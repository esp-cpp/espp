#!/usr/bin/env bash
# In-container runner for the QEMU interop matrix (see run_qemu.sh and
# .github/workflows/rtps_qemu.yml): the embedded RTPS engine, as the esp32
# firmware in qemu/, running in Espressif's QEMU on a tap device, against ROS 2
# (FastDDS, then CycloneDDS) and the espp host engine. Every case drives the
# host side and asserts on the device's serial log (the "RTPSTEST ..." markers
# the firmware prints; see qemu/main/rtps_qemu.cpp).
#
# Usage: run_qemu_interop.sh /work/components/rtps/interop/qemu/build/qemu_flash.bin
#
# Network: the container runs with --network none, so the tap device created
# here is its only interface and every DDS implementation uses it without
# per-vendor interface configuration. Host side 192.168.100.1, device
# 192.168.100.2 (sdkconfig.ci.defaults).
#
# NOTE: no `set -u` - ROS 2's setup.bash references unset variables.
set -o pipefail

FLASH="${1:?usage: run_qemu_interop.sh <qemu_flash.bin>}"
QEMU="${QEMU:-/opt/qemu/bin/qemu-system-xtensa}"
TAP=tap0
HOST_IP=192.168.100.1
DEVICE_IP=192.168.100.2
# the device log goes next to the image (on the bind mount) so CI can show it
LOG="$(dirname "$FLASH")/qemu-serial.log"

PASS=0
FAIL=0
note() { echo -e "\n===== $* ====="; }
result() { # name exit_code
  if [ "$2" -eq 0 ]; then echo "RESULT PASS: $1"; PASS=$((PASS+1));
  else echo "RESULT FAIL: $1"; FAIL=$((FAIL+1)); fi
}
# device-log helpers: lines are counted from a mark taken before each case
mark() { wc -l < "$LOG"; }
since() { tail -n +"$(( $1 + 1 ))" "$LOG"; }           # lines after mark $1
count_since() { since "$1" | grep -c -- "$2"; }         # occurrences of $2 after mark $1
wait_for_marker() { # marker timeout_s
  local i
  for i in $(seq 1 "$2"); do grep -q -- "$1" "$LOG" 2>/dev/null && return 0; sleep 1; done
  return 1
}
# Between cases: the ros2 CLI participants of the previous case are gone, but
# the device forgets a peer only when its (capped, 8 s) lease expires; give
# it that, so no case starts against pools still holding dead peers. Also
# print the device's latest status line, the first thing to read on a failure.
settle() {
  sleep 10
  grep -o "RTPSTEST status.*" "$LOG" | tail -1 | sed 's/^/  device: /'
  echo "  device: pool-exhaustion messages so far $(grep -c 'RESSOURCE LIMIT EXCEEDED' "$LOG"), RX frames dropped $(grep -c 'RX frame dropped' "$LOG")"
}

# --- tap device ----------------------------------------------------------------
note "Tap device $TAP ($HOST_IP/24), the container's only interface"
ip tuntap add dev $TAP mode tap || { echo "cannot create $TAP (needs --cap-add NET_ADMIN --device /dev/net/tun)"; exit 2; }
ip addr add $HOST_IP/24 dev $TAP
ip link set $TAP up
ip route add 224.0.0.0/4 dev $TAP 2>/dev/null || true
ip addr show $TAP | sed 's/^/  /'

# --- host side: espp lib + the two interop binaries ------------------------------
note "Copy sources to container-local tree, build the espp host engine's interop binaries"
rsync -a --delete --exclude '.git/' --exclude 'build/' --exclude 'build-*/' \
  --exclude 'managed_components/' --exclude 'docs/' --exclude 'dependencies.lock' /work/ /tmp/espp/
cd /tmp/espp
cmake -S lib -B lib/build -DCMAKE_BUILD_TYPE=Release -DESPP_INSTALL=ON -DCMAKE_INSTALL_PREFIX=/tmp/espp/install > /tmp/cmake_lib.log 2>&1 \
  && cmake --build lib/build -j"$(nproc)" --target install > /tmp/build_lib.log 2>&1 \
  && cmake -S pc -B pc/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/tmp/espp/install > /tmp/cmake.log 2>&1 \
  && cmake --build pc/build -j"$(nproc)" --target rtps_interop_pub rtps_interop_sub > /tmp/build.log 2>&1
build_rc=$?
result "host_build" $build_rc
if [ $build_rc -ne 0 ]; then tail -30 /tmp/cmake_lib.log /tmp/build_lib.log /tmp/cmake.log /tmp/build.log; exit 1; fi
BIN=pc/build

# --- the device -------------------------------------------------------------------
note "Boot the firmware in QEMU ($FLASH)"
: > "$LOG"
"$QEMU" -M esp32 -m 4M \
  -drive file="$FLASH",if=mtd,format=raw \
  -global driver=timer.esp32.timg,property=wdt_disable,value=true \
  -nic tap,ifname=$TAP,script=no,downscript=no,model=open_eth \
  -nographic -monitor none -serial file:"$LOG" > /tmp/qemu-stderr.log 2>&1 &
QEMU_PID=$!
cleanup() { kill $QEMU_PID 2>/dev/null; wait $QEMU_PID 2>/dev/null || true; }
trap cleanup EXIT
if wait_for_marker "RTPSTEST ready" 90; then
  grep -o "RTPSTEST ready.*" "$LOG" | head -1
  result "device_boot" 0
else
  echo "device did not come up; log tail:"; sed 's/\x1b\[[0-9;]*m//g' "$LOG" | tail -40; cat /tmp/qemu-stderr.log
  result "device_boot" 1
  exit 1
fi
ping -c 2 -W 2 $DEVICE_IP > /tmp/ping.log 2>&1; result "device_ping" $?
wait_for_marker "RTPSTEST status" 10
FIRST_HEAP=$(grep -o " heap=[0-9]*" "$LOG" | tail -1 | cut -d= -f2)

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 # matches the engine's Config::DOMAIN_ID

# --- the ROS 2 cases, run once per rmw -----------------------------------------------
ros2_cases() { # prefix log-tag
  local p="$1" t="$2" m

  note "${p}device publisher -> ROS 2 subscriber (reliable, /qemu_tx)"
  timeout 30 ros2 topic echo --once /qemu_tx std_msgs/msg/String > "/tmp/${t}_echo.log" 2>&1
  local rc=$?; grep -q "qemu " "/tmp/${t}_echo.log" || rc=1
  head -3 "/tmp/${t}_echo.log"; result "${p}device_pub->ros2_echo" $rc
  settle

  note "${p}ROS 2 publisher -> device subscriber (reliable, /qemu_rx)"
  m=$(mark)
  timeout 32 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'ros2 to qemu'" > /dev/null 2>&1 &
  local pub=$!; sleep 30; kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
  local n; n=$(count_since "$m" "RTPSTEST rx ros2 to qemu"); echo "device received $n"
  [ "$n" -ge 5 ]; result "${p}ros2_pub->device_sub" $?
  settle

  note "${p}device best-effort publisher -> ROS 2 subscriber (/qemu_be_tx)"
  timeout 30 ros2 topic echo --once --qos-reliability best_effort /qemu_be_tx std_msgs/msg/String > "/tmp/${t}_beecho.log" 2>&1
  rc=$?; grep -q "qemu_be " "/tmp/${t}_beecho.log" || rc=1
  head -3 "/tmp/${t}_beecho.log"; result "${p}device_be_pub->ros2_echo" $rc
  settle

  note "${p}ROS 2 best-effort publisher -> device subscriber (/qemu_be_rx)"
  m=$(mark)
  timeout 32 ros2 topic pub --qos-reliability best_effort -r 5 /qemu_be_rx std_msgs/msg/String "data: 'be to qemu'" > /dev/null 2>&1 &
  pub=$!; sleep 30; kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
  n=$(count_since "$m" "RTPSTEST be_rx be to qemu"); echo "device received $n"
  [ "$n" -ge 3 ]; result "${p}ros2_be_pub->device_be_sub" $?
  settle

  note "${p}ROS 2 publisher -> device subscriber, samples still arriving 22 s in (liveliness past the lease)"
  timeout 36 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'still alive'" > /dev/null 2>&1 &
  pub=$!; sleep 24; m=$(mark); sleep 10; kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
  n=$(count_since "$m" "RTPSTEST rx still alive"); echo "device received $n in the 24-34 s window"
  [ "$n" -ge 10 ]; result "${p}ros2_pub->device_sub_liveliness" $?
  settle

  note "${p}twelve other participants on the domain, then the publisher the device needs joins last"
  local crowd=() i
  for i in $(seq 1 12); do
    timeout 45 ros2 topic pub -r 1 "/crowd$i" std_msgs/msg/String "data: 'crowd'" > /dev/null 2>&1 &
    crowd+=("$!")
  done
  sleep 10
  m=$(mark)
  timeout 32 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'late joiner'" > /dev/null 2>&1 &
  pub=$!; sleep 30; kill $pub "${crowd[@]}" 2>/dev/null; wait $pub "${crowd[@]}" 2>/dev/null || true
  n=$(count_since "$m" "RTPSTEST rx late joiner"); echo "device received $n"
  [ "$n" -ge 3 ]; result "${p}many_participants_late_joiner" $?
  settle

  note "${p}ROS 2 publisher -> device subscriber, 30 KB samples (DATA_FRAG over lwIP, /qemu_big)"
  m=$(mark)
  local big; big=$(head -c 30000 /dev/zero | tr '\0' x)
  timeout 40 ros2 topic pub -t 12 -r 0.5 /qemu_big std_msgs/msg/String "data: '$big'" > /dev/null 2>&1
  n=$(count_since "$m" "RTPSTEST big_rx len=30000"); echo "device reassembled $n"
  [ "$n" -ge 1 ]; result "${p}ros2_pub->device_sub_frag_30k" $?
  settle
}

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2_cases "fastdds: " fastdds

note "fastdds: ROS 2 service client -> device service server (/qemu_add_two_ints)"
m=$(mark)
timeout 50 ros2 service call /qemu_add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}" > /tmp/srv.log 2>&1
rc=$?; grep -q "sum=42" /tmp/srv.log || rc=1
[ "$(count_since "$m" "RTPSTEST srv 7+35=42")" -ge 1 ] || rc=1
tail -2 /tmp/srv.log; result "fastdds: ros2_client->device_service" $rc
settle

# --- espp host engine <-> device ---------------------------------------------------------
note "espp host publisher -> device subscriber (reliable, /qemu_rx)"
m=$(mark)
"$BIN"/rtps_interop_pub rt/qemu_rx std_msgs::msg::dds_::String_ 1 60 200 $HOST_IP > /tmp/hostpub.log 2>&1 &
pub=$!; sleep 16; kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
n=$(count_since "$m" "RTPSTEST rx "); echo "device received $n"
[ "$n" -ge 5 ]; result "espp_host_pub->device_sub" $?
settle

note "device publisher -> espp host subscriber (reliable, /qemu_tx)"
"$BIN"/rtps_interop_sub rt/qemu_tx std_msgs::msg::dds_::String_ 1 5 30 $HOST_IP > /tmp/hostsub.log 2>&1
rc=$?; tail -2 /tmp/hostsub.log; result "device_pub->espp_host_sub" $rc
settle

note "espp host publisher -> device subscriber, 30 KB samples as DATA_FRAG (1400-byte fragments, /qemu_big)"
# The ROS 2 peers send a 30 KB sample as one IP-fragmented UDP datagram (lwIP
# reassembly); the host engine with a fragment size sends RTPS DATA_FRAG
# submessages, so this exercises the engine's own reassembly instead.
m=$(mark)
"$BIN"/rtps_interop_pub rt/qemu_big std_msgs::msg::dds_::String_ 1 12 2000 $HOST_IP 30000 1400 > /tmp/hostfrag.log 2>&1 &
pub=$!; sleep 30; kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
n=$(count_since "$m" "RTPSTEST big_rx len=30000"); echo "device reassembled $n"
[ "$n" -ge 1 ]; result "espp_host_frag_30k->device_sub" $?
settle

# --- fault injection: loss and delay on the device's link ------------------------------------
note "15% loss + 10 ms jitter on $TAP: ROS 2 publisher -> device reliable subscriber still delivers"
tc qdisc add dev $TAP root netem loss 15% delay 10ms 3ms
m=$(mark)
timeout 42 ros2 topic pub -r 10 /qemu_rx std_msgs/msg/String "data: 'lossy'" > /dev/null 2>&1 &
pub=$!; sleep 40; kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
n=$(count_since "$m" "RTPSTEST rx lossy"); echo "device received $n of ~400 under 15% loss"
[ "$n" -ge 100 ]; result "netem_loss_reliable" $?
tc qdisc del dev $TAP root
settle

# --- CycloneDDS pass ---------------------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2_cases "cyclone: " cyclone
unset CYCLONEDDS_URI

# --- the device after all of that -----------------------------------------------------------------
note "Device health over the run"
LAST_HEAP=$(grep -o " heap=[0-9]*" "$LOG" | tail -1 | cut -d= -f2)
echo "free heap first=$FIRST_HEAP last=$LAST_HEAP"
[ -n "$FIRST_HEAP" ] && [ -n "$LAST_HEAP" ] && [ "$LAST_HEAP" -ge $((FIRST_HEAP - 65536)) ]; result "device_heap_stable" $?
! grep -qE "Guru Meditation|abort\(\)|assert failed|RTPSTEST fatal" "$LOG"; result "device_no_crash" $?
grep -o "RTPSTEST status.*" "$LOG" | tail -1
# informational: pressure the run put on the device (pool exhaustion, frames
# the emulated MAC dropped, endpoints skipped for their type-name length)
echo "device pool-exhaustion messages: $(grep -c 'RESSOURCE LIMIT EXCEEDED' "$LOG")"
echo "device RX frames dropped by the emulated MAC: $(grep -c 'RX frame dropped' "$LOG")"
echo "device type names over the cap: $(grep -c 'exceeds maximum allowed length' "$LOG")"

echo ""
echo "==================== SUMMARY ===================="
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "QEMU INTEROP PASS" || echo "QEMU INTEROP FAIL"
exit $FAIL
