#!/usr/bin/env bash
# In-container runner for the QEMU interop matrix (see run_qemu.sh and
# .github/workflows/rtps_qemu.yml): the embedded RTPS engine, as the esp32
# firmware in qemu/, running in Espressif's QEMU on a tap device, against ROS 2
# (FastDDS, then CycloneDDS) and the espp host engine. Every case drives the
# host side and asserts on the device's serial log (the "RTPSTEST ..." markers
# the firmware prints; see qemu/main/rtps_qemu.cpp).
#
# Usage: run_qemu_interop.sh /work/components/rtps/interop/qemu/build/qemu_flash.bin [matrix|soak]
#
# Suites (CI runs both, in parallel, on one firmware build):
#   matrix  the interop cases against FastDDS, CycloneDDS and the host engine,
#           then fault injection on the link (tc netem: loss, reordering,
#           duplication, delay, corruption) and the device's health
#   soak    minutes of churn - participants coming and going, matches, service
#           calls, 30 KB samples, both rmws alternating - and the heap must not
#           trend down afterwards
#
# Network: the container runs with --network none, so the tap device created
# here is its only interface and every DDS implementation uses it without
# per-vendor interface configuration. Host side 192.168.100.1, device
# 192.168.100.2 (sdkconfig.ci.defaults).
#
# NOTE: no `set -u` - ROS 2's setup.bash references unset variables.
set -o pipefail

FLASH="${1:?usage: run_qemu_interop.sh <qemu_flash.bin> [matrix|soak]}"
SUITE="${2:-matrix}"
QEMU="${QEMU:-/opt/qemu/bin/qemu-system-xtensa}"
[ -x "$QEMU" ] || { echo "no $QEMU in this image: Espressif's QEMU is x86_64-only (build the image on an amd64 host, or docker build --platform linux/amd64; or point QEMU= at another build)"; exit 2; }
TAP=tap0
HOST_IP=192.168.100.1
DEV_IP=192.168.100.2 # sdkconfig.ci.defaults
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
# Stop background peers: TERM, up to 10 s for them to go, then KILL. A ros2 CLI
# process occasionally does not exit on TERM (nor on the TERM its `timeout`
# wrapper sends at expiry, which has no -k), and a plain `wait` on it then hung
# the whole suite until the job's 75-minute limit.
stop_pids() { # pids...
  [ $# -gt 0 ] || return 0
  kill "$@" 2>/dev/null
  local i
  for i in $(seq 1 50); do
    kill -0 "$@" 2>/dev/null || break
    sleep 0.2
  done
  kill -9 "$@" 2>/dev/null
  wait "$@" 2>/dev/null || true
}
# A capture of the frames towards the device, in two lines: how many, how
# many of them IP fragments (more-fragments set or a non-zero offset) against
# whole datagrams, how many to multicast, the biggest burst (frames within any
# 20 ms), and the busiest destinations. The emulated MAC's ring is the
# device's only buffer against a burst, and the emulator puts no wire time
# between frames.
wire_summary() { # pcap
  [ -s "$1" ] || { echo "  wire: no capture"; return; }
  local total frag mcast burst
  total=$(tcpdump -nn -r "$1" 2>/dev/null | wc -l)
  frag=$(tcpdump -nn -r "$1" 'ip[6:2] & 0x3fff != 0' 2>/dev/null | wc -l)
  mcast=$(tcpdump -nn -r "$1" 'ip dst net 224.0.0.0/4' 2>/dev/null | wc -l)
  burst=$(tcpdump -nn -tt -r "$1" 2>/dev/null | awk '{t[NR]=$1} END {b=0; j=1; for (i=1; i<=NR; i++) { while (t[i]-t[j] > 0.02) j++; if (i-j+1 > b) b=i-j+1 } print b}')
  echo "  wire: $total frames towards the device: $frag IP fragments, $((total - frag)) whole datagrams, $mcast to multicast; biggest burst $burst frames within 20 ms"
  echo "  wire: busiest destinations: $(tcpdump -nn -r "$1" 'ip[6:2] & 0x3fff = 0' 2>/dev/null | awk '{print $5}' | sed 's/:$//' | sort | uniq -c | sort -rn | head -4 | awk '{printf "%s x%s  ", $2, $1}')"
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
cleanup() { stop_pids $QEMU_PID; }
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

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export ROS_DOMAIN_ID=0 # matches the engine's Config::DOMAIN_ID

run_matrix() {
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
  local pub=$!; sleep 30; stop_pids $pub
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
  pub=$!; sleep 30; stop_pids $pub
  n=$(count_since "$m" "RTPSTEST be_rx be to qemu"); echo "device received $n"
  [ "$n" -ge 3 ]; result "${p}ros2_be_pub->device_be_sub" $?
  settle

  note "${p}ROS 2 publisher -> device subscriber, samples still arriving 22 s in (liveliness past the lease)"
  timeout 36 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'still alive'" > /dev/null 2>&1 &
  pub=$!; sleep 24; m=$(mark); sleep 10; stop_pids $pub
  n=$(count_since "$m" "RTPSTEST rx still alive"); echo "device received $n in the 24-34 s window"
  [ "$n" -ge 10 ]; result "${p}ros2_pub->device_sub_liveliness" $?
  settle

  note "${p}ROS 2 service client -> device service server (/qemu_add_two_ints)"
  # the one RPC the device serves: under rmw_cyclonedds_cpp the request carries
  # the in-payload {guid, seq} header instead of related_sample_identity, and
  # the device must answer in that form (rtps/rpc/cyclonedds_request_header.hpp)
  m=$(mark)
  timeout 50 ros2 service call /qemu_add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}" > "/tmp/${t}_srv.log" 2>&1
  rc=$?; grep -q "sum=42" "/tmp/${t}_srv.log" || rc=1
  [ "$(count_since "$m" "RTPSTEST srv 7+35=42")" -ge 1 ] || rc=1
  tail -2 "/tmp/${t}_srv.log"; result "${p}ros2_client->device_service" $rc
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
  pub=$!; sleep 30; stop_pids $pub "${crowd[@]}"
  n=$(count_since "$m" "RTPSTEST rx late joiner"); echo "device received $n"
  [ "$n" -ge 3 ]; result "${p}many_participants_late_joiner" $?
  settle

  note "${p}ROS 2 publisher -> device subscriber, 30 KB samples (DATA_FRAG over lwIP, /qemu_big)"
  m=$(mark)
  local big; big=$(head -c 30000 /dev/zero | tr '\0' x)
  # the wire during this case, summarised below: what one 30 KB sample is
  # as frames, and how tight the burst is (the emulated MAC's ring is the
  # device's only buffer against a burst with no wire time between frames)
  local cap="/tmp/${t}_frag.pcap"
  tcpdump -i "$TAP" -nn -w "$cap" "udp and (dst host $DEV_IP or dst net 224.0.0.0/4)" > /dev/null 2>&1 &
  local tcpd=$!; sleep 1
  timeout 40 ros2 topic pub -t 12 -r 0.5 /qemu_big std_msgs/msg/String "data: '$big'" > /dev/null 2>&1
  sleep 1; stop_pids $tcpd
  n=$(count_since "$m" "RTPSTEST big_rx len=30000"); echo "device reassembled $n"
  wire_summary "$cap"
  [ "$n" -ge 1 ]; result "${p}ros2_pub->device_sub_frag_30k" $?
  settle

  note "${p}more participants than the remote-participant pool holds (28 against 24), then all leave: discovery keeps going and the pool recovers"
  crowd=()
  for i in $(seq 1 28); do
    timeout 50 ros2 topic pub -r 1 "/flood$i" std_msgs/msg/String "data: 'flood'" > /dev/null 2>&1 &
    crowd+=("$!")
  done
  sleep 25
  echo "  device: $(grep -o 'RTPSTEST status.*' "$LOG" | tail -1)"
  stop_pids "${crowd[@]}"
  settle
  sleep 6 # the capped lease (8 s) plus one liveliness sweep: every one of them forgotten
  m=$(mark)
  timeout 32 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'after the flood'" > /dev/null 2>&1 &
  pub=$!; sleep 30; stop_pids $pub
  n=$(count_since "$m" "RTPSTEST rx after the flood"); echo "device received $n"
  [ "$n" -ge 5 ]; result "${p}participant_pool_overflow_recovers" $?
  settle
}

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2_cases "fastdds: " fastdds

# --- a full pool, and a late participant the device needs -------------------------------
note "pool full of participants the device has no use for (28 against 24) and they STAY; the publisher it needs joins last: an unused one is forgotten for it"
# Unlike participant_pool_overflow_recovers (everyone leaves and the leases free
# the slots) the crowd keeps announcing here, so the only way in for the late
# publisher is the eviction of a participant none of the device's endpoints
# match (Config::SPDP_EVICT_UNUSED_PARTICIPANTS: oldest first, one per
# announcement period, logged by the device). With this build's knobs the
# builtin proxy pools (20) fill before the participant pool (24), so this is
# the proxy-pool trigger of the eviction that is exercised here. The rejected
# members of the crowd re-announce every few seconds and each takes the slot
# of another unused one; the publisher gets its slot the same way, and once
# its writer matches the device's reader it is in use and stays.
#
# Timing: 28 Python ROS 2 nodes started at once take 30 to 60 s to come up on
# a CI runner (they were all admitted within the same few seconds in the first
# run of this case, 40 s after launch), and the publisher started afterwards
# needs as long itself. So wait for the crowd to be up - the device's first
# eviction says the pools are full and the crowd keeps knocking - before the
# publisher starts, and give the publisher's samples time to show up.
crowd=()
for i in $(seq 1 28); do
  timeout 240 ros2 topic pub -r 1 "/stay$i" std_msgs/msg/String "data: 'stay'" > /dev/null 2>&1 &
  crowd+=("$!")
done
m0=$(mark)
for i in $(seq 1 120); do
  [ "$(count_since "$m0" "forgetting an unused participant")" -ge 1 ] && break
  sleep 1
done
echo "  crowd up after ${i} s (first eviction seen: $([ "$i" -lt 120 ] && echo yes || echo no))"
sleep 10
echo "  device: $(grep -o 'RTPSTEST status.*' "$LOG" | tail -1)"
m=$(mark)
timeout 100 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'made room for me'" > /dev/null 2>&1 &
pub=$!
n=0
for i in $(seq 1 45); do
  n=$(count_since "$m" "RTPSTEST rx made room for me")
  [ "$n" -ge 5 ] && break
  sleep 2
done
stop_pids $pub "${crowd[@]}"
e=$(count_since "$m" "forgetting an unused participant")
echo "device received $n (after $((i * 2)) s); evictions logged since the publisher started: $e"
[ "$n" -ge 5 ]; result "pool_full_evicts_unused_for_late_joiner" $?
settle
sleep 10 # the crowd's capped leases (8 s) run out before the next case

# --- espp host engine <-> device ---------------------------------------------------------
note "espp host publisher -> device subscriber (reliable, /qemu_rx)"
m=$(mark)
"$BIN"/rtps_interop_pub rt/qemu_rx std_msgs::msg::dds_::String_ 1 60 200 $HOST_IP > /tmp/hostpub.log 2>&1 &
pub=$!; sleep 16; stop_pids $pub
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
pub=$!; sleep 30; stop_pids $pub
n=$(count_since "$m" "RTPSTEST big_rx len=30000"); echo "device reassembled $n"
[ "$n" -ge 1 ]; result "espp_host_frag_30k->device_sub" $?
settle

# --- fault injection: loss and delay on the device's link ------------------------------------
note "15% loss + 10 ms jitter on $TAP: ROS 2 publisher -> device reliable subscriber still delivers"
tc qdisc add dev $TAP root netem loss 15% delay 10ms 3ms
m=$(mark)
timeout 42 ros2 topic pub -r 10 /qemu_rx std_msgs/msg/String "data: 'lossy'" > /dev/null 2>&1 &
pub=$!; sleep 40; stop_pids $pub
n=$(count_since "$m" "RTPSTEST rx lossy"); echo "device received $n of ~400 under 15% loss"
[ "$n" -ge 100 ]; result "netem_loss_reliable" $?
tc qdisc del dev $TAP root
settle

# --- fault injection beyond loss: reordering, duplication, corruption, delay ---------------------
# tc netem on $TAP shapes what the host sends towards the device. The host
# engine's samples are numbered ("espp interop N", printed by the device as
# "RTPSTEST rx espp interop N" / "be_rx ..."), so order is checkable: a reliable
# reader must deliver every sample exactly once and in order whatever the link
# does; a best-effort reader may lose samples but must never deliver one twice
# or out of order (RTPS 2.5 8.4.12.1).
numbers_since() { since "$1" | grep -o "RTPSTEST $2 espp interop [0-9]*" | awk '{print $NF}'; }
strictly_increasing() { awk 'NR > 1 && $1 <= prev { bad = 1 } { prev = $1 } END { exit bad }'; }
netem_case() { # name netem-args topic reliable(0|1) marker min-count max-count
  local name="$1" args="$2" topic="$3" reliable="$4" marker="$5" min="$6" max="$7" m n rc=0
  tc qdisc add dev $TAP root netem $args
  m=$(mark)
  "$BIN"/rtps_interop_pub "$topic" std_msgs::msg::dds_::String_ "$reliable" 200 100 $HOST_IP > "/tmp/${name}.log" 2>&1 &
  local pub=$!; sleep 27; stop_pids $pub
  tc qdisc del dev $TAP root
  n=$(numbers_since "$m" "$marker" | wc -l | tr -d ' ')
  if numbers_since "$m" "$marker" | strictly_increasing; then
    echo "device received $n of 200, in order, none twice"
  else
    echo "device received $n of 200, OUT OF ORDER or a duplicate:"; numbers_since "$m" "$marker" | tr '\n' ' ' | cut -c1-400; echo; rc=1
  fi
  [ "$n" -ge "$min" ] && [ "$n" -le "$max" ] || rc=1
  result "$name" $rc
  settle
}

# The delay has to exceed the 100 ms sample spacing for samples (not just
# packets) to cross: a reordered packet goes out at once, the rest 150 ms later.
note "30% reordering across a 150 ms delay: host reliable publisher -> device, every sample once and in order"
netem_case netem_reorder_reliable "delay 150ms 20ms reorder 30% 50%" rt/qemu_rx 1 rx 150 200
note "30% reordering: host best-effort publisher -> device, late samples rejected, never out of order"
netem_case netem_reorder_best_effort "delay 150ms 20ms reorder 30% 50%" rt/qemu_be_rx 0 be_rx 60 200
note "30% duplication: host reliable publisher -> device, every sample exactly once"
netem_case netem_duplicate_reliable "duplicate 30%" rt/qemu_rx 1 rx 190 200
note "30% duplication: host best-effort publisher -> device, no sample twice"
netem_case netem_duplicate_best_effort "duplicate 30%" rt/qemu_be_rx 0 be_rx 150 200
note "2% corruption: host reliable publisher -> device keeps delivering (the engine survives the rest of the run)"
netem_case netem_corrupt_reliable "corrupt 2%" rt/qemu_rx 1 rx 150 200

note "300 ms (+-50) delay on the link from before discovery: a ROS 2 publisher is still discovered, matched and delivered"
tc qdisc add dev $TAP root netem delay 300ms 50ms
m=$(mark)
timeout 32 ros2 topic pub -r 5 /qemu_rx std_msgs/msg/String "data: 'slow link'" > /dev/null 2>&1 &
pub=$!; sleep 30; stop_pids $pub
tc qdisc del dev $TAP root
n=$(count_since "$m" "RTPSTEST rx slow link"); echo "device received $n over a 600 ms round trip"
[ "$n" -ge 20 ]; result "netem_delay_discovery" $?
settle

note "15% loss + 10 ms jitter: device reliable publisher -> host subscriber (the device writer's repairs, against lost ACKNACKs)"
tc qdisc add dev $TAP root netem loss 15% delay 10ms 3ms
"$BIN"/rtps_interop_sub rt/qemu_tx std_msgs::msg::dds_::String_ 1 100 45 $HOST_IP > /tmp/hostsub_loss.log 2>&1
rc=$?; tail -1 /tmp/hostsub_loss.log; result "netem_loss_device_pub" $rc
tc qdisc del dev $TAP root
settle

note "15% loss + 10 ms jitter both ways: ROS 2 service call -> device service, each RMW (a lost request or reply is repaired, not retried by the client)"
# ros2 service call sends ONE request and waits: the only way the reply comes
# back under loss is the reliable writers on both ends repairing the lost
# DATA (and the HEARTBEAT/ACKNACK that carry the repair) within the timeout.
# The root netem only impairs the host -> device direction (the request); the
# reply comes back on the TAP's ingress, so it gets its own ~14% drop there
# (an ingress qdisc with a random gact, no IFB needed) or the device's reply
# writer would never have to repair anything. Best effort: an iproute2 without
# the gact action leaves the case host->device only, and says so.
tc qdisc add dev $TAP root netem loss 15% delay 10ms 3ms
reply_loss=""
if tc qdisc add dev $TAP ingress 2>/dev/null \
   && tc filter add dev $TAP parent ffff: protocol ip prio 1 u32 match u32 0 0 action pass random netrand drop 7 2>/dev/null; then
  reply_loss="yes"
else
  echo "  (no ingress drop available in this image: the device -> host reply leg runs lossless)"
  tc qdisc del dev $TAP ingress 2>/dev/null
fi
for rmw in rmw_fastrtps_cpp rmw_cyclonedds_cpp; do
  export RMW_IMPLEMENTATION=$rmw
  tag=${rmw#rmw_}; tag=${tag%_cpp}; [ "$tag" = fastrtps ] && tag=fastdds; [ "$tag" = cyclonedds ] && tag=cyclone
  m=$(mark)
  timeout 50 ros2 service call /qemu_add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}" > "/tmp/srv_loss_${tag}.log" 2>&1
  rc=$?; grep -q "sum=7" "/tmp/srv_loss_${tag}.log" || rc=1
  [ "$(count_since "$m" "RTPSTEST srv 3+4=7")" -ge 1 ] || rc=1
  tail -1 "/tmp/srv_loss_${tag}.log"; result "netem_loss_service_${tag}" $rc
  settle
done
[ -n "$reply_loss" ] && tc qdisc del dev $TAP ingress 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
tc qdisc del dev $TAP root

# --- CycloneDDS pass ---------------------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2_cases "cyclone: " cyclone
unset CYCLONEDDS_URI

}

device_health() {
note "Device health over the run"
LAST_HEAP=$(grep -o " heap=[0-9]*" "$LOG" | tail -1 | cut -d= -f2)
echo "free heap first=$FIRST_HEAP last=$LAST_HEAP"
# Known, bounded growth: two buffers grow once to the largest sample (30 KB
# each, measured 60 KB over a run) when the first 30 KB samples arrive; the
# slack leaves 36 KB on top of that for anything that leaks per case.
[ -n "$FIRST_HEAP" ] && [ -n "$LAST_HEAP" ] && [ "$LAST_HEAP" -ge $((FIRST_HEAP - 98304)) ]; result "device_heap_stable" $?
! grep -qE "Guru Meditation|abort\(\)|assert failed|RTPSTEST fatal" "$LOG"; result "device_no_crash" $?
grep -o "RTPSTEST status.*" "$LOG" | tail -1
# informational: pressure the run put on the device (pool exhaustion, frames
# the emulated MAC dropped, endpoints skipped for their type-name length)
echo "device pool-exhaustion messages: $(grep -c 'RESSOURCE LIMIT EXCEEDED' "$LOG")"
echo "device RX frames dropped by the emulated MAC: $(grep -c 'RX frame dropped' "$LOG")"
echo "device type names over the cap: $(grep -c 'exceeds maximum allowed length' "$LOG")"
}

# --- the soak ------------------------------------------------------------------------------------
latest_heap() { grep -o " heap=[0-9]*" "$LOG" | tail -1 | cut -d= -f2; }
latest_min_heap() { grep -o "min_heap=[0-9]*" "$LOG" | tail -1 | cut -d= -f2; }
run_soak() {
  local cycles="${SOAK_CYCLES:-20}"
  note "Soak: $cycles cycles of churn, FastDDS and CycloneDDS alternating; the heap must not trend down"
  local big; big=$(head -c 30000 /dev/zero | tr '\0' x)
  # One cycle: six ROS 2 participants and two host-engine ones come, match the
  # device's endpoints every way they can (reliable and best-effort in, the
  # device's publisher out, a 30 KB sample, a service call), and go. 12 s of
  # traffic, 2 s of quiet, so the previous cycle's leases (capped at 8 s) run
  # out part way into the next: pools and proxies churn as peers do.
  soak_cycle() { # rmw cycle
    export RMW_IMPLEMENTATION="$1"
    if [ "$1" = rmw_cyclonedds_cpp ]; then
      export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
    else
      unset CYCLONEDDS_URI
    fi
    local pids=()
    timeout 14 ros2 topic pub -r 20 /qemu_rx std_msgs/msg/String "data: 'soak $2'" > /dev/null 2>&1 & pids+=($!)
    timeout 14 ros2 topic pub --qos-reliability best_effort -r 10 /qemu_be_rx std_msgs/msg/String "data: 'soak be $2'" > /dev/null 2>&1 & pids+=($!)
    timeout 14 ros2 topic pub -t 3 -r 0.5 /qemu_big std_msgs/msg/String "data: '$big'" > /dev/null 2>&1 & pids+=($!)
    timeout 14 ros2 topic echo --once /qemu_tx std_msgs/msg/String > /dev/null 2>&1 & pids+=($!)
    timeout 14 ros2 topic echo --once --qos-reliability best_effort /qemu_be_tx std_msgs/msg/String > /dev/null 2>&1 & pids+=($!)
    # the service call keeps its log: it is asserted below (soak_service_calls),
    # not just run for the churn
    timeout 14 ros2 service call /qemu_add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: $2}" > "/tmp/soak_srv_$2.log" 2>&1 & pids+=($!)
    "$BIN"/rtps_interop_pub rt/qemu_rx std_msgs::msg::dds_::String_ 1 40 200 $HOST_IP > /dev/null 2>&1 & pids+=($!)
    "$BIN"/rtps_interop_sub rt/qemu_tx std_msgs::msg::dds_::String_ 1 5 12 $HOST_IP > /dev/null 2>&1 & pids+=($!)
    sleep 12
    stop_pids "${pids[@]}"
    if grep -q "sum=$((1 + $2))" "/tmp/soak_srv_$2.log"; then
      srv_ok=$((srv_ok + 1))
    else
      echo "  service call of cycle $2 (${1#rmw_}) was not answered: $(tail -1 "/tmp/soak_srv_$2.log" | cut -c1-120)"
    fi
    sleep 2
  }

  # warm-up: whatever grows once to the largest sample grows here, outside the measurement
  local srv_ok=0
  soak_cycle rmw_fastrtps_cpp 0
  srv_ok=0 # the warm-up's call is not part of the count
  sleep 14 # lease + one sweep: the warm-up's peers forgotten before the baseline
  local base_heap base_min; base_heap=$(latest_heap); base_min=$(latest_min_heap)
  echo "baseline after the warm-up: heap=$base_heap min_heap=$base_min"
  echo "  device: $(grep -o 'RTPSTEST status.*' "$LOG" | tail -1)"

  local i rmw m_last h mn new_lows=0 prev_min="$base_min"
  local -a heaps=()
  for i in $(seq 1 "$cycles"); do
    rmw=rmw_fastrtps_cpp; [ $((i % 2)) -eq 0 ] && rmw=rmw_cyclonedds_cpp
    m_last=$(mark)
    soak_cycle "$rmw" "$i"
    h=$(latest_heap); heaps+=("$h")
    # a new low-water mark counts when it is a real step down (1 KB or more):
    # the Cyclone cycles' transient allocations (a service call's request and
    # reply buffers among them) move the mark by tens of bytes from cycle to
    # cycle, which is fragmentation jitter, not growth
    # prev_min is the last COUNTED low, so a decline that creeps down by less
    # than 1 KB per cycle still adds up to a counted step
    mn=$(latest_min_heap)
    if [ "$mn" -le $((prev_min - 1024)) ]; then new_lows=$((new_lows + 1)); prev_min="$mn"; fi
    echo "cycle $i ${rmw#rmw_}: $(grep -o 'RTPSTEST status.*' "$LOG" | tail -1 | sed 's/RTPSTEST status //')"
  done
  local last_rx last_be last_big; last_rx=$(count_since "$m_last" "RTPSTEST rx "); last_be=$(count_since "$m_last" "RTPSTEST be_rx "); last_big=$(count_since "$m_last" "RTPSTEST big_rx ")
  sleep 14 # every soak peer forgotten
  local end_heap end_min; end_heap=$(latest_heap); end_min=$(latest_min_heap)
  echo "heap after the warm-up $base_heap, after cycle $((cycles / 2)) ${heaps[$((cycles / 2 - 1))]}, at the end $end_heap (free, bytes); low-water mark $base_min -> $end_min, $new_lows new low(s) of 1 KB or more over the cycles"
  echo "last cycle: reliable $last_rx, best-effort $last_be, 30 KB $last_big samples delivered"
  echo "pool-exhaustion messages over the soak: $(grep -c 'RESSOURCE LIMIT EXCEEDED' "$LOG"), RX frames dropped: $(grep -c 'RX frame dropped' "$LOG")"
  # Free heap at the end within 8 KB of the baseline: a leak of 500 bytes per
  # cycle would show. The low-water mark: a transient that grows with the
  # cycles sets a new low at most of them, so at most 5 new lows are allowed
  # over the 20, and the mark must stay within 96 KB of the baseline. A global
  # low-water mark finds a few new combinations of transients early in a run
  # (three in 20 cycles, 40 KB in all, seen; a 64 KB one-off seen too, at a
  # different point each run, the free heap unmoved - most likely the 64 KB
  # per-datagram read buffer each receive handler uses with fragmentation on,
  # the first time one more handler than before ran at once, bounded by the
  # number of sockets).
  [ -n "$base_heap" ] && [ -n "$end_heap" ] && [ "$end_heap" -ge $((base_heap - 8192)) ]; result "soak_heap_no_leak" $?
  [ -n "$base_min" ] && [ -n "$end_min" ] && [ "$end_min" -ge $((base_min - 98304)) ] && [ "$new_lows" -le 5 ]; result "soak_min_heap_stable" $?
  [ "$last_rx" -ge 50 ] && [ "$last_be" -ge 20 ] && [ "$last_big" -ge 1 ]; result "soak_still_delivering" $?
  # Every cycle's service call (Fast DDS and Cyclone alternating) must be
  # answered: the service path under churn, not just the topics. One miss over
  # the run is tolerated: a call is a single request with a 14 s budget during
  # the busiest seconds of a cycle, and a lost DATA repaired just past that
  # budget is not the failure this guards against (the heap and the matrix's
  # loss cases cover repair); two or more misses are.
  echo "service calls answered: $srv_ok of $cycles"
  [ "$srv_ok" -ge $((cycles - 1)) ]; result "soak_service_calls" $?
}

case "$SUITE" in
  matrix) run_matrix ;;
  soak) run_soak ;;
  *) echo "unknown suite: $SUITE (matrix|soak)"; exit 2 ;;
esac
device_health

echo ""
echo "==================== SUMMARY ===================="
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "QEMU INTEROP PASS" || echo "QEMU INTEROP FAIL"
exit $FAIL
