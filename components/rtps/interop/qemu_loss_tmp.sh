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
  # per pid (kill -0 on the whole list fails as soon as ONE is gone), and an
  # exited child that is not yet reaped still has a /proc entry: a zombie
  # counts as gone
  alive() { [ -d "/proc/$1" ] && ! grep -q '^State:.*Z' "/proc/$1/status" 2>/dev/null; }
  local i p left=""
  for i in $(seq 1 50); do
    left=""
    for p in "$@"; do alive "$p" && left="$left $p"; done
    [ -z "$left" ] && break
    sleep 0.2
  done
  # The pids are mostly `timeout` wrappers: a TERM is forwarded to their
  # command, a KILL is not, so take the children down first or a ros2 process
  # outlives its wrapper and keeps talking to the device in the next case.
  for p in $left; do pkill -KILL -P "$p" 2>/dev/null; done
  # shellcheck disable=SC2086
  [ -n "$left" ] && kill -9 $left 2>/dev/null
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


loss_on() {
  tc qdisc add dev $TAP root netem loss 15% delay 10ms 3ms
  tc qdisc add dev $TAP ingress 2>/dev/null && tc filter add dev $TAP parent ffff: protocol ip prio 1 u32 match u32 0 0 action pass random netrand drop 7 2>/dev/null || echo "  (no ingress drop)"
}
loss_off() { tc qdisc del dev $TAP root 2>/dev/null; tc qdisc del dev $TAP ingress 2>/dev/null; }
attempt() { # rmw tag loss(yes/no) n
  local rmw=$1 tag=$2 loss=$3 n=$4
  note "attempt $n: $tag, loss=$loss"
  [ "$loss" = yes ] && loss_on
  local cap=/work/components/rtps/interop/qemu/build/cap_${tag}_$n.pcap
  tcpdump -i "$TAP" -nn -w "$cap" udp > /dev/null 2>&1 &
  local tp=$!
  sleep 1
  local m; m=$(mark)
  export RMW_IMPLEMENTATION=$rmw
  local t0; t0=$(date +%s)
  timeout 50 ros2 service call /qemu_add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}" > /work/components/rtps/interop/qemu/build/srv_${tag}_$n.log 2>&1
  local rc=$?
  local dt=$(( $(date +%s) - t0 ))
  sleep 1; stop_pids $tp
  [ "$loss" = yes ] && loss_off
  local srv; srv=$(count_since "$m" "RTPSTEST srv 3+4=7")
  echo "  rc=$rc after ${dt}s, device srv markers=$srv, client: $(tail -1 /work/components/rtps/interop/qemu/build/srv_${tag}_$n.log | cut -c1-100)"
  echo "  frames dev->host: $(tcpdump -nn -r "$cap" "src host $DEV_IP" 2>/dev/null | wc -l), host->dev unicast: $(tcpdump -nn -r "$cap" "dst host $DEV_IP" 2>/dev/null | wc -l), host->mcast: $(tcpdump -nn -r "$cap" 'ip dst net 224.0.0.0/4' 2>/dev/null | wc -l)"
  if [ $rc -ne 0 ]; then
    echo "  --- device log since the attempt started (non-tx) ---"
    since "$m" | sed 's/\x1b\[[0-9;]*m//g' | grep -v "RTPSTEST tx\|RTPSTEST status" | head -30
    echo "  --- dev->host frames after the srv marker time (size, dst) ---"
    tcpdump -nn -tt -r "$cap" "src host $DEV_IP" 2>/dev/null | awk '{print $1, $5, $NF}' | tail -40
    echo "  --- host->dev unicast frames (size) ---"
    tcpdump -nn -tt -r "$cap" "dst host $DEV_IP" 2>/dev/null | awk '{print $1, $3, $NF}' | tail -25
  fi
  settle
}
attempt rmw_cyclonedds_cpp cyclone no 0
for i in 1 2 3 4 5 6 7 8 9 10; do attempt rmw_cyclonedds_cpp cyclone yes $i; done
echo "DONE"
