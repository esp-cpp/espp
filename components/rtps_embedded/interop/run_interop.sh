#!/usr/bin/env bash
# FastDDS / ROS 2 interop test for the embeddedRTPS engine (runs INSIDE the
# harness container; use ./run.sh from the host).
#
# All peers share one network namespace so RTPS multicast works unconditionally.
# NOTE: the espp side is always started FIRST: the engine currently binds its
# unicast ports with SO_REUSE at participantId 0 without probing (see
# REFACTOR_PLAN.md Phase 2), while FastDDS probes past occupied ports.
#
# NOTE: no `set -u` - ROS 2's setup.bash references unset variables.

# Work on a container-local copy: the pc tests link the lib installed into
# lib/pc inside the source tree, which on the bind mount holds the developer's
# host-platform (e.g. macOS) artifacts. Building in-place would either link
# incompatible objects or clobber them with linux ones.
echo "===== Copy sources to container-local tree ====="
rsync -a --delete   --exclude '.git/' --exclude 'build/' --exclude 'build-*/'   --exclude 'managed_components/' --exclude 'docs/' --exclude 'dependencies.lock'   /work/ /tmp/espp/
cd /tmp/espp

PASS=0
FAIL=0
note() { echo -e "\n===== $* ====="; }
result() { # name exit_code
  if [ "$2" -eq 0 ]; then echo "RESULT PASS: $1"; PASS=$((PASS+1));
  else echo "RESULT FAIL: $1"; FAIL=$((FAIL+1)); fi
}

note "Build espp lib + host binaries (linux)"
cmake -S lib -B lib/build -DCMAKE_BUILD_TYPE=Release > /tmp/cmake_lib.log 2>&1 \
  && cmake --build lib/build -j"$(nproc)" --target install > /tmp/build_lib.log 2>&1 \
  && cmake -S pc -B pc/build -DCMAKE_BUILD_TYPE=Release > /tmp/cmake.log 2>&1 \
  && cmake --build pc/build -j"$(nproc)" --target \
       rtps_embedded_pubsub rtps_embedded_golden \
       rtps_embedded_interop_pub rtps_embedded_interop_sub > /tmp/build.log 2>&1
build_rc=$?
result "build" $build_rc
if [ $build_rc -ne 0 ]; then tail -30 /tmp/cmake_lib.log /tmp/build_lib.log /tmp/build.log; exit 1; fi
BIN=pc/build

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0    # matches the engine's Config::DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

note "Golden wire-format tests"
"$BIN"/rtps_embedded_golden; result "golden" $?

note "espp <-> espp in-process loopback"
"$BIN"/rtps_embedded_pubsub; result "loopback" $?

note "espp publisher -> ROS 2 subscriber (reliable, rt/chatter)"
"$BIN"/rtps_embedded_interop_pub rt/chatter std_msgs::msg::dds_::String_ 1 60 200 > /tmp/pub1.log 2>&1 &
ESPP_PID=$!
sleep 3
timeout 30 ros2 topic echo --once /chatter std_msgs/msg/String > /tmp/echo.log 2>&1
echo_rc=$?
kill $ESPP_PID 2>/dev/null; wait $ESPP_PID 2>/dev/null
cat /tmp/echo.log
result "espp_pub->ros2_echo" $echo_rc

note "ROS 2 publisher -> espp subscriber (reliable, rt/chatter)"
"$BIN"/rtps_embedded_interop_sub rt/chatter std_msgs::msg::dds_::String_ 1 3 30 > /tmp/sub1.log 2>&1 &
SUB_PID=$!
sleep 3
timeout 35 ros2 topic pub -r 5 /chatter std_msgs/msg/String "data: 'ros2 to espp'" > /tmp/rospub.log 2>&1 &
ROS_PID=$!
wait $SUB_PID
sub_rc=$?
kill $ROS_PID 2>/dev/null; wait $ROS_PID 2>/dev/null
cat /tmp/sub1.log
result "ros2_pub->espp_sub" $sub_rc

note "espp best-effort publisher -> ROS 2 best-effort subscriber"
"$BIN"/rtps_embedded_interop_pub rt/chatter std_msgs::msg::dds_::String_ 0 60 200 > /tmp/pub2.log 2>&1 &
ESPP_PID=$!
sleep 3
timeout 30 ros2 topic echo --once --qos-reliability best_effort /chatter std_msgs/msg/String > /tmp/echo2.log 2>&1
echo2_rc=$?
kill $ESPP_PID 2>/dev/null; wait $ESPP_PID 2>/dev/null
result "espp_be_pub->ros2_be_echo" $echo2_rc

echo ""
echo "==================== SUMMARY ===================="
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "INTEROP PASS" || echo "INTEROP FAIL"
exit $FAIL
