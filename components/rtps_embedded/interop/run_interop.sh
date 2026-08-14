#!/usr/bin/env bash
# FastDDS / ROS 2 interop test for the embeddedRTPS engine (runs INSIDE the
# harness container; use ./run.sh from the host).
#
# All peers share one network namespace so RTPS multicast works unconditionally.
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
       rtps_embedded_pubsub rtps_embedded_golden rtps_facade_pubsub rtps_typed_pubsub \
       rtps_facade_frag rtps_facade_backlog rtps_facade_frag_sizes rtps_service_loopback \
       rtps_service_naming rtps_action_naming rtps_action_types rtps_action_loopback \
       rtps_service_interop_server rtps_service_interop_client \
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

note "ROS 2 service + action name/type mangling (host)"
"$BIN"/rtps_service_naming; result "service_naming" $?
"$BIN"/rtps_action_naming; result "action_naming" $?

note "ROS 2 action envelope codec vs captured Fibonacci bytes (host)"
"$BIN"/rtps_action_types; result "action_types" $?

note "espp <-> espp in-process loopback"
"$BIN"/rtps_embedded_pubsub; result "loopback" $?

note "facade <-> facade in-process (two participants, port probing)"
"$BIN"/rtps_facade_pubsub; result "facade_loopback" $?

note "typed pub/sub in-process"
"$BIN"/rtps_typed_pubsub; result "typed_loopback" $?

# Regression guard: a reliable writer under backlog must retain + send every
# sample on the dynamic (host) storage path (no cursor-advance-as-drop skip).
# Non-fragmented small samples, so robust in the shared-netns container.
note "reliable backlog: no sample skipped (dynamic history growth)"
"$BIN"/rtps_facade_backlog; result "backlog_no_skip" $?

# Service (RMI) request/reply in-process: exercises name mangling + the
# related_sample_identity inline-QoS emit/parse + pending-request correlation.
# Non-fragmented small payloads, so robust in the shared-netns container.
note "service (RMI) request/reply loopback (related_sample_identity correlation)"
"$BIN"/rtps_service_loopback; result "service_loopback" $?

# Action (AMI) in-process: send_goal + feedback + get_result (deferred) over the
# 3 services + 2 topics, correlated by goal UUID. Non-fragmented, container-robust.
note "action (AMI) goal server loopback (Fibonacci: feedback + deferred result)"
"$BIN"/rtps_action_loopback; result "action_loopback" $?

# NOTE: the in-process fragmented loopbacks (rtps_facade_frag, rtps_facade_frag_sizes)
# are BUILT above (compile guard) but run as standalone host gates (docker-free),
# not here: two participants sharing one
# process + reactor in the container, publishing many small MTU-capped fragments,
# is an environment artifact (it passes on the host). The espp<->espp
# fragmentation path is proven in-container by cross_process_frag_200k below.

note "espp <-> espp cross-process 200 KB DATA_FRAG (reliable, large 60000 frag size, byte-exact)"
# Few large DATA_FRAG submessages (~4 per 200 KB sample), matching FastDDS's
# default large-fragment style; reliable QoS recovers any dropped fragment by
# whole-sample retransmit. Publish slowly so each SN completes (single reassembly
# slot) before the next.
"$BIN"/rtps_embedded_interop_sub xprocfrag std_msgs::msg::dds_::String_ 1 1 40 "" 200000 > /tmp/xsubfrag.log 2>&1 &
XSUBF=$!
sleep 2
"$BIN"/rtps_embedded_interop_pub xprocfrag std_msgs::msg::dds_::String_ 1 12 3000 "" 200000 60000 > /tmp/xpubfrag.log 2>&1 &
XPUBF=$!
wait $XSUBF
xsubf_rc=$?
kill $XPUBF 2>/dev/null; wait $XPUBF 2>/dev/null
tail -2 /tmp/xsubfrag.log
result "cross_process_frag_200k" $xsubf_rc

note "espp <-> espp cross-process on one host (port probing)"
"$BIN"/rtps_embedded_interop_sub xproc std_msgs::msg::dds_::String_ 0 3 20 > /tmp/xsub.log 2>&1 &
XSUB=$!
sleep 2
"$BIN"/rtps_embedded_interop_pub xproc std_msgs::msg::dds_::String_ 0 30 200 > /tmp/xpub.log 2>&1 &
XPUB=$!
wait $XSUB
xsub_rc=$?
kill $XPUB 2>/dev/null; wait $XPUB 2>/dev/null
tail -2 /tmp/xsub.log
result "cross_process" $xsub_rc

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

note "espp best-effort publisher -> ROS 2 (200 KB String, DATA_FRAG)"
# espp fragments the 200 KB sample into DATA_FRAG submessages; rmw_fastrtps
# reassembles it. Best-effort (v1 scope): in the shared-netns container there is
# ~no loss, so this proves the espp->ROS 2 DATA_FRAG WIRE encoding interops.
"$BIN"/rtps_embedded_interop_pub rt/bignum std_msgs::msg::dds_::String_ 0 20 2000 "" 200000 60000 > /tmp/pubbig.log 2>&1 &
ESPP_PID=$!
sleep 3
timeout 45 ros2 topic echo --once --full-length --qos-reliability best_effort /bignum std_msgs/msg/String > /tmp/echobig.log 2>&1
echobig_rc=$?
kill $ESPP_PID 2>/dev/null; wait $ESPP_PID 2>/dev/null
echo "echo exit=$echobig_rc, echo bytes=$(wc -c < /tmp/echobig.log)"
echo "--- pubbig tail ---"; tail -3 /tmp/pubbig.log
echo "--- echobig head ---"; head -c 160 /tmp/echobig.log; echo
# Require the echo to succeed AND to have carried a large (fragment-reassembled)
# payload (>150 KB of YAML), proving the 200 KB sample crossed intact.
big1_rc=1
if [ "$echobig_rc" -eq 0 ] && [ "$(wc -c < /tmp/echobig.log)" -gt 150000 ]; then big1_rc=0; fi
result "espp_pub->ros2_echo_200k" $big1_rc

note "ROS 2 publisher -> espp subscriber (200 KB String, DATA_FRAG both vendors)"
# ros2/FastDDS fragments the 200 KB String (its own fragment size); espp must
# reassemble it byte-exact against the shared 'A'+(i%26) pattern. Best-effort
# (v1): proves espp decodes FastDDS DATA_FRAG. Reliable frag recovery is v2.
"$BIN"/rtps_embedded_interop_sub rt/bignum2 std_msgs::msg::dds_::String_ 0 1 45 "" 200000 > /tmp/subbig.log 2>&1 &
SUBBIG_PID=$!
sleep 3
# 200 KB cannot be passed as a `ros2 topic pub` CLI arg (ARG_MAX); publish it
# from a small rclpy node that generates the pattern in-process (best-effort).
timeout 45 python3 /work/components/rtps_embedded/interop/ros2_big_publisher.py /bignum2 200000 40 > /tmp/rospubbig.log 2>&1 &
ROSBIG_PID=$!
wait $SUBBIG_PID
big2_rc=$?
kill $ROSBIG_PID 2>/dev/null; wait $ROSBIG_PID 2>/dev/null
echo "--- subbig tail ---"; tail -20 /tmp/subbig.log
echo "--- rospubbig tail ---"; tail -5 /tmp/rospubbig.log
result "ros2_pub->espp_sub_200k" $big2_rc

# --- Services (RMI): live ROS 2 <-> espp request/reply -----------------------
# The espp service uses the same rq/rr topics + _Request_/_Response_ types +
# related_sample_identity inline QoS that rmw_fastrtps uses, so a real ROS 2
# client/server interoperates with no type-hash exchange (verified: the service
# even appears in `ros2 service list`).

note "espp service server <- ROS 2 client (ros2 service call add_two_ints)"
"$BIN"/rtps_service_interop_server /add_two_ints example_interfaces::srv::dds_::AddTwoInts 40 \
  > /tmp/svcsrv.log 2>&1 &
SVCSRV=$!
sleep 4
timeout 25 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}" \
  > /tmp/svccall.log 2>&1
# Pass iff ROS 2 got the correct response (sum=42).
grep -q "sum=42" /tmp/svccall.log; svccall_rc=$?
sleep 1
kill $SVCSRV 2>/dev/null; wait $SVCSRV 2>/dev/null
echo "--- ros2 service call ---"; tail -3 /tmp/svccall.log
echo "--- espp server ---"; grep "server:" /tmp/svcsrv.log | tail -3
result "espp_service_server<-ros2_client" $svccall_rc

note "espp service client -> ROS 2 server (rclpy add_two_ints)"
python3 /work/components/rtps_embedded/interop/ros2_add_two_ints_server.py > /tmp/rossvcsrv.log 2>&1 &
ROSSVC=$!
sleep 4
timeout 35 "$BIN"/rtps_service_interop_client /add_two_ints example_interfaces::srv::dds_::AddTwoInts \
  20 22 30 > /tmp/svcclient.log 2>&1
svcclient_rc=$?
kill $ROSSVC 2>/dev/null; wait $ROSSVC 2>/dev/null
echo "--- espp client ---"; tail -2 /tmp/svcclient.log
echo "--- rclpy server ---"; grep -iE "= [0-9]" /tmp/rossvcsrv.log | tail -2
result "espp_service_client->ros2_server" $svcclient_rc

echo ""
echo "==================== SUMMARY ===================="
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "INTEROP PASS" || echo "INTEROP FAIL"
exit $FAIL
