#!/usr/bin/env bash
# FastDDS / ROS 2 interop test for the embeddedRTPS engine (runs INSIDE the
# harness container; use ./run.sh from the host).
#
# All peers share one network namespace so RTPS multicast works unconditionally.
#
# NOTE: no `set -u` - ROS 2's setup.bash references unset variables.

# Work on a container-local copy: the build installs espp into a staging prefix
# (/tmp/espp/install) and the pc tests find_package it from there. Doing this on
# the bind mount would either mix in the developer's host-platform (e.g. macOS)
# artifacts or clobber them with linux ones, so copy to a container-local tree.
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

# --- Liveliness past the remote participant lease ----------------------------
# A peer keeps re-announcing itself over SPDP; if those re-announcements are not
# delivered to the SPDP reader, espp lets the peer's lease expire (10 s for
# CycloneDDS, which addresses the repeats to readerId ENTITYID_UNKNOWN) and
# drops every matched endpoint with it - discovery looked fine, data stopped
# after the lease. `ros2 topic echo --once` cannot see that, so both legs here
# only count samples that arrive from 22 s in: discovery (a few seconds), the
# 10 s lease and the liveliness check cadence all fit before that, so a
# dropped peer contributes nothing to the count (count_after_s, see
# rtps_interop_sub / ros2_count_subscriber.py). The FastDDS pass is the
# control: it names the SPDP reader in every announcement.
lease_liveliness_cases() { # prefix log-tag
  local prefix="$1" tag="$2"
  note "${prefix}ROS 2 publisher -> espp subscriber, samples still arriving 22 s in (liveliness past the lease)"
  "$BIN"/rtps_interop_sub rt/lease std_msgs::msg::dds_::String_ 1 10 34 "" 0 2 22 > "/tmp/${tag}leasesub.log" 2>&1 &
  local sub=$!
  sleep 3
  timeout 36 ros2 topic pub -r 5 /lease std_msgs/msg/String "data: 'still alive'" > "/tmp/${tag}leaserospub.log" 2>&1 &
  local rospub=$!
  wait $sub
  local sub_rc=$?
  kill $rospub 2>/dev/null; wait $rospub 2>/dev/null || true
  tail -2 "/tmp/${tag}leasesub.log"
  result "${prefix}ros2_pub->espp_sub_liveliness" $sub_rc

  note "${prefix}espp publisher -> ROS 2 subscriber, samples still arriving 22 s in (liveliness past the lease)"
  "$BIN"/rtps_interop_pub rt/lease2 std_msgs::msg::dds_::String_ 1 220 200 > "/tmp/${tag}leasepub.log" 2>&1 &
  local pub=$!
  sleep 3
  timeout 40 python3 /work/components/rtps/interop/ros2_count_subscriber.py /lease2 --seconds 34 --required 10 --count-after 22 --reliability reliable \
    > "/tmp/${tag}leasecount.log" 2>&1
  local count_rc=$?
  kill $pub 2>/dev/null; wait $pub 2>/dev/null || true
  tail -2 "/tmp/${tag}leasecount.log"
  result "${prefix}espp_pub->ros2_sub_liveliness" $count_rc
}

# --- A dozen participants on the domain, the publisher joins last -------------
# Every ROS 2 process is a DDS participant, and espp tracks remote participants
# in a fixed pool (SPDP_MAX_NUMBER_FOUND_PARTICIPANTS: 32 on the host profile
# this harness builds, 10 on the embedded profile). A participant beyond the
# pool is ignored silently - never matched, its data dropped. With twelve
# unrelated participants already on the domain, the publisher espp actually
# needs joins last and must still be matched. On an ESP32 the same scenario
# needs the pool raised (menuconfig RTPS -> Custom capacity overrides).
many_participants_case() { # prefix log-tag
  local prefix="$1" tag="$2"
  note "${prefix}espp subscriber with twelve other participants on the domain; the publisher joins last"
  local crowd=()
  local i
  for i in $(seq 1 12); do
    timeout 45 ros2 topic pub -r 1 "/crowd$i" std_msgs/msg/String "data: 'crowd'" > /dev/null 2>&1 &
    crowd+=("$!")
  done
  sleep 8 # let the crowd come up (each is its own participant) before espp starts
  "$BIN"/rtps_interop_sub rt/crowd std_msgs::msg::dds_::String_ 1 3 30 > "/tmp/${tag}crowdsub.log" 2>&1 &
  local sub=$!
  sleep 3
  timeout 32 ros2 topic pub -r 5 /crowd std_msgs/msg/String "data: 'late joiner'" > "/tmp/${tag}crowdpub.log" 2>&1 &
  local pub=$!
  wait $sub
  local sub_rc=$?
  kill $pub "${crowd[@]}" 2>/dev/null; wait $pub "${crowd[@]}" 2>/dev/null || true
  tail -2 "/tmp/${tag}crowdsub.log"
  result "${prefix}many_participants_late_joiner" $sub_rc
}

# --- Endpoint churn: a peer's publisher comes and goes ------------------------
# A peer that deletes a publisher, or leaves the domain, says so with a dispose
# (a DATA whose inline QoS carries PID_STATUS_INFO + PID_KEY_HASH, RTPS 2.5
# 8.5.4.3); espp must unmatch the writer right then, not when the peer's lease
# would have run out (20 s Fast DDS, 10 s Cyclone DDS). The ROS 2 node creates,
# uses and destroys a publisher five times (3 s on, 2 s off) and exits, which
# also disposes its participant; the espp subscriber must count five
# matched->unmatched transitions, each after samples arrived, within 40 s -
# lease expiry alone would yield at most two (rtps_interop_churn,
# ros2_churn_publisher.py). Guards the receiver's inline-QoS handling
# (pc/tests/rtps_sedp_inline_qos_dispose.cpp is the host-side test).
endpoint_churn_case() { # prefix log-tag
  local prefix="$1" tag="$2"
  note "${prefix}ROS 2 publisher created and destroyed in rounds -> espp subscriber unmatches on each dispose"
  "$BIN"/rtps_interop_churn rt/churn std_msgs::msg::dds_::String_ 5 40 > "/tmp/${tag}churnsub.log" 2>&1 &
  local sub=$!
  sleep 3
  timeout 45 python3 /work/components/rtps/interop/ros2_churn_publisher.py /churn --rounds 5 --on 3 --off 2 > "/tmp/${tag}churnpub.log" 2>&1
  local pub_rc=$?
  wait $sub
  local sub_rc=$?
  echo "--- churn publisher ---"; tail -2 "/tmp/${tag}churnpub.log"
  echo "--- espp subscriber ---"; tail -4 "/tmp/${tag}churnsub.log"
  [ $pub_rc -eq 0 ] || sub_rc=1
  result "${prefix}ros2_pub_churn->espp_sub_unmatch" $sub_rc
}

note "Build espp lib + host binaries (linux)"
echo "compiler: $(${CXX:-c++} --version | head -1); cmake $(cmake --version | head -1 | awk '{print $3}'); ROS_DISTRO=${ROS_DISTRO:-?}"
cmake -S lib -B lib/build -DCMAKE_BUILD_TYPE=Release -DESPP_INSTALL=ON -DCMAKE_INSTALL_PREFIX=/tmp/espp/install > /tmp/cmake_lib.log 2>&1 \
  && cmake --build lib/build -j"$(nproc)" --target install > /tmp/build_lib.log 2>&1 \
  && cmake -S pc -B pc/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/tmp/espp/install > /tmp/cmake.log 2>&1 \
  && cmake --build pc/build -j"$(nproc)" --target \
       rtps_pubsub rtps_golden rtps_locator_subnet rtps_facade_pubsub rtps_typed_pubsub \
       rtps_facade_frag rtps_facade_backlog rtps_facade_frag_sizes rtps_service_loopback \
       rtps_service_naming rtps_action_naming rtps_action_types rtps_native_protocol \
       rtps_cyclonedds_rpc_header rtps_preemptive_acknack_count \
       rtps_action_loopback \
       rtps_native_service_loopback rtps_native_action_loopback rtps_typed_rpc_loopback \
       rtps_service_interop_server rtps_service_interop_client rtps_trigger_interop_server \
       rtps_action_interop_server rtps_action_interop_client \
       rtps_sedp_dedicated_locator rtps_banded_pubsub rtps_banded_deferred rtps_banded_ration \
       rtps_banded_churn rtps_service_rollback rtps_deferred_recovery rtps_guaranteed_submit \
       rtps_remove_reader_deadlock rtps_guaranteed_fairness rtps_writer_churn \
       rtps_stateless_saturation rtps_callback_reentrancy rtps_best_effort_order rtps_acknack_bitmap \
       rtps_interop_pub rtps_interop_sub rtps_interop_churn rtps_sedp_inline_qos_dispose \
       rtps_sedp_history_overflow rtps_sedp_history_ring > /tmp/build.log 2>&1
build_rc=$?
result "build" $build_rc
if [ $build_rc -ne 0 ]; then tail -30 /tmp/cmake_lib.log /tmp/build_lib.log /tmp/build.log; exit 1; fi
BIN=pc/build

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export ROS_DOMAIN_ID=0    # matches the engine's Config::DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

note "Golden wire-format tests"
"$BIN"/rtps_golden; result "golden" $?

note "Unicast locator selection for a peer on several networks (host)"
"$BIN"/rtps_locator_subnet; result "locator_subnet" $?

note "SEDP/SPDP disposes from the inline QoS unmatch the endpoint / forget the participant (host)"
"$BIN"/rtps_sedp_inline_qos_dispose; result "sedp_inline_qos_dispose" $?

note "SEDP writer history never drops a live announcement; a deleted endpoint's slot is reusable (host)"
"$BIN"/rtps_sedp_history_ring; result "sedp_history_ring" $?
"$BIN"/rtps_sedp_history_overflow; result "sedp_history_overflow" $?

note "ROS 2 service + action name/type mangling (host)"
"$BIN"/rtps_service_naming; result "service_naming" $?
"$BIN"/rtps_action_naming; result "action_naming" $?

note "ROS 2 action envelope codec vs captured Fibonacci bytes (host)"
"$BIN"/rtps_action_types; result "action_types" $?

note "rmw_cyclonedds_cpp in-payload request/reply header codec (host)"
"$BIN"/rtps_cyclonedds_rpc_header; result "cyclonedds_rpc_header" $?
"$BIN"/rtps_preemptive_acknack_count; result "preemptive_acknack_count" $?

note "native (espp<->espp) protocol codec: header + action framing (host)"
"$BIN"/rtps_native_protocol; result "native_protocol" $?

note "espp <-> espp in-process loopback"
"$BIN"/rtps_pubsub; result "loopback" $?

note "facade <-> facade in-process (two participants, port probing)"
"$BIN"/rtps_facade_pubsub; result "facade_loopback" $?

note "typed pub/sub in-process"
"$BIN"/rtps_typed_pubsub; result "typed_loopback" $?

note "per-endpoint priority: dedicated ports (SEDP locator + ration) + banded loopbacks"
"$BIN"/rtps_sedp_dedicated_locator; result "sedp_dedicated_locator" $?
"$BIN"/rtps_banded_pubsub; result "banded_pubsub" $?
"$BIN"/rtps_banded_deferred; result "banded_deferred" $?
"$BIN"/rtps_banded_ration; result "banded_ration" $?
# Teardown-under-load regression (the CI shutdown-hang class): dedicated-port
# churn + stop() with deferred deliveries in flight must complete promptly.
timeout 120 "$BIN"/rtps_banded_churn; result "banded_churn" $?
# Transactional composite creation: partial failures must leak nothing.
timeout 60 "$BIN"/rtps_service_rollback; result "service_rollback" $?
# Deferred-arm recovery: a rejected drain arm must never strand a delivery.
timeout 60 "$BIN"/rtps_deferred_recovery; result "deferred_recovery" $?
# Guaranteed writer-progress submission: a pool-rejected progress poke must be
# re-armed by the retry mechanism, never dropped.
timeout 60 "$BIN"/rtps_guaranteed_submit; result "guaranteed_submit" $?
# remove_reader must not deadlock when the removed reader's callback calls back
# into the participant (e.g. publish()).
timeout 60 "$BIN"/rtps_remove_reader_deadlock; result "remove_reader_deadlock" $?
# Guaranteed-retry admission fairness: under sustained higher-band overload a
# lower-band producer must still be admitted (eventual-run), and no owed run
# may be lost.
timeout 90 "$BIN"/rtps_guaranteed_fairness; result "guaranteed_fairness" $?
# Writer create/publish/delete churn under load: deletion racing parked/queued
# progress() jobs must neither crash nor wedge the SEDP announcement stream.
timeout 120 "$BIN"/rtps_writer_churn; result "writer_churn" $?
# Best-effort saturation must not collapse: a burst far faster than the send
# path is (near-)losslessly drained with growable history.
timeout 90 "$BIN"/rtps_stateless_saturation; result "stateless_saturation" $?
# Reentrant cross-callback removal: a callback removing a later-registered
# callback (and freeing its arg) must prevent the stale snapshot entry from
# being invoked.
timeout 30 "$BIN"/rtps_callback_reentrancy; result "callback_reentrancy" $?
timeout 30 "$BIN"/rtps_best_effort_order; result "best_effort_order" $?
timeout 30 "$BIN"/rtps_acknack_bitmap; result "acknack_bitmap" $?

# The same saturation test against a STATIC-storage build of the engine with a
# 2-slot best-effort history: this exercises the KEEP_LAST overwrite path (the
# test requires overflow drops to be counted) and the progress() cursor clamp
# (the test requires overflow drops to be COUNTED, delivered+dropped to
# conserve the burst, and delivery above the total-collapse level - the
# pre-fix cursor bug delivered only the final ring contents).
# Built as a separate lib/test pair because the storage model and the history
# depth are baked into the library at compile time.
note "static-storage saturation gate (KEEP_LAST drop-oldest, cursor clamp)"
STATIC_FLAGS="-DRTPS_STORAGE_STATIC -DRTPS_CFG_HISTORY_SIZE_STATELESS=2"
cmake -S lib -B lib/build-static -DCMAKE_BUILD_TYPE=Release -DESPP_INSTALL=ON \
      -DESPP_BUILD_PYTHON=OFF -DCMAKE_INSTALL_PREFIX=/tmp/espp/install-static \
      -DCMAKE_CXX_FLAGS="$STATIC_FLAGS" > /tmp/cmake_static.log 2>&1 \
  && cmake --build lib/build-static -j"$(nproc)" --target install > /tmp/build_static.log 2>&1 \
  && cmake -S pc -B pc/build-static -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_PREFIX_PATH=/tmp/espp/install-static \
       -DCMAKE_CXX_FLAGS="$STATIC_FLAGS" > /tmp/cmake_static_pc.log 2>&1 \
  && cmake --build pc/build-static -j"$(nproc)" --target rtps_stateless_saturation > /tmp/build_static_pc.log 2>&1
static_build_rc=$?
result "static_build" $static_build_rc
if [ $static_build_rc -ne 0 ]; then tail -20 /tmp/cmake_static.log /tmp/build_static.log /tmp/build_static_pc.log; fi
timeout 90 pc/build-static/rtps_stateless_saturation; result "stateless_saturation_static" $?

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

# Native (espp<->espp) lean request/reply: 20-byte in-band header over pub/sub,
# all three client styles (sync/async/future). Not ROS-interoperable by design.
note "native (espp<->espp) service loopback (in-band correlation header)"
"$BIN"/rtps_native_service_loopback; result "native_service_loopback" $?

note "native (espp<->espp) action loopback (lean AMI: goal + feedback + result)"
"$BIN"/rtps_native_action_loopback; result "native_action_loopback" $?

# Typed espp-idiomatic wrappers (ServiceServer/Client, ActionServer/Client) over
# reflectable structs - the RMI/AMI analogue of Publisher<T>/Subscriber<T>.
note "typed RMI/AMI wrappers loopback (ServiceServer/Client + ActionServer/Client)"
"$BIN"/rtps_typed_rpc_loopback; result "typed_rpc_loopback" $?

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
"$BIN"/rtps_interop_sub xprocfrag std_msgs::msg::dds_::String_ 1 1 40 "" 200000 > /tmp/xsubfrag.log 2>&1 &
XSUBF=$!
sleep 2
"$BIN"/rtps_interop_pub xprocfrag std_msgs::msg::dds_::String_ 1 12 3000 "" 200000 60000 > /tmp/xpubfrag.log 2>&1 &
XPUBF=$!
wait $XSUBF
xsubf_rc=$?
kill $XPUBF 2>/dev/null; wait $XPUBF 2>/dev/null
tail -2 /tmp/xsubfrag.log
result "cross_process_frag_200k" $xsubf_rc

note "espp <-> espp cross-process on one host (port probing)"
"$BIN"/rtps_interop_sub xproc std_msgs::msg::dds_::String_ 0 3 20 > /tmp/xsub.log 2>&1 &
XSUB=$!
sleep 2
"$BIN"/rtps_interop_pub xproc std_msgs::msg::dds_::String_ 0 30 200 > /tmp/xpub.log 2>&1 &
XPUB=$!
wait $XSUB
xsub_rc=$?
kill $XPUB 2>/dev/null; wait $XPUB 2>/dev/null
tail -2 /tmp/xsub.log
result "cross_process" $xsub_rc

note "espp publisher -> ROS 2 subscriber (reliable, rt/chatter)"
"$BIN"/rtps_interop_pub rt/chatter std_msgs::msg::dds_::String_ 1 60 200 > /tmp/pub1.log 2>&1 &
ESPP_PID=$!
sleep 3
timeout 30 ros2 topic echo --once /chatter std_msgs/msg/String > /tmp/echo.log 2>&1
echo_rc=$?
kill $ESPP_PID 2>/dev/null; wait $ESPP_PID 2>/dev/null
cat /tmp/echo.log
result "espp_pub->ros2_echo" $echo_rc

note "ROS 2 publisher -> espp subscriber (reliable, rt/chatter)"
"$BIN"/rtps_interop_sub rt/chatter std_msgs::msg::dds_::String_ 1 3 30 > /tmp/sub1.log 2>&1 &
SUB_PID=$!
sleep 3
timeout 35 ros2 topic pub -r 5 /chatter std_msgs/msg/String "data: 'ros2 to espp'" > /tmp/rospub.log 2>&1 &
ROS_PID=$!
wait $SUB_PID
sub_rc=$?
kill $ROS_PID 2>/dev/null; wait $ROS_PID 2>/dev/null
cat /tmp/sub1.log
result "ros2_pub->espp_sub" $sub_rc

note "ROS 2 publisher -> espp BANDED subscriber (dedicated unicast port)"
# The espp reader runs at QosBand::High (band=1), so it is granted a dedicated
# unicast port (7400+250*domain+100+n) announced via its SEDP per-endpoint
# unicast locator. FastDDS honors that locator and sends the topic's DATA to
# the dedicated port - delivery here proves a dedicated-port endpoint
# interoperates with FastDDS/ROS 2.
"$BIN"/rtps_interop_sub rt/chatter std_msgs::msg::dds_::String_ 1 3 30 "" 0 1 > /tmp/subband.log 2>&1 &
SUBBAND_PID=$!
sleep 3
timeout 35 ros2 topic pub -r 5 /chatter std_msgs/msg/String "data: 'ros2 to banded espp'" > /tmp/rospubband.log 2>&1 &
ROSBAND_PID=$!
wait $SUBBAND_PID
subband_rc=$?
kill $ROSBAND_PID 2>/dev/null; wait $ROSBAND_PID 2>/dev/null
cat /tmp/subband.log
result "ros2_pub->espp_banded_sub" $subband_rc

lease_liveliness_cases "" "f"
many_participants_case "" "f"
endpoint_churn_case "" "f"

note "espp best-effort publisher -> ROS 2 best-effort subscriber"
"$BIN"/rtps_interop_pub rt/chatter std_msgs::msg::dds_::String_ 0 60 200 > /tmp/pub2.log 2>&1 &
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
"$BIN"/rtps_interop_pub rt/bignum std_msgs::msg::dds_::String_ 0 20 2000 "" 200000 60000 > /tmp/pubbig.log 2>&1 &
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
"$BIN"/rtps_interop_sub rt/bignum2 std_msgs::msg::dds_::String_ 0 1 45 "" 200000 > /tmp/subbig.log 2>&1 &
SUBBIG_PID=$!
sleep 3
# 200 KB cannot be passed as a `ros2 topic pub` CLI arg (ARG_MAX); publish it
# from a small rclpy node that generates the pattern in-process (best-effort).
timeout 45 python3 /work/components/rtps/interop/ros2_big_publisher.py /bignum2 200000 40 > /tmp/rospubbig.log 2>&1 &
ROSBIG_PID=$!
wait $SUBBIG_PID
big2_rc=$?
kill $ROSBIG_PID 2>/dev/null; wait $ROSBIG_PID 2>/dev/null
echo "--- subbig tail ---"; tail -20 /tmp/subbig.log
echo "--- rospubbig tail ---"; tail -5 /tmp/rospubbig.log
result "ros2_pub->espp_sub_200k" $big2_rc

# --- Services (RMI) + actions (AMI): live ROS 2 <-> espp -------------------
# The espp service uses the same rq/rr topics + _Request_/_Response_ types as
# ROS 2, so a real ROS 2 client/server interoperates with no type-hash exchange
# (verified: the service even appears in `ros2 service list`). Correlation
# follows the peer's RMW: related_sample_identity inline QoS for
# rmw_fastrtps_cpp, the in-payload {guid, seq} header for rmw_cyclonedds_cpp
# (see rpc/cyclonedds_request_header.hpp) - so the four cases run under both.
rpc_cases() { # prefix log-tag
  local prefix="$1" tag="$2"
  note "${prefix}espp service server <- ROS 2 client (ros2 service call add_two_ints)"
  "$BIN"/rtps_service_interop_server /add_two_ints example_interfaces::srv::dds_::AddTwoInts 40 \
    > "/tmp/${tag}svcsrv.log" 2>&1 &
  local svcsrv=$!
  sleep 4
  timeout 25 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}" \
    > "/tmp/${tag}svccall.log" 2>&1
  # Pass iff ROS 2 got the correct response (sum=42).
  grep -q "sum=42" "/tmp/${tag}svccall.log"; local svccall_rc=$?
  sleep 1
  kill $svcsrv 2>/dev/null; wait $svcsrv 2>/dev/null
  echo "--- ros2 service call ---"; tail -3 "/tmp/${tag}svccall.log"
  echo "--- espp server ---"; grep "server:" "/tmp/${tag}svcsrv.log" | tail -3
  result "${prefix}espp_service_server<-ros2_client" $svccall_rc

  note "${prefix}espp service client -> ROS 2 server (rclpy add_two_ints)"
  python3 /work/components/rtps/interop/ros2_add_two_ints_server.py > "/tmp/${tag}rossvcsrv.log" 2>&1 &
  local rossvc=$!
  sleep 4
  timeout 35 "$BIN"/rtps_service_interop_client /add_two_ints example_interfaces::srv::dds_::AddTwoInts \
    20 22 30 > "/tmp/${tag}svcclient.log" 2>&1
  local svcclient_rc=$?
  kill $rossvc 2>/dev/null; wait $rossvc 2>/dev/null
  echo "--- espp client ---"; tail -2 "/tmp/${tag}svcclient.log"
  echo "--- rclpy server ---"; grep -iE "= [0-9]" "/tmp/${tag}rossvcsrv.log" | tail -2
  result "${prefix}espp_service_client->ros2_server" $svcclient_rc

  note "${prefix}espp action server <- ROS 2 client (ros2 action send_goal -f fibonacci)"
  "$BIN"/rtps_action_interop_server /fibonacci example_interfaces::action::dds_::Fibonacci 45 \
    > "/tmp/${tag}actsrv.log" 2>&1 &
  local actsrv=$!
  sleep 5
  timeout 30 ros2 action send_goal -f /fibonacci example_interfaces/action/Fibonacci "{order: 5}" \
    > "/tmp/${tag}actsend.log" 2>&1
  # Pass iff the ROS 2 client's goal completed on the espp server (full round-trip:
  # send_goal accepted -> feedback -> deferred get_result -> SUCCEEDED). The result
  # sequence itself is byte-validated separately by the action_types codec test.
  grep -q "status: SUCCEEDED" "/tmp/${tag}actsend.log"
  local actsend_rc=$?
  sleep 1
  kill $actsrv 2>/dev/null; wait $actsrv 2>/dev/null
  echo "--- ros2 action send_goal ---"; grep -iE "Result:|sequence=|status|Goal finished" "/tmp/${tag}actsend.log" | tail -4
  echo "--- espp action server ---"; grep "server:" "/tmp/${tag}actsrv.log" | tail -3
  result "${prefix}espp_action_server<-ros2_client" $actsend_rc

  note "${prefix}espp action client -> ROS 2 server (rclpy Fibonacci)"
  python3 /work/components/rtps/interop/ros2_fibonacci_server.py > "/tmp/${tag}rosactsrv.log" 2>&1 &
  local rosact=$!
  sleep 5
  timeout 40 "$BIN"/rtps_action_interop_client /fibonacci example_interfaces::action::dds_::Fibonacci \
    5 30 > "/tmp/${tag}actclient.log" 2>&1
  local actclient_rc=$?
  kill $rosact 2>/dev/null; wait $rosact 2>/dev/null
  echo "--- espp action client ---"; tail -2 "/tmp/${tag}actclient.log"
  echo "--- rclpy server ---"; grep -iE "order=" "/tmp/${tag}rosactsrv.log" | tail -2
  result "${prefix}espp_action_client->ros2_server" $actclient_rc

  # --- beyond one happy-path call: concurrency, ordering, timing, sizes, goal paths ---
  note "${prefix}espp service client -> ROS 2 server, 8 requests in flight at once (each reply must land on its own request)"
  python3 /work/components/rtps/interop/ros2_add_two_ints_server.py > "/tmp/${tag}rossvcsrv8.log" 2>&1 &
  local rossvc8=$!
  sleep 4
  timeout 40 "$BIN"/rtps_service_interop_client /add_two_ints example_interfaces::srv::dds_::AddTwoInts \
    100 7 30 "" 8 > "/tmp/${tag}svcclient8.log" 2>&1
  local svcclient8_rc=$?
  kill $rossvc8 2>/dev/null; wait $rossvc8 2>/dev/null
  echo "--- espp client ---"; tail -2 "/tmp/${tag}svcclient8.log"
  result "${prefix}espp_service_client_x8->ros2_server" $svcclient8_rc

  note "${prefix}two espp service clients -> one ROS 2 server at the same time (replies are filtered per client)"
  python3 /work/components/rtps/interop/ros2_add_two_ints_server.py > "/tmp/${tag}rossvcsrv2.log" 2>&1 &
  local rossvc2=$!
  sleep 4
  timeout 40 "$BIN"/rtps_service_interop_client /add_two_ints example_interfaces::srv::dds_::AddTwoInts \
    1 2 30 "" 4 > "/tmp/${tag}svcclientA.log" 2>&1 &
  local client_a=$!
  timeout 40 "$BIN"/rtps_service_interop_client /add_two_ints example_interfaces::srv::dds_::AddTwoInts \
    1000 2000 30 "" 4 > "/tmp/${tag}svcclientB.log" 2>&1 &
  local client_b=$!
  wait $client_a; local client_a_rc=$?
  wait $client_b; local client_b_rc=$?
  kill $rossvc2 2>/dev/null; wait $rossvc2 2>/dev/null
  echo "--- espp client A ---"; tail -1 "/tmp/${tag}svcclientA.log"
  echo "--- espp client B ---"; tail -1 "/tmp/${tag}svcclientB.log"
  [ $client_a_rc -eq 0 ] && [ $client_b_rc -eq 0 ]; result "${prefix}two_espp_clients->ros2_server" $?

  note "${prefix}espp service client up first, the ROS 2 server joins 8 s later (requests sent before any server was matched)"
  timeout 50 "$BIN"/rtps_service_interop_client /add_two_ints example_interfaces::srv::dds_::AddTwoInts \
    5 6 40 > "/tmp/${tag}svclate.log" 2>&1 &
  local late_client=$!
  sleep 8
  python3 /work/components/rtps/interop/ros2_add_two_ints_server.py > "/tmp/${tag}rossvclate.log" 2>&1 &
  local rossvclate=$!
  wait $late_client; local late_rc=$?
  kill $rossvclate 2>/dev/null; wait $rossvclate 2>/dev/null
  echo "--- espp client ---"; tail -1 "/tmp/${tag}svclate.log"
  echo "--- rclpy server ---"; grep -iE "= [0-9]" "/tmp/${tag}rossvclate.log" | tail -2
  result "${prefix}espp_service_client->late_ros2_server" $late_rc

  note "${prefix}espp Trigger server <- ROS 2 client: an empty request, a 48000-character reply in one DATA"
  "$BIN"/rtps_trigger_interop_server /trigger std_srvs::srv::dds_::Trigger 40 48000 > "/tmp/${tag}trgsrv.log" 2>&1 &
  local trgsrv=$!
  sleep 4
  timeout 25 ros2 service call /trigger std_srvs/srv/Trigger "{}" > "/tmp/${tag}trgcall.log" 2>&1
  local trg_rc=0
  grep -q "success=True" "/tmp/${tag}trgcall.log" || trg_rc=1
  local xs; xs=$(grep -o "message='x*'" "/tmp/${tag}trgcall.log" | tr -cd 'x' | wc -c | tr -d ' ')
  [ "$xs" -eq 48000 ] || trg_rc=1
  sleep 1
  kill $trgsrv 2>/dev/null; wait $trgsrv 2>/dev/null
  echo "--- ros2 service call --- (message of $xs characters)"; grep -o "success=[A-Za-z]*" "/tmp/${tag}trgcall.log" | tail -1
  echo "--- espp server ---"; grep "server:" "/tmp/${tag}trgsrv.log" | tail -2
  result "${prefix}espp_trigger_server<-ros2_client" $trg_rc

  note "${prefix}espp action server <- ROS 2 client: a goal cancelled at its first feedback, and a goal the server refuses"
  "$BIN"/rtps_action_interop_server /fibonacci example_interfaces::action::dds_::Fibonacci 90 \
    > "/tmp/${tag}actsrv2.log" 2>&1 &
  local actsrv2=$!
  sleep 5
  timeout 35 python3 /work/components/rtps/interop/ros2_fibonacci_client.py cancel 12 > "/tmp/${tag}actcancel.log" 2>&1
  local cancel_rc=$?; grep -q "^CANCELED" "/tmp/${tag}actcancel.log" || cancel_rc=1
  timeout 35 python3 /work/components/rtps/interop/ros2_fibonacci_client.py reject 0 > "/tmp/${tag}actreject.log" 2>&1
  local reject_rc=$?; grep -q "^REJECTED" "/tmp/${tag}actreject.log" || reject_rc=1
  sleep 1
  kill $actsrv2 2>/dev/null; wait $actsrv2 2>/dev/null
  echo "--- cancel ---"; tail -1 "/tmp/${tag}actcancel.log"
  echo "--- reject ---"; tail -1 "/tmp/${tag}actreject.log"
  echo "--- espp action server ---"; grep "server:" "/tmp/${tag}actsrv2.log" | tail -3
  result "${prefix}espp_action_server<-ros2_cancel" $cancel_rc
  result "${prefix}espp_action_server<-ros2_reject" $reject_rc
}
rpc_cases "" ""

# --- CycloneDDS: the ROS 2 pairs again on rmw_cyclonedds_cpp -----------------
# CycloneDDS is stricter than FastDDS on the wire: it addresses HEARTBEAT/GAP to
# ENTITYID_UNKNOWN, resolves ACKNACKs through INFO_DST, rejects a pre-emptive
# ACKNACK with base 0 from vendors other than eProsima/RTI, and leaves the
# per-endpoint unicast locator out of SEDP. Each of these alone stops all samples.
# Services and actions run here too: rmw_cyclonedds_cpp correlates requests with
# a {guid, seq} header in the payload, not the related_sample_identity inline
# QoS of rmw_fastrtps_cpp, and espp's endpoints answer each peer in its own form.
note "CycloneDDS (rmw_cyclonedds_cpp): ROS 2 pub/sub pairs, services, actions"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CycloneDDS caps the participants one host may create on a domain through its
# participant-index range (MaxAutoParticipantIndex, default 9 = ten
# participants); the busy-domain case below creates thirteen, so lift it.
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 daemon stop > /dev/null 2>&1

"$BIN"/rtps_interop_pub rt/chatter std_msgs::msg::dds_::String_ 1 60 200 > /tmp/cpub1.log 2>&1 &
ESPP_PID=$!
sleep 3
timeout 30 ros2 topic echo --once /chatter std_msgs/msg/String > /tmp/cecho1.log 2>&1
cecho1_rc=$?
kill $ESPP_PID 2>/dev/null; wait $ESPP_PID 2>/dev/null
cat /tmp/cecho1.log
result "cyclone: espp_pub->ros2_echo" $cecho1_rc

"$BIN"/rtps_interop_sub rt/chatter std_msgs::msg::dds_::String_ 1 3 30 > /tmp/csub1.log 2>&1 &
SUB_PID=$!
sleep 3
timeout 35 ros2 topic pub -r 5 /chatter std_msgs/msg/String "data: 'cyclone to espp'" > /tmp/crospub1.log 2>&1 &
ROS_PID=$!
wait $SUB_PID
csub1_rc=$?
kill $ROS_PID 2>/dev/null; wait $ROS_PID 2>/dev/null
tail -2 /tmp/csub1.log
result "cyclone: ros2_pub->espp_sub" $csub1_rc

"$BIN"/rtps_interop_pub rt/chatter std_msgs::msg::dds_::String_ 0 60 200 > /tmp/cpub2.log 2>&1 &
ESPP_PID=$!
sleep 3
timeout 30 ros2 topic echo --once --qos-reliability best_effort /chatter std_msgs/msg/String > /tmp/cecho2.log 2>&1
cecho2_rc=$?
kill $ESPP_PID 2>/dev/null; wait $ESPP_PID 2>/dev/null
result "cyclone: espp_be_pub->ros2_be_echo" $cecho2_rc

"$BIN"/rtps_interop_sub rt/chatter std_msgs::msg::dds_::String_ 0 3 30 > /tmp/csub2.log 2>&1 &
SUB_PID=$!
sleep 3
timeout 35 ros2 topic pub -r 5 --qos-reliability best_effort /chatter std_msgs/msg/String "data: 'cyclone be to espp'" > /tmp/crospub2.log 2>&1 &
ROS_PID=$!
wait $SUB_PID
csub2_rc=$?
kill $ROS_PID 2>/dev/null; wait $ROS_PID 2>/dev/null
tail -2 /tmp/csub2.log
result "cyclone: ros2_be_pub->espp_be_sub" $csub2_rc

lease_liveliness_cases "cyclone: " "c"
many_participants_case "cyclone: " "c"
endpoint_churn_case "cyclone: " "c"
rpc_cases "cyclone: " "c"

if grep -l "malformed packet received from vendor" /tmp/c*.log > /dev/null 2>&1; then
  echo "CycloneDDS reported malformed packets from espp:"; grep -h "malformed packet" /tmp/c*.log | head -3
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI

# --- Mixed RMWs on one espp service ----------------------------------------
# The server decides the reply form per request from the client's vendor id,
# so one espp service must serve a Fast DDS client and a Cyclone client at the
# same time, each in its own form, with no cross-talk between their replies.
note "mixed: a Fast DDS client and a Cyclone client call one espp service at the same time, three rounds"
"$BIN"/rtps_service_interop_server /add_two_ints example_interfaces::srv::dds_::AddTwoInts 60 > /tmp/mixsrv.log 2>&1 &
MIXSRV=$!
sleep 4
mix_rc=0
for round in 1 2 3; do
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout 25 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: $round}" > "/tmp/mixf$round.log" 2>&1 &
  mixf=$!
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp timeout 25 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 100, b: $round}" > "/tmp/mixc$round.log" 2>&1 &
  mixc=$!
  wait $mixf || mix_rc=1
  wait $mixc || mix_rc=1
  grep -q "sum=$((1 + round))" "/tmp/mixf$round.log" || mix_rc=1
  grep -q "sum=$((100 + round))" "/tmp/mixc$round.log" || mix_rc=1
done
sleep 1
kill $MIXSRV 2>/dev/null; wait $MIXSRV 2>/dev/null
echo "--- fastdds client, round 3 ---"; tail -1 /tmp/mixf3.log
echo "--- cyclone client, round 3 ---"; tail -1 /tmp/mixc3.log
echo "--- espp server ---"; grep "server:" /tmp/mixsrv.log | tail -3
result "mixed_rmw_clients->espp_service_server" $mix_rc

# --- Python bindings: functional round-trip of every RMI/AMI mechanism --------
# Build the espp wheel in-container and run the demo (multicast discovery works
# here via the shared network namespace), exercising the GIL-wrapped service /
# action / native bindings that the C++ tests cannot reach.
note "Python bindings functional demo (services + actions + native, 5 mechanisms)"
# The container-local copy has no .git (rsync excludes it), so setuptools-scm
# cannot derive a version - pin one via SETUPTOOLS_SCM_PRETEND_VERSION.
SETUPTOOLS_SCM_PRETEND_VERSION=0.0.0+interop \
  /opt/espp-venv/bin/pip install --no-build-isolation --no-deps -q . > /tmp/pywheel.log 2>&1
pywheel_rc=$?
if [ $pywheel_rc -ne 0 ]; then
  echo "python wheel build failed"; tail -20 /tmp/pywheel.log; result "python_rpc_demo" 1
else
  /opt/espp-venv/bin/python python/rtps_rpc_demo.py > /tmp/pydemo.log 2>&1
  pydemo_rc=$?
  grep -E "PASS|FAIL|ALL PASS|FAILURES" /tmp/pydemo.log | tail -8
  result "python_rpc_demo" $pydemo_rc
fi

echo ""
echo "==================== SUMMARY ===================="
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "INTEROP PASS" || echo "INTEROP FAIL"
exit $FAIL
