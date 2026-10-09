# YASMIN logic, ROS 2, and pointcloud audit

Date: 2026-10-09

Repository: `https://github.com/jcfurey/yasmin.git`

Reviewed commit: `1a724c401fbd2b86b27777074cd9e0e1d64426cc` (`main`, package version 6.1.2)

## Assessment and scope

At the reviewed commit, the existing core, ROS, and PCL tests passed on this machine. Additional checks nevertheless reproduced unsafe blackboard casts, broken cancellation of orthogonal regions, failure propagation that waits indefinitely for a sibling, cancellation that leaves a Python action waiting, ROS clients shared across different node namespaces, acceptance of malformed pointclouds, and a process crash from invalid projection coefficients.

Implementation has started with the crash, lifecycle, cancellation, and client-owner isolation fixes below. The original findings and reproduction evidence are retained for traceability; they describe the reviewed commit, not the current working tree.

This review concentrates on `yasmin`, `yasmin_ros`, and `yasmin_pcl`, with selected factory, viewer, package manifest, CMake, and CI paths. It is a source and local execution audit, not an exhaustive review of the editor, CLI, plugin manager, browser UI, or every supported ROS distribution. The original audit made no implementation changes. The following progress section tracks subsequent fixes. Source links below are relative to this report; line numbers in the original findings refer to the reviewed commit.

Evidence labels:

- **Reproduced:** observed in an additional local execution; mocked clients are identified explicitly.
- **Source-confirmed:** the implementation establishes the path; its timing or end-to-end effect was not reproduced.
- **Integration gap:** a supported-use boundary or ROS convention to address in application wiring, rather than an unconditional library bug.

Priorities: **P1** = crash, unsafe access, indefinite wait, incorrect cancellation, or cross-robot routing; **P2** = incorrect state/data handling or significant integration gap; **P3** = lower-impact configuration/namespace issue. These are remediation priorities, not a formal ROS quality-level declaration.

## Implementation progress — first fixes (2026-10-09)

The first fix batch is committed as the following reviewable series. **10 findings are fixed in this batch, 3 are partially addressed, and 11 remain open.** “Fixed” refers to the specific defect described in the inventory, not exhaustive verification across every ROS/PCL version.

| Commit | Change |
| --- | --- |
| `ccb03c48` | Checked blackboard reads and Python integer conversion contract |
| `97f5511a` | State lifecycle, parallel failure propagation, and orthogonal cancellation |
| `279194b0` | Factory Python proxy lifecycle |
| `96422b64` | ROS client ownership and Python action execution isolation |
| `45314fe2` | PCL cloud layout, model, and filter input validation |

| ID | Status | Implemented behavior and regression coverage |
| --- | --- | --- |
| L01 | Fixed | `Blackboard::get<T>()` checks the recorded type under the storage lock and throws a descriptive error before casting. Tests cover scalar/container/pointer mismatches, shared copies, remapping, and replacement with another type. |
| L02 | Fixed | Orthogonal regions execute through the state lifecycle wrapper. Active cancellation reaches region children; canceled regions can execute again. Parent cancellation returns the container's default outcome. |
| L03 | Fixed | A shared parallel runner records the first exception, cancels siblings, releases orthogonal barriers, joins workers, and rethrows the original failure. Cancellation repeats while workers stop, covering startup races. Tests exercise a waiting sibling and a failure before a barrier. |
| L05 | Fixed | Exceptions restore idle status, preserving cancellation. Completion and exception cleanup use atomic status transitions so they cannot overwrite a concurrent cancel. Reuse and null-blackboard regressions are included. |
| L06 | Fixed | The factory proxy calls the inner lifecycle wrapper and marks itself canceled when forwarding cancellation. An XML-loaded Python regression checks running/completed status and reuse after cancellation. |
| L07 | Partial | Duplicate region state-machine instances and multiple joins with the same sync ID inside one region are rejected. Validation of nested joins and paths that skip synchronization remains open. |
| R01 | Fixed | C++ caches use a unique token for the owning node's shared ownership identity; Python keys use weak references to the node instance. Identical names and namespaces cannot alias owners. `clear_for_node()` provides explicit cleanup; the default node uses it on destruction. C++ also prunes expired owners on subsequent lookup. Tests cover all endpoint types, distinct namespaces, duplicate fully qualified node names, reuse, and cleanup. |
| R02 | Fixed | Python action cancellation wakes the local result wait immediately and requests remote cancellation asynchronously. Tests cover missing goal responses and absent cancellation acknowledgements, including late goal acceptance. |
| R03 | Fixed | Each Python action invocation owns its completion event, result, status, and goal handle. Old result callbacks cannot complete a later invocation; late accepted old goals receive cancellation. Tests exercise timeout followed by immediate reuse. |
| R04 | Fixed | Python action/service discovery uses at most 100 ms wait slices, a monotonic deadline for each retry, and cancellation/context checks. Existing timeout/retry semantics are preserved. |
| P01 | Fixed | Projection validates supported model IDs, coefficient counts and finiteness, nonzero normals/axes, positive radii, and valid cone angles before PCL dispatch. Tests include empty, short, zero-normal, NaN, infinite, and unsupported-model inputs. |
| P02 | Partial | Filters validate required XYZ fields, configured filter fields, limits, voxel sizes, radius/statistical parameters, and crop bounds. A present but mistyped indices key now aborts instead of silently selecting all points. Missing-field and invalid-voxel regressions are included. Other PCL operations that report failures only through logs still need individual checks. |
| P03 | Partial | Shared schema validation checks dimensions, row/data lengths, field datatypes/counts/offsets, and index range before native filtering and conversion; file outputs are also checked. Filters pack padded rows without modifying their input and explicitly reject foreign-endian data. Conversion preserves valid padding and mixed fields. Exact ROS nanoseconds across PCL's microsecond header remain unresolved. |

Compatibility: checked blackboard reads require the exact stored C++ type. Python scalar integers are stored as `std::int64_t` and Python scalar floats as `double` ([conversion contract](../yasmin/include/yasmin/blackboard_pywrapper.hpp)). The factory suite contained an existing `get<int>()` read of a Python-produced integer; it now uses `get<std::int64_t>()`.

Parallel cancellation remains cooperative: a custom state must return when canceled. The runner cannot forcibly terminate user code or interrupt a PCL operation already in progress. Explicit application-owned nodes should call `ROSClientsCache.clear_for_node(node)` before destruction; cache cleanup does not destroy endpoints still held by their states.

Pointcloud validation currently supports the common ROS/PCL datatype IDs 1–10. Unsupported datatypes and foreign-endian native filtering return `aborted`; converters can preserve foreign-endian data without filtering it. P04's Python/native representation bridge is still open: checked blackboard reads now turn a mismatch into a reported error instead of undefined behavior.

Regression sources: [core blackboard](../yasmin/test/test_blackboard.cpp), [state lifecycle](../yasmin/test/test_state.cpp), [concurrence](../yasmin/test/test_concurrence.cpp), [orthogonal regions](../yasmin/test/test_orthogonal_state.cpp), [factory proxy](../yasmin_factory/test/test_yasmin_factory.cpp), [Python client cancellation and cache](../yasmin_ros/test/test_client_cancellation.py), [C++ cache](../yasmin_ros/test/test_publisher_state.cpp), and [PCL tests](../yasmin_pcl/test).

Verification: all affected packages and their dependencies built in Debug mode with `BUILD_TESTING=ON` on ROS 2 Lyrical / PCL 1.15.1. **419 individual test cases passed, with zero errors, failures, or skips**, including 27 newly added cases. `colcon test-result` reports 466 records because it also counts 47 CTest wrapper entries.

| Package | CTest entries | Individual cases passed |
| --- | ---: | ---: |
| `yasmin` | 14 | 241 |
| `yasmin_pcl` | 14 | 46 |
| `yasmin_ros` | 16 | 79 |
| `yasmin_factory` | 3 | 53 |

Build, install, and test artifacts are under `/tmp/yasmin-audit`. ROS tests used localhost discovery and domain 177. `git diff --check` passed. Older ROS/PCL versions and a real sensor/bag-to-publisher pipeline have not been verified in this batch.

## Implementation progress — C++ client cleanup (2026-10-09)

Commit `56719da2` addresses **R05–R07**. Across both batches, **12 findings are fixed, 4 are partially addressed, and 8 remain open**. R06 remains partial because legacy rclcpp releases do not expose the required cleanup API.

| ID | Status | Implemented behavior and regression coverage |
| --- | --- | --- |
| R05 | Fixed | Each C++ action execution retains its own goal handle, completion state, and cancellation intent. Response timeout requests remote cancellation. Acceptance arriving after cancellation, timeout, reuse, or state destruction still triggers cancellation. Local cancellation wakes the waiter without waiting for the server's acknowledgment. Old result callbacks cannot complete a later execution. Feedback for abandoned goals is suppressed, and a new goal waits for an in-flight feedback handler to finish. |
| R06 | Partial | A scope guard retains the returned service request and removes only that invocation's pending entry on timeout, cancellation, or exception where rclcpp exposes `remove_pending_request()`. A regression preserves an unrelated request on the same cached client; another cancels and reuses a state 20 times with no pending entries retained. Foxy lacks a public removal API; feature detection retains compilation compatibility but cannot provide cleanup there. Older distributions have not been built or tested. See the primary [Foxy client implementation](https://github.com/ros2/rclcpp/blob/foxy/rclcpp/include/rclcpp/client.hpp). |
| R07 | Fixed | Action completion/rejection and all three states' cancellation transitions use the corresponding wait mutex. Monitor timed waits use a predicate, including cancellation already requested before waiting. Discovery runs outside the wait mutex so cancellation can interrupt polling. User request/goal/result handlers run outside it and may cancel their own state. Tests cover fast responses/rejections, cancellation and reuse, pre-canceled monitors, absent endpoints, and cancellation from user handlers. The narrow check-to-sleep race is prevented by synchronization; it is not forced with instrumentation. |

Cancellation is a request to the remote action server, which may reject it or be unavailable. The local outcome does not wait for remote termination. Service request cleanup releases client bookkeeping and cannot undo side effects already executing on the server. Delayed goal acceptance still requires the owning action client and executor to remain available to process that callback. Feedback handlers must return promptly: rclcpp can hold its client mutex during them, delaying cancellation dispatch. Execution reuse waits for an in-flight feedback handler to finish.

Regression source: [C++ client cleanup and waits](../yasmin_ros/test/test_client_cleanup.cpp). All 17 new cases passed on ROS 2 Lyrical. The affected packages and their dependencies rebuilt successfully in Debug mode. The complete ROS suite (96 cases) and factory suite (53 cases) passed after the final change. Together with the unchanged first-batch core (241) and PCL (46) results, **436 individual cases passed with zero errors, failures, or skips**. `colcon test-result` reports 484 records, including 48 CTest wrapper entries. Final ROS/factory logs are under `/tmp/yasmin-audit/test-log-final-handoff`. `git diff --check` passed.

Next work: **L04** signal handling; **R08–R09** Python node shutdown and TF clock ownership; **P04** a Python/native cloud bridge; completion of **L07/P02/P03** and legacy **R06** cleanup; then **S01–S04** sensor QoS, composition/callback contracts, and viewer namespacing.

## Implementation progress — ROS 2 and Nav2 conventions (2026-10-09)

This batch addresses L04, L07, R08, R09 and S01–S04, and seven additional findings (N01–N07) from a follow-up review of ROS 2 and Nav2 integration. Of the original 24 findings, **17 are now fixed, 4 are partially addressed (R06, P02, P03, S02), 2 are addressed by documented contracts (S01, S03), and 1 remains open (P04)**. All seven new findings are fixed.

| ID | Status | Implemented behavior and regression coverage |
| --- | --- | --- |
| L04 | Fixed | The SIGINT handler only updates lock-free atomics and writes to a self-pipe. A dispatcher thread, with SIGINT blocked, runs cancellation callbacks under the registry lock, so unregistering waits for an in-flight cancellation. The first SIGINT cancels the state machine and leaves the ROS context valid, so remote goals can still be canceled. A repeated SIGINT is forwarded to the previous handler (rclcpp/rclpy, or the default action). A generation token discards wake-ups from registrations that have ended. If another handler was installed on top during execution, YASMIN stays in its chain as a pass-through instead of removing it. Tests cover execution off the signal context (C++ and a Python `cancel_state` override), escalation, repeated executions, the disabled handler, and a handler installed later; 50/50 stress runs passed. |
| L07 | Fixed | Joins are discovered in nested state machines of a region, with the one-join-per-region check applied across nesting. A region that finishes, including through a path that skips its JoinState, drops out of its barriers (`arrive_and_drop` semantics), so siblings are released. `reset()` restores the participant count for the next execution. The skip and nested cases fail on the previous implementation. |
| R08 | Fixed | Python cleanup (executor, spin thread, node) runs whether or not the context is valid and is idempotent. `get_instance()` replaces an instance whose context was shut down, in Python and C++. C++ releases the stale node before calling `rclcpp::init()`, since rclcpp aborts when re-initializing the default context while nodes of the previous one are alive. |
| R09 | Fixed | The Python buffer is created with the node, so it uses the node's ROS clock and its jump handling. Replacing the buffer unregisters the old listener's subscriptions, its `tf2_frames` service, and its clock jump callback. Both TF states accept an optional node. |
| S01 | Documented | Both monitors keep the reliable default, which is the ROS default for generic topics; documentation directs sensor consumers to `SensorDataQoS` / `qos_profile_sensor_data`. The existing monitor demos already use it. `msg_queue < 1` is rejected (N07). |
| S02 | Partial | `YasminNode::get_instance(name, NodeOptions)` and `YasminNode.get_instance(name)` name the singleton when they create it; a `__node` remap still takes precedence. The factory executables use their executable names, so parameter files keyed by node name apply. Custom contexts for the singleton are rejected with an explicit error; application-owned nodes remain the supported route. The C++ auto-initialization still passes no command-line arguments. The viewer node is now a registered component (`yasmin_viewer::YasminViewerNode`), verified by loading it into a container in a namespace. |
| S03 | Documented | The README states the execution contract: states block their caller, so run the state machine outside callbacks of an executor that must deliver its responses, or use a separate or reentrant callback group. |
| S04 | Fixed | Publishers and the viewer node use the relative topic `fsm_viewer`. For root-namespace nodes, this resolves to the previous `/fsm_viewer`. Verified end to end: a publisher in `/robot1` is shown by a viewer started in `/robot1`. Name collisions between unnamed machines in the same namespace remain. |
| N01 | Fixed | **P1, reproduced.** Destroying the C++ `YasminNode` could hang: `Executor::cancel()` is lost if the spin thread has not yet entered `spin()`, which then blocks indefinitely while the destructor joins it. A create-then-destroy test sequence hung in 6 of 8 runs, with the backtrace in `stop_executor()`. Cancellation is now repeated until `spin()` returns: 0 of 40 runs hung. Python is unaffected because its executor's shutdown flag persists. |
| N02 | Fixed | **P2, integration gap.** Action states discarded the result of aborted goals, so Nav2's `error_code` and `error_msg` (e.g. `NO_VALID_PATH`, `TF_ERROR`) were unreachable. `set_abort_handler()` (C++) and the `abort_handler` keyword argument (Python) map that result to an outcome. Rejected goals, which have no result, still return ABORT. Verified with the C++/Python test servers, and end to end with `nav2_msgs/NavigateToPose` in a namespace (`TF_ERROR` 9002 surfaced). |
| N03 | Fixed | **P2, source-confirmed.** The C++ TF state created a new buffer and listener on every execution, which discarded TF history. Its buffer had no timer interface, so `waitForTransform()` threw. The listener writes into the buffer through a raw reference while the blackboard released the two independently, leaving a window in which the listener could outlive its buffer. The pair is now reused until `cache_time_sec` changes, the buffer has a `CreateTimerROS` interface (as in Nav2), and the listener owns a reference to its buffer. |
| N04 | Fixed | **P2, reproduced with a mocked client.** The Python service state shared one response/event across invocations: a late response could complete a later invocation, and a cancellation between the check and the wait could be lost. Each invocation now owns its completion state, and the cancellation check and registration share a lock. The regression fails on the previous implementation. |
| N05 | Fixed | **P3, reproduced.** The README/HTML Nav2 demos used the absolute `/navigate_to_pose`, which defeats namespacing. The Python demo passed `None` as outcomes, which raised `TypeError`, and the C++ demo did not compile (`std::map` transitions). Both now use the relative name and show the abort handler. The C++ demo compiles against `nav2_msgs`; the Python state ran against a NavigateToPose server. `setup_outcomes()` accepts `None`. |
| N06 | Fixed | **P3, source-confirmed.** The Python viewer timer used the node's ROS clock, so it stopped while simulated time was paused; it now uses steady time, like the C++ wall timer. |
| N07 | Fixed | **P3, source-confirmed.** Monitors accepted `msg_queue < 1`, which discards every message; both languages now reject it. |

Other changes: the C++ `PublisherState` publishes an owned message, avoiding a copy with intra-process communication.

Compatibility:

- **Namespaced viewer users:** a namespaced node now publishes to `<ns>/fsm_viewer`. Start the viewer in that namespace or remap the topic.
- **Ctrl-C:** a second SIGINT during a `handle_sigint` execution now reaches the previous handler.
- **Factory node names:** the factory executables now have fixed node names, so two instances in one namespace need distinct `__node`/launch names.
- **TF state reuse:** the C++ TF state returns the same buffer/listener across executions.
- **API changes:** `yasmin_viewer` depends on `rclcpp_components`. Python `ServiceState.response_callback` is now private.

Regression sources: [SIGINT](../yasmin/test/test_sigint_handler.cpp) and [Python SIGINT](../yasmin/test/test_state_machine.py), [orthogonal joins](../yasmin/test/test_orthogonal_state.cpp), [C++ node lifecycle](../yasmin_ros/test/test_yasmin_node.cpp), [Python node lifecycle and TF clock](../yasmin_ros/test/test_yasmin_node.py), [C++ TF](../yasmin_ros/test/test_tf_buffer_state.cpp), [action abort results](../yasmin_ros/test/test_action_client_state.cpp) ([Python](../yasmin_ros/test/test_action_client_state.py)), [service invocation isolation](../yasmin_ros/test/test_client_cancellation.py), and [monitor queue](../yasmin_ros/test/test_monitor_state.cpp) ([Python](../yasmin_ros/test/test_monitor_state.py)).

Verification: `yasmin`, `yasmin_ros`, `yasmin_viewer`, `yasmin_factory` and `yasmin_pcl` rebuilt in Debug mode with `BUILD_TESTING=ON` on ROS 2 Lyrical (rclcpp 32, Cyclone DDS), with no new compiler warnings. **464 individual cases passed, with zero errors, failures, or skips.** `colcon test-result` reports 515 records, which include CTest wrapper entries.

| Package | `colcon` records | Individual cases passed |
| --- | ---: | ---: |
| `yasmin` | 266 | 251 |
| `yasmin_ros` | 133 | 114 |
| `yasmin_factory` | 56 | 53 |
| `yasmin_pcl` | 60 | 46 |

Each new regression for L07, N01 and N04 was also run against the previous implementation, where it failed or hung. Additional checks:

- The C++ Nav2 demo from the README compiles against `nav2_msgs`.
- The README's Python `Nav2State` ran against a NavigateToPose server in `/robot1`.
- The viewer component loaded into `component_container` in `/robot1` and served HTTP, and the standalone executable still runs.
- A namespaced `YasminViewerPub` appeared in a viewer started in the same namespace.

Builds used a scratch workspace outside the repository, with a private `ROS_DOMAIN_ID` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. `git diff --check` passed.

Remaining work: **P04** (Python/native cloud bridge); **P02/P03** completion; **S02** custom contexts for the singleton and passing C++ command-line arguments through auto-initialization; distinct identities for same-named machines in one viewer namespace. **R06** cleanup remains unavailable on Foxy, which is end-of-life. Older ROS distributions advertised by CI have not been built in this batch.

## Implementation progress — point cloud pipeline and remaining items (2026-10-09)

This batch completes P02, P03, P04 and S02, finishes the S04 viewer identity work, and fixes two new findings (N08–N09). R06 is closed as won't fix: it only affects Foxy, which is end-of-life and not a target. Of the original 24 findings, **21 are fixed, 2 are addressed by documented contracts (S01, S03), and 1 is won't fix (R06)**.

| ID | Status | Implemented behavior and regression coverage |
| --- | --- | --- |
| P02 | Fixed | Two more PCL failures reported only through logs now return `aborted`. VoxelGrid returned its input unfiltered when voxel indices would overflow; the state now repeats PCL's check over the same selected points (verified to honor input indices). StatisticalOutlierRemoval returned an empty cloud for a zero multiplier, which the `PCLPointCloud2` filter treats as unset; it now requires a positive value. Both regressions fail on the previous implementation. |
| P03 | Fixed | `RosToPclPointCloud2State` writes the exact ROS header to `output_header`. `PclToRosPointCloud2State` restores the nanosecond stamp from an optional `input_header`, but only while the cloud keeps that frame and microsecond time, so a stale or unrelated header is ignored. PCD/PLY writers address points as packed rows and silently wrote corrupt files for row-padded clouds (reproduced in all five writer modes); the save states now pack rows and reject foreign-endian data first. |
| P04 | Fixed | **Native path:** new `PointCloud2MonitorState` and `PointCloud2PublisherState` plugins receive and publish clouds in C++ and store shared pointers, so a Python-authored state machine can keep the data path native through `CppStateFactory`. The monitor offers QoS profiles (default `sensor_data`), a latest-message queue, a timeout, and a freshness option. Its callback owns a shared inbox rather than the state, and cancellation updates the wait predicate under the inbox lock. **Checked bridge:** conversion and publisher states accept a `PointCloud2` shared pointer, by value, or as serialized bytes (Python `serialize_message`); `output_format: serialized` produces bytes for Python. A Python message object is rejected with a hint instead of a type error. |
| S02 | Fixed | When the C++ singleton initializes rclcpp, it now passes the process arguments from `/proc/self/cmdline`, as `rclpy.init()` uses `sys.argv`; if parsing fails it falls back to no arguments with a warning. `--ros-args` namespaces, remappings and parameters therefore apply to C++ states, including inside Python processes. Custom contexts are supported through a `MultiThreadedExecutor` built with `ExecutorOptions`; an uninitialized custom context is rejected. A re-exec test checks namespace and `use_sim_time` from the real command line; 30/30 stress runs passed. |
| S04 | Fixed | The viewer keys its cache by publisher GID. Same-named machines from different publishers are listed as `NAME`, `NAME (2)` in first-seen order instead of replacing each other. A new viewer test covers namespaced reception and isolation from another namespace; it fails on the previous cache. |
| R06 | Won't fix | Pending-request removal is unavailable only on Foxy, which is end-of-life. |
| N08 | Fixed | **P1, reproduced with Valgrind** (`Invalid write of size 4` in `pcl::ExtractIndices<PCLPointCloud2>::applyFilter`). With `keep_organized`, PCL writes a 4-byte float at every field offset of removed points. With fields narrower than 4 bytes, such as a `uint8`/`uint16` ring, this corrupts the next point (`x` 106 → 63.94 in the probe) and writes past the buffer at the last point. RandomSample, CropBox, PassThrough, SOR and RadiusOutlierRemoval were probed and do not write outside the removed point. `ExtractIndicesState` now builds the organized output itself: removed points get the user value in their floating-point fields and keep integer fields. |
| N09 | Fixed | **P2, reproduced.** C++ plugin states could not be configured with numeric parameters from XML or Python. XML stores `int`/`double` and Python `int64`/`double`, while the plugins read `float`/`int`; after L01, `configure()` threw `has type 'double', requested 'float'`. Before L01, the bytes were silently reinterpreted. `State::get_parameter<T>()` now converts numeric values when representable and rejects fractional→integer and out-of-range values; `bool` and non-numeric types stay exact. |

**Python vs. C++ on the cloud path** (Release build, one core, Ouster-like 1024×128 cloud of 6.3 MB):

| Step | C++ | Python |
| --- | ---: | ---: |
| Deserialize received message | 0.25 ms | 2.5 ms |
| Serialize to cross Python→C++ | 0.5 ms | 3.5 ms |
| 0.5 m voxel downsample (PCL / NumPy) | 19 ms | 120 ms |

Python subscriptions deserialize every message while holding the GIL, including while their state is inactive. **Recommendation:** keep the cloud path in C++ states and use Python for decisions. Verified end to end: a Python state machine running under `--ros-args -r __ns:=/robot1` received a 10 Hz Ouster-like stream with the C++ monitor (`/robot1/points`), ran conversion → VoxelGrid → conversion → publish, and delivered 20/20 frames (131k → 112k points) on `/robot1/points_filtered`, preserving the 123456789 ns stamp.

Compatibility:

- **Dependencies:** `yasmin_pcl` now depends on `rclcpp`, `std_msgs` and `yasmin_ros`.
- **SOR:** `stddev_mul_thresh` must be positive.
- **ExtractIndices:** organized output keeps integer fields of removed points.
- **Saving:** the save states reject foreign-endian clouds.
- **Auto-init arguments:** C++ auto-initialization honors the process `--ros-args`, including `__node`, which then also names the C++ singleton in Python processes.
- **ABI:** `YasminNode` holds its executor through `std::unique_ptr<rclcpp::Executor>`.
- **Viewer:** viewer JSON keys may carry a ` (2)` suffix.
- **Parameters:** numeric parameter reads convert instead of throwing.

Verification: all five packages rebuilt in Debug mode with `BUILD_TESTING=ON` on ROS 2 Lyrical with no compiler warnings. **487 individual cases ran with zero errors or failures.** One case skips by design: it is the child half of the command-line re-exec test, which runs and passes inside its parent. `colcon test-result` reports 540 records, which include CTest wrapper entries.

| Package | `colcon` records | Individual cases |
| --- | ---: | ---: |
| `yasmin` | 268 | 253 |
| `yasmin_ros` | 136 | 117 (1 skipped by design) |
| `yasmin_viewer` | 2 | 1 |
| `yasmin_factory` | 56 | 53 |
| `yasmin_pcl` | 78 | 63 |

The N08, P02, P03 save and S04 regressions were also run against the previous implementation, where they fail. The XML and Python parameter probes for `VoxelGridState` and `StatisticalOutlierRemovalState` failed before N09 and pass after it. `git diff --check` passed. Older ROS distributions advertised by CI have not been built.

## Verification — CI checks and other distributions (2026-10-09)

GitHub Actions has not run on the fork, so none of the upstream CI checks had verified this branch. They were reproduced locally with the same tools.

| Check | Result before | Fix |
| --- | --- | --- |
| clang-format 18.1.8 (`cpp-formatter.yml`) | 17 changed C++ files failed; `main` passes | Formatted; the whole repository now passes |
| black, line length 90 (`python-formatter.yml`) | 4 changed Python files failed | Formatted; the whole repository now passes |
| Jazzy (`ros:jazzy`, PCL 1.14) build | `yasmin_pcl` failed to compile | See below |
| Humble (`ros:humble`, PCL 1.12) build | `yasmin_pcl` failed to compile | See below |

**Compile failure.** The VoxelGrid overflow pre-check (P02) used `getMinMax3D` overloads with indices, which first appear in PCL 1.15. Older PCL also computes the VoxelGrid bounds over the whole cloud, so the pre-check now follows the installed version.

**Humble TF clock.** On Humble, the Python tf2 `Buffer` has no clock or time-jump handling; passing the node only adds the `tf2_frames` service. The R09 change is harmless there, and the clock assertion is skipped on that version.

The Docker builds mirror `rosdep install` plus `colcon build/test`. They cover `yasmin`, `yasmin_msgs`, `yasmin_ros`, `yasmin_viewer`, `yasmin_factory` and `yasmin_pcl`; `yasmin_editor` was not changed and is not included.

| Distribution | Build | `colcon test-result` |
| --- | --- | --- |
| Lyrical (host, PCL 1.15) | OK, no warnings | 540 records, 0 errors/failures, 1 skipped by design |
| Jazzy (Docker) | OK | 540 records, 0 errors/failures, 1 skipped by design |
| Humble (Docker) | OK | 540 records, 0 errors/failures, after the TF skip above |

Kilted and Rolling have not been built.

## Implementation progress — Nav2 goal acceptance (2026-10-09)

| ID | Status | Implemented behavior and regression coverage |
| --- | --- | --- |
| N10 | Fixed | **P2, integration gap.** `response_timeout` bounds goal acceptance and result together, so the only way to bound a server that never accepts a goal also capped how long navigation could run. Nav2's behavior tree action nodes bound only acceptance (`server_timeout`). `set_goal_response_timeout()` (C++) and `goal_response_timeout` (Python) add that separate bound: on expiry the state returns `timeout` and requests cancellation, so a goal accepted later is canceled. Tested against a real server that accepts after 1.5 s (`timeout` after 0.3 s, then the late goal receives a cancel request). Also tested with a 5 s goal that still succeeds under a 0.3 s bound, and with mocked late responses in Python. |
| N11 | Fixed | **P3, source-confirmed.** The Python `ActionState` built its goal before waiting for the server, while C++ builds it afterwards, so a long wait could send a goal built from stale blackboard data. It now builds the goal once the server is available. A regression checks that no goal is built while the server is unavailable. |

Verification: `yasmin_ros` 141 records and `yasmin_factory` 56 records passed on Lyrical (one skip by design). `yasmin_ros` also passed rebuilt in the Jazzy and Humble images. Both formatters pass repository-wide.

## Findings inventory

| ID | Priority | Area | Finding | Evidence |
| --- | --- | --- | --- | --- |
| L01 | P1 | Blackboard | `get<T>()` casts without checking the stored type | Reproduced |
| L02 | P1 | Orthogonal execution | Regions bypass state lifecycle; cancellation skips active regions | Reproduced |
| L03 | P1 | Parallel execution | A failed child does not unblock/cancel its siblings before joining | Reproduced for orthogonal barrier |
| L04 | P1 | Signal handling | SIGINT handler locks mutexes and invokes arbitrary cancellation code | Source-confirmed; opt-in path |
| L05 | P2 | State lifecycle | Exceptions leave a state marked running | Reproduced |
| L06 | P2 | Python factory proxy | Proxy bypasses inner lifecycle and does not mark itself canceled | Source-confirmed |
| L07 | P1 | Region synchronization | Barrier participants are counted as join objects, not distinct regions | Source-confirmed |
| R01 | P1 | ROS client cache | Cache keys identify nodes by short name only | Reproduced with Python node stubs; same C++ design |
| R02 | P1 | Python action | Cancellation does not wake the result wait | Reproduced with a pending mock client |
| R03 | P1 | Python action | Late callbacks can complete a subsequent execution with an old result | Reproduced by callback injection |
| R04 | P2 | Python clients | Indefinite server discovery waits do not observe cancellation promptly | Source-confirmed |
| R05 | P1 | C++ action | Timeout leaves the remote goal running; early cancellation misses a late goal handle | Source-confirmed |
| R06 | P2 | C++ service | Timeout/cancellation does not remove pending requests | Source-confirmed |
| R07 | P1 | C++ waits | Action completion and cancellation notifications can be lost | Source-confirmed; timing-dependent |
| R08 | P2 | Python node | Destroying the singleton after context shutdown skips explicit cleanup | Source-confirmed |
| R09 | P2 | Python TF | Buffer is created without the node's ROS clock | Source-confirmed |
| P01 | P1 | PCL projection | Invalid model coefficients can crash instead of returning `aborted` | Reproduced; ASan stack trace |
| P02 | P2 | PCL filtering | PCL error returns/logs are treated as successful filtering | Reproduced for missing field |
| P03 | P1 | Pointcloud validation | Conversion accepts inconsistent layouts and payload lengths | Reproduced; unsafe downstream use is a risk |
| P04 | P1 | Python/C++ cloud boundary | Python ROS messages and native PCL state inputs have incompatible stored types | Source-confirmed integration gap; interacts with L01 |
| S01 | P2 | Sensor QoS | Default monitor QoS cannot receive best-effort sensor publishers | Conditional interoperability gap |
| S02 | P2 | ROS contexts/composition | Default runtime nodes hide context, naming, and executor ownership | Integration gap |
| S03 | P2 | Callback execution | Blocking state execution can deadlock a caller's ROS callback group | Conditional integration gap |
| S04 | P3 | Viewer namespace | Absolute `/fsm_viewer` topic bypasses node namespaces | Source-confirmed convention deviation |
| N01 | P1 | C++ default node | Destruction can hang when executor cancellation precedes `spin()` | Reproduced (follow-up review) |
| N02 | P2 | Action results | Aborted-goal results (Nav2 `error_code`) are discarded | Integration gap (follow-up review) |
| N03 | P2 | C++ TF state | Buffer recreated per execution, no timer interface, listener may outlive buffer | Source-confirmed (follow-up review) |
| N04 | P2 | Python service | Responses and cancellation are not tied to an invocation | Reproduced with a mocked client (follow-up review) |
| N05 | P3 | Nav2 demos | Absolute action name; Python demo raises, C++ demo does not compile | Reproduced (follow-up review) |
| N06 | P3 | Python viewer | Publication stops while simulated time is paused | Source-confirmed (follow-up review) |
| N07 | P3 | Monitors | `msg_queue < 1` silently discards every message | Source-confirmed (follow-up review) |
| N08 | P1 | PCL ExtractIndices | Organized output writes past narrow fields and the buffer end | Reproduced with Valgrind (follow-up review) |
| N09 | P2 | State parameters | C++ plugins reject numeric parameters from XML and Python | Reproduced (follow-up review) |
| N10 | P2 | Action goal acceptance | No bound on goal acceptance without also capping goal duration | Integration gap (follow-up review) |
| N11 | P3 | Python action | Goal built before waiting for the server | Source-confirmed (follow-up review) |

### L01 — Unchecked C++ blackboard casts

**Location:** [blackboard.hpp](../yasmin/include/yasmin/blackboard.hpp), `get<T>()`, lines 164–185; contrast `set<T>()`, lines 122–149, which records type information.

`get<T>()` checks that a key exists, then dereferences a `static_pointer_cast<T>` from `shared_ptr<void>`. It never compares `T` with the type registry. Storing `float(1.0)` and requesting `int` returned `1065353216` without an exception in the reproduction. Wrong pointer, container, or ROS-message types can cause undefined behavior and process failure. A surrounding `catch (std::exception&)` does not make this safe.

**Remediation:** perform a checked lookup and cast under the same storage lock; use a reliable type token and throw a descriptive type error. Cover scalar, pointer, message, container, and parameter mismatches, including a Python object passed to a native C++ consumer.

### L02 — Orthogonal regions cannot be canceled through their lifecycle

**Location:** [orthogonal_state.cpp](../yasmin/src/yasmin/orthogonal_state.cpp), lines 151–163 and 220–232; [state_machine.cpp](../yasmin/src/yasmin/state_machine.cpp), lines 567–588.

Region workers call `sm->execute(bb_copy)` directly. Only `State::operator()` marks a state running. `OrthogonalState::cancel_state()` then calls cancellation only for regions whose `is_running()` is true; `StateMachine::cancel_state_machine()` has the same running guard. The reproduction observed an actively executing region with `is_running() == false` and an uncanceled child after canceling its parent.

**Impact:** long-running service, action, or monitor states can keep the orthogonal container blocked in `join()` after cancellation. Releasing a barrier does not cancel ordinary work after that barrier.

**Remediation:** execute each region through its lifecycle wrapper, propagate cancellation consistently, and test cancellation while a child is actively waiting, including re-execution afterward.

### L03 — Child failure can leave parallel execution waiting forever

**Location:** [orthogonal_state.cpp](../yasmin/src/yasmin/orthogonal_state.cpp), lines 157–162 and 182–208; [concurrence.cpp](../yasmin/src/yasmin/concurrence.cpp), lines 181–193 and 209–233.

Workers save exceptions, but the parent joins every thread before checking them. A region throwing before a shared `JoinState` leaves another region waiting at the barrier. The failure did not propagate until the reproduction explicitly canceled the parent to release the barrier. A `Concurrence` has the same ordering problem if another child waits indefinitely.

**Remediation:** record the first failure, signal sibling cancellation and barrier release immediately, then join and rethrow. Preserve the original exception and bound cleanup where the child contract permits it.

### L04 — SIGINT handling is not signal-safe and replaces the ROS handler

**Location:** [state_machine.cpp](../yasmin/src/yasmin/state_machine.cpp), lines 42–78 and 508–512.

When signal handling is enabled, the installed handler locks a `std::mutex`, traverses a container, and invokes cancellation callbacks. Those callbacks can log, lock other mutexes, or wait for action cancellation. These operations are unsuitable inside an asynchronous signal handler and can deadlock if SIGINT interrupts code holding a needed lock. The previous handler is saved for restoration, but is not invoked while the YASMIN handler is installed; ROS context shutdown can therefore be suppressed during execution.

**Remediation:** let ROS/application code own SIGINT, or notify a normal execution context through an appropriate signal-safe mechanism and perform cancellation there. This finding applies to `handle_sigint=true`, not every execution.

### L05 — Exception paths leave stale running status

**Location:** [state.cpp](../yasmin/src/yasmin/state.cpp), lines 96–149.

`operator()` marks the state running, injects defaults, and calls `execute()` without an exception cleanup guard. The invalid-outcome path resets status, but a thrown execution/default-injection exception does not. A throwing callback left `is_running() == true` after unwinding.

**Remediation:** define the failure status and restore it on all exception paths through a scope guard or catch/rethrow. Verify both a leaf and an enclosing state machine, and verify reuse after failure.

### L06 — Python factory holders have a separate, inconsistent lifecycle

**Location:** [yasmin_factory.cpp](../yasmin_factory/src/yasmin_factory.cpp), lines 390–413 and 424–444.

`PythonStateHolder::execute()` calls the inner state's `execute()` directly, while the parent invokes the holder's lifecycle wrapper. The inner state's status is therefore not reset for each invocation. `cancel_state()` forwards to the inner state without marking the holder canceled. A Python state that checks `is_canceled()` can remain canceled on later invocations; the holder can meanwhile be reported completed. Python-loaded containers with running-status guards are also exposed to this mismatch.

**Remediation:** make the proxy and inner lifecycle consistent, including cancellation, status reporting, default injection, and reuse. Verify a factory-loaded Python state in a loop, canceled and subsequently invoked again.

### L07 — Invalid barrier topology is accepted

**Location:** [orthogonal_state.cpp](../yasmin/src/yasmin/orthogonal_state.cpp), lines 80–105; [region_barrier.cpp](../yasmin/src/yasmin/region_barrier.cpp), lines 28–45.

Configuration groups every top-level join object by `sync_id` and uses the number of objects as the barrier party count. Two sequential joins with the same ID in one region satisfy the minimum count of two, although that region cannot reach its second join before the first releases. Duplicate region state-machine instances are also not rejected by `add_region()`.

**Remediation:** validate participants by distinct region and reject shared region instances. Either reject multiple joins with one ID in a region or define and implement their generation semantics. Nested joins and branches that can skip a join need explicit validation/documentation as well.

### R01 — Cached clients can belong to the wrong node or robot

**Location:** [ros_clients_cache.hpp](../yasmin_ros/include/yasmin_ros/ros_clients_cache.hpp), lines 69–75, 118–124, and 179–186; [ros_clients_cache.py](../yasmin_ros/yasmin_ros/ros_clients_cache.py), lines 106–109, 139–142, and 174–184.

Keys use `node->get_name()` / `node.get_name()`, not the node instance, namespace, resolved endpoint, or ROS context. Two `processor` nodes in `/robot_a` and `/robot_b` asking for relative topic `cloud` can share the publisher created for robot A. The stub reproduction returned the same publisher with owner `/robot_a` for both requests. C++ uses the same identifying fields. Recreating a node with the same name can also return stale cached entities.

**Remediation:** scope caches to node identity/lifetime and context, with resolved endpoint and complete QoS identity; clean them when their owner is destroyed. A fully qualified node name alone still does not distinguish distinct nodes with identical names.

### R02 — Python action cancellation does not wake the waiting state

**Location:** [action_state.py](../yasmin_ros/yasmin_ros/action_state.py), lines 136–154 and 205–218.

`cancel_state()` requests goal cancellation and changes status, but does not set `_action_done_event`. With the default `response_timeout=None`, execution stays in `Event.wait()` until a result arrives. A pending mock goal response demonstrated that the worker remained alive after cancellation; manually setting the event allowed it to return `canceled`. Cancellation rejection or a disconnected server can produce the same persistent wait.

The method also calls the rclpy future's `.result()` as though it waited for completion; the installed rclpy implementation returns the current result or `None` rather than blocking.

**Remediation:** wake the local waiter immediately and manage remote cancellation separately with bounded confirmation. Test cancellation before goal acceptance, after acceptance, on server loss, and when cancellation is rejected.

### R03 — Python action callbacks have no execution identity

**Location:** [action_state.py](../yasmin_ros/yasmin_ros/action_state.py), lines 187–203 and 236–285. C++ already has an epoch mechanism in [action_state.hpp](../yasmin_ros/include/yasmin_ros/action_state.hpp), lines 410–425.

Every run resets shared fields and reuses the same event. Goal-response/result callbacks carry no generation or goal identity check. An old goal completing after timeout and after the next run starts can replace the new goal handle, set the new run's event, and deliver the old result. Injecting a previous result callback after resetting the event accepted `old_goal` and set the event.

**Remediation:** associate callbacks with a per-execution object or generation, and reject stale results, feedback, and goal handles. An old accepted goal may still need explicit cancellation rather than merely ignoring it.

### R04 — Python discovery waits are not cancellation-aware

**Location:** [ros_state_utils.py](../yasmin_ros/yasmin_ros/ros_state_utils.py), lines 47–63; [action_state.py](../yasmin_ros/yasmin_ros/action_state.py), lines 175–180; [service_state.py](../yasmin_ros/yasmin_ros/service_state.py), lines 142–147.

`wait_with_retry()` checks cancellation only after `condition_fn()` returns false. Default `wait_for_server(None)` and `wait_for_service(timeout_sec=None)` can wait indefinitely when the endpoint is absent. Changing state status does not interrupt those calls. C++ discovery uses 100 ms slices instead.

**Remediation:** use short discovery slices with an overall deadline and cancellation/context checks. Verify shutdown and cancellation with an endpoint that never appears.

### R05 — C++ action timeout/cancellation can leave a goal executing remotely

**Location:** [action_state.hpp](../yasmin_ros/include/yasmin_ros/action_state.hpp), lines 289–328, 450–472, and 584–593.

Response timeout returns `TIMEOUT` without issuing goal cancellation. Cancellation before the goal-response callback has supplied a handle cannot cancel the goal; the later callback stores the accepted handle without checking a pending cancel request. The previous handle is also not explicitly cleared when sending a new goal. A state machine can consequently advance to another behavior while the old remote action continues.

**Remediation:** retain cancellation intent per goal, cancel a handle that arrives late, reset handle state per execution, and specify timeout cleanup semantics. Test with delayed goal acceptance and a server that continues running after the local timeout.

### R06 — C++ service requests accumulate after timeout

**Location:** [service_state.hpp](../yasmin_ros/include/yasmin_ros/service_state.hpp), lines 313–347 and 369–371.

The returned request identifier/future is discarded. Timeout or cancellation exits without `remove_pending_request()`. Repeated requests to a server that receives but never responds can leave pending requests in the cached client. Epoch checks prevent an old response from being used, but do not remove the pending entry. Python has explicit pending-request removal on these paths.

**Remediation:** retain the request identifier and remove it when abandoning a wait. This is client-side bookkeeping; it does not cancel side effects already executing on a service server.

### R07 — C++ condition-variable notifications can be lost

**Location:** [action_state.hpp](../yasmin_ros/include/yasmin_ros/action_state.hpp), lines 456–468 and 646–650; [monitor_state.hpp](../yasmin_ros/include/yasmin_ros/monitor_state.hpp), lines 167–175 and 219–221; [service_state.hpp](../yasmin_ros/include/yasmin_ros/service_state.hpp), lines 335–345 and 369–371.

Action result/rejection handlers change the completion predicate and notify without holding `action_done_mutex`. Cancellation changes the status predicate and notifies without the wait mutex in action, service, and monitor states. A notifier can run after the waiter checks its predicate but before it actually blocks. An atomic predicate prevents a data race on the flag; it does not prevent this missed notification. With no timeout, the wait can persist indefinitely.

**Remediation:** synchronize predicate transitions with the corresponding wait mutex, preserving a consistent lock order. Use predicate-based timed waits in the monitor as well. Add controlled interleaving tests; this review did not force the narrow notification race at runtime.

### R08 — Python singleton cleanup depends on the context still being alive

**Location:** [yasmin_node.py](../yasmin_ros/yasmin_ros/yasmin_node.py), lines 67–77 and 108–124.

`destroy_instance()` calls the executor/thread/node cleanup method only while `rclpy.ok()` is true. After application shutdown or SIGINT invalidates the context, it clears the singleton reference without performing that explicit cleanup. `get_instance()` can also reinitialize the default context while still returning an existing node associated with the old context. Static client caches are not cleared here.

**Remediation:** make cleanup idempotent and independent of context validity, clear owner-specific caches, and recreate invalid-context nodes. Test shutdown-before-destroy and context reinitialization, including the EventsExecutor and fallback executor variants.

### R09 — Python TF buffer does not inherit the node's ROS clock

**Location:** [tf_buffer_state.py](../yasmin_ros/yasmin_ros/tf_buffer_state.py), lines 86–87; compare [tf_buffer_state.cpp](../yasmin_ros/src/yasmin_ros/tf_buffer_state.cpp), lines 82–83.

Python constructs `Buffer(cache_time=...)` without its `node` argument, then gives the node only to the listener. The installed tf2 buffer uses `node.get_clock()` when a node is supplied and a standalone system clock otherwise. Consequently, the buffer's clock-jump clearing behavior is disconnected from the listener node's simulated ROS clock. C++ explicitly supplies the node clock. Bag replay/rewind with `use_sim_time` needs verification; this review did not execute that scenario.

**Remediation:** associate the Python buffer with the intended node/ROS clock and test forward playback, backward clock jumps, and context/clock changes. The dependency behavior was checked in `/opt/ros/lyrical/lib/python3.14/site-packages/tf2_ros/buffer.py`, lines 85–97.

### P01 — Invalid projection coefficients cause a process crash

**Location:** [project_inliers_state.cpp](../yasmin_pcl/src/filters/project_inliers_state.cpp), lines 82–104.

The state checks only that the coefficients key exists and contains a non-null pointer. It does not validate coefficient count, values, or model compatibility. With a valid one-point XYZ cloud, the default plane model, and a non-null `ModelCoefficients` with an empty `values` vector, PCL logged an invalid model and the process exited with SIGSEGV (139). AddressSanitizer identified a null read in `pcl::ProjectInliers<pcl::PCLPointCloud2>::applyFilter()`, called from this state's line 103, on PCL 1.15.1.

**Remediation:** validate supported model IDs and their coefficient contracts before invoking PCL, including finite values and nondegenerate models. Reject invalid configurations with `aborted`. The crash is in PCL, but the reusable state currently exposes it to an ordinary bad blackboard input; a C++ exception handler cannot catch SIGSEGV.

### P02 — Filtering failure is reported as success

**Location:** [filter_state_utils.hpp](../yasmin_pcl/include/yasmin_pcl/common/filter_state_utils.hpp), lines 147–175; [voxel_grid_state.cpp](../yasmin_pcl/src/filters/voxel_grid_state.cpp), lines 90–143; [crop_box_state.cpp](../yasmin_pcl/src/filters/crop_box_state.cpp), lines 168–209.

The wrapper assumes that a returning `filter.filter()` means success. PCL can log an error and return without throwing. A `PassThroughState` configured with field `does_not_exist` returned `succeeded` with output width zero. This makes a configuration error indistinguishable from a valid empty filtering result.

Parameter validation is also incomplete: voxel leaf sizes are not checked for positivity/finiteness; radius and statistical parameters are forwarded without a range contract. There are limited checks elsewhere, such as nonnegative sample count and minimum points per voxel, so validation is not uniformly absent.

**Remediation:** validate the input schema, field type, and filter parameters before dispatch. Establish postconditions appropriate to each filter; do not reject all empty outputs, since an empty result may be correct. Add invalid-field, invalid-range, NaN, and degenerate-model cases.

### P03 — Malformed pointcloud layouts pass through conversion unchecked

**Location:** [ros_to_pcl_point_cloud2_state.cpp](../yasmin_pcl/src/io/ros_to_pcl_point_cloud2_state.cpp), lines 48–61; [pcl_to_ros_point_cloud2_state.cpp](../yasmin_pcl/src/io/pcl_to_ros_point_cloud2_state.cpp), lines 48–61; [filter_state_utils.hpp](../yasmin_pcl/include/yasmin_pcl/common/filter_state_utils.hpp), lines 89–99 and 153–165.

The conversion states validate only pointer presence. A ROS message with `width=1`, `height=1`, `point_step=12`, `row_step=12`, and an empty data array returned `succeeded`. Dimensions are later used to validate point indices, without validating that the cloud's byte storage can contain those points.

**Impact:** invalid clouds can reach native PCL operations; their precise failure depends on the filter and PCL version. This review reproduced acceptance, not every possible downstream out-of-bounds access.

**Remediation:** check dimension arithmetic/overflow, payload size, row stride, field datatype/count/offset bounds, required fields, and supported endian/layout combinations before native processing. Account for valid row padding. The official [PointCloud2 definition](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/PointCloud2.msg) defines the binary layout and requires data size to match `row_step * height`.

### P04 — Python ROS clouds are not native C++ cloud pointers

**Location:** [cloud_types.hpp](../yasmin_pcl/include/yasmin_pcl/common/cloud_types.hpp), lines 31–38; [blackboard_pywrapper.hpp](../yasmin/include/yasmin/blackboard_pywrapper.hpp), lines 314–445 and 456–497; [supported_interface_serialization.cpp](../yasmin_ros/src/yasmin_ros/supported_interface_serialization.cpp), lines 30–58.

The ROS-to-PCL state requires an exact `std::shared_ptr<sensor_msgs::msg::PointCloud2>` entry. A Python `PointCloud2` placed in the blackboard is stored as `py::object`; neither a Python message nor a by-value C++ message satisfies that pointer contract. Likewise, native PCL pointers are not supported by the Python blackboard getter's standard conversion table.

The provided C++ ROS serialization states have a fixed interface allowlist that does not include `sensor_msgs/msg/PointCloud2`. Their native message storage is by value, so extending that allowlist alone would still require adapting to the pointer expected by the PCL conversion state.

**Remediation:** provide a checked cloud bridge or keep the cloud pipeline native C++. Do not connect a Python monitor directly to a native PCL conversion state merely by giving both keys the same name: L01 makes the mismatch potentially unsafe rather than reliably returning `aborted`.

## ROS 2 standards and integration conventions

### S01 — Set sensor subscription QoS explicitly

**Location:** [monitor_state.hpp](../yasmin_ros/include/yasmin_ros/monitor_state.hpp), lines 72–76 and 114–121; [monitor_state.py](../yasmin_ros/yasmin_ros/monitor_state.py), lines 37–49 and 96–102.

Both monitors default to depth 10 with the ordinary reliable profile. A reliable subscriber is incompatible with a best-effort publisher. This is appropriate for some generic topics, but creates an interoperability gap when the monitor is used as a raw cloud consumer. The states already accept a QoS override.

**Recommendation:** use `rclcpp::SensorDataQoS()` / `qos_profile_sensor_data` for the first raw-sensor consumption stage and document output QoS separately. [REP-2003](https://reps.openrobotics.org/rep-2003/) recommends SensorDataQoS for sensor consumers; it remains a draft and does not prescribe every later processing stage. This is a conditional sensor-use deviation, not a claim that all generic subscribers must default to best effort.

### S02 — Prefer application-owned node, context, executor, and clock

**Location:** [yasmin_node.cpp](../yasmin_ros/src/yasmin_ros/yasmin_node.cpp), lines 57–86; [yasmin_node.py](../yasmin_ros/yasmin_ros/yasmin_node.py), lines 57–64 and 92–106; [tf_buffer_state.cpp](../yasmin_ros/src/yasmin_ros/tf_buffer_state.cpp), lines 67–97.

The convenience singleton implicitly initializes the default ROS context and creates a randomly named node with a private executor. The C++ auto-init path uses `rclcpp::init(0, nullptr)`, so callers relying on that path do not pass command-line ROS arguments through initialization. It offers no `NodeOptions`/custom-context constructor. TF creation also chooses the singleton by default. Many other ROS states do support node injection, which is useful and should be the preferred integration path.

**Recommendation:** keep convenience defaults, but allow explicit node/context/clock injection throughout, make runtime ownership clear, and verify namespaces, parameter YAML, remappings, `use_sim_time`, and component-style embedding. Lack of lifecycle-node or component support is an optional architecture limitation, not by itself a ROS standards violation.

### S03 — Calling blocking states from ROS callbacks needs an execution contract

**Location:** [service_state.py](../yasmin_ros/yasmin_ros/service_state.py), lines 160–170; [action_state.hpp](../yasmin_ros/include/yasmin_ros/action_state.hpp), lines 450–468; [monitor_state.hpp](../yasmin_ros/include/yasmin_ros/monitor_state.hpp), lines 162–211.

The APIs make an asynchronous ROS request and then block a thread on an event or condition variable. If an application invokes them from a callback sharing a mutually exclusive group with the response/subscription callback, progress can stop. A multithreaded executor alone does not fix that grouping. This is a conditional caller integration hazard; ordinary state execution on a separate application worker is a valid design.

**Recommendation:** document that states run outside dependent ROS callback groups, or provide an asynchronous execution adapter. ROS's official [callback-group guidance](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/nodes/Working-with-nodes/Using-callback-groups.rst) describes the grouping requirements and deadlock conditions.

### S04 — Viewer traffic uses a global namespace

**Location:** [yasmin_viewer_pub.cpp](../yasmin_viewer/src/yasmin_viewer/yasmin_viewer_pub.cpp), lines 54–62; [yasmin_viewer_pub.py](../yasmin_viewer/yasmin_viewer/yasmin_viewer_pub.py), lines 62–71; [yasmin_viewer_node.cpp](../yasmin_viewer/src/yasmin_viewer/yasmin_viewer_node.cpp), lines 262–264.

The absolute `/fsm_viewer` topic is shared across node namespaces. Multiple unnamed machines also default to `Unnamed_FSM`, while the viewer cache is keyed by machine name, allowing one to replace another's display entry. Explicit remapping/naming can avoid the collision.

**Recommendation:** expose a configurable relative topic and stable unique machine identity, while allowing an intentional shared viewer deployment. Absolute topic names are legal ROS names; this is a namespace-composability deviation.

## Pointcloud input/output inventory

### Available paths and their contracts

`yasmin_pcl` contains **14 C++ plugin states**: two ROS/PCL converters, four file I/O states, and eight filters. It does not provide its own cloud subscriber/publisher executable, a Python PCL API, or a ready-made topic-to-topic launch pipeline. Topic I/O must be supplied through typed `yasmin_ros` states or custom states. There is a projection filter, but no general segmentation/registration state in this package at this commit.

| Stage | Input | Output | Important behavior |
| --- | --- | --- | --- |
| ROS subscription | `MonitorState<sensor_msgs::msg::PointCloud2>` or Python equivalent | Handler-defined blackboard entry | Default DDS depth 10 and separate application queue 10; use explicit sensor QoS. Store a native ROS shared pointer for the C++ conversion path. |
| ROS → PCL | `RosToPclPointCloud2State`: `input_cloud` as native ROS shared pointer | `output_cloud` as `pcl::PCLPointCloud2::Ptr` | Calls `pcl_conversions::toPCL`; copies the binary message representation; schema validation was added in the first fix batch; no TF validation. |
| File input | `LoadPcdState`, `LoadPlyState`; configured `file_path` | PCL `output_cloud`, `sensor_origin`, `sensor_orientation`; PCD also `pcd_version` | Pose metadata is separate from the cloud. File formats do not restore a ROS acquisition header contract. |
| Selection filters | PassThrough, CropBox, ExtractIndices | PCL `output_cloud`; optional/derived index outputs | Optional `input_indices` refers to the input cloud. Organized output can contain replacement NaNs. |
| Downsampling | VoxelGrid, RandomSample | PCL `output_cloud`; RandomSample also `output_indices` | VoxelGrid changes topology and field values; RandomSample performs a second sampling pass to derive indices. |
| Outlier filters | StatisticalOutlierRemoval, RadiusOutlierRemoval | PCL `output_cloud`; optional `removed_indices` | The first fix batch adds schema and basic parameter checks; retained NaNs and `is_dense` still matter to downstream consumers. |
| Projection | ProjectInliers | PCL `output_cloud` | Requires compatible native coefficients; invalid coefficient vectors now abort before PCL dispatch (P01). |
| PCL → ROS | `PclToRosPointCloud2State`: native PCL `input_cloud` | Native ROS shared pointer in `output_cloud` | Calls `pcl_conversions::fromPCL`; does not assign a new frame or acquisition time. |
| ROS publication | Typed `PublisherState<PointCloud2>` or custom state | Topic message | Caller constructs/returns the message and selects QoS. A successful publish call is not delivery acknowledgement. |
| File output | `SavePcdState`, `SavePlyState`; PCL `input_cloud` | File at configured path | PCD modes: ASCII, binary, binary-compressed. PLY has binary and camera options. Sensor pose comes from state parameters. |

Sources: [plugins.xml](../yasmin_pcl/plugins.xml), [cloud_types.hpp](../yasmin_pcl/include/yasmin_pcl/common/cloud_types.hpp), and the individual [I/O](../yasmin_pcl/src/io) / [filter](../yasmin_pcl/src/filters) implementations.

### Data fidelity, freshness, and metadata

- **Extra point fields:** the two conversion states pass the complete binary representation through `pcl_conversions`; they do not intentionally reduce clouds to XYZ. The first fix batch adds a byte-for-byte XYZ + intensity roundtrip with organized padded rows and a frame/header check. Ring, per-point time, RGB, normals, and the exact driver-produced field combinations still need pipeline coverage.
- **Timestamp precision:** the installed conversion library rounds ROS nanoseconds to PCL microseconds, then multiplies back to nanoseconds. A roundtrip changed `1.123456789` to `1.123457000`. This is a dependency representation limit, not a YASMIN-specific arithmetic error. Preserve the original ROS header separately when exact acquisition time matters. See the primary [pcl_conversions implementation](https://raw.githubusercontent.com/ros-perception/perception_pcl/ros2/pcl_conversions/include/pcl_conversions/pcl_conversions.h), and the tested installed header under `/opt/ros/lyrical/include/pcl_conversions/pcl_conversions/pcl_conversions.h`.
- **Frames and TF:** converters preserve the supplied header fields subject to timestamp conversion; they do not check that coordinates agree with `frame_id`. `TfBufferState` creates a buffer/listener, not a cloud transform. CropBox's configured transform is used for selection and should not be treated as a general transform-and-relabel operation. File-loaded clouds need an explicitly assigned frame and a deliberate timestamp policy before ROS publication.
- **File pose metadata:** loaders expose pose in blackboard outputs, but savers read origin/quaternion from configured scalar parameters, defaulting to zero translation and identity rotation. A load→save graph must explicitly transfer that metadata; it is not automatically consumed from `sensor_origin` / `sensor_orientation`. Quaternion arrays use `[x, y, z, w]` in [cloud_types.hpp](../yasmin_pcl/include/yasmin_pcl/common/cloud_types.hpp), lines 67–71. PCL < 1.14 has an ASCII-only PLY camera metadata fallback; binary PLY viewpoint metadata is skipped with a warning in [load_ply_state.cpp](../yasmin_pcl/src/io/load_ply_state.cpp), lines 32–162 and 199–204. That older-version path was not run here.
- **Freshness:** monitors subscribe when constructed, retain messages while inactive, and process the oldest retained message on execution. After an idle interval, a state may process a stale cloud. There is no acquisition-age check or latest-only mode. Set the queue to one for latest retained data, or implement an explicit freshness policy. Validate queue sizes: zero currently discards every message rather than providing a meaningful input mode.
- **Storage and copying:** callbacks retain shared message objects, while both conversion states allocate an output and copy the cloud payload. Defaults allow 10 DDS samples plus a 10-message application queue per monitor; actual DDS memory behavior varies. For large clouds this is material memory and bandwidth overhead, not a zero-copy pipeline.
- **Topology and indices:** organized filtering can replace points with NaNs; voxelization aggregates points and fields. Do not interpret averaged ring/time/label fields as original discrete measurements, and do not reuse indices after topology-changing operations without a mapping. RandomSample's cloud and index outputs should receive a concurrent determinism stress test for the deployed PCL implementation; two-pass sampling assumes repeatable RNG state.
- **Output validity:** failed conversions/loads/filters return without removing a previous `output_cloud`. Optional `removed_indices` is written only when requested. Consumers must follow outcomes and must not infer freshness from key presence alone. Prefer per-stage keys, execution metadata, or explicit invalidation to make this contract visible.
- **Cancellation during PCL work:** the PCL states mostly expose `succeeded`/`aborted`, with no interruptible native filter execution. SavePCD checks cancellation before writing, but cannot interrupt a write already inside PCL. Large synchronous operations can delay parent shutdown even after the lifecycle bugs are fixed.

### Suggested native ROS pipeline

```text
PointCloud2 subscription (explicit sensor QoS, freshness policy)
  → blackboard: ros_input as native ROS shared pointer
  → validate layout, fields, acquisition header
  → ROS-to-PCL conversion: pcl_input
  → filter(s): separate pcl_filtered keys, explicit per-state remapping
  → PCL-to-ROS conversion: ros_output
  → restore exact header or deliberately assign transformed frame/time
  → PointCloud2 publisher (explicit output QoS)
```

Each stock PCL state uses the local names `input_cloud` and `output_cloud`; connect stages with state-machine blackboard remappings. There is no automatic connection merely because two stages declare those names. Keep unfiltered/filtered outputs separate when branches run concurrently. Checked reads now reject mismatched stored types (L01); the Python/native adapter gap remains open (P04).

## Baseline audit verification

The inspected packaging uses ament, package manifests, and pluginlib exports in the expected ROS 2 structure. The local build/install and installed-target consumer checks succeeded; this review found no demonstrated packaging failure on Lyrical. That does not establish compatibility with every older distribution advertised by the CI workflows.

Environment: ROS 2 Lyrical, Python 3.14.4, GCC 15.2, PCL 1.15.1, Cyclone DDS. Build/test output was directed to `/tmp/yasmin-audit`; no workspace build/install tree was changed.

| Check | Result |
| --- | --- |
| Build `yasmin`, `yasmin_ros`, `yasmin_pcl`, Debug, testing enabled | All three built successfully |
| Core suite | 14 CTest entries; 231 individual Python/C++ cases passed |
| PCL suite | 14 CTest entries; 37 individual C++ cases passed |
| ROS suite | 15 CTest entries; 72 individual Python/C++ cases passed |
| Installed-package consumer | Separate CMake project found and linked `yasmin_pcl::yasmin_pcl_states` successfully |
| Additional native harness | Reproduced L01, L02, L03, L05, P01, P02, P03, and timestamp quantization |
| Mocked Python checks | Reproduced R01 and R02; injected a stale callback for R03 |
| Crash diagnosis | Normal harness exited 139; ASan traced P01 to PCL `ProjectInliers::applyFilter`, called by state line 103 |

The first ROS run failed because sandboxed Cyclone DDS could not enumerate UDP interfaces; even a minimal node creation failed. Re-running with local DDS access, `ROS_DOMAIN_ID=177`, and localhost-only discovery passed. Those first-run errors are environmental and are not counted as package defects. `colcon test-result` reports 383 successful records because it includes 43 CTest wrapper records in addition to the 340 individual cases.

Build command used:

```bash
source /opt/ros/lyrical/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon --log-base /tmp/yasmin-audit/log build \
  --base-paths /home/jcfurey/robotics/resple_test_ws/src/yasmin \
  --packages-up-to yasmin_ros yasmin_pcl \
  --build-base /tmp/yasmin-audit/build \
  --install-base /tmp/yasmin-audit/install \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
```

Tests were run with the matching build/install bases and `colcon test`; the ROS rerun additionally set `ROS_LOG_DIR=/tmp/yasmin-audit/ros-logs`, `ROS_DOMAIN_ID=177`, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Temporary reproduction sources and results are in `/tmp/yasmin-audit/repro` and `/tmp/yasmin-audit/python_repro.py`; `/tmp` artifacts are not permanent repository assets.

Selected native harness observations:

```text
wrong_type_get<int>=1065353216 (no exception)
exception_state_still_running=1
active_region_is_running=0
region_child_canceled=0
failed_region_propagated_before_external_cancel=0
malformed_ros_conversion=succeeded
roundtrip_stamp_nanosec=123457000
invalid_filter_field=succeeded
invalid_filter_output_width=0
```

Minimal P01 trigger, given a valid XYZ PCL cloud already stored as native `input_cloud`:

```cpp
yasmin_pcl::filters::ProjectInliersState state;  // default: plane
state.configure();
auto coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
blackboard->set<pcl::ModelCoefficients::Ptr>("input_model_coefficients", coefficients);
state(blackboard);  // PCL 1.15.1: SIGSEGV rather than "aborted"
```

## Remediation order and missing regression coverage

1. **Prevent process failure:** checked blackboard types; PointCloud2 schema validation; model/parameter validation before PCL dispatch. Add the reproduced crash and invalid-field cases.
2. **Make cancellation and failures terminate:** fix orthogonal/proxy lifecycle, propagate sibling failures, repair notification synchronization, wake Python action waits, and cancel late/expired goals. Test active cancellation and reuse, not just canceling an idle container.
3. **Isolate ROS owners and executions:** node/context-specific caches, execution identity for Python callbacks, pending-request removal, and cleanup after context shutdown. Add two namespaces with the same node name and timeout→immediate rerun scenarios.
4. **Exercise a real cloud pipeline:** sensor QoS and publisher compatibility, byte-for-byte mixed-field roundtrip, organized clouds with row padding, endian handling, NaNs, empty clouds, exact header restoration, file viewpoint handling, and publication into a subscriber/RViz-compatible interface.
5. **Verify deployment behavior:** bag playback with simulated time, shutdown during unavailable endpoints/PCL operations, bounded memory on large clouds, and the older PCL/ROS distributions advertised by CI. Factory/viewer integration tests and a full editor/CLI review remain separate work.

At the reviewed commit, the orthogonal cancellation test only checked the flag on an idle parent ([test_orthogonal_state.cpp](../yasmin/test/test_orthogonal_state.cpp), lines 218–227), and PCL tests mostly use small XYZ fixtures. The first fix batch extends these scenarios with active cancellation, reuse, malformed coefficients/layouts, and padded/mixed-field cloud regressions; further pipeline coverage remains necessary.
