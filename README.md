# Robotac ROS Noetic Workspace

This workspace is staged on macOS and built on Ubuntu 20.04 with ROS Noetic.
It contains a Livox MID360s driver, FAST-LIO, a V4L2 RGB webcam, MAVROS for
PX4, AprilTag detection, and the local `robotac_servo` switch package. No
Sunray private control packages or messages are included.

## Source layout

- `src/livox_ros_driver2`: Sunray-validated Livox ROS1 driver.
- `src/Livox-SDK2`: Livox SDK required by the driver.
- `src/fast_lio`: Sunray-validated FAST-LIO source.
- `src/web_cam`: V4L2 RGB camera driver.
- `src/mavros`: official MAVROS 1.21.1 source.
- `src/apriltag` and `src/apriltag_ros`: official AprilTag 3 source and ROS wrapper.
- `src/robotac_bringup`: project launch files and runtime checks.
- `src/robotac_flight`: FAST-LIO vision adapter and safety-gated local ENU
  waypoint flight state machine.
- `src/robotac_servo`: Boolean switch for the channel-1 USB PWM servo controller
  used by `yundrone_blink`.

Exact source revisions are recorded in `versions.lock`. Official git sources
are also described in `.repos` for `vcs import` users.

## Ubuntu build

Copy this workspace to Ubuntu, then run:

```bash
cd ~/robotac_ws
./scripts/bootstrap_ubuntu20.sh
source devel/setup.bash
./scripts/verify_workspace.sh
```

The bootstrap script installs the Livox SDK and the native AprilTag library,
then installs ROS dependencies, MAVROS GeographicLib data, and performs a ROS1
catkin build. Do not install the apt MAVROS package alongside this source build.
The bootstrap script recreates `src/apriltag/CATKIN_IGNORE` before building;
this is required because AprilTag is installed as a native CMake library and
then linked by `apriltag_ros`.

## Configuration

Before connecting hardware, update these files:

- `config/lidar/mid360s.json`: host and LiDAR IP addresses.
- `config/fastlio/mid360s.yaml`: LiDAR-to-IMU extrinsics and timing.
- `config/camera/rgb.yaml`: camera calibration.
- `config/apriltag/tags.yaml`: tag36h11 IDs 0 and 1, each with a 0.15 m
  pose-estimation side length and 0.25 m printed total size metadata.
- `config/mavros/px4.yaml`: PX4 frame and plugin settings.
- `config/fastlio/vision_bridge.yaml`: FAST-LIO world and airframe alignment,
  covariance, timestamp, rate, and jump limits.
- `config/flight/local_waypoints.yaml`: takeoff height, local relative ENU
  waypoints, landing profile, and flight limits.
- `config/flight/posearray_waypoints_example.yaml`: position-only runtime
  waypoint example for `/robotac/flight/waypoints`.
- `config/deployment.yaml`: deployment gate checklist.

Install `config/udev/99-robotac-rgb-camera.rules.template` as
`/etc/udev/rules.d/99-robotac-rgb-camera.rules`, reload udev rules, and ensure
the runtime user belongs to the `video` group. The template matches the tested
Realtek `0bda:5858` camera and only aliases its capture node (V4L2 index 0) as
`/dev/robotac_rgb_camera`. If a different camera is used, replace the IDs and
add its serial attribute before installing the rule. The launch can be pointed
at a temporary device path with `video_device:=/dev/video0` while diagnosing
udev.

Install a corresponding PX4 rule from
`config/udev/99-robotac-px4.rules.template`, and ensure the runtime user is in
the `dialout` group. MAVROS defaults to `serial:///dev/px4_fcu:921600`.
The default `config/mavros/px4_pluginlists.yaml` is intentionally local-only:
it loads MAVROS command, IMU, local-position, parameter, system-status/time,
setpoint_raw, and vision-pose plugins, while `global_position`, `gps_status`,
and `waypoint` remain blacklisted. The local waypoint controller therefore does
not require GPS fixes, latitude/longitude, global mission upload, or
GeographicLib datasets. Only re-enable the GeographicLib launch check after you
deliberately add global-position plugins back to the MAVROS plugin list.
Do not launch `camera_extrinsics.launch` with its zero defaults on an aircraft:
pass the measured `base_link -> camera_rgb_optical_frame` transform first.

For the servo controller, install `config/udev/99-robotac-servo.rules.template`
as `/etc/udev/rules.d/99-robotac-servo.rules`, reload udev, and ensure the
runtime user can access the `dialout` group. The known HL-340 controller is
`1a86:7523` and is exposed as `/dev/robotac_servo`.

The servo package exposes only `/robotac/servo/open` (`std_msgs/Bool`):
`false` commands 0 degrees and `true` commands the configured opening angle,
45 degrees by default. The PWM protocol is 115200 baud, channel 1, 50 Hz;
integer duty quantization means nearby angles can share one duty value. The
default launch closes on shutdown; explicitly set `close_on_shutdown:=false`
only for a bench test that must retain the last PWM output.

`full_system.launch` supports passive sensor/SLAM/AprilTag observation before
the flight calibration gates are complete. `enable_mavros` still defaults to
`false`; setting it to `true` explicitly opens a telemetry link only and does
not send flight commands. Enabling external-vision output or flight control
requires the sensor, FCU, PX4, and deployment gates in
`config/deployment.yaml`; it is a configuration error to request either without
explicitly enabling MAVROS. A checked-in `192.168.1.5` LiDAR host address is
allowed only for passive diagnostics and is rejected for vision or flight output.

The two `robotac_flight` launches also load `deployment.yaml`. Passive preview
can run independently, but `enable_mavros_output:=true` or
`enable_control:=true` is rejected by the node itself until the relevant gates
are `true`. Flight control additionally requires recorded PX4 Offboard-loss
failsafe behavior and a ground-test result; payload control also requires a
stable `/dev/robotac_servo` device. Do not set those values until the FAST-LIO
axes and `body -> base_link` extrinsics have been measured, PX4 external-vision
fusion has been confirmed, and ground checks have passed. The checked-in
transforms are identity bench defaults, not flight calibration.

`full_system.launch` exposes `flight_enable_control`, `flight_auto_mode`,
`flight_auto_arm`, `flight_auto_land`, and `flight_enable_payload` separately.
All default to `false`; setting `flight_enable_payload:=true` without
`flight_enable_control:=true` causes the flight node to reject startup rather
than send a payload command. It exposes `enable_servo:=false`, `servo_port`,
and `servo_open_angle` separately as well; the servo process never starts
unless `enable_servo:=true` is explicit.

FAST-LIO keeps the upstream local-coordinate contract: `/Odometry`,
`/cloud_registered`, `/PointCloud`, and `/path` use `camera_init` as the world
frame and `body` as the estimator IMU frame. These are not aliases for
`map`/`base_link`; do not add a static transform until the LiDAR/IMU and airframe
extrinsics have been measured and recorded in `config/fastlio/mid360s.yaml`.

FAST-LIO PCD writing is disabled by default. Set `pcd_save:=true` on the
FAST-LIO launch only after verifying sufficient free disk and memory.

## Launches

```bash
roslaunch robotac_bringup camera_rgb.launch
roslaunch robotac_bringup lidar_mid360s.launch
roslaunch robotac_bringup fastlio_mid360s.launch
roslaunch robotac_bringup mapping_demo.launch rviz:=true
roslaunch robotac_bringup mavros_px4.launch fcu_url:=serial:///dev/px4_fcu:921600
roslaunch robotac_bringup apriltag_rgb.launch
roslaunch robotac_servo servo.launch
roslaunch robotac_bringup full_system.launch enable_mavros:=false
roslaunch robotac_flight fastlio_vision_bridge.launch enable_mavros_output:=false
roslaunch robotac_flight local_flight_preflight.launch require_vision_output:=false
roslaunch robotac_flight local_waypoint_flight.launch enable_control:=false
roslaunch robotac_flight active_flight_observer.launch
```

For a read-only FCU telemetry session, use
`full_system.launch enable_mavros:=true` with the intended `fcu_url`; it does
not enable vision output, mode changes, arming, or setpoint transmission. The
two `robotac_flight` launches are passive by default. The vision bridge
publishes only `/robotac/fastlio_vision/pose_preview`; the waypoint node
publishes only `/robotac/flight/setpoint_preview`. Nothing is sent to MAVROS
until the corresponding output gate is explicitly enabled, and the flight
state machine still requires a separate `/robotac/flight/start` service call.

For the staged path from offline checks to read-only aircraft observation, run:

```bash
./scripts/flight_test_ladder.sh
```

This helper is offline by default: it runs `verify_workspace.sh`, previews the
configured local route, prints the deployment-gate status, and then prints the
next read-only aircraft commands. It never starts ROS nodes, opens serial
devices, sends setpoints, changes modes, arms, or calls MAVROS services. Active
flight commands are hidden unless `--show-active` is supplied, and even then the
script refuses to print them until both the active-flight deployment gates in
`config/deployment.yaml`, the `active_local_flight` readiness report, and a
passed read-only evidence bundle supplied via `--evidence-dir` all pass. That
evidence bundle must include both subscriber-only topic evidence and the
`ev_acceptance_observer.json` result proving PX4/MAVROS local position moved in
the same direction and scale as the FAST-LIO vision-pose input while the vehicle
was connected, disarmed, and on the ground. The final payload mission block is
hidden separately until `payload_local_flight` readiness passes. Use
`--skip-verify` only when you are iterating on the printed route/command ladder
and have just run the full workspace verification separately.

To inspect the same evidence matrix directly, run the offline readiness report:

```bash
rosrun robotac_flight local_flight_readiness.py --config-root ~/robotac_ws/config
```

It reports separate readiness for `vision_output`, `active_local_flight`, and
`payload_local_flight`. By default it only reports; add
`--require-phase vision_output`, `--require-phase active_local_flight`, or
`--require-phase payload_local_flight` when you want the command to fail closed
until that stage is truly ready.

To check the source/configuration contract for the whole local-flight goal
without ROS or aircraft access, run:

```bash
./scripts/check_flight_contract.py
```

It verifies that MAVROS remains local-only, the configured mission is local and
relative, the controller cannot publish raw setpoints before explicit control
enable plus `/robotac/flight/start`, FAST-LIO vision output is gated before
`/mavros/vision_pose/pose_cov`, and the read-only/active evidence tools still
cover target reach, landing, and payload evidence.

## FAST-LIO vision input

`fastlio_vision_bridge.py` converts `/Odometry` from
`camera_init -> body` into a `PoseWithCovarianceStamped` with local ENU and
implicit `base_link` semantics. `PoseWithCovarianceStamped` can name only its
parent frame, so `input_child_to_output_child_*` is the sole place to describe
the measured `body -> base_link` transform. MAVROS then performs the ENU-to-NED
and FLU-to-FRD conversion and sends `VISION_POSITION_ESTIMATE`; do not perform
that conversion again in project code. The bridge preserves the LiDAR timestamp, rejects stale,
backward, non-finite, low-rate, or jumping poses, repairs invalid covariance
with conservative configured values, and reports unhealthy on timeout. The
flight controller requires all three live signals before active control: an
`ok ...` bridge status, a fresh healthy signal, and a fresh valid message on
`/mavros/vision_pose/pose_cov` with frame `odom`. A latched
`output_enabled` value alone is never treated as proof that MAVROS is still
receiving vision data. Before `/robotac/flight/start` accepts an active
mission, it also checks that `/mavros` is subscribed to that exact vision-pose
topic, which confirms the MAVROS vision-pose plugin is loaded. Active control
also requires `/mavros` to subscribe to `/mavros/setpoint_raw/local`; this
confirms the MAVROS setpoint_raw plugin is present before the controller tries
to stream OFFBOARD setpoints. During the active mission, those MAVROS consumer
checks are repeated at the configured `consumer_check_interval`; losing either
required consumer enters `ABORT` and closes the raw setpoint transmission gate.

Interfaces:

```text
/Odometry                                  nav_msgs/Odometry (input)
/robotac/fastlio_vision/pose_preview       PoseWithCovarianceStamped
/robotac/fastlio_vision/healthy            std_msgs/Bool
/robotac/fastlio_vision/status             std_msgs/String
/mavros/vision_pose/pose_cov               PoseWithCovarianceStamped (opt-in)
```

First run preview only and inspect position/orientation while moving the
aircraft along each positive body/local axis. After the transform and PX4 EKF
configuration are confirmed, set `frame_alignment_approved: true` in
`config/fastlio/vision_bridge.yaml`:

```bash
rosrun robotac_flight check_px4_vision_config.py
roslaunch robotac_flight fastlio_vision_bridge.launch enable_mavros_output:=true
rostopic hz /mavros/vision_pose/pose_cov
```

The PX4 checker only reads parameters. Newer PX4 uses `EKF2_EV_CTRL`; older
firmware may use `EKF2_AID_MASK`. Parameter changes must be made deliberately
for the installed firmware and are never written by this workspace.

## Local relative flight

Waypoints are metres relative to the MAVROS local position captured by the
explicit start request. The default `robotac_start_body` frame uses the captured
aircraft FLU axes: `x` forward, `y` left, and `z` up. The controller converts
those fixed start-heading offsets into local ENU before publishing the MAVROS
setpoint. Use `yaw_deg` for degrees or `yaw` for radians; either one is
relative to the captured takeoff heading, and a waypoint must not define both.
`robotac_local_enu` remains available for clients that deliberately want fixed
ENU axes. No GPS, latitude, longitude, global mission item, or `CommandTOL`
takeoff/land service is used. The estimator gate requires relative horizontal
position and either absolute or AGL vertical validity; it does not require a
global position fix.

The state machine is:

```text
IDLE -> PAYLOAD_PREPARE -> PRESTREAM -> WAIT_OFFBOARD -> WAIT_ARMED -> TAKEOFF
     -> WAYPOINTS -> WAIT_LAND/LANDING -> COMPLETE
```

Any connection, local-pose, FAST-LIO, estimator, mode, bounds, or timeout
failure enters `ABORT`. During `TAKEOFF`, `WAYPOINTS`, and `WAIT_LAND`, the FCU
must continuously report `OFFBOARD`; `LANDING` permits either `OFFBOARD` or the
configured `AUTO.LAND` mode. MAVROS `State` and `ExtendedState` have freshness
limits, and `COMPLETE` is reached only after fresh confirmation of
`ON_GROUND && !armed`.

Automatic health, estimator, payload, MAVROS, and OFFBOARD faults use
`critical_fault_action: release` by default: the controller closes its raw
MAVROS transmission gate immediately and publishes only its preview topic.
PX4's already verified Offboard-loss behavior is then authoritative. An
operator-triggered `/robotac/flight/abort` defaults to `release` and can only
use a separately configured `operator_abort_action` (`hold`, `release`, or
best-effort `land`). The `land` option never arms the aircraft and does not
resume raw setpoints if the mode request fails or MAVROS is disconnected.

Setpoints use `/mavros/setpoint_raw/local` at 20 Hz with ROS ENU values; MAVROS
performs the NED conversion. Landing descends vertically at the configured rate
and changes to `AUTO.LAND` near the captured local ground height. A missing
mode change or landing confirmation times out into the same critical release
path. It never automatically disarms.

Runtime interfaces:

```text
/robotac/flight/waypoints          geometry_msgs/PoseArray (IDLE only)
/robotac/flight/start              std_srvs/Trigger
/robotac/flight/land               std_srvs/Trigger
/robotac/flight/abort              std_srvs/Trigger
/robotac/flight/reset              std_srvs/Trigger
/robotac/flight/status             std_msgs/String
/robotac/flight/setpoint_preview   mavros_msgs/PositionTarget
/mavros/setpoint_raw/local         mavros_msgs/PositionTarget (opt-in)
/robotac/servo/open                std_msgs/Bool (payload opt-in only)
/robotac/servo/status              std_msgs/String (serial-write feedback)
```

To replace the configured route at runtime, publish a `PoseArray` only while
the controller is `IDLE`. Its `header.frame_id` must exactly match
`robotac_start_body` (or the configured `waypoint_frame`); a missing or other
frame is rejected. Each pose position is metres in that frame and its yaw is
relative to the heading captured at `/robotac/flight/start`. The controller
locks the accepted route once a mission starts; call reset before replacing it.
`PoseArray` can carry only position and yaw, so it is for position-only routes.
The checked-in payload mission with per-waypoint hold and `payload_action` must
be loaded through `config/flight/local_waypoints.yaml` at launch.

To publish a position-only waypoint file without starting the mission or
touching MAVROS, first dry-run the parser, then publish while the controller is
idle:

```bash
rosrun robotac_flight audit_local_mission.py \
  --file ~/robotac_ws/config/flight/local_waypoints.yaml \
  --origin-x 0 --origin-y 0 --origin-z 0 --origin-yaw-deg 0 \
  --require-payload-open

rosrun robotac_flight preview_local_route.py \
  --file ~/robotac_ws/config/flight/local_waypoints.yaml \
  --origin-x 0 --origin-y 0 --origin-z 0 --origin-yaw-deg 0

rosrun robotac_flight publish_waypoints.py \
  --file ~/robotac_ws/config/flight/posearray_waypoints_example.yaml \
  --dry-run

rosrun robotac_flight publish_waypoints.py \
  --file ~/robotac_ws/config/flight/posearray_waypoints_example.yaml
```

The helper intentionally refuses YAML containing fields `PoseArray` cannot
carry, such as `hold` or `payload_action`, unless `--allow-metadata-drop` is
explicitly supplied. It publishes only `/robotac/flight/waypoints`; it never
calls `/robotac/flight/start`, never requests OFFBOARD, never arms, and never
sends MAVROS setpoints.

`audit_local_mission.py` is the first check for a new route file. It is fully
offline and fails if the mission contains GPS/global keys, lacks
`require_auto_land: true`, violates the route limits, or, when requested,
contains no payload-open action. `preview_local_route.py` is also fully offline
and prints both the controller's local ENU targets and the MAVLink local-NED
route after MAVROS conversion. Use them together to confirm
front/left/right/rear directions, yaw units, landing return point, and payload
event location before any armed test.

Safe dry-run launch:

```bash
roslaunch robotac_flight local_waypoint_flight.launch \
  enable_control:=false auto_mode:=false auto_arm:=false auto_land:=false
```

With `enable_control:=false`, the node registers only the preview publisher; it
does not register a publisher on `/mavros/setpoint_raw/local`. This keeps
read-only evidence bundles from showing a phantom control publisher.

The following regression test is isolated from the aircraft: it starts a
separate loopback ROS master, a MAVROS/PX4 contract simulator, and the
controller. It uses a non-zero local start position and a 90 degree heading to
verify the configured body-relative route, the ROS ENU to MAVLink local-NED
conversion, the `AUTO.LAND` handoff, and the simulated `closed -> open` payload
sequence. It uses `config/deployment_sim.yaml`, never reads the aircraft
deployment gates, opens no serial device, and never starts MAVROS:

```bash
src/robotac_flight/test/run_closed_loop_sim.sh
```

The runtime `PoseArray` API has its own loopback regression. It uses the public
`publish_waypoints.py` helper to load the position-only YAML route before
mission start, then verifies the resulting ENU and MAVLink NED routes after a
non-zero start position and 90 degree heading. It likewise opens no serial
device or MAVROS node:

```bash
src/robotac_flight/test/run_dynamic_waypoints_sim.sh
```

The start gate also has an isolated MAVROS setpoint-consumer regression. It
starts the same controller against a simulator that deliberately does not
subscribe to `/mavros/setpoint_raw/local`, then verifies that
`/robotac/flight/start` is rejected with
`mavros_setpoint_raw_consumer_unavailable`. This protects against entering an
active mission when the raw setpoint path is not actually being consumed; it
opens no serial device and never starts MAVROS:

```bash
src/robotac_flight/test/run_setpoint_consumer_gate_sim.sh
```

The corresponding FAST-LIO bridge regression starts only a loopback ROS master
and publishes simulated `camera_init -> body` odometry. It verifies that the
bridge preserves the FAST-LIO timestamp, emits the configured local pose, and
requires a new health window after an invalid input frame. It never starts
MAVROS or opens a hardware device:

```bash
src/robotac_flight/test/run_vision_bridge_sim.sh
```

The controller also has an isolated FAST-LIO-health-loss regression. It
deliberately stops the fake vision-health stream after OFFBOARD/arming, then
verifies that the controller enters `ABORT` and sends no raw MAVROS setpoints
whose source timestamp is later than the controller's abort-status timestamp.
This covers the controller's health-timeout behavior; it is separate from the
bridge regression above:

```bash
src/robotac_flight/test/run_flight_fault_sim.sh
```

The same regression can exercise loss of the actual MAVROS vision-pose stream
while the bridge health signal remains true:

```bash
ROBOTAC_FLIGHT_FAULT=vision_output_loss \
  src/robotac_flight/test/run_flight_fault_sim.sh
```

It can also exercise loss of MAVROS's raw-setpoint consumer after the mission
has already entered OFFBOARD/armed simulation. The controller must enter
`ABORT` with `mavros_setpoint_raw_consumer_lost` and stop raw setpoint output:

```bash
ROBOTAC_FLIGHT_FAULT=setpoint_consumer_loss \
  src/robotac_flight/test/run_flight_fault_sim.sh
```

The same consumer-loss regression exists for the MAVROS vision-pose input:

```bash
ROBOTAC_FLIGHT_FAULT=vision_consumer_loss \
  src/robotac_flight/test/run_flight_fault_sim.sh
```

Before any active test, run the read-only local-flight preflight against the
existing ROS graph. It subscribes to MAVROS state, grounded/disarmed status,
local `map -> base_link` odometry, estimator health, FAST-LIO
`camera_init -> body` odometry, bridge preview/health/status and their matching
timestamps, MAVROS time-sync status, and optionally the MAVROS vision-pose
input. The launch starts only the preflight node: it creates no publishers,
sends no setpoints, opens no serial device, and does not call flight-control
services:

```bash
roslaunch robotac_flight local_flight_preflight.launch \
  observe_seconds:=30 require_vision_output:=false
```

After the measured transforms, PX4 external-vision parameters, and deployment
gates have been approved, repeat it with
`require_vision_output:=true require_timesync:=true check_px4_vision_params:=true require_setpoint_consumer:=true`.
This also checks that `/mavros` is subscribed to the exact vision-pose and
setpoint_raw topics. The PX4 parameter query is read-only and, by default, also
requires `EKF2_EV_POS_X/Y/Z` to be zero within 0.01 m; Robotac's bridge already
outputs the airframe `base_link` pose, so non-zero PX4 EV offsets would apply
the external-vision lever arm twice. After measuring the FAST-LIO to FCU timing
chain, add `require_ev_delay:=true expected_ev_delay_ms:=...` with a suitable
`ev_delay_tolerance_ms:=...` so `EKF2_EV_DELAY` is checked instead of merely
printed. The following regression exercises that preflight against a loopback
ROS graph; it opens no serial device and starts no MAVROS:

```bash
src/robotac_flight/test/run_flight_preflight_sim.sh
```

After the read-only preflight passes with MAVROS vision output enabled, run the
read-only EV acceptance observer on the ground before any armed test. Keep the
vehicle disarmed and on the ground, then slowly move it by at least the
configured `min_motion_m` so the script can compare PX4/MAVROS
`/mavros/local_position/odom` motion against the FAST-LIO vision input on
`/mavros/vision_pose/pose_cov`. It publishes nothing, calls no services, and
does not open the FCU serial device:

```bash
evidence_dir=~/robotac_ws/logs/read_only_evidence/$(date +%Y%m%d_%H%M%S)
mkdir -p "${evidence_dir}"
roslaunch robotac_flight ev_acceptance_observer.launch \
  observe_seconds:=20 min_motion_m:=0.30 \
  evidence_file:="${evidence_dir}/ev_acceptance_observer.json"
```

This observer is an acceptance evidence step, not a flight controller. It
checks that local-position motion follows the external-vision input direction
and scale while MAVROS is connected, disarmed, on ground, and the bridge reports
healthy output.

To capture the read-only evidence bundle for later review, run:

```bash
./scripts/collect_readonly_flight_evidence.sh \
  --duration 8 --bag-seconds 0 \
  --output-dir "${evidence_dir}"
./scripts/analyze_readonly_flight_evidence.py "${evidence_dir}"
./scripts/flight_goal_audit.py --readonly-evidence "${evidence_dir}"
```

The collector only subscribes and inspects the existing ROS graph. It records
topic lists, topic info, one-message samples, short `rostopic hz` windows, and,
only when requested with `--bag-seconds`, a rosbag of the relevant MAVROS,
FAST-LIO, Livox, and vision topics. Use `--output-dir` to append this topic
evidence to the same directory that contains `ev_acceptance_observer.json`. It
never launches ROS nodes, publishes
topics, calls services, changes PX4 mode, arms, or sends setpoints. The analyzer
reads that bundle offline and reports whether `mavros_safe_state`,
`vision_to_mavros`, and `active_preflight_evidence` are ready. Its default
required phase is `active_preflight_evidence`, so it exits non-zero until MAVROS
is connected, disarmed, on ground, FAST-LIO vision is healthy, MAVROS consumes
`/mavros/vision_pose/pose_cov`, MAVROS consumes `/mavros/setpoint_raw/local`,
no node publishes `/mavros/setpoint_raw/local` during this read-only evidence
window, EV acceptance passed from the same directory, and the required
local-position, vision-pose, FAST-LIO odometry, and time-sync streams meet the
configured rate thresholds.

For a controlled test, `enable_control`, `auto_mode`, `auto_arm`, and
`auto_land` are independent gates. Keep all automatic gates false for the first
connected test. The checked-in route has `require_auto_land: true`, so an active
mission start is refused unless `auto_land:=true` was explicitly supplied; this
prevents the specified route from silently ending in a 1 m hover. Switching to
OFFBOARD or arming manually still requires the explicit start request and all
health checks to pass.

The default route is: take off to 1 m, forward 1 m and return, left 1 m and
return, right 1 m and return, rearward 1 m, open the payload servo, return to
the start point, then enter `AUTO.LAND`. Payload motion has its own explicit
gate and cannot occur unless `enable_payload:=true` is supplied together with
`enable_control:=true`. Start `robotac_servo` separately; the flight node
requires an active subscriber on `/robotac/servo/open`, commands it closed at
mission start, waits for the servo node's successful serial-write acknowledgement,
and only commands it open after the rear waypoint's hold time. The status topic
confirms a USB serial write, not a measured servo angle or a physical payload lock.
The flight node must receive an initial latched
`state=... success=... seq=... boot=...` status before it accepts a start
request; each mission command then requires a strictly newer sequence number
from the same servo process instance.

During any active local-flight test, start the read-only active-flight observer
in a separate terminal before calling `/robotac/flight/start`:

```bash
flight_evidence_dir=~/robotac_ws/logs/active_flight_evidence/$(date +%Y%m%d_%H%M%S)
mkdir -p "${flight_evidence_dir}"
roslaunch robotac_flight active_flight_observer.launch \
  evidence_file:="${flight_evidence_dir}/active_flight_observer.json"
```

The observer subscribes only to `/robotac/flight/status`,
`/robotac/flight/setpoint_preview`, `/mavros/local_position/odom`,
`/mavros/state`, `/mavros/extended_state`, and servo status. It never publishes
setpoints, calls services, arms, changes mode, or commands landing. A passing
`active_flight_observer.json` requires the controller to reach `COMPLETE`, all
waypoints to be consumed, relative airborne altitude to have been observed, MAVROS to
finish disarmed/on-ground, and every `TAKEOFF`/`WAYPOINTS` setpoint target to
have been observed within the configured `waypoint_reach_tolerance` by
`/mavros/local_position/odom`. If `require_payload_open:=true`, it also requires
a successful payload-open acknowledgement.

After the observer exits, analyze the evidence offline:

```bash
./scripts/analyze_active_flight_evidence.py "${flight_evidence_dir}"
./scripts/analyze_active_flight_evidence.py \
  "${flight_evidence_dir}" --require-phase payload_local_flight
```

The first command proves the local waypoint/takeoff/landing contract. The
payload phase additionally requires a successful servo open acknowledgement.
Like the read-only evidence analyzer, it only reads JSON and never interacts
with ROS, MAVROS, or the aircraft.

To roll up the whole objective after a test, use the top-level audit:

```bash
./scripts/flight_goal_audit.py \
  --readonly-evidence ~/robotac_ws/logs/read_only_evidence/YYYYMMDD_HHMMSS \
  --active-evidence ~/robotac_ws/logs/active_flight_evidence/YYYYMMDD_HHMMSS
```

It reports configuration gates, read-only FAST-LIO→MAVROS/PX4 evidence, and
active local-flight evidence together. The default required phase is
`active_local_flight`; add `--require-phase payload_local_flight` for the final
payload mission.

The camera publishes `/camera/rgb/image_raw`, `/camera/rgb/camera_info`, and
rectified `/camera/rgb/image_rect`. The default tested profile is MJPEG
1920x1080 at 30 Hz; YUYV at this resolution is only 5 Hz on the tested camera.
This camera's MJPEG stream decodes as YUV422, so the driver profile uses
`color_format: yuv422p`. JPEG compressed image transport is disabled for this
camera. AprilTag consumes the rectified topic by default. The checked-in
1920x1080 calibration has 30 valid samples and 0.1000 px RMS reprojection error.
The original calibration is retained as `config/camera/rgb_640x480.yaml`.
AprilTag defaults to `tag36h11` and loads standalone IDs `0` and `1` from
`config/apriltag/tags.yaml`. Each tag uses `0.15 m` for pose estimation; the
print's total outer size is recorded as `0.25 m`. To temporarily test one
different tag without editing the config, use
`use_config_tags:=false tag_id:=N tag_size:=S`.

Servo switch examples (with a running `roscore`):

```bash
rostopic pub -1 /robotac/servo/open std_msgs/Bool "data: true"
rostopic pub -1 /robotac/servo/open std_msgs/Bool "data: false"
rosrun robotac_servo servo_cycle_test.py
```

The camera and AprilTag chain can be observed without MAVROS:

```bash
rostopic hz /camera/rgb/image_raw
rostopic echo -n 1 /tag_detections
```

`camera_extrinsics.launch` is intentionally standalone and defaults to a zero
transform. Pass measured values before using its TF for navigation, for example
`roslaunch robotac_bringup camera_extrinsics.launch x:=... y:=... z:=...`.

## macOS to Ubuntu sync

From macOS:

```bash
./scripts/sync_to_ubuntu.sh user@ubuntu-host
```

Pass `user@ubuntu-host:/path` to select a different remote destination. The
sync intentionally does not use `--delete`.
