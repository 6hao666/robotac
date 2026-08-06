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
- `config/apriltag/tags.yaml`: tag36h11 IDs 0 and 1, each with a 0.20 m
  pose-estimation side length and 0.25 m printed total size metadata.
- `config/mavros/px4.yaml`: PX4 frame and plugin settings.
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

`full_system.launch` rejects the default workspace until the five sensor
checks in `config/deployment.yaml` are set to `true` and the MID360s JSON no
longer contains its sample IP addresses. Its `enable_mavros` argument defaults
to `false`; set it to `true` only after the FCU device rule and the sixth
`stable_fcu_device_configured` check are complete. This guard deliberately does
not run for individual component launches, so bench diagnosis remains possible.

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
```

For a read-only FCU telemetry session, use
`full_system.launch enable_mavros:=true` only after the deployment gate is
complete. No arming, mode, setpoint, takeoff, or other flight-control command
is sent by this workspace's launch files.

The camera publishes `/camera/rgb/image_raw`, `/camera/rgb/camera_info`, and
rectified `/camera/rgb/image_rect`. JPEG compressed image transport is disabled
for this camera. AprilTag consumes the rectified topic by default.
AprilTag defaults to `tag36h11` and loads standalone IDs `0` and `1` from
`config/apriltag/tags.yaml`. Each tag uses `0.20 m` for pose estimation; the
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

## Docker build (optional)

On a host with Docker Desktop running, build the Ubuntu 20.04/ROS Noetic
validation image from this directory:

```bash
docker build --progress=plain -t robotac-noetic:local .
```

The image runs the same native-library and catkin build steps as Ubuntu. It is
a compile check only; it does not contain or access aircraft hardware and does
not launch MAVROS.

## macOS to Ubuntu sync

From macOS:

```bash
./scripts/sync_to_ubuntu.sh user@ubuntu-host
```

Pass `user@ubuntu-host:/path` to select a different remote destination. The
sync intentionally does not use `--delete`.
