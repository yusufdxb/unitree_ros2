# Unitree robot models

This repository carries two Unitree message families. They are not
interchangeable: a robot publishes one of them on `lowstate`, and a ROS node
subscribed with the other type never matches it.

| Family | LowState | Motor slots | Robots (per official SDK2 sources) |
| --- | --- | ---: | --- |
| `unitree_go` | `unitree_go/msg/LowState` | 20 | GO2, GO2-W, B2, B2-W, H1 |
| `unitree_hg` | `unitree_hg/msg/LowState` | 35 | G1, H1-2, H2, R1 |

Request/response APIs use `unitree_api` for every robot. The family, topics and
motor counts below come from `unitree_sdk2` (aaf69cb),
`unitree_sdk2_python` (814556d), `unitree_mujoco` (1eb6642) and `unitree_ros`
(725b59b). A model file or a compiling message type is not evidence that
a robot's firmware behaves as described.

## Support status

Each column is a separate claim. "Software" columns are established on
loopback against software peers; they say nothing about a firmware release.

| Robot | Family | Native build | Serialization | DDS software peers | Recorded data | Real firmware telemetry | Command behavior |
| --- | --- | --- | --- | --- | --- | --- | --- |
| GO2 | `unitree_go` | Verified | Verified | SDK2 types; MuJoCo bridge | Verified (2 GO2 datasets) | Verified, read-only (2026-11-10) | Not verified |
| GO2-W | `unitree_go` | Verified | Verified | SDK2 types; MuJoCo bridge | None | None | Not verified |
| B2 | `unitree_go` | Verified | Verified | SDK2 types; MuJoCo bridge | None | None | Not verified |
| B2-W | `unitree_go` | Verified | Verified | SDK2 types; MuJoCo bridge | None | None | Not verified |
| H1 | `unitree_go` | Verified | Verified | SDK2 types; MuJoCo bridge | None | None | Not verified |
| G1 (23/29 DOF) | `unitree_hg` | Verified | Verified | SDK2 types; MuJoCo bridge | None | None | Not verified |
| H1-2 | `unitree_hg` | Verified | Verified | SDK2 types; MuJoCo model (family forced) | None | None | Not verified |
| H2 | `unitree_hg` | Verified | Verified | SDK2 types; MuJoCo model (family forced) | None | None | Not verified |
| R1 | `unitree_hg` | Verified | Verified | SDK2 types only | None | None | Not verified |
| A2, AS2 | `unitree_go` sport state only | Verified | Verified | SDK2 types only (`SportModeState`) | None | None | Not verified |
| G1-D, H2-D, R1 Air, R1-A5/A7, AS2-W, G1+ | Unknown | — | — | — | None | None | Unsupported |

What the columns rest on:

- **Native build**: Ubuntu 24.04 / Jazzy on x86_64 locally and in CI (x86_64
  and ARM64), Humble on x86_64 in CI, Foxy compilation. It is per message
  family; no robot-specific code is needed to build.
- **Serialization**: CDR round trips of every generated interface, and the
  field-by-field exchange with SDK2 types below.
- **SDK2 types**: `scripts/test_sdk2_peer.sh` builds a peer from
  `unitree_sdk2`'s own generated C++ types and its bundled Cyclone DDS 0.10.2,
  with no ROS IDL. It covers 17 types of both families (LowState, LowCmd, IMU,
  motor, hand, battery and board states, sport state, LiDAR state, wireless
  controller, `MotorCmds`, `Point32`). An SDK2 sample, with every scalar and the
  ends of every array filled, reaches ROS with all fields equal to it or to
  their defaults. For the 16 types SDK2 accepts from ROS (all but
  `SportModeState`, see below), a fully populated ROS sample reaches SDK2 with
  those named fields equal and SDK2's echo of it comes back unchanged.
  `unitree_api` is covered by the plain DDS peer, not by this one. This is
  evidence about a message family, not about any one robot.
- **MuJoCo bridge**: Unitree's own `unitree_mujoco` Python bridge (1eb6642, with
  `unitree_sdk2_python` 814556d and cyclonedds 0.10.2) published LowState for
  the official models on private topics. ROS received all 20 or 35 motor slots
  unchanged, the populated slots matched the model (GO2 12, GO2-W 16, B2 12,
  B2-W 16, H1 19 of 20, G1 29-DOF 29 and 23-DOF 23 of 29), and the readers
  printed the matching slots. The official Python bridge selects `unitree_hg`
  only for `g1`, so it cannot publish H1-2 or H2; with the family forced to
  `unitree_hg`, their models published 27 and 31 slots that ROS received
  unchanged. The simulator sets `mode_machine` to 0. This was a local check on
  x86_64 Jazzy; the harness is not part of this repository or CI.
- **Recorded data and firmware**: only GO2. Two offline GO2 recordings, and a
  read-only GO2 session on 2026-11-10 in which `/lowstate` and
  `/lf/sportmodestate` from firmware were decoded by `read_low_state` and
  `read_motion_state`, built from the commit this work starts from. No command
  was sent.
- **Command behavior**: no robot, including GO2, has had a command or API call
  verified by this work.

## Topics

Topic names are given as ROS names; the DDS topic adds the `rt/` prefix.
Never put `rt/` in a ROS topic name: ROS adds it again, so `rt/lowstate` in ROS
is the DDS topic `rt/rt/lowstate`, which no Unitree component publishes.

| Topic | Type | Robots | Notes |
| --- | --- | --- | --- |
| `lowstate` | `unitree_go/msg/LowState` | GO2, GO2-W, B2, B2-W, H1 | Motor slots used: 12, 16, 12, 16, 20 |
| `lowstate` | `unitree_hg/msg/LowState` | G1, H1-2, H2, R1 | Motor slots used: 29, 27, 31, 26 |
| `lf/lowstate` | same as `lowstate` | per robot | Low-rate copy where provided |
| `sportmodestate`, `lf/sportmodestate` | `unitree_go/msg/SportModeState` | GO2, GO2-W, B2, B2-W, A2, AS2 | SDK2 examples use `sportmodestate` for GO2 and `lf/sportmodestate` for the others |
| `wirelesscontroller` | `unitree_go/msg/WirelessController` | GO2, B2, G1 and others | Publisher QoS differs between observations; subscribe best-effort |
| `secondary_imu` | `unitree_hg/msg/IMUState` | G1, H2, R1 | Torso IMU |
| `dex3/left/state`, `dex3/right/state` | `unitree_hg/msg/HandState` | G1 with Dex3-1 | `lf/` variants exist |
| `waistpitchstate` | `unitree_hg/msg/MotorState` | G1-D, H2-D | Not covered by an example here |
| `hispeed_state` | `geometry_msgs/msg/Point32` | G1-D, H2-D | Lift height in `y` |
| `api/<service>/request`, `api/<service>/response` | `unitree_api/msg/Request`, `Response` | all | Service names differ per robot; see below |

Command topics (`lowcmd`, `arm_sdk`, `dex3/*`, `hand_sdk`, `waistpitchcmd`,
`user_lowcmd`) move hardware. They are listed only so that they can be avoided.

## Motor layout

`motor_state` is a fixed-size array. Only the first N slots carry joints, and
their order is robot-specific (see the joint index tables in `unitree_sdk2`
examples and `unitree_ros` model READMEs):

- GO2, B2: 12 leg motors. GO2-W, B2-W: slots 12–15 are the wheels.
- H1: 20 slots for 19 joints; slot 9 is unused.
- G1: 29 slots. 23-DOF configurations leave the waist roll/pitch and wrist
  pitch/yaw slots (13, 14, 20, 21, 27, 28) invalid, and lock-waist
  configurations leave 13 and 14 invalid. `mode_machine` in `LowState`
  identifies the configuration; the `unitree_ros` `g1_description` README maps
  each value (1–18, 23) to a model file.
- H1-2: 27, H2: 31, R1: 26 slots.

## Reading telemetry safely

Build as described in [the Jazzy guide](jazzy.md), configure the interface for
the robot, then run only subscriber examples:

```bash
ros2 run unitree_ros2_example read_low_state --ros-args -p robot:=b2
ros2 run unitree_ros2_example read_low_state_hg --ros-args -p robot:=g1
ros2 run unitree_ros2_example read_low_state_hg --ros-args -p robot:=h2 -p secondary_imu:=true
ros2 run unitree_ros2_example read_motion_state
ros2 run unitree_ros2_example read_wireless_controller
```

`robot` selects the motor slots that are printed: `go2` (default), `go2w`,
`b2`, `b2w`, `h1` for `read_low_state`; `g1` (default), `h1_2`, `h2`, `r1`
for `read_low_state_hg`. A model of the other family, or an unknown name, is
rejected with exit status 1 before any subscription is created. These examples
subscribe only to their telemetry topics, publish nothing but the `/rosout` and
`/parameter_events` topics every `rclcpp` node has, and create no service
clients; the test suite checks this from the ROS graph. Like every `rclcpp`
node they also offer the standard parameter services.

`ros2 topic echo /lowstate unitree_hg/msg/LowState` works as well. Check the
publisher QoS with `ros2 topic info -v` before relying on a reliable
subscription.

## Robot-specific APIs

Request/response services are robot-specific. Their API ids and parameters are
not shared between robots, even where the service name is the same.

| Robot | Clients in this repository | SDK2 service names |
| --- | --- | --- |
| GO2 | `ros2_sport_client`, `ros2_robot_state_client` (`go2_*` examples) | `sport`, `robot_state`, `vui`, `videohub`, `obstacles_avoid`, `config` |
| B2, B2-W | `ros2_b2_sport_client`, `b2_motion_switch_client` | `sport`, `motion_switcher`, `robot_state`, `config`, `front_videohub`, `back_videohub` |
| G1 | `g1_loco_client`, `g1_arm_action_client`, `g1_audio_client`, `g1_motion_switch_client` | `sport` (loco), `arm`, `voice`, `agv` |
| H2 | `h2_loco_client`, `h2_motion_switch_client` | `sport` (loco), `arm` |
| H1 | none | `loco` (API version 2.0.0.0) |
| H1-2, R1, A2, AS2 | none | R1: `sport`, `voice`; A2/AS2: `sport`, `voice` |

All clients share the timeout, correlation and single-flight behavior described
in [the Jazzy guide](jazzy.md#validation-matrix-and-limits). Calls are never
retried automatically. Every motion, mode, arm, hand, audio and low-level
command example can move or reconfigure the robot: run them only under the
robot's documented safety procedure, with the robot supervised and able to stop.
None is executed by the test suite.

## Known definition differences

These are the differences between this repository's interfaces and the
`unitree_sdk2` C++ and Python definitions. None changes the wire layout of the
fields ROS defines. The SDK2 peer tests check each of them except
`Response.binary`.

- `unitree_go` IMU, motor, battery and board temperatures, and
  `unitree_api/Response.binary`, are `int8` in ROS and `uint8` in SDK2. The bytes
  are identical; values above 127 read as negative in ROS.
- `unitree_go/LidarState`: SDK2 adds `uint8 dirty_percentage` after
  `error_state`. It occupies alignment padding in the ROS layout, so ROS readers
  ignore it and the remaining fields are unaffected.
- `unitree_go/SportModeState`: SDK2 appends `PathPoint[10] path_point`. ROS
  readers ignore the 280 trailing bytes (recorded GO2 data carries them as
  zeros). An SDK2 reader rejects a ROS-published `SportModeState`, because the
  ROS type does not serialize that array.
- Not in this repository: `unitree_go` `ConfigChangeStatus`, `VoxelMapCompressed`;
  `unitree_hg` `SportModeState`, `AgvBmsState`; `hg_doubleimu` `doubleIMUState`.

## Needs official documentation or hardware

- Firmware telemetry for every robot except GO2.
- Which topics, rates and QoS each firmware release publishes, including `lf/`
  variants and `secondary_imu`.
- The LowState family and motor layout of A2, AS2, AS2-W, G1-D, H2-D, R1 Air and
  R1-A5/A7: SDK2 shows only sport/audio APIs or arm examples for them.
- Topics and types for `unitree_hg/SportModeState`, `AgvBmsState` and
  `doubleIMUState`, which no SDK2 example uses.
- The behavior of every command and API call on any robot.
