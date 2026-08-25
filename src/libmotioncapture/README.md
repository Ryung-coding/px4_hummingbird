[![CI](https://github.com/9iant/libmotioncapture/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/9iant/libmotioncapture/actions/workflows/CI.yml)

# libmotioncapture

Interface abstraction for motion capture system APIs.

Supported backends include Motion Analysis (Cortex), Vicon, OptiTrack, Qualisys, VRPN, Nokov, and FZMotion.

## ROS2 Integration

This branch targets ROS2 with `ament_cmake` and provides:

- Python bridge node: `scripts/mocap_bridge.py`
- ROS2 launch file: `launch/mocap_bridge.launch.py`
- Python bindings package: `motioncapture_ros`

In this cleaned workspace, `deps/pybind11` and `deps/cortex_sdk_linux` are normal vendored source directories, not git submodules.

## Prerequisites

```bash
sudo apt install libboost-system-dev libboost-thread-dev libeigen3-dev python3-dev python3-numpy
```

ROS2 Jazzy and `colcon` are required.

## Build (ROS2 / colcon)

```bash
cd <workspace>
colcon build --symlink-install
source install/setup.bash
```

This cleaned copy vendors the MotionAnalysis/Cortex SDK and pybind11 path needed for this workspace. Other backends are disabled by default; only enable them after adding the matching SDK directory under `deps/`.

## Run Bridge Node

```bash
ros2 launch libmotioncapture mocap_bridge.launch.py
```

Parameters:

- `host_ip` (default: `202.169.1.197`): local network interface IP used to receive Cortex UDP
- `mocap_ip` (default: `202.169.1.100`): MotionAnalysis/Cortex server IP
- `type` (default: `motionanalysis`): backend type
- `mocap_pub_name` (default: `/opti_raw`): raw PoseStamped output consumed by `px4_hummingbird_opti`
- `max_radius` (default: `3.0`): max allowed distance (m)
- `fps` (default: `500.0`): publish cap (Hz)

## Examples

```bash
python3 examples/python.py motionanalysis 127.0.0.1
install/lib/libmotioncapture/motioncapture_example motionanalysis 127.0.0.1
```

## License

[MIT](LICENSE)
