# Livox ROS 2 messages

This package contains the unmodified `CustomMsg` and `CustomPoint` definitions
from [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2/tree/6b9356cadf77084619ba406e6a0eb41163b08039),
commit `6b9356cadf77084619ba406e6a0eb41163b08039`. It supplies the simulator's ROS
interfaces, without the hardware driver, Livox SDK, PCL, or APR. The package
maintainer metadata and full upstream MIT license notice are retained; this
message-only packaging is maintained in simple_rbt.

| File | SHA256 |
| --- | --- |
| `msg/CustomMsg.msg` | `f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96` |
| `msg/CustomPoint.msg` | `b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59` |

The package name `livox_ros_driver2`, message fields, generator target
`livox_interfaces2`, and `LIBRARY_NAME livox_ros_driver2` are preserved. C++
consumers link `livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp`;
Python consumers import `livox_ros_driver2.msg`.

From the repository root, `./setup.sh --native-only` builds this package into
`build/ros_interfaces/install/livox_ros_driver2`, then builds the native runtime
using the already prepared SDK and assets. `./setup.sh` also prepares those SDK
and asset dependencies. The launchers use this local prefix and leave any other
ROS workspace's source and installation untouched. This package provides no
hardware-driver node; hardware operation requires the upstream driver separately.
