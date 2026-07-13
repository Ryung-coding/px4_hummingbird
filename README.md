## HummingBird SITL Notes

This branch adds a Gazebo HummingBird mode for the `4022_gz_hummingbird`
airframe. The airframe uses `CA_AIRFRAME=16` to select the HummingBird
actuator effectiveness path.

The main mode parameters are:

| Parameter | Value | Meaning |
| --- | --- | --- |
| `HB_CTRL_MODE` | `0` | Conventional underactuated multicopter control. PX4 uses the normal position-to-attitude path and holds HummingBird servo outputs at zero. |
| `HB_CTRL_MODE` | `1` | Fully actuated HummingBird control. PX4 uses the HummingBird attitude/force path and allocator to publish motor plus theta/phi servo commands. |
| `HB_CMD_SOURCE` | `0` | RC/PX4 command source. The controller follows the normal PX4 trajectory setpoint path. |
| `HB_CMD_SOURCE` | `1` | DDS command source. The controller reads external ROS 2 `trajectory_setpoint6dof` input through the uXRCE-DDS bridge. |

`HB_CTRL_MODE` selects underactuated versus fully actuated control behavior.
`HB_CMD_SOURCE` selects whether the command comes from the normal PX4/RC path or
from the external DDS bridge.

## Documentation & Resources

| Resource | Description |
| --- | --- |
| [User Guide](https://docs.px4.io/main/en/) | Build, configure, and fly with PX4 |
| [Developer Guide](https://docs.px4.io/main/en/development/development.html) | Modify the flight stack, add peripherals, port to new hardware |
| [Airframe Reference](https://docs.px4.io/main/en/airframes/airframe_reference.html) | Full list of supported frames |
| [Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/) | Compatible flight controllers |
| [Release Notes](https://docs.px4.io/main/en/releases/) | What's new in each release |
| [Contribution Guide](https://docs.px4.io/main/en/contribute/) | How to contribute to PX4 |
