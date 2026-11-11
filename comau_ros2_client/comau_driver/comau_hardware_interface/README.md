# comau_hardware_interface

`ros2_control` system interface of the Comau C5G (`comau_hardware_interface/ComauHardwareInterface`). Hardware
parameters are read from the `<hardware><param>` tags of the arm's `<ros2_control>` block (see `src/comau_hw_interface.cpp`,
`on_init`): `robot_ip`, `state_port`, `robot_port`, `arm1_port`, the `fly_*` and `stop_*` tuning, `fk_base_link`,
`fk_tip_link`, `max_resume_deviation`, `speed_override_percent`.

## Joint mapping: `joint_signs`, `joint_offsets_deg`

The controller reports joints in degrees with its own sign and zero; the robot model uses radians with its own. Per joint:

```
model [deg] = sign * controller [deg] + offset_deg
controller [deg] = sign * (model [deg] - offset_deg)
```

Without parameters the driver uses the mapping measured on the Racer 5-0.80 (`include/comau_hardware_interface/joint_mapping.hpp`):

| joint | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| `sign` | -1 | 1 | -1 | -1 | 1 | -1 |
| `offset_deg` | 0 | 0 | -90 | 0 | 0 | 34.5 |

Other robots set both parameters, comma separated, with one value per joint of the arm (signs are 1 or -1), e.g.

```xml
<param name="joint_signs">-1,1,-1,-1</param>
<param name="joint_offsets_deg">0.0,0.0,-85.0,0.0</param>
```

Only one of the two may be given: the other keeps the built-in values for those positions. A wrong count or value makes
`on_init` fail with the reason in the log. In RAPT OS the export writes these parameters from the project's
`robot_driver.json` (`joint_mapping`), see `robots/drivers/README.md`.

The mapping has been measured on the Racer 5-0.80 only; any other robot (e.g. the P180 palletizer) has to be verified on
the real robot before use.

## Tests

`test/test_joint_mapping.cpp` (gtest, no ROS needed) checks that the built-in table gives exactly the values of the former
hard-coded mapping, and the override, parsing and error cases.
