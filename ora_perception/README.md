# Perception line protection

For [issue #68](https://github.com/oaklandrobotics/ros_ora26/issues/68),
the clearing mask excludes detected lines **and a dilated margin around them**.
The marking cloud still uses the original cleaned lines; the extra margin is
neither marked nor cleared. Horizon and robot masks remain excluded.

`free_space_dilation_radius` is a startup-only integer ROS parameter, from 0 to
100 pixels. The default is **2**, retaining the existing 5x5 square dilation
kernel. A radius of 0 excludes the lines themselves without an extra margin.
A radius of 4 uses a 9x9 kernel. Larger margins reduce clearing near lines;
the physical width depends on image resolution, depth, and camera calibration.

Standalone example (after sourcing ROS and the workspace):

```bash
ros2 run ora_perception edge_detection --ros-args \
  -p use_sim_time:=true -p free_space_dilation_radius:=4
```

Run only one edge-detection node at a time. In the full robot launch, set the
parameter in the `edge_detection_node` parameters in `ora_launch/launch/robot.launch.py`.

Regression checks exercise margins, frame boundaries, robot/horizon exclusions,
and the actual image-to-point-cloud callback in both simulated and real modes:

```bash
colcon test --packages-select ora_perception --ctest-args -R free_space_mask
colcon test-result --verbose
```

These checks validate the image-mask and point-cloud behavior. They do not
establish that downstream costmap raytracing cannot cross a line or replace
field validation on the robot. Camera alignment is tracked separately from #68.
