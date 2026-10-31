# Controller Reference

Quick reference for joystick feedback patterns and ROS 2 service callbacks.

---

## Feedback Patterns

- **`k_north_course_feedback_`**
  - **Pattern:** 1 long pulse
  - **Trigger:** Course selection set to North Course (Course 1).

- **`k_south_course_feedback_`**
  - **Pattern:** 2 long pulses
  - **Trigger:** Course selection set to South Course (Course 2).

- **`k_toggle_enabled_feedback_`**
  - **Pattern:** 1 short pulse
  - **Trigger:** Feature enabled (ex: Enable Auton, Enable Drive Forward).

- **`k_toggle_disabled_feedback_`**
  - **Pattern:** 2 short pulses
  - **Trigger:** Feature disabled (ex: Disable Auton, Disable Drive Forward).

- **`k_update_feedback_`**
  - **Pattern:** 1 long pulse, 1 short pulse
  - **Trigger:** Reload operation complete (ex: Reload Waypoints).

- **`k_high_impact_feedback_`**
  - **Pattern:** Sustained long pulse
  - **Trigger:** System reset or safety action (ex: Reset Navigation, E-Stop Activation).

---

## Service Callbacks

- **`setCourseClientCallback`**
  Cycles between North and South courses. Triggers `k_north_course_feedback_` when set to True (North Course) and `k_south_course_feedback_` when set to False (South Course).

- **`setAutonClientFeedback`**
  Triggers `k_toggle_enabled_feedback_` when autonomous mode is enabled and `k_toggle_disabled_feedback_` when disabled.

- **`reloadWaypointClientFeedback`**
  Triggers `k_update_feedback_` upon successful completion of waypoint reload.

- **`resetNavigationClientFeedback`**
  Triggers `k_high_impact_feedback_` upon successful completion of navigation reset.