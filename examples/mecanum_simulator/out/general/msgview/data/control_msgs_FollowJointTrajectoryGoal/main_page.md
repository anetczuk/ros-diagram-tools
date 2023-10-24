<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `control_msgs/FollowJointTrajectoryGoal` |

```
trajectory_msgs/JointTrajectory trajectory
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string[] joint_names
  trajectory_msgs/JointTrajectoryPoint[] points
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration time_from_start
control_msgs/JointTolerance[] path_tolerance
  string name
  float64 position
  float64 velocity
  float64 acceleration
control_msgs/JointTolerance[] goal_tolerance
  string name
  float64 position
  float64 velocity
  float64 acceleration
duration goal_time_tolerance


```


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
