<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Topic

[![/cmd_vel](t__cmd_vel.png "/cmd_vel")](t__cmd_vel.png)

|     |     |
| --- | --- |
| Topic name: | `/cmd_vel` |
| Data type: | `geometry_msgs/Twist` |
| Publishers: | `` |
| Subscribers: | `/gazebo` |

Message:
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z

```


| ROS nodes (1): | Description: |
| -------------- | ------------ |
| [`/gazebo`](n__gazebo.md) | Gazebo node |

| ROS topics (1): | Description: |
| --------------- | ------------ |
| [`/cmd_vel`](t__cmd_vel.md) |  |


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
