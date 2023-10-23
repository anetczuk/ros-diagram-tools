<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Service


|  |  |
| --------------------------------- | -------- | ------------ |
| Name: | `/gazebo/spawn_sdf_model` |
| Data type | `gazebo_msgs/SpawnModel` |
| Listener: | [`/gazebo`](n__gazebo.html) |

Message:
```
string model_name
string model_xml
string robot_namespace
geometry_msgs/Pose initial_pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
string reference_frame
---
bool success
string status_message


```



<font size="1">
    File was automatically generated using [*ros-diagram-tools*]("https://github.com/anetczuk/ros-diagram-tools") project.
    Project is distributed under the BSD 3-Clause license.
</font>
