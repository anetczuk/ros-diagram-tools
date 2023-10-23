<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Service


|  |  |
| --------------------------------- | -------- | ------------ |
| Name: | `/gazebo/get_link_properties` |
| Data type | `gazebo_msgs/GetLinkProperties` |
| Listener: | [`/gazebo`](n__gazebo.html) |

Message:
```
string link_name
---
geometry_msgs/Pose com
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
bool gravity_mode
float64 mass
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
bool success
string status_message


```



<font size="1">
    File was automatically generated using [*ros-diagram-tools*]("https://github.com/anetczuk/ros-diagram-tools") project.
    Project is distributed under the BSD 3-Clause license.
</font>
