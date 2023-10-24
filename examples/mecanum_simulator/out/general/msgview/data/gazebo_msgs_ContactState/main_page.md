<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `gazebo_msgs/ContactState` |

```
string info
string collision1_name
string collision2_name
geometry_msgs/Wrench[] wrenches
  geometry_msgs/Vector3 force
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 torque
    float64 x
    float64 y
    float64 z
geometry_msgs/Wrench total_wrench
  geometry_msgs/Vector3 force
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 torque
    float64 x
    float64 y
    float64 z
geometry_msgs/Vector3[] contact_positions
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3[] contact_normals
  float64 x
  float64 y
  float64 z
float64[] depths


```


</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
