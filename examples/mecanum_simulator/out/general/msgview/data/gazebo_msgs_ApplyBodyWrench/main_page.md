<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `gazebo_msgs/ApplyBodyWrench` |
| Usage: | `/gazebo/apply_body_wrench` |

```
string body_name
string reference_frame
geometry_msgs/Point reference_point
  float64 x
  float64 y
  float64 z
geometry_msgs/Wrench wrench
  geometry_msgs/Vector3 force
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 torque
    float64 x
    float64 y
    float64 z
time start_time
duration duration
---
bool success
string status_message


```


</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
