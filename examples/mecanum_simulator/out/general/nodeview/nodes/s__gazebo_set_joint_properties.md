<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Service


|     |     |
| --- | --- |
| Name: | `/gazebo/set_joint_properties` |
| Data type: | `gazebo_msgs/SetJointProperties` |
| Listener: | [`/gazebo`](n__gazebo.md) |

Message:
```
string joint_name
gazebo_msgs/ODEJointProperties ode_joint_config
  float64[] damping
  float64[] hiStop
  float64[] loStop
  float64[] erp
  float64[] cfm
  float64[] stop_erp
  float64[] stop_cfm
  float64[] fudge_factor
  float64[] fmax
  float64[] vel
---
bool success
string status_message


```



</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
