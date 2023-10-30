<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->


## Message details

|     |     |
| --- | --- |
| Data type: | `gazebo_msgs/GetJointProperties` |
| Used by: | `/gazebo/get_joint_properties` |

```
string joint_name
---
uint8 REVOLUTE=0
uint8 CONTINUOUS=1
uint8 PRISMATIC=2
uint8 FIXED=3
uint8 BALL=4
uint8 UNIVERSAL=5
uint8 type
float64[] damping
float64[] position
float64[] rate
bool success
string status_message

```


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
