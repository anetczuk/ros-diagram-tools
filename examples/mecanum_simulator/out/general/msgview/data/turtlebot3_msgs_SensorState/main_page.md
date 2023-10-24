<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `turtlebot3_msgs/SensorState` |

```
uint8 BUMPER_FORWARD=1
uint8 BUMPER_BACKWARD=2
uint8 CLIFF=1
uint8 SONAR=1
uint8 ILLUMINATION=1
uint8 BUTTON0=1
uint8 BUTTON1=2
uint8 ERROR_LEFT_MOTOR=1
uint8 ERROR_RIGHT_MOTOR=2
uint8 TORQUE_ON=1
uint8 TORQUE_OFF=2
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 bumper
float32 cliff
float32 sonar
float32 illumination
uint8 led
uint8 button
bool torque
int32 left_encoder
int32 right_encoder
float32 battery


```


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
