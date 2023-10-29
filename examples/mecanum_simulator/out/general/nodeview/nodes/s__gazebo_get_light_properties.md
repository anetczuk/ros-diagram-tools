<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Service


|     |     |
| --- | --- |
| Name: | `/gazebo/get_light_properties` |
| Data type: | `gazebo_msgs/GetLightProperties` |
| Listener: | [`/gazebo`](n__gazebo.md) |

Message:
```
string light_name
---
std_msgs/ColorRGBA diffuse
  float32 r
  float32 g
  float32 b
  float32 a
float64 attenuation_constant
float64 attenuation_linear
float64 attenuation_quadratic
bool success
string status_message

```



</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
