<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Service


|  |  |
| --------------------------------- | -------- | ------------ |
| Name: | `/gazebo/set_light_properties` |
| Data type | `gazebo_msgs/SetLightProperties` |
| Listener: | [`/gazebo`](n__gazebo.html) |

Message:
```
string light_name
bool cast_shadows
std_msgs/ColorRGBA diffuse
  float32 r
  float32 g
  float32 b
  float32 a
std_msgs/ColorRGBA specular
  float32 r
  float32 g
  float32 b
  float32 a
float64 attenuation_constant
float64 attenuation_linear
float64 attenuation_quadratic
geometry_msgs/Vector3 direction
  float64 x
  float64 y
  float64 z
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
---
bool success
string status_message


```



</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
