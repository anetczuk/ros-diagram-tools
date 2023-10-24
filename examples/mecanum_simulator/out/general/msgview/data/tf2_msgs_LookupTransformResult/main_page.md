<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `tf2_msgs/LookupTransformResult` |

```
geometry_msgs/TransformStamped transform
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
tf2_msgs/TF2Error error
  uint8 NO_ERROR=0
  uint8 LOOKUP_ERROR=1
  uint8 CONNECTIVITY_ERROR=2
  uint8 EXTRAPOLATION_ERROR=3
  uint8 INVALID_ARGUMENT_ERROR=4
  uint8 TIMEOUT_ERROR=5
  uint8 TRANSFORM_ERROR=6
  uint8 error
  string error_string


```


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
