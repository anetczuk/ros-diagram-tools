<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `visualization_msgs/ImageMarker` |

```
uint8 CIRCLE=0
uint8 LINE_STRIP=1
uint8 LINE_LIST=2
uint8 POLYGON=3
uint8 POINTS=4
uint8 ADD=0
uint8 REMOVE=1
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string ns
int32 id
int32 type
int32 action
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
float32 scale
std_msgs/ColorRGBA outline_color
  float32 r
  float32 g
  float32 b
  float32 a
uint8 filled
std_msgs/ColorRGBA fill_color
  float32 r
  float32 g
  float32 b
  float32 a
duration lifetime
geometry_msgs/Point[] points
  float64 x
  float64 y
  float64 z
std_msgs/ColorRGBA[] outline_colors
  float32 r
  float32 g
  float32 b
  float32 a


```


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
