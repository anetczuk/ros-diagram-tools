<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `map_msgs/GetPointMapROI` |

```
float64 x
float64 y
float64 z
float64 r
float64 l_x
float64 l_y
float64 l_z
---
sensor_msgs/PointCloud2 sub_map
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  uint32 height
  uint32 width
  sensor_msgs/PointField[] fields
    uint8 INT8=1
    uint8 UINT8=2
    uint8 INT16=3
    uint8 UINT16=4
    uint8 INT32=5
    uint8 UINT32=6
    uint8 FLOAT32=7
    uint8 FLOAT64=8
    string name
    uint32 offset
    uint8 datatype
    uint32 count
  bool is_bigendian
  uint32 point_step
  uint32 row_step
  uint8[] data
  bool is_dense


```


</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
