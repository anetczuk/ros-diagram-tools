<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `diagnostic_msgs/DiagnosticArray` |

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
diagnostic_msgs/DiagnosticStatus[] status
  byte OK=0
  byte WARN=1
  byte ERROR=2
  byte STALE=3
  byte level
  string name
  string message
  string hardware_id
  diagnostic_msgs/KeyValue[] values
    string key
    string value


```


</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
