<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Main page

|     |     |
| --- | --- |
| Data type: | `controller_manager_msgs/ControllersStatistics` |

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
controller_manager_msgs/ControllerStatistics[] controller
  string name
  string type
  time timestamp
  bool running
  duration max_time
  duration mean_time
  duration variance_time
  int32 num_control_loop_overruns
  time time_last_control_loop_overrun


```


</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
