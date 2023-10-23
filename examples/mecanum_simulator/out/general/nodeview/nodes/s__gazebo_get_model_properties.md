<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Service


|  |  |
| --------------------------------- | -------- | ------------ |
| Name: | `/gazebo/get_model_properties` |
| Data type | `gazebo_msgs/GetModelProperties` |
| Listener: | [`/gazebo`](n__gazebo.html) |

Message:
```
string model_name
---
string parent_model_name
string canonical_body_name
string[] body_names
string[] geom_names
string[] joint_names
string[] child_model_names
bool is_static
bool success
string status_message


```



</br>
File was automatically generated using [*ros-diagram-tools*](https://github.com/anetczuk/ros-diagram-tools) project.
Project is distributed under the BSD 3-Clause license.
