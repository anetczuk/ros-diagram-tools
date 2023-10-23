<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## Topic

[![/gazebo/model_states](t__gazebo_model_states.png "/gazebo/model_states")](t__gazebo_model_states.png)

|  |  |
| --------------------------------- | -------- | ------------ |
| Topic name: | `/gazebo/model_states` |
| Data type | `gazebo_msgs/ModelStates` |
| Publishers: | `/gazebo` |
| Subscribers: | `` |

Message:
```
string[] name
geometry_msgs/Pose[] pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist[] twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z


```


| ROS nodes (1): | Description: |
| ----------------------------------- | ------------ |
| [`/gazebo`](n__gazebo.html) | Gazebo node |

| ROS topics (1): | Description: |
| ----------------------------------- | ------------ |
| [`/gazebo/model_states`](t__gazebo_model_states.html) |  |


<font size="1">
    File was automatically generated using [*ros-diagram-tools*]("https://github.com/anetczuk/ros-diagram-tools") project.
    Project is distributed under the BSD 3-Clause license.
</font>
