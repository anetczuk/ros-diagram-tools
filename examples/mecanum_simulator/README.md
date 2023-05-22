# Example of diagrams over Nexus 4WD Mecanum Simulator

*Nexus 4WD Mecanum Simulator* project is taken from (https://github.com/RBinsonB/nexus_4wd_mecanum_simulator) (commit `e26640c600050b3da95312885d6a6e4de0786ff6`).
Project is published on GPL-3.0 license.



## Generation

Dumping required data is done by script `dump.sh`. Script have to be run from environment with installed *ROS*.
Script `generate.sh` generates diagrams and views from duped data.



## `codedistribution`

[![code distribution chart](out/codedistribution-small.png "code distribution chart")](out/codedistribution.png)


## `catkintree`

Generated diagram:

[![catkin packages tree](out/catkintree/full_graph-small.png "catkin packages tree")](out/catkintree/full_graph.png)

Generated main graph view:

[![main graph view](out/catkintree/main-page-small.png "main graph view")](out/catkintree/main-page.png)

Generated package graph view:

[![package graph view](out/catkintree/node-page-small.png "package graph view")](out/catkintree/node-page.png)


## `catkinschedule`

Generated diagram:

[![catkin packages tree](out/catkinschedule/schedule-small.png "catkin packages tree")](out/catkinschedule/schedule.png)

Generated main graph view:

[![main graph view](out/catkinschedule/main-page-small.png "main graph view")](out/catkinschedule/main-page.png)


## `classifynodes`

```
{
    "gazebo_ros": {
        "path": "/opt/ros/noetic/share/gazebo_ros",
        "nodes": [
            "/gazebo",
            "/gazebo_gui"
        ]
    },
    "nexus_4wd_mecanum_description": {
        "path": "/home/vbox/rosdiagrams/mecanum/catkin_ws/src/nexus_4wd_mecanum_description",
        "nodes": [
            "/robot_state_publisher"
        ]
    },
    "nexus_4wd_mecanum_gazebo": {
        "path": "/home/vbox/rosdiagrams/mecanum/catkin_ws/src/nexus_4wd_mecanum_gazebo",
        "nodes": [
            "/gazebo",
            "/gazebo_gui",
            "/robot_state_publisher",
            "/urdf_spawner"
        ]
    }
}

```


## `rosverify`

```
INFO:rosdiagram.tool.rosverify:found workspaces: ['/home/vbox/rosdiagrams/turtlebot3/catkin_ws', '/opt/ros/noetic']
INFO:rosdiagram.tool.rosverify:all overlay packages: []
INFO:rosdiagram.tool.rosverify:workspace overlay packages: []

```


## `rosnodetree`

Generated diagram:

[![ROS nodes, topics and services graph](out/nodetree/full_graph-small.png "ROS nodes, topics and services graph")](out/nodetree/full_graph.png)

Generated main graph view:

[![main graph view](out/nodetree/main-page-small.png "main graph view")](out/nodetree/main-page.png)

Generated node graph view:

[![node graph view](out/nodetree/node-page-small.png "node graph view")](out/nodetree/node-page.png)

Generated topic graph view:

[![topic graph view](out/nodetree/topic-page-small.png "topic graph view")](out/nodetree/topic-page.png)

Generated service graph view:

[![service graph view](out/nodetree/service-page-small.png "service graph view")](out/nodetree/service-page.png)


## `rostopictree`

[![ROS nodes and topics graph](out/topictree/graph-small.png "ROS nodes and topics graph")](out/topictree/graph.png)
