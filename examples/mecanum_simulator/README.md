# Example of diagrams over Nexus 4WD Mecanum Simulator

*Nexus 4WD Mecanum Simulator* project is taken from (https://github.com/RBinsonB/nexus_4wd_mecanum_simulator) (commit `e26640c600050b3da95312885d6a6e4de0786ff6`).
Project is published on GPL-3.0 license.


## Generation

Dumping required data is done by script `dump.sh`. Script have to be run from environment with installed *ROS*.
Script `generate.sh` generates diagrams and views from duped data.


## `codedistribution` over `dumpclocdir` dump

[![code distribution chart](out/codedistribution_src-small.png "code distribution chart")](out/codedistribution_src.png)


## `codedistribution` over `dumpclocpack` dump

[![code distribution chart](out/general/clockpackview/graph-small.png "code distribution chart")](out/general/clockpackview/graph.png)


## `packagetree` over `dumpcatkindeps` dump

Generated diagram:

[![catkin packages tree](out/catkintree/full_graph-small.png "catkin packages tree")](out/catkintree/full_graph.png)

Generated main graph view:

[![main graph view](out/catkintree/main-page-small.png "main graph view")](out/catkintree/main-page.png)

Generated package graph view:

[![package graph view](out/catkintree/node-page-small.png "package graph view")](out/catkintree/node-page.png)


## `packagetree` over `dumprospack` dump

Generated diagram:

[![runtime packages tree](out/general/packageview/full_graph-small.png "runtime packages tree")](out/general/packageview/full_graph.png)

Generated main graph view:

[![main graph view](out/general/packageview-main-page-small.png "main graph view")](out/general/packageview-main-page.png)


## `buildtime`

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


## `rosnodegraph`

Generated full diagram:

[![ROS nodes, topics and services graph](out/nodetree/whole_graph-small.png "ROS nodes, topics and services graph")](out/nodetree/whole_graph.png)

Generated main graph view:

[![main graph view](out/general/nodeview-main-page-small.png "main graph view")](out/general/nodeview-main-page.png)

Generated node graph view:

[![node graph view](out/general/nodeview-node-page-small.png "node graph view")](out/general/nodeview-node-page.png)

Generated topic graph view:

[![topic graph view](out/general/nodeview-topic-page-small.png "topic graph view")](out/general/nodeview-topic-page.png)

Generated service graph view:

[![service graph view](out/general/nodeview-service-page-small.png "service graph view")](out/general/nodeview-service-page.png)


## `rostopicgraph`

[![ROS nodes and topics graph](out/topictree/graph-small.png "ROS nodes and topics graph")](out/topictree/graph.png)
