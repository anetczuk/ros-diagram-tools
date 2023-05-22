## Example of diagrams over Turtlebot3

*Nexus 4WD Mecanum Simulator* project is taken from (https://github.com/RBinsonB/nexus_4wd_mecanum_simulator) (commit `e26640c600050b3da95312885d6a6e4de0786ff6`).
Project is published on GPL-3.0 license.



### Generation

Dumping required data is done by script `dump.sh`. Script have to be run from environment with installed *ROS*.
Script `generate.sh` generates diagrams and views from duped data.



### Diagrams

#### `codedistribution`

[![code distribution chart](out/codedistribution-small.png "code distribution chart")](out/codedistribution.png)

#### `catkintree`

[![catkin packages tree](out/catkintree/full_graph-small.png "catkin packages tree")](out/catkintree/full_graph.png)

#### `catkinschedule`

[![catkin packages tree](out/catkinschedule/schedule-small.png "catkin packages tree")](out/catkinschedule/schedule.png)

#### `rosverify`

```
INFO:rosdiagram.tool.rosverify:found workspaces: ['/home/vbox/rosdiagrams/turtlebot3/catkin_ws', '/opt/ros/noetic']
INFO:rosdiagram.tool.rosverify:all overlay packages: []
INFO:rosdiagram.tool.rosverify:workspace overlay packages: []

```

#### `rosnodetree`

[![ROS nodes, topics and services graph](out/nodetree/full_graph-small.png "ROS nodes, topics and services graph")](out/nodetree/full_graph.png)

#### `rostopictree`

[![ROS nodes and topics graph](out/topictree/graph-small.png "ROS nodes and topics graph")](out/topictree/graph.png)
