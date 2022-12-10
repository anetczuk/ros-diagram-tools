# ROS diagram tools

Generate various aspects of *Robot Operating System* workspace in form of *Graphviz* diagrams.

Package can produce following diagrams:
- source code distribution chart
- packages dependency tree
- *ROS* nodes and topics graphs



## code distribution diagram

Tool presents distribution of code in source subdirectories.

Execution: `./src/rosdiagram/codedistribution.py --help`

[![code distribution chart](example/codedistribution/out/graph-small.png "code distribution chart")](example/codedistribution/out/graph.png)



## catkin packages tree

Show workspace packages in form of dependency tree.

Execution: `./src/rosdiagram/catkintree.py --help`

[![catkin packages tree](example/catkinlist/out/graph-small.png "catkin packages tree")](example/catkinlist/out/graph.png)



## rostopic tree

Present dependecy of *ROS* nodes and topics in form of dependency graph.

Execution: `./src/rosdiagram/rostopictree.py --help`

[![ROS nodes and topics graph](example/rostopiclist/out/graph-small.png "ROS nodes and topics graph")](example/rostopiclist/out/graph.png)



## rosnode tree

Presents dependecy of *ROS* nodes, topics and services in form of data flow graph.

Execution: `./src/rosdiagram/rosnodetree.py --help`

[![ROS nodes, topics and services graph](example/rosnodelist/out/full_graph-small.png "ROS nodes, topics and services graph")](example/rosnodelist/out/full_graph.png)

In addition, for given graph interactive web page can be generated, [example here](example/rosnodelist/out/full_graph.html).

Following animation shows navigation thorough nodes of graph:

![HTML graph](doc/html_graph.gif "HTML graph")



## Dumping information

There are additional scripts:
- `./src/dump_rostopic.sh`
- `./src/dump_rosnode.sh`

that dump *ROS* data for further processing. It can be usefull, e.g. if one does not have unrestricted access to *ROS* workspace (Docker container for example).



## References

- [pydotplus](https://pypi.org/project/pydotplus/)
- [Texthon](texthon.chipsforbrain.org/)
