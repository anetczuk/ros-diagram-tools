# ROS diagram tools

Generate various aspects *ROS* workspace in form of diagrams.


## Code distribution

Tool presents distribution of code in source subdirectories.

Execution: `./src/rosdiagram/codedistribution.py --help`

[![code distribution chart](doc/codedistribution/out/graph-small.png "code distribution chart")](doc/codedistribution/out/graph.png)


## catkin packages tree

Show workspace packages in form of dependency tree.

Execution: `./src/rosdiagram/catkintree.py --help`

[![catkin packages tree](doc/catkintree/out/graph-small.png "catkin packages tree")](doc/catkintree/out/graph.png)


## rostopic tree

Present dependecy of *ROS* nodes and topics in form of dependency graph.

Execution: `./src/rosdiagram/rostopictree.py --help`

[![ROS nodes and topics graph](doc/rostopictree/out/graph-small.png "ROS nodes and topics graph")](doc/rostopictree/out/graph.png)


## References

- [pydotplus](https://pypi.org/project/pydotplus/)
