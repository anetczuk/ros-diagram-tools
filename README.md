# ROS diagram tools

Generate various aspects *ROS* workspace in form of diagrams.


## Code distribution

Tool presents distribution of code in source subdirectories.

Execution: `./src/rosdiagram/codedistribution.py --help`

<p align="center" width="100%">
[![code distribution chart](example/codedistribution/out/graph-small.png "code distribution chart")](example/codedistribution/out/graph.png)
</p>


## catkin packages tree

Show workspace packages in form of dependency tree.

Execution: `./src/rosdiagram/catkintree.py --help`

<p align="center" width="100%">
[![catkin packages tree](example/catkinlist/out/graph-small.png "catkin packages tree")](example/catkinlist/out/graph.png)
</p>


## rostopic tree

Present dependecy of *ROS* nodes and topics in form of dependency graph.

Execution: `./src/rosdiagram/rostopictree.py --help`

<p align="center" width="100%">
[![ROS nodes and topics graph](example/rostopiclist/out/graph-small.png "ROS nodes and topics graph")](example/rostopiclist/out/graph.png)
</p>


## References

- [pydotplus](https://pypi.org/project/pydotplus/)
