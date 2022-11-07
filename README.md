# ROS diagram tools

Generate various aspects *ROS* workspace in form of diagrams.


## Code distribution

Tool presents distribution of code in source subdirectories.

Execution: `./src/rosdiagram/codedistribution.py --help`

<span style="display:block;text-align:center">
[![code distribution chart](example/codedistribution/out/graph-small.png "code distribution chart")](example/codedistribution/out/graph.png)
</span>


## catkin packages tree

Show workspace packages in form of dependency tree.

Execution: `./src/rosdiagram/catkintree.py --help`

<span style="display:block;text-align:center">
[![catkin packages tree](example/catkinlist/out/graph-small.png "catkin packages tree")](example/catkinlist/out/graph.png)
</span>


## rostopic tree

Present dependecy of *ROS* nodes and topics in form of dependency graph.

Execution: `./src/rosdiagram/rostopictree.py --help`

<span style="display:block;text-align:center">
[![ROS nodes and topics graph](example/rostopiclist/out/graph-small.png "ROS nodes and topics graph")](example/rostopiclist/out/graph.png)
</span>


## References

- [pydotplus](https://pypi.org/project/pydotplus/)
