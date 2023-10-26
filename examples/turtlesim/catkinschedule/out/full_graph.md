<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->

## build schedule graph


[![missing image](schedule.svg "missing image")](schedule.svg)


**Stats:**

|     |     |
| --- | --- |
| **Build time:** | 000 m 08.5 s |
| **Packages time sum:** | 000 m 08.5 s |
| **High load duration:** | 000 m 00.0 s |
| **Start gap duration:** | 000 m 00.0 s |
| **Critical path gap duration:** | 000 m 00.0 s |


**Pipeline:**

| Name: | Busy time: | Efficiency: |
| ----- | ---------- | ----------- |
| thread 0 | 000 m 08.5 s | 100.0 % |
| OVERALL | 000 m 08.5 s | 100.0 % |


**Critical path:**

| Name: | Duration: | Gap: | Start time: | End time: |
| ----- | --------- | ---- | ----------- | --------- |
| catkin_tools_prebuild | 000 m 04.1 s | 000 m 04.1 s | 000 m 00.0 s | 000 m 04.1 s |
| turtlepy | 000 m 04.4 s | 000 m 04.4 s | 000 m 04.1 s | 000 m 08.5 s |


**Packages (duration order):**

| Name: | Duration: | Start time: | End time: |
| ----- | --------- | ----------- | --------- |
| turtlepy | 000 m 04.4 s | 000 m 04.1 s | 000 m 08.5 s |
| catkin_tools_prebuild | 000 m 04.1 s | 000 m 00.0 s | 000 m 04.1 s |


**Packages (name order):**

| Name: | Duration: | Start time: | End time: |
| ----- | --------- | ----------- | --------- |
| catkin_tools_prebuild | 000 m 04.1 s | 000 m 00.0 s | 000 m 04.1 s |
| turtlepy | 000 m 04.4 s | 000 m 04.1 s | 000 m 08.5 s |


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
