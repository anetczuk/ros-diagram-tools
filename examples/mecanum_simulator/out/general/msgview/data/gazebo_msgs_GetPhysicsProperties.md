<!--
File was automatically generated using 'ros-diagram-tools' project.
Project is distributed under the BSD 3-Clause license.
-->


## Message details

|     |     |
| --- | --- |
| Data type: | `gazebo_msgs/GetPhysicsProperties` |
| Used by: | `/gazebo/get_physics_properties` |

```
---
float64 time_step
bool pause
float64 max_update_rate
geometry_msgs/Vector3 gravity
  float64 x
  float64 y
  float64 z
gazebo_msgs/ODEPhysics ode_config
  bool auto_disable_bodies
  uint32 sor_pgs_precon_iters
  uint32 sor_pgs_iters
  float64 sor_pgs_w
  float64 sor_pgs_rms_error_tol
  float64 contact_surface_layer
  float64 contact_max_correcting_vel
  float64 cfm
  float64 erp
  uint32 max_contacts
bool success
string status_message

```


</br>
<font size="1">
File was automatically generated using <a href="https://github.com/anetczuk/ros-diagram-tools"><i>ros-diagram-tools</i></a> project.
Project is distributed under the BSD 3-Clause license.
</font>
