mrsl_quadrotor
==========
package of simulation for mrsl UAV
### Requirements
General Requirements
 - `ros` (indigo)
 - `gazebo` (2.2)
 - `gazebo_ros` (2.2)

For control robot
 - `so3_control`
 - `quadrotor_msgs`

For planning and mapping
- `omnimapper_upenn`
 
### mrsl_quadrotor_launch
the package for launch demo
```           
$cd ./launch
$roslaunch gazebo.launch
$roslaunch spawn.launch
```
 
### Todo's    
 - add Aerodynamics model

