mrsl_quadrotor
==========    
package of simulation for mrsl UAV
### Requirements
 - `ros` (indigo)
 - `gazebo` (2.2)
 - `gazebo_ros` (2.2)
 - `quadrotor_msgs`
              
### mrsl_quadrotor_launch
the package for launch demo
```           
$cd ./launch  
$roslaunch gazebo.launch
$roslaunch spawn.launch
$roslaunch controller.launch
$rosrun mrsl_quadrotor_launch start_motor_waypoints.sh
$rosrun mrsl_quadrotor_launch send_waypoints.sh
```           
              
### Todo's    
 - add Aerodynamics model
 - add TRPY control plugin

