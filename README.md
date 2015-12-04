mrsl_quadrotor
==========
package of simulation for mrsl UAV
### Requirements
General Requirements
 - `ros` (indigo)
 - `gazebo` (2.2)
 - `gazebo_ros` (2.2)

Robot types list
 - `hummingbird`
 - `hummingbird_rgbd`
 - `pelican_laser_rgbd`

Mobile object list
 - `mobile object`

Sensor list
 - `rgbd`
 - `laser scanner`

World models list
- `empty`
- `levine`
- `skir`
- `corridor`
### mrsl_quadrotor_launch
the package for launch demo
```           
$cd ./launch
$roslaunch gazebo.launch world:=corridor
$roslaunch spawn.launch robot_type:=pelican_laser_rgbd
```
 
