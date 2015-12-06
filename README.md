mrsl_quadrotor
==========
package of simulation for mrsl UAV
### Requirements
General Requirements
 - `ros` (indigo)
 - `gazebo` (2.2)
 - `gazebo_ros` (2.2)

### List
  Robot Type         | Mobile object | Sensor | World    
  :----------------- | :------------ | :----- | :------
  hummingbird        | mobile object | rgbd   | empty    
  hummingbird_rgbd   |               | laser  | levine   
  pelican_laser_rgbd |               |        | corridor 
                     |               |        | warehouse

### mrsl_quadrotor_launch
the package for launch demo
```           
$cd ./launch
$roslaunch gazebo.launch world:=corridor
$roslaunch spawn.launch robot_type:=pelican_laser_rgbd
```
 
### Samples
  hummingbird | hummingbird_rgbd | pelican_laser_rgbd | mobile object
  :---------- | :-------------- | :----------------- | :-----------
  <img src="./mrsl_models/samples/hummingbird.jpg" width="96"> | <img src="./mrsl_models/samples/hummingbird_rgbd.jpg" width="96"> | <img src="./mrsl_models/samples/pelican_laser_rgbd.jpg" width="96"> | <img src="./mrsl_models/samples/mobile_object.jpg" width="128">

  levine | corridor | warehouse 
  :---------- | :-------------- | :----------- 
  <img src="./mrsl_models/samples/levine.jpg" width="256"> | <img src="./mrsl_models/samples/corridor.jpg" width="256"> | <img src="./mrsl_models/samples/warehouse.jpg" width="256">


