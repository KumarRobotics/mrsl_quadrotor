MRSL Quadrotor Simulator
=============
Package of Gazebo quadrotor simulator. User can easily set up the simulation environments and the quadrotor platform using sensors provided in this package.

Stacks include:
  - `mrsl_models`: all the CAD models
  - `mrsl_quadrotor_description`: all the `urdf` and world config files
  - `mrsl_quadrotor_launch`: example launch files
  - `mrsl_quadrotor_simulator`: quadrotor dynamics simulator
  - `mrsl_quadrotor_utils`: ros utils
  - `mrsl_mobile_object`: gazebo mobile obtacle plugin
  - `velodyne_simulator`: gazebo velodyne plugin
Check each package for more details.

### Requirements
Requirements
 - `ros` (indigo+)
 - `gazebo` (2.2+)
 - `gazebo_ros` (2.2+)
 - `quadrotor_control` [KumarRobotics](https://github.com/KumarRobotics/quadrotor_control)

### List of Components
  Robot Frame          |  Sensor | World | Platform Examples
  :------------------- |:-----   | :-----| :-------
  Hummingbird          |  rgbd   | empty | pelican
  Pelican              |  camera | levine| pelican\_laser
  FLA Platform         |  stereo | skir  | pelican\_laser\_rgbd
  |                    |  hokuyo | wg    | pelican\_laser\_rotate
  |                    |velodyne | pillars| pelican\_rgbd
  |                    | laser\_beam | iros2017\_corridor| pelican\_velodyne

### Test Example
Using the example launch file in the repo
```
$cd ./mrsl_quadrotor_launch/launch
$roslaunch gazebo.launch world:=levine
$roslaunch spawn.launch robot_type:=pelican
```

### Samples
  hummingbird | hummingbird\_rgbd | pelican\_laser\_rgbd | mobile object
  :---------- | :-------------- | :------------------ | :-----------
  <img src="./mrsl_models/samples/hummingbird.jpg" width="96"> | <img src="./mrsl_models/samples/hummingbird_rgbd.jpg" width="96"> | <img src="./mrsl_models/samples/pelican_laser_rgbd.jpg" width="96"> | <img src="./mrsl_models/samples/mobile_object.jpg" width="128">

  levine | pillars | iros2017\_corridor
  :---------- | :-------------- | :-----------
  <img src="./mrsl_models/samples/levine.jpg" width="256"> | <img src="./mrsl_models/samples/pillars.png" width="256"> | <img src="./mrsl_models/samples/iros2017_corridor.png" width="256">


