MRSL Quadrotor Simulator
=============
Package of Gazebo quadrotor simulator. Latest version works for Gazebo > 7.0.
User can easily set up the simulation environments and the quadrotor platform using sensors provided in this package.

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
  Hummingbird          |  empty  | empty | hummingbird
  Pelican              |  empty  | empty | pelican
  |                    |  hokuyo | levine| pelican\_laser
  FLA Platform         |  stereo | skir  | pelican\_laser\_rgbd
  |                    |  hokuyo | wg    | pelican\_laser\_rotate
  |                    |velodyne | pillars| pelican\_rgbd
  |                    | laser\_beam | D60| pelican\_velodyne

### Test Example
Using the example launch file in the repo
```
roslaunch mrsl_quadrotor_launch gazebo.launch world:=levine
roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=pelican mav_name:=juliett
```

### Dragon DDK:

Sensors: Downward facing camera, stereo, rgbd, imu.

Dependency: Clone and compile `https://github.com/ATLFlight/qflight_descriptions`
Note: The controller launch file depends on `quadrotor_control` package which will be soon released (currently closed source)
- `quadrotor_control` [KumarRobotics](https://github.com/KumarRobotics/quadrotor_control)
```
roslaunch mrsl_quadrotor_launch gazebo.launch world:=empty
roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=dragon_ddk mav_name:=ddk
roslaunch mrsl_quadrotor_launch controller.launch mav_type:=dragon_ddk mav_name:=ddk mass:=0.25 odom_topic:=ground_truth/odom
rosrun rqt_mav_manager rqt_mav_manager
```

### Falcon4:

Simulating the Ouster OS1 64 beam lidar requires using a [fork of the ouster_example code](https://github.com/wilselby/ouster_example)
and installing the hector gazebo plugins
```
sudo apt install ros-melodic-hector-gazebo-plugins
```
 a. Sensor: OVC3, Two OVC3 side Cams, downward facing Lidar
```
roslaunch mrsl_quadrotor_launch gazebo.launch world:=forest2
roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=falcon4 mav_name:=falcon4
roslaunch mrsl_quadrotor_launch falcon4_control.launch mav_type:=falcon4 mav_name:=falcon4 odom_topic:=ground_truth/odom
rosrun rqt_mav_manager rqt_mav_manager
```
 b. Sensor: Ouster OS1 64 x 512, OVC3, Two OVC3 side Cams, downward facing Lidar
```
roslaunch mrsl_quadrotor_launch gazebo.launch world:=forest2
roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=falcon4_os1_64_512 mav_name:=falcon4_os1
roslaunch mrsl_quadrotor_launch falcon4_os1_control.launch mav_type:=falcon4_os1_64_512 mav_name:=falcon4_os1 odom_topic:=ground_truth/odom
rosrun rqt_mav_manager rqt_mav_manager
```

 c. Sensor: Ouster OS1 16 x 256, OVC3, Two OVC3 side Cams, downward facing Lidar
```
roslaunch mrsl_quadrotor_launch gazebo.launch world:=forest2
roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=falcon4_os1_16_256 mav_name:=falcon4_os1
roslaunch mrsl_quadrotor_launch falcon4_os1_control.launch mav_type:=falcon4_os1_16_256 mav_name:=falcon4_os1 odom_topic:=ground_truth/odom
rosrun rqt_mav_manager rqt_mav_manager
```
### Samples
  hummingbird | hummingbird\_rgbd | pelican\_laser\_rgbd | Falcon4\_OS1 | mobile object
  :---------- | :-------------- | :------------------ | :----------- | :-----------
  <img src="./mrsl_models/samples/hummingbird.jpg" width="128"> | <img src="./mrsl_models/samples/hummingbird_rgbd.jpg" width="128"> | <img src="./mrsl_models/samples/pelican_laser_rgbd.jpg" width="128"> | <img src="./mrsl_models/samples/Falcon4.jpg" width="128"> | <img src="./mrsl_models/samples/mobile_object.jpg" width="128">

  levine | pillars | D60 | forest
  :---------- | :-------------- | :----------- | :-----------
  <img src="./mrsl_models/samples/levine.jpg" width="180"> | <img src="./mrsl_models/samples/pillars.png" width="180"> | <img src="./mrsl_models/samples/D60.png" width="180"> | <img src="./mrsl_models/samples/Forest.jpg" width="180">


