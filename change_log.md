# Change Log

All notable changes to this project will be documented in this file.

This package is updated to be used with ros2 jazzy and gazebo harmonic.

### files changed
- mrsl_models: updated the cmakelist.txt and package.xml to ros2
- mrsl_quadrotor_description: changed the falcon4_os1_16_256.urdf to have the updated sensor plugins in gazebo harmonic. Updated levin.world to have required plugins
- mrsl_quadrotor_launch: updated the launch files to ROS2 format
- mrsl_quadrotor_utils: only updated the change_header.cpp, change_odom.cpp, msg_to_tf.cpp to ROS2. The other source files are not updated nor compiled as they are not used to launch the autonomy stack for now.

### TODOs:
- mrsl_mobile_object: not updated for now as it's not core function of the autonomy stack 
- mrsl_quadrotor_simulator: changed not pushed as some required functions (eg. apply torque on quadrotor) are not implemeted yet by gzebobo harmonic 
- mrsl_quadrotor_description: only falcon4_os1_16_256 robot model and levin.world is updated to be used in gazebo harmonic. The other robot and world models are to be updated.