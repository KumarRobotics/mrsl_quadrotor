#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <mrsl_quadrotor_simulator/Quadrotor.h>
#include <mrsl_quadrotor_simulator/QuadrotorTRPYControl.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <stdio.h>

namespace mrsl_quadrotor_simulator {
class QuadrotorPropulsionTRPY : public gazebo::ModelPlugin {
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadrotorPropulsionTRPY() : initialized_(false) { robot_name_ = std::string(""); }

  ~QuadrotorPropulsionTRPY() {
    node_handle_->shutdown();
    delete node_handle_;
  }

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable "
                       "to load plugin. "
                       << "Load the Gazebo system plugin "
                          "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
                          "package)");
      return;
    }

    if (_sdf->HasElement("robotNamespace"))
      robot_name_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

    node_handle_ = new ros::NodeHandle(robot_name_);
    link = _parent->GetLink();

    gazebo::physics::InertialPtr I = link->GetInertial();
    float Im[3][3];
#if GAZEBO_MAJOR_VERSION >= 8
    Im[0][0] = I->IXX();
    Im[0][1] = I->IXY();
    Im[1][0] = I->IXY();
    Im[1][1] = I->IYY();
    Im[1][2] = I->IYZ();
    Im[2][1] = I->IYZ();
    Im[2][2] = I->IZZ();
    Im[2][0] = I->IXZ();
    Im[0][2] = I->IXZ();
#else
    Im[0][0] = I->GetIXX();
    Im[0][1] = I->GetIXY();
    Im[1][0] = I->GetIXY();
    Im[1][1] = I->GetIYY();
    Im[1][2] = I->GetIYZ();
    Im[2][1] = I->GetIYZ();
    Im[2][2] = I->GetIZZ();
    Im[2][0] = I->GetIXZ();
    Im[0][2] = I->GetIXZ();
#endif

    quad_attitude_control.init(quad, Im);

    if (_sdf->HasElement("command_topic")) {
      command_topic_ = _sdf->GetElement("command_topic")->Get<std::string>();

      ros::SubscribeOptions ops;
      ops.callback_queue = &callback_queue_;
      ops.init<QuadrotorTRPYControl::TRPYCmdMsg>(
          command_topic_, 10,
          boost::bind(&QuadrotorTRPYControl::cmdCallback,
                      &quad_attitude_control, _1));
      command_sub_ = node_handle_->subscribe(ops);
    } else
      ROS_ERROR("command_topic has not been set up correctly, use either "
                "'so3_cmd' or 'trpy_cmd'");

    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&QuadrotorPropulsionTRPY::OnUpdate, this, _1));
  }

  void OnUpdate(const gazebo::common::UpdateInfo &_info) {
    gazebo::common::Time current_time = _info.simTime;
    if (!initialized_) {
      last_time_ = current_time;
      initialized_ = true;
      return;
    }

    double dt = (current_time - last_time_).Double();
    last_time_ = current_time;

    callback_queue_.callAvailable();

    Quadrotor::State quad_state;
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = link->WorldPose();
    ignition::math::Vector3d angular_velocity = link->RelativeAngularVel();
#else
    ignition::math::Pose3d pose = link->GetWorldPose().Ign();
    ignition::math::Vector3d angular_velocity = link->GetRelativeAngularVel().Ign();
#endif
    quad_state.omega(0) = angular_velocity.X();
    quad_state.omega(1) = angular_velocity.Y();
    quad_state.omega(2) = angular_velocity.Z();
    quad_state.ypr(0) = pose.Rot().Yaw();
    quad_state.ypr(1) = pose.Rot().Pitch();
    quad_state.ypr(2) = pose.Rot().Roll();

    Quadrotor::MotorState des_rpm =
        quad_attitude_control.getControl(quad_state);
    Quadrotor::Wrench wrench = quad.update(des_rpm, dt);

    link->AddRelativeForce(ignition::math::Vector3d(0, 0, wrench(0)));

    link->AddRelativeTorque(
        ignition::math::Vector3d(wrench(1), wrench(2), wrench(3)));
  }

private:
  // Pointer to the model
  gazebo::event::ConnectionPtr updateConnection;
  gazebo::physics::LinkPtr link; // Link for robot
  gazebo::common::Time last_time_;
  bool initialized_; // initialize time stamp
  std::string robot_name_;
  std::string command_topic_; // command that robot subscribe to

  ros::NodeHandle *node_handle_;
  ros::Subscriber command_sub_;
  ros::CallbackQueue callback_queue_;

  Quadrotor quad;
  QuadrotorTRPYControl quad_attitude_control;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(QuadrotorPropulsionTRPY)
}
