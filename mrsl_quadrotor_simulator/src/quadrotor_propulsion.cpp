#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <mrsl_quadrotor_simulator/Quadrotor.h>
#include <mrsl_quadrotor_simulator/QuadrotorSO3AttitudeControl.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <stdio.h>
 
namespace mrsl_quadrotor_simulator
{
  class QuadrotorPropulsion : public gazebo::ModelPlugin
  {
   public:
    QuadrotorPropulsion() :
        initialized_(false)
    {
    }

    ~QuadrotorPropulsion()
    {
      node_handle_->shutdown();
      delete node_handle_;
    }
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      node_handle_ = new ros::NodeHandle("");                                                 
      link = _parent->GetLink();

      gazebo::physics::InertialPtr I = link->GetInertial();
      float Im[3][3];
      Im[0][0] = I->GetIXX();
      Im[0][1] = I->GetIXY();
      Im[1][0] = I->GetIXY();
      Im[1][1] = I->GetIYY();
      Im[1][2] = I->GetIYZ();
      Im[2][1] = I->GetIYZ();
      Im[2][2] = I->GetIZZ();
      Im[2][0] = I->GetIXZ();
      Im[0][2] = I->GetIXZ();

      quad_attitude_control.init(quad, Im);

      if (_sdf->HasElement("command_topic"))
        command_topic_ = _sdf->GetElement("command_topic")->Get<std::string>();

      if (command_topic_.compare("so3_cmd") == 0)
      {
        ros::SubscribeOptions ops;
        ops.callback_queue = &callback_queue_;
        ops.init<QuadrotorSO3AttitudeControl::CmdMsg>(command_topic_, 1, 
              boost::bind(&QuadrotorSO3AttitudeControl::cmdCallback, &quad_attitude_control, _1));
        command_sub_ = node_handle_->subscribe(ops);
      }
      else
        ROS_ERROR("command_topic has not been set up correctly, use either 'so3_cmd' or 'trpy_cmd'");

      updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&QuadrotorPropulsion::OnUpdate, this, _1));
    }
 


    void OnUpdate(const gazebo::common::UpdateInfo &_info)
    {
      gazebo::common::Time current_time = _info.simTime;
      if(!initialized_)
      {
        last_time_ = current_time;
        initialized_ = true;
        return;
      }

      double dt = (current_time - last_time_).Double();
      last_time_ = current_time;

      callback_queue_.callAvailable();

      gazebo::math::Pose pose = link->GetWorldPose();
      gazebo::math::Matrix3 R = pose.rot.GetAsMatrix3();
      gazebo::math::Vector3 angular_velocity = link->GetRelativeAngularVel();

      Quadrotor::State quad_state;
      quad_state.omega(0) = angular_velocity.x;
      quad_state.omega(1) = angular_velocity.y;
      quad_state.omega(2) = angular_velocity.z;
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          quad_state.R(i, j) = R[i][j];

      Quadrotor::MotorState des_rpm = quad_attitude_control.getControl(quad_state);
      Quadrotor::Wrench wrench = quad.update(des_rpm, dt);

      link->AddRelativeForce(gazebo::math::Vector3(0, 0, wrench(0)));
      link->AddRelativeTorque(gazebo::math::Vector3(wrench(1), wrench(2), wrench(3)));
    }
 
   private:
    // Pointer to the model
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::physics::LinkPtr link; // Link for robot
    gazebo::common::Time last_time_;
    bool initialized_; // initialize time stamp
    std::string command_topic_; // command that robot subscribe to

    ros::NodeHandle* node_handle_;
    ros::Subscriber command_sub_;
    ros::CallbackQueue callback_queue_;

    Quadrotor quad;
    QuadrotorSO3AttitudeControl quad_attitude_control;
  };
 
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QuadrotorPropulsion)
}

