/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin 
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/**
 * A diff drive plugin supporting multiple wheels per vehicle side. Based on
 * existing plugins as stated above this notice.
 */

/*
    Copyright (c) 2014, Stefan Kohlbrecher
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <algorithm>
#include <assert.h>

#include <hector_gazebo_plugins/diffdrive_plugin_multi_wheel.h>

#if (GAZEBO_MAJOR_VERSION < 8)
#include <gazebo/math/gzmath.hh>
#endif
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <gazebo/gazebo_config.h>

namespace gazebo {

  enum {
    RIGHT,
    LEFT,
  };

  GazeboRosDiffDriveMultiWheel::GazeboRosDiffDriveMultiWheel() {}

  // Destructor
  GazeboRosDiffDriveMultiWheel::~GazeboRosDiffDriveMultiWheel() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  // Load the controller
  void GazeboRosDiffDriveMultiWheel::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboRosDiffDriveMultiWheel Plugin missing <robotNamespace>, defaults to \"%s\"", 
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ = 
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    //this->left_joint_names_ = "left_joint";
    if (!_sdf->HasElement("leftJoints")) {
      gzthrow("Have to specify space separated left side joint names via <leftJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("leftJoints")->Get<std::string>();
      boost::split( joint_names_[LEFT], joint_string, boost::is_any_of(" ") );
    }

    //this->right_joint_names_ = "right_joint";
    if (!_sdf->HasElement("rightJoints")) {
      gzthrow("Have to specify space separated right side joint names via <rightJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("rightJoints")->Get<std::string>();
      boost::split( joint_names_[RIGHT], joint_string, boost::is_any_of(" ") );
    }

    this->wheel_separation_ = 0.34;
    if (!_sdf->HasElement("wheelSeparation")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <wheelSeparation>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
      this->wheel_separation_ = 
        _sdf->GetElement("wheelSeparation")->Get<double>();
    }

    this->wheel_diameter_ = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
      this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }


    this->publish_odometry_tf_ = true;
    if (!_sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = _sdf->GetElement("publishOdometryTf")->Get<bool>();
    }

    this->publish_odometry_msg_ = true;
    if (!_sdf->HasElement("publishOdometryMsg")) {
      ROS_WARN("GazeboRosDiffDriveMultiWheel Plugin (ns = %s) missing <publishOdometryMsg>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_msg_ ? "true" : "false");
    } else {
      this->publish_odometry_msg_ = _sdf->GetElement("publishOdometryMsg")->Get<bool>();
    }



    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
#if (GAZEBO_MAJOR_VERSION >= 8)
    last_update_time_ = this->world->SimTime();
#else
    last_update_time_ = this->world->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    for (size_t side = 0; side < 2; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){
        joints_[side].push_back(this->parent->GetJoint(joint_names_[side][i]));
        if (!joints_[side][i]){
          char error[200];
          snprintf(error, 200,
                   "GazeboRosDiffDriveMultiWheel Plugin (ns = %s) couldn't get hinge joint named \"%s\"",
                   this->robot_namespace_.c_str(), joint_names_[side][i].c_str());
          gzthrow(error);
        }
#if (GAZEBO_MAJOR_VERSION > 4)
        joints_[side][i]->SetEffortLimit(0, torque);
#else
        joints_[side][i]->SetMaxForce(0, torque);
#endif
      }
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO("Starting GazeboRosDiffDriveMultiWheel Plugin (ns = %s)!", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosDiffDriveMultiWheel::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosDiffDriveMultiWheel::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosDiffDriveMultiWheel::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosDiffDriveMultiWheel::UpdateChild() {
#if (GAZEBO_MAJOR_VERSION >= 8)
    common::Time current_time = this->world->SimTime();
#else
    common::Time current_time = this->world->GetSimTime();
#endif
    double seconds_since_last_update = 
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      if (this->publish_odometry_tf_ || this->publish_odometry_msg_){
        publishOdometry(seconds_since_last_update);
      }

      // Update robot in case new velocities have been requested
      getWheelVelocities();
      //joints[LEFT]->SetVelocity(0, wheel_speed_[LEFT] / wheel_diameter_);
      //joints[RIGHT]->SetVelocity(0, wheel_speed_[RIGHT] / wheel_diameter_);

      for (size_t side = 0; side < 2; ++side){
        for (size_t i = 0; i < joints_[side].size(); ++i){
          joints_[side][i]->SetVelocity(0, wheel_speed_[side] / (0.5 * wheel_diameter_));
        }
      }

      last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void GazeboRosDiffDriveMultiWheel::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosDiffDriveMultiWheel::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double vr = x_;
    double va = rot_;

    wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
  }

  void GazeboRosDiffDriveMultiWheel::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosDiffDriveMultiWheel::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosDiffDriveMultiWheel::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = 
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = this->parent->WorldPose();

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
#else
    math::Pose pose = this->parent->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
#endif

    tf::Transform base_footprint_to_odom(qt, vt);

    if (this->publish_odometry_tf_){
      transform_broadcaster_->sendTransform(
            tf::StampedTransform(base_footprint_to_odom, current_time,
                                 odom_frame, base_footprint_frame));
    }

    // publish odom topic
#if (GAZEBO_MAJOR_VERSION >= 8)
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
#else
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
#endif
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d linear;
    linear = this->parent->WorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->WorldAngularVel().Z();
#else
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;
#endif

    // convert velocity to child_frame_id (aka base_footprint)
#if (GAZEBO_MAJOR_VERSION >= 8)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
#else
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;
#endif

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (this->publish_odometry_msg_){
      odometry_publisher_.publish(odom_);
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDriveMultiWheel)
}
