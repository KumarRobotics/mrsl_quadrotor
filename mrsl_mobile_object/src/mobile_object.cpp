#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

namespace mrsl_quadrotor_simulator
{
  class MobileObject : public gazebo::ModelPlugin
  {
   public:
    MobileObject():
        initialzed_(false),
        lower_bound_(-1.0),
        upper_bound_(1.0),
        vx_(0.0),
        vy_(0.0)
    {
    }

    ~MobileObject()
    {
    }

    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      ns_ = "";
      if (_sdf->HasElement("robotNamespace"))
        ns_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

      if (!_sdf->HasElement("updateRate"))
        update_rate_ = 10.0;
      else
        update_rate_ = _sdf->GetElement("updateRate")->Get<double>();


      if (!_sdf->HasElement("topicName"))
        topic_name_ = "pose";
      else
        topic_name_ = _sdf->Get<std::string>("topicName");

      node_handle_ = new ros::NodeHandle(ns_);

      if (topic_name_ != "")
        pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(topic_name_, 1);

      if(_sdf->HasElement("vx"))
        vx_ = _sdf->GetElement("vx")->Get<double>();
      else
        vx_ = 0;
      if(_sdf->HasElement("vy"))
        vy_ = _sdf->GetElement("vy")->Get<double>();
      else
        vy_ = 0;
      if (_sdf->HasElement("w"))
        w_ = _sdf->GetElement("w")->Get<double>();
      else
        w_ = 0;



      link = _parent->GetLink();

      updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&MobileObject::OnUpdate, this, _1));
    }


    void OnUpdate(const gazebo::common::UpdateInfo &_info)
    {
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d pose = link->WorldPose();
#else
      ignition::math::Pose3d pose = link->GetWorldPose().Ign();
#endif
      if(!initialzed_){
        initialzed_ = true;
        init_pose_ = pose;
      }

      if((fabs(pose.Pos().X() - (init_pose_.Pos().X()  + lower_bound_)) < 1e-1 && vx_ < 0)
         || (fabs(pose.Pos().X() - (init_pose_.Pos().X() + upper_bound_)) < 1e-1 && vx_ > 0))
        vx_ = -vx_;

      if((fabs(pose.Pos().Y() - (init_pose_.Pos().Y()  + lower_bound_)) < 1e-1 && vy_ < 0)
         || (fabs(pose.Pos().Y() - (init_pose_.Pos().Y() + upper_bound_)) < 1e-1 && vy_ > 0))
        vy_ = -vy_;

      link->SetLinearVel(ignition::math::Vector3d(vx_, vy_, 0));
      link->SetAngularVel(ignition::math::Vector3d(0, 0, w_));

      //if (pub_.getNumSubscribers() > 0 && topic_name_ != "" && update_rate_ > 0)
      if (topic_name_ != "" && update_rate_ > 0)
      {
        static tf::TransformBroadcaster br;
        static ros::Time prev_time = ros::Time::now();

        if((ros::Time::now() - prev_time).toSec() >= 1./update_rate_) {
          geometry_msgs::PoseStamped msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "map";
          msg.pose.position.x = pose.Pos().X();
          msg.pose.position.y = pose.Pos().Y();
          msg.pose.position.z = pose.Pos().Z();
          msg.pose.orientation.w = pose.Rot().W();
          msg.pose.orientation.x = pose.Rot().X();
          msg.pose.orientation.y = pose.Rot().Y();
          msg.pose.orientation.z = pose.Rot().Z();

          pub_.publish(msg);

          tf::Transform bt(tf::Quaternion(msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w),
              tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
          br.sendTransform(tf::StampedTransform(bt, msg.header.stamp, msg.header.frame_id, ns_ + "base_link"));

          prev_time = msg.header.stamp;
        }
      }
    }

   private:
    // Pointer to the model
    ignition::math::Pose3d init_pose_;
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::physics::LinkPtr link; // Link for robot
    bool initialzed_;
    double lower_bound_;
    double upper_bound_;
    double vx_, vy_, w_;

    // Ros interface
    ros::NodeHandle *node_handle_;
    ros::Publisher pub_;
    std::string ns_;
    std::string topic_name_;
    double update_rate_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MobileObject)
}
