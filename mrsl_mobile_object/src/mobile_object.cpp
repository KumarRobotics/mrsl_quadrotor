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
      gazebo::math::Pose pose = link->GetWorldPose();

      if(!initialzed_){
        initialzed_ = true;
        init_pose_ = pose;
      }

      if((fabs(pose.pos.x - (init_pose_.pos.x  + lower_bound_)) < 1e-1 && vx_ < 0)
         || (fabs(pose.pos.x - (init_pose_.pos.x + upper_bound_)) < 1e-1 && vx_ > 0))
        vx_ = -vx_;

      if((fabs(pose.pos.y - (init_pose_.pos.y  + lower_bound_)) < 1e-1 && vy_ < 0)
         || (fabs(pose.pos.y - (init_pose_.pos.y + upper_bound_)) < 1e-1 && vy_ > 0))
        vy_ = -vy_;

      link->SetLinearVel(gazebo::math::Vector3(vx_, vy_, 0));
      link->SetAngularVel(gazebo::math::Vector3(0, 0, w_));

      //if (pub_.getNumSubscribers() > 0 && topic_name_ != "" && update_rate_ > 0)
      if (topic_name_ != "" && update_rate_ > 0)
      {
        static tf::TransformBroadcaster br;
        static ros::Time prev_time = ros::Time::now();

        if((ros::Time::now() - prev_time).toSec() >= 1./update_rate_) {
          geometry_msgs::PoseStamped msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "map";
          msg.pose.position.x = pose.pos.x;
          msg.pose.position.y = pose.pos.y;
          msg.pose.position.z = pose.pos.z;
          msg.pose.orientation.w = pose.rot.w;
          msg.pose.orientation.x = pose.rot.x;
          msg.pose.orientation.y = pose.rot.y;
          msg.pose.orientation.z = pose.rot.z;

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
    gazebo::math::Pose init_pose_;
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
