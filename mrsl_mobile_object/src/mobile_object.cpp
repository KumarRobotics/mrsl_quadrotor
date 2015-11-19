#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <stdio.h>
 
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
      if(_sdf->HasElement("vx"))
        vx_ = _sdf->GetElement("vx")->Get<double>();
      if(_sdf->HasElement("vy"))
        vy_ = _sdf->GetElement("vy")->Get<double>();

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
      link->SetAngularVel(gazebo::math::Vector3(0, 0, 0));
    }
 
   private:
    // Pointer to the model
    gazebo::math::Pose init_pose_;
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::physics::LinkPtr link; // Link for robot
    bool initialzed_;
    double lower_bound_;
    double upper_bound_;
    double vx_, vy_;
  };
 
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MobileObject)
}
