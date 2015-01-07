#ifndef QUADROTOR_ATTITUDE_CONTROL_H_
#define QUADROTOR_ATTITUDE_CONTROL_H_
#include <mrsl_quadrotor_simulator/Quadrotor.h> 
#include <quadrotor_msgs/SO3Command.h>

namespace mrsl_quadrotor_simulator
{
  class QuadrotorSO3AttitudeControl
  {
   public:
    typedef quadrotor_msgs::SO3Command CmdMsg;
    struct Command
    {
      float force[3]; 
      float qx, qy, qz, qw; 
      float angvel_x, angvel_y, angvel_z; 
      float kR[3]; 
      float kOm[3]; 
      float kf_correction;
    };

    void init(const Quadrotor &quad, float I[][3]); 
    Quadrotor::MotorState getControl(const Quadrotor::State &state);
    void cmdCallback(const CmdMsg::ConstPtr &msg);
   private:
    float kf_;
    float km_;
    float arm_length_;
    float I_[3][3];
    Command cmd;
  };
}
#endif
