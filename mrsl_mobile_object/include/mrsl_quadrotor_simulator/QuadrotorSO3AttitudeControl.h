#ifndef QUADROTOR_ATTITUDE_CONTROL_H_
#define QUADROTOR_ATTITUDE_CONTROL_H_
#include <mrsl_quadrotor_simulator/Quadrotor.h>
#include <quadrotor_msgs/SO3Command.h>

namespace mrsl_quadrotor_simulator {
class QuadrotorSO3AttitudeControl {
public:
  typedef quadrotor_msgs::SO3Command CmdMsg;
  void init(const Quadrotor &quad, float I[][3]);
  Quadrotor::MotorState getControl(const Quadrotor::State &state);
  void cmdCallback(const CmdMsg::ConstPtr &msg);

private:
  float kf_;
  float km_;
  float arm_length_;
  float I_[3][3];
  boost::optional<CmdMsg> cmd;
};
}
#endif
