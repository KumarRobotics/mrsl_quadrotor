#ifndef QUADROTOR_TRPY_CONTROL_H_
#define QUADROTOR_TRPY_CONTROL_H_

#include <boost/optional.hpp>
#include <mrsl_quadrotor_simulator/Quadrotor.h>
#include <quadrotor_msgs/TRPYCommand.h>

namespace mrsl_quadrotor_simulator {
class QuadrotorTRPYControl {
public:
  typedef quadrotor_msgs::TRPYCommand TRPYCmdMsg;
  void init(const Quadrotor &quad, float I[][3]);
  Quadrotor::MotorState getControl(const Quadrotor::State &state);
  void cmdCallback(const TRPYCmdMsg::ConstPtr &msg);

private:
  float kf_;
  float km_;
  float arm_length_;
  float I_[3][3];
  boost::optional<TRPYCmdMsg> cmd;
};
}
#endif
