#include <mrsl_quadrotor_simulator/QuadrotorTRPYControl.h>

namespace mrsl_quadrotor_simulator {
void QuadrotorTRPYControl::init(const Quadrotor &quad, float I[][3]) {
  kf_ = quad.getPropellerThrustCoefficient();
  km_ = quad.getPropellerMomentCoefficient();
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      I_[i][j] = I[i][j];
  arm_length_ = quad.getArmLength();
}

void QuadrotorTRPYControl::cmdCallback(
    const QuadrotorTRPYControl::TRPYCmdMsg::ConstPtr &msg) {
  cmd = *msg;
}

Quadrotor::MotorState
QuadrotorTRPYControl::getControl(const Quadrotor::State &state) {
  if (!cmd)
    return Quadrotor::MotorState::Zero();

  const double kf = kf_;
  const double km = km_ / kf_ * kf;


  //const float I[3][3] = {{J_(0, 0), J_(0, 1), J_(0, 2)},
                         //{J_(1, 0), J_(1, 1), J_(1, 2)},
                         //{J_(2, 0), J_(2, 1), J_(2, 2)}};
  double yaw;
  yaw = state.ypr(0);
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(state.ypr(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollAngle(state.ypr(2), Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  const Eigen::Matrix3d R(q);

  const float R11 = R(0, 0);
  const float R12 = R(0, 1);
  const float R13 = R(0, 2);
  const float R21 = R(1, 0);
  const float R22 = R(1, 1);
  const float R23 = R(1, 2);
  const float R31 = R(2, 0);
  const float R32 = R(2, 1);
  const float R33 = R(2, 2);

  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  float Rd11 = cos(cmd->yaw) * cos(cmd->pitch);
  float Rd12 = cos(cmd->yaw) * sin(cmd->pitch) * sin(cmd->roll) -
               cos(cmd->roll) * sin(cmd->yaw);
  float Rd13 = sin(cmd->yaw) * sin(cmd->roll) +
               cos(cmd->yaw) * cos(cmd->roll) * sin(cmd->pitch);
  float Rd21 = cos(cmd->pitch) * sin(cmd->yaw);
  float Rd22 = cos(cmd->yaw) * cos(cmd->roll) +
               sin(cmd->yaw) * sin(cmd->pitch) * sin(cmd->roll);
  float Rd23 = cos(cmd->roll) * sin(cmd->yaw) * sin(cmd->pitch) -
               cos(cmd->yaw) * sin(cmd->roll);
  float Rd31 = -sin(cmd->pitch);
  float Rd32 = cos(cmd->pitch) * sin(cmd->roll);
  float Rd33 = cos(cmd->pitch) * cos(cmd->roll);

  float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                              Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                              Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  float force = 0;
  if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
    force = cmd->thrust;

  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 +
                      R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 -
                      R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 +
                      R31 * Rd32 - R32 * Rd31);

  float Omd1 =
      cmd->angular_velocity.x * (R11 * Rd11 + R21 * Rd21 + R31 * Rd31) +
      cmd->angular_velocity.y * (R11 * Rd12 + R21 * Rd22 + R31 * Rd32) +
      cmd->angular_velocity.z * (R11 * Rd13 + R21 * Rd23 + R31 * Rd33);
  float Omd2 =
      cmd->angular_velocity.x * (R12 * Rd11 + R22 * Rd21 + R32 * Rd31) +
      cmd->angular_velocity.y * (R12 * Rd12 + R22 * Rd22 + R32 * Rd32) +
      cmd->angular_velocity.z * (R12 * Rd13 + R22 * Rd23 + R32 * Rd33);
  float Omd3 =
      cmd->angular_velocity.x * (R13 * Rd11 + R23 * Rd21 + R33 * Rd31) +
      cmd->angular_velocity.y * (R13 * Rd12 + R23 * Rd22 + R33 * Rd32) +
      cmd->angular_velocity.z * (R13 * Rd13 + R23 * Rd23 + R33 * Rd33);

  float eOm1 = Om1 - Omd1;
  float eOm2 = Om2 - Omd2;
  float eOm3 = Om3 - Omd3;

#if 0
  float in1 = Om2 * (I_[2][0] * Om1 + I_[2][1] * Om2 + I_[2][2] * Om3) -
              Om3 * (I_[1][0] * Om1 + I_[1][1] * Om2 + I_[1][2] * Om3);
  float in2 = Om3 * (I_[0][0] * Om1 + I_[0][1] * Om2 + I_[0][2] * Om3) -
              Om1 * (I_[2][0] * Om1 + I_[2][1] * Om2 + I_[2][2] * Om3);
  float in3 = Om1 * (I_[1][0] * Om1 + I_[1][1] * Om2 + I_[1][2] * Om3) -
              Om2 * (I_[0][0] * Om1 + I_[0][1] * Om2 + I_[0][2] * Om3);
#else
  float in1 = Omd2 * (I_[2][0] * Omd1 + I_[2][1] * Omd2 + I_[2][2] * Omd3) -
              Omd3 * (I_[1][0] * Omd1 + I_[1][1] * Omd2 + I_[1][2] * Omd3);
  float in2 = Omd3 * (I_[0][0] * Omd1 + I_[0][1] * Omd2 + I_[0][2] * Omd3) -
              Omd1 * (I_[2][0] * Omd1 + I_[2][1] * Omd2 + I_[2][2] * Omd3);
  float in3 = Omd1 * (I_[1][0] * Omd1 + I_[1][1] * Omd2 + I_[1][2] * Omd3) -
              Omd2 * (I_[0][0] * Omd1 + I_[0][1] * Omd2 + I_[0][2] * Omd3);
#endif

  float M1 = -cmd->kR[0] * eR1 - cmd->kOm[0] * eOm1 + in1;
  float M2 = -cmd->kR[1] * eR2 - cmd->kOm[1] * eOm2 + in2;
  float M3 = -cmd->kR[2] * eR3 - cmd->kOm[2] * eOm3 + in3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * arm_length_ * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * arm_length_ * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * arm_length_ * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * arm_length_ * kf) - M3 / (4 * km);


  Quadrotor::MotorState control;
  for (int i = 0; i < 4; i++) {
    if(cmd->aux.enable_motors) {
      if (w_sq[i] < 0)
        w_sq[i] = 0;
      control[i] = std::sqrt(w_sq[i]);
    } else {
      control[i] = 0;
    }
  }

  return control;
}
}
