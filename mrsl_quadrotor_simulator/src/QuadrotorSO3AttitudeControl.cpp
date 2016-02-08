#include <mrsl_quadrotor_simulator/QuadrotorSO3AttitudeControl.h>
#include <stdio.h>
namespace mrsl_quadrotor_simulator
{
  void QuadrotorSO3AttitudeControl::init(const Quadrotor &quad, float I[][3])
  {
    kf_ = quad.getPropellerThrustCoefficient();
    km_ = quad.getPropellerMomentCoefficient();
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        I_[i][j] = I[i][j];
    arm_length_ = quad.getArmLength();
  }

  void QuadrotorSO3AttitudeControl::cmdCallback(const QuadrotorSO3AttitudeControl::CmdMsg::ConstPtr &msg)
  {
    cmd = *msg;
  }

  Quadrotor::MotorState QuadrotorSO3AttitudeControl::getControl(const Quadrotor::State &state)
  {
    Quadrotor::MotorState control;
    if(!cmd)
      return control;

    const float kf = kf_ - cmd->aux.kf_correction;
    const float km = km_ / kf_ * kf;

    double yaw;
    if(cmd->aux.use_external_yaw)
      yaw = cmd->aux.current_yaw;
    else
      yaw = state.ypr(0);
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(state.ypr(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(state.ypr(2), Eigen::Vector3d::UnitX());

    const Eigen::Matrix3d R = yawAngle * pitchAngle * rollAngle;

    const float R11 = R(0,0);
    const float R12 = R(0,1);
    const float R13 = R(0,2);
    const float R21 = R(1,0);
    const float R22 = R(1,1);
    const float R23 = R(1,2);
    const float R31 = R(2,0);
    const float R32 = R(2,1);
    const float R33 = R(2,2);

    const float Om1 = state.omega(0);
    const float Om2 = state.omega(1);
    const float Om3 = state.omega(2);

    Eigen::Quaterniond q_des(cmd->orientation.w,
                             cmd->orientation.x,
                             cmd->orientation.y,
                             cmd->orientation.z);

    Eigen::Matrix3d Rd(q_des);

    const float Rd11 = Rd(0,0);
    const float Rd12 = Rd(0,1);
    const float Rd13 = Rd(0,2);
    const float Rd21 = Rd(1,0);
    const float Rd22 = Rd(1,1);
    const float Rd23 = Rd(1,2);
    const float Rd31 = Rd(2,0);
    const float Rd32 = Rd(2,1);
    const float Rd33 = Rd(2,2);

   
    const float des_angvel_x = cmd->angular_velocity.x;
    const float des_angvel_y = cmd->angular_velocity.y;
    const float des_angvel_z = cmd->angular_velocity.z;


    float Psi = 0.5f*(3.0f - (Rd11*R11 + Rd21*R21 + Rd31*R31 +
                              Rd12*R12 + Rd22*R22 + Rd32*R32 +
                              Rd13*R13 + Rd23*R23 + Rd33*R33));

    float force = 0;
    if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
      force = cmd->force.x * R13 + cmd->force.y * R23 + cmd->force.z * R33;

    float eR1 = 0.5f*(R12*Rd13 - R13*Rd12 + R22*Rd23 - R23*Rd22 + R32*Rd33 - R33*Rd32);
    float eR2 = 0.5f*(R13*Rd11 - R11*Rd13 - R21*Rd23 + R23*Rd21 - R31*Rd33 + R33*Rd31);
    float eR3 = 0.5f*(R11*Rd12 - R12*Rd11 + R21*Rd22 - R22*Rd21 + R31*Rd32 - R32*Rd31);

    // Omd = R.transpose()*Rd*des_ang_vel;
    const float Omd1 = des_angvel_x*(R11*Rd11 + R21*Rd21 + R31*Rd31) + des_angvel_y*(R11*Rd12 + R21*Rd22 + R31*Rd32) +
      des_angvel_z*(R11*Rd13 + R21*Rd23 + R31*Rd33);
    const float Omd2 = des_angvel_x*(R12*Rd11 + R22*Rd21 + R32*Rd31) + des_angvel_y*(R12*Rd12 + R22*Rd22 + R32*Rd32) +
      des_angvel_z*(R12*Rd13 + R22*Rd23 + R32*Rd33);
    const float Omd3 = des_angvel_x*(R13*Rd11 + R23*Rd21 + R33*Rd31) + des_angvel_y*(R13*Rd12 + R23*Rd22 + R33*Rd32) +
      des_angvel_z*(R13*Rd13 + R23*Rd23 + R33*Rd33);
    const float eOm1 = Om1 - Omd1;
    const float eOm2 = Om2 - Omd2;
    const float eOm3 = Om3 - Omd3;

    float in1 = Om2*(I_[2][0]*Om1 + I_[2][1]*Om2 + I_[2][2]*Om3) - Om3*(I_[1][0]*Om1 + I_[1][1]*Om2 + I_[1][2]*Om3);
    float in2 = Om3*(I_[0][0]*Om1 + I_[0][1]*Om2 + I_[0][2]*Om3) - Om1*(I_[2][0]*Om1 + I_[2][1]*Om2 + I_[2][2]*Om3);
    float in3 = Om1*(I_[1][0]*Om1 + I_[1][1]*Om2 + I_[1][2]*Om3) - Om2*(I_[0][0]*Om1 + I_[0][1]*Om2 + I_[0][2]*Om3);

    float M1 = -cmd->kR[0]*eR1 - cmd->kOm[0]*eOm1 + in1;
    float M2 = -cmd->kR[1]*eR2 - cmd->kOm[1]*eOm2 + in2;
    float M3 = -cmd->kR[2]*eR3 - cmd->kOm[2]*eOm3 + in3;

    float w_sq[4];
    w_sq[0] = force/(4*kf) - M2/(2*arm_length_*kf) + M3/(4*km);
    w_sq[1] = force/(4*kf) + M2/(2*arm_length_*kf) + M3/(4*km);
    w_sq[2] = force/(4*kf) + M1/(2*arm_length_*kf) - M3/(4*km);
    w_sq[3] = force/(4*kf) - M1/(2*arm_length_*kf) - M3/(4*km);

    for(int i = 0; i < 4; i++)
    {
      //if(cmd->aux.enable_motors)
      if(1)
      {
        if(w_sq[i] < 0)
          w_sq[i] = 0;
        control[i] = sqrtf(w_sq[i]);
      }
      else
      {
        control[i] = 0;
      }
    }

    return control;
  }


}
