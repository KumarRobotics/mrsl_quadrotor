#include <mrsl_quadrotor_simulator/QuadrotorSO3AttitudeControl.h>
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

    cmd.force[0] = 0;                                                     
    cmd.force[1] = 0;                                                      
    cmd.force[2] = 0;                                                      
    cmd.qw = 1.0;                                                      
    cmd.qx = 0.0;                                                      
    cmd.qy = 0.0;                                                      
    cmd.qz = 0.0;                                                      
    cmd.angvel_x = 0.0;                                           
    cmd.angvel_y = 0.0;                                           
    cmd.angvel_z = 0.0;                                           
    cmd.kR[0] = 0.0;                                                           
    cmd.kR[1] = 0.0;                                                           
    cmd.kR[2] = 0.0;                                                           
    cmd.kOm[0] = 0.0;                                                         
    cmd.kOm[1] = 0.0;                                                         
    cmd.kOm[2] = 0.0;                                                         
    cmd.kf_correction = 0;                                       
  }


  void QuadrotorSO3AttitudeControl::cmdCallback(const QuadrotorSO3AttitudeControl::CmdMsg::ConstPtr &msg)
  {
    cmd.force[0] = msg->force.x;                                                      
    cmd.force[1] = msg->force.y;                                                      
    cmd.force[2] = msg->force.z;                                                      
    cmd.qw = msg->orientation.w;                                                      
    cmd.qx = msg->orientation.x;                                                      
    cmd.qy = msg->orientation.y;                                                      
    cmd.qz = msg->orientation.z;                                                      
    cmd.angvel_x = msg->angular_velocity.x;                                           
    cmd.angvel_y = msg->angular_velocity.y;                                           
    cmd.angvel_z = msg->angular_velocity.z;                                           
    cmd.kR[0] = msg->kR[0];                                                           
    cmd.kR[1] = msg->kR[1];                                                           
    cmd.kR[2] = msg->kR[2];                                                           
    cmd.kOm[0] = msg->kOm[0];                                                         
    cmd.kOm[1] = msg->kOm[1];                                                         
    cmd.kOm[2] = msg->kOm[2];                                                         
    cmd.kf_correction = msg->aux.kf_correction;                                       
  }

  Quadrotor::MotorState QuadrotorSO3AttitudeControl::getControl(const Quadrotor::State &state)
  {
    const float kf = kf_ - cmd.kf_correction;
    const float km = km_ / kf_ * kf;
    const float R11 = state.R(0,0);
    const float R12 = state.R(0,1);
    const float R13 = state.R(0,2);
    const float R21 = state.R(1,0);
    const float R22 = state.R(1,1);
    const float R23 = state.R(1,2);
    const float R31 = state.R(2,0);
    const float R32 = state.R(2,1);
    const float R33 = state.R(2,2);

    const float Om1 = state.omega(0);
    const float Om2 = state.omega(1);
    const float Om3 = state.omega(2);

    const float Rd11 = cmd.qw*cmd.qw + cmd.qx*cmd.qx - cmd.qy*cmd.qy - cmd.qz*cmd.qz;
    const float Rd12 = 2*(cmd.qx*cmd.qy - cmd.qw*cmd.qz);
    const float Rd13 = 2*(cmd.qx*cmd.qz + cmd.qw*cmd.qy);
    const float Rd21 = 2*(cmd.qx*cmd.qy + cmd.qw*cmd.qz);
    const float Rd22 = cmd.qw*cmd.qw - cmd.qx*cmd.qx + cmd.qy*cmd.qy - cmd.qz*cmd.qz;
    const float Rd23 = 2*(cmd.qy*cmd.qz - cmd.qw*cmd.qx);
    const float Rd31 = 2*(cmd.qx*cmd.qz - cmd.qw*cmd.qy);
    const float Rd32 = 2*(cmd.qy*cmd.qz + cmd.qw*cmd.qx);
    const float Rd33 = cmd.qw*cmd.qw - cmd.qx*cmd.qx - cmd.qy*cmd.qy + cmd.qz*cmd.qz;

    const float des_angvel_x = cmd.angvel_x;
    const float des_angvel_y = cmd.angvel_y;
    const float des_angvel_z = cmd.angvel_z;


    float Psi = 0.5f*(3.0f - (Rd11*R11 + Rd21*R21 + Rd31*R31 +
                              Rd12*R12 + Rd22*R22 + Rd32*R32 +
                              Rd13*R13 + Rd23*R23 + Rd33*R33));

    float force = 0;
    if(Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
      force = cmd.force[0]*R13 + cmd.force[1]*R23 + cmd.force[2]*R33;

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

    float M1 = -cmd.kR[0]*eR1 - cmd.kOm[0]*eOm1 + in1;
    float M2 = -cmd.kR[1]*eR2 - cmd.kOm[1]*eOm2 + in2;
    float M3 = -cmd.kR[2]*eR3 - cmd.kOm[2]*eOm3 + in3;

    float w_sq[4];
    w_sq[0] = force/(4*kf) - M2/(2*arm_length_*kf) + M3/(4*km);
    w_sq[1] = force/(4*kf) + M2/(2*arm_length_*kf) + M3/(4*km);
    w_sq[2] = force/(4*kf) + M1/(2*arm_length_*kf) - M3/(4*km);
    w_sq[3] = force/(4*kf) - M1/(2*arm_length_*kf) - M3/(4*km);

    Quadrotor::MotorState control;
    for(int i = 0; i < 4; i++)
    {
      if(w_sq[i] < 0)
        w_sq[i] = 0;

      control[i] = sqrtf(w_sq[i]);
    }
    return control;
  }


}
