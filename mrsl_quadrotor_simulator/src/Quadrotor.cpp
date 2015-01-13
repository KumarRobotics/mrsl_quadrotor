#include <mrsl_quadrotor_simulator/Quadrotor.h>

namespace mrsl_quadrotor_simulator
{
  Quadrotor::Quadrotor(void)
  {
    double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
    prop_radius_ = 0.099;
    J_ = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

    kf_ = 5.55e-8;
    // km_ = 2.5e-9; // from Nate
    // km = (Cq/Ct)*Dia*kf
    // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
    km_ = 0.07 * ( 2 * prop_radius_) * kf_;

    arm_length_ = 0.17;
    motor_time_constant_ = 1.0/20;
    min_rpm_ = 1500;
    max_rpm_ = 7500;

    curr_motor_rpm_ = Quadrotor::MotorState::Zero();
  }
  Quadrotor::~Quadrotor()
  {
  }

  Quadrotor::Wrench Quadrotor::update(Quadrotor::MotorState des_motor_rpm, double dt)
  {
    for(int i = 0; i < 4; i++)
    {
      if(des_motor_rpm(i) > max_rpm_)
        des_motor_rpm(i) = max_rpm_;
      else if(des_motor_rpm(i) < min_rpm_)
        des_motor_rpm(i) = min_rpm_;
    }
    Quadrotor::MotorState motor_rpm_dot = (des_motor_rpm - curr_motor_rpm_)/motor_time_constant_;
    curr_motor_rpm_ += dt * motor_rpm_dot;
    Quadrotor::MotorState motor_rpm_sq = curr_motor_rpm_.square();

    Quadrotor::Wrench output; // thrust, moments
    output(0) = kf_ * motor_rpm_sq.sum();
    output(1) = kf_*(motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
    output(2) = kf_*(motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
    output(3) = km_*(motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) - motor_rpm_sq(3));
    return output;
  }

  const Quadrotor::MotorState &Quadrotor::getMotorState(void) const
  {
    return curr_motor_rpm_;
  }

  void Quadrotor::setMotorState(const Quadrotor::MotorState &motor_rpm)
  {
    curr_motor_rpm_ = motor_rpm;
  }

  double Quadrotor::getArmLength(void) const
  {
    return arm_length_;
  }
  double Quadrotor::getPropRadius(void) const
  {
    return prop_radius_;
  }
  double Quadrotor::getPropellerThrustCoefficient(void) const
  {
    return kf_;
  }
  double Quadrotor::getPropellerMomentCoefficient(void) const
  {
    return km_;
  }
  double Quadrotor::getMotorTimeConstant(void) const
  {
    return motor_time_constant_;
  }
  double Quadrotor::getMaxRPM(void) const
  {
    return max_rpm_;
  }
  double Quadrotor::getMinRPM(void) const
  {
    return min_rpm_;
  }
  const Eigen::Matrix3d &Quadrotor::getInertia(void) const
  {                                  
    return J_;                       
  }         
}
