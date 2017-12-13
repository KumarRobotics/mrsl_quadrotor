#ifndef QUADROTOR_SIMULATOR_QUADROTOR_H_
#define QUADROTOR_SIMULATOR_QUADROTOR_H_
#include <iostream>
#include <boost/bind.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

namespace mrsl_quadrotor_simulator {
class Quadrotor {
public:
  typedef Eigen::Array4d MotorState;
  typedef Eigen::Array4d Wrench;

  struct State {
    Eigen::Vector3d ypr;
    Eigen::Vector3d omega;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  Quadrotor();
  ~Quadrotor();

  const Quadrotor::MotorState &getMotorState(void) const;
  const Eigen::Matrix3d &getInertia(void) const;
  void setMotorState(const Quadrotor::MotorState &state);

  double getArmLength(void) const;
  double getPropRadius(void) const;
  double getPropellerThrustCoefficient(void) const;
  double getPropellerMomentCoefficient(void) const;
  double getMotorTimeConstant(void) const;
  double getMaxRPM(void) const;
  double getMinRPM(void) const;

  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  Wrench update(MotorState des_motor_rpm, double dt);

private:
  double kf_;
  double km_;
  double prop_radius_;
  double arm_length_;
  double motor_time_constant_; // unit: sec
  double max_rpm_;
  double min_rpm_;
  Eigen::Matrix3d J_;

  MotorState curr_motor_rpm_;
};
}
#endif
