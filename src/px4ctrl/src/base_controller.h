/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __BASE_CONTROLLER_H
#define __BASE_CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

    // c++的结构体与类相似，也可以定义函数，构造函数，析构函数，只是成员的访问类型不同
	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class BaseController
{
public:
  BaseController(Parameter_t &);
  virtual ~BaseController() = default;
  virtual quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                        const Odom_Data_t &odom,
                                                        const Imu_Data_t &imu, 
                                                        Controller_Output_t &u) = 0;
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
                            const Parameter_t &param);
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
