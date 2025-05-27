#ifndef __MPCCONTROLLER_H
#define __MPCCONTROLLER_H

#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <queue>
#include "input.h"
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <stdio.h>
#include "NonlinearESO.hpp"

// 定义模型函数指针类型
using ModelPtr = casadi::SX (*)(const casadi::SX &, const casadi::SX &);

struct Desired_State_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;

    Desired_State_t() {};

    Desired_State_t(Odom_Data_t &odom)
        : p(odom.p),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
          yaw_rate(0) {};
};

struct Controller_Output_t
{

    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;

    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]

    // Collective mass normalized thrust
    double thrust;

    // Eigen::Vector3d des_v_real;
};

class MPCController
{
private:
    // 控制器参数
    struct
    {
        int horizon; // 预测时域长度
        double dt;   // 采样时间

        // 权重矩阵
        Eigen::Matrix<double, 3, 3> Q_p; // 位置误差权重
        Eigen::Matrix<double, 3, 3> Q_v; // 速度误差权重
        Eigen::Matrix<double, 4, 4> R;   // 控制输入权重

        // 四旋翼物理参数
        double mass;         // 质量 [kg]
        double gravity;      // 重力加速度 [m/s^2]
        double thrust_limit; // 最大推力 [N]
    } param_;
    
    // 状态向量: [位置(3), 速度(3)]
    Eigen::Matrix<double, 6, 1> state_;

    // 控制输入: [总推力(1), 姿态角(3)]
    Eigen::Matrix<double, 4, 1> control_;

    // 没有使用
    ModelPtr model;

    // CasADi优化器
    casadi::Function solver_;
    bool solver_initialized_;

    // 调试信息
    quadrotor_msgs::Px4ctrlDebug debug_msg_;

    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

    // Thrust-accel mapping params
    const double rho2_ = 0.998; // do not change
    double thr2acc_;
    double P_;   

    double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
    double fromQuaternion2yaw(Eigen::Quaterniond q);

public:
    
    // 构造函数
    MPCController();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 将Eigen矩阵转换为CasADi SX矩阵
    template <typename Derived>
    casadi::SX eigenToCasadi(const Eigen::MatrixBase<Derived> &mat);
    
    // 初始化非线性MPC求解器
    void initializeSolver();

    // 初始化扰动模型下的MPC求解器
    void initializeCompleteSolver();

    // 四旋翼位移四元数模型
    casadi::SX nonlinearQuadrotorTranslationQuaternionModel(const casadi::SX &x, const casadi::SX &u);

    casadi::SX eulerAnglesToRotationMatrix(const casadi::SX &phi,
        const casadi::SX &theta,
        const casadi::SX &psi);

    // 四旋翼位移欧拉角模型
    casadi::SX nonlinearQuadrotorTranslationEulerModel(const casadi::SX &x, const casadi::SX &u);

    // 四旋翼位移欧拉角扰动模型
    casadi::SX nonlinearQuadrotorTranslationEulerDisturbanceModel(const casadi::SX &x, const casadi::SX &u, const casadi::SX &d);

    // 四元数乘法
    casadi::SX quaternionMultiply(const casadi::SX &q1, const casadi::SX &q2);

    // 四元数求逆
    casadi::SX quaternionInverse(const casadi::SX &q);

    // 四元数转旋转矩阵
    casadi::SX quaternionToRotationMatrix(const casadi::SX &q);

    // 计算期望的姿态四元数
    Eigen::Quaterniond computeDesiredAttitude(const Eigen::Vector3d &des_acc, double des_yaw);

    // 主控制计算函数
    quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                  const Odom_Data_t &odom,
                                                  const Imu_Data_t &imu,
                                                  Controller_Output_t &u);

    quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                  const Odom_Data_t &odom,
                                                  const Imu_Data_t &imu,
                                                  Controller_Output_t &u,
                                                  NonlinearESO &observer);

    bool estimateThrustModel(const Eigen::Vector3d &est_a,
                             const Parameter_t &param);

    void resetThrustMapping(void);                                              
};

#endif 