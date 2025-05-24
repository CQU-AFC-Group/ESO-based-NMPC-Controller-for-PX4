#include <cmath>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

class NonlinearESO
{
public:
    // ESO配置参数结构体
    struct Config
    {
        double beta1;  // 位置误差增益
        double beta2;  // 速度误差增益
        double beta3;  // 扰动误差增益
        double alpha1; // 位置非线性因子 (0 < alpha < 1)
        double alpha2; // 速度非线性因子
        double alpha3; // 扰动非线性因子
        double delta;  // 线性区间阈值
        double mass;   // 系统质量
        double dt;     // 控制周期(秒)
        double gravity;
    };

    // 析构函数
    ~NonlinearESO() {};

    // 构造函数：初始化ESO参数和初始状态
    NonlinearESO(const Config &config)
        : config_(config),
          position_hat_{0, 0, 0},
          velocity_hat_{0, 0, 0},
          disturbance_hat_{0, 0, 0}
    {
        validateParameters();
    }

    // 构造函数：默认初始化参数
    NonlinearESO():
          position_hat_{0, 0, 0},
          velocity_hat_{0, 0, 0},
          disturbance_hat_{0, 0, 0}
    {
        const double omega = 20; // 观测器带宽(rad/s)
        config_.alpha1 = 3 * omega;
        config_.alpha2 = 3 * std::pow(omega, 2);
        config_.alpha3 = std::pow(omega, 3);
        config_.beta1 = 0.3;
        config_.beta2 = 0.3;
        config_.beta3 = 0.3;
        config_.delta = 0.1;
        config_.mass = 1.62;
        config_.dt = 0.01;
        config_.gravity = 9.81;
        validateParameters();
    }

    // 设置初始状态 (可选)
    void initialize(const double position[3],
                    const double velocity[3],
                    const double disturbance[3])
    {
        for (int i = 0; i < 3; ++i)
        {
            position_hat_[i] = position[i];
            velocity_hat_[i] = velocity[i];
            disturbance_hat_[i] = disturbance[i];
        }
    }

    /**
     * @brief ESO更新函数（输入为推力和欧拉角）
     * @param position_meas 测量位置 [x, y, z] (m)
     * @param thrust 总推力 (N)
     * @param euler 欧拉角 [roll, pitch, yaw] (rad)
     */
    void update(const double position_meas[3],
                const double thrust,
                const double euler[3])
    {
        // 1. 计算推力在世界坐标系的分量
        const double roll = euler[0];   // Roll
        const double pitch = euler[1]; // Pitch
        const double yaw = euler[2];   // Yaw

        // 计算三角函数值
        const double cr = cos(roll); // 横滚角余弦
        const double sr = sin(roll);  // 横滚角正弦
        const double cp = cos(pitch);  // 俯仰角余弦
        const double sp = sin(pitch);   // 俯仰角正弦
        const double cy = cos(yaw);   // 偏航角余弦
        const double sy = sin(yaw);   // 偏航角正弦

        // 构建ZYX顺序的旋转矩阵
        Eigen::Matrix3d R;
        R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr;

        const Eigen::Vector3d thrust_body(0.0, 0.0, thrust); // 机体坐标系推力
        const Eigen::Vector3d thrust_world = R * thrust_body;

        // 重力向量
        const Eigen::Vector3d gravity_force(0.0, 0.0, -config_.mass * config_.gravity);

        // 总控制输入力 = 推力 + 补偿重力
        const Eigen::Vector3d u_total = thrust_world + gravity_force;

        // 2. 对各轴独立更新ESO
        for (int axis = 0; axis < 3; ++axis)
        {
            const double e = position_meas[axis] - position_hat_[axis];

            // 非线性误差处理
            const double e1 = fal(e, config_.alpha1, config_.delta);
            const double e2 = fal(e, config_.alpha2, config_.delta);
            const double e3 = fal(e, config_.alpha3, config_.delta);

            // 状态更新 (欧拉离散化)
            position_hat_[axis] += config_.dt * (velocity_hat_[axis] + config_.beta1 * e1);

            velocity_hat_[axis] += config_.dt * ((u_total[axis] + disturbance_hat_[axis]) / config_.mass + config_.beta2 * e2);

            disturbance_hat_[axis] += config_.dt * (config_.beta3 * e3);
        }
    }
    // 获取估计状态
    void getPositionEstimate(double out[3]) const
    {
        std::copy(position_hat_, position_hat_ + 3, out);
    }

    void getVelocityEstimate(double out[3]) const
    {
        std::copy(velocity_hat_, velocity_hat_ + 3, out);
    }

    void getDisturbanceEstimate(double out[3]) const
    {
        std::copy(disturbance_hat_, disturbance_hat_ + 3, out);
    }

private:
    Config config_;             // 观测器参数
    double position_hat_[3];    // 估计位置 [x,y,z]
    double velocity_hat_[3];    // 估计速度 [vx,vy,vz]
    double disturbance_hat_[3]; // 估计扰动 [dx,dy,dz]

    // 参数有效性检查
    void validateParameters() const
    {
        if (config_.dt <= 0)
            throw std::invalid_argument("dt must > 0");
        if (config_.mass <= 0)
            throw std::invalid_argument("mass must > 0");
        if (config_.alpha1 <= 0 || config_.alpha1 >= 1 ||
            config_.alpha2 <= 0 || config_.alpha2 >= 1 ||
            config_.alpha3 <= 0 || config_.alpha3 >= 1)
        {
            throw std::invalid_argument("alpha must in (0,1)");
        }
    }

    // 非线性函数实现
    static double fal(double e, double alpha, double delta)
    {
        const double abs_e = std::abs(e);
        if (abs_e > delta)
        {
            return std::copysign(std::pow(abs_e, alpha), e);
        }
        else
        {
            return e / std::pow(delta, 1.0 - alpha);
        }
    }
};

