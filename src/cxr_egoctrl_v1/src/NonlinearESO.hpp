#include <cmath>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

class NonlinearESO
{
public:
    // ESO配置参数结构体（分离水平和高度参数）
    struct Config
    {
        // 水平通道参数
        double beta1_xy;  // 水平位置误差增益
        double beta2_xy;  // 水平速度误差增益
        double beta3_xy;  // 水平扰动误差增益
        double alpha1_xy; // 水平位置非线性因子
        double alpha2_xy; // 水平速度非线性因子
        double alpha3_xy; // 水平扰动非线性因子

        // 高度通道参数
        double beta1_z;  // 高度位置误差增益
        double beta2_z;  // 高度速度误差增益
        double beta3_z;  // 高度扰动误差增益
        double alpha1_z; // 高度位置非线性因子
        double alpha2_z; // 高度速度非线性因子
        double alpha3_z; // 高度扰动非线性因子

        // 公共参数
        double delta; // 线性区间阈值
        double mass;  // 系统质量
        double dt;    // 控制周期(秒)
        double gravity;
    };

    // 析构函数
    ~NonlinearESO() = default;

    // 构造函数：自定义参数初始化
    NonlinearESO(const Config &config)
        : config_(config),
          position_hat_{0, 0, 0},
          velocity_hat_{0, 0, 0},
          disturbance_hat_{0, 0, 0}
    {
        validateParameters();
    }

    // 构造函数：默认参数初始化（分离水平/高度参数）
    NonlinearESO()
    {
        // 水平通道参数
        const double omega_xy = 5; // 水平观测带宽
        config_.beta1_xy = 3 * omega_xy;
        config_.beta2_xy = 3 * std::pow(omega_xy, 2);
        config_.beta3_xy = std::pow(omega_xy, 3);
        config_.alpha1_xy = 0.6;
        config_.alpha2_xy = 0.6;
        config_.alpha3_xy = 0.6;

        // 高度通道参数
        const double omega_z = 6.5; // 高度观测带宽
        config_.beta1_z = 3 * omega_z;
        config_.beta2_z = 3 * std::pow(omega_z, 2);
        config_.beta3_z = std::pow(omega_z, 3);
        config_.alpha1_z = 0.5;
        config_.alpha2_z = 0.5;
        config_.alpha3_z = 0.5;

        // 公共参数
        config_.delta = 0.1;
        config_.mass = 2.3;
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

    // 更新函数（示例保留一个版本，实际可根据需要保留多个重载）
    void update(const Eigen::Vector3d &position_meas,
                double thrust,
                const Eigen::Vector3d &euler)
    {
        // 1. 计算推力在世界坐标系的分量
        const double roll = euler(0);  // Roll
        const double pitch = euler(1); // Pitch
        const double yaw = euler(2);   // Yaw

        // 计算三角函数值
        const double cr = cos(roll);  // 横滚角余弦
        const double sr = sin(roll);  // 横滚角正弦
        const double cp = cos(pitch); // 俯仰角余弦
        const double sp = sin(pitch); // 俯仰角正弦
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

        // 分通道更新
        for (int axis = 0; axis < 3; ++axis)
        {
            const bool is_z_axis = (axis == 2); // 判断当前轴

            // 选择对应通道参数
            const double beta1 = is_z_axis ? config_.beta1_z : config_.beta1_xy;
            const double beta2 = is_z_axis ? config_.beta2_z : config_.beta2_xy;
            const double beta3 = is_z_axis ? config_.beta3_z : config_.beta3_xy;
            const double alpha1 = is_z_axis ? config_.alpha1_z : config_.alpha1_xy;
            const double alpha2 = is_z_axis ? config_.alpha2_z : config_.alpha2_xy;
            const double alpha3 = is_z_axis ? config_.alpha3_z : config_.alpha3_xy;

            const double e = position_meas(axis) - position_hat_[axis];

            // 非线性误差处理
            const double e1 = fal(e, alpha1, config_.delta);
            const double e2 = fal(e, alpha2, config_.delta);
            const double e3 = fal(e, alpha3, config_.delta);

            // 状态更新
            position_hat_[axis] += config_.dt * (velocity_hat_[axis] + beta1 * e1);
            velocity_hat_[axis] += config_.dt * ((u_total[axis] + disturbance_hat_[axis]) / config_.mass + beta2 * e2);
            disturbance_hat_[axis] += config_.dt * beta3 * e3;
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
    Config config_;
    double position_hat_[3];
    double velocity_hat_[3];
    double disturbance_hat_[3];

    // 参数验证（扩展参数检查）
    void validateParameters() const
    {
        if (config_.dt <= 0)
            throw std::invalid_argument("dt must > 0");
        if (config_.mass <= 0)
            throw std::invalid_argument("mass must > 0");

        // 检查水平通道参数
        auto check_alpha = [](double a, const char *name)
        {
            if (a <= 0 || a >= 1)
                throw std::invalid_argument(name + std::string(" alpha must in (0,1)"));
        };

        check_alpha(config_.alpha1_xy, "alpha1_xy");
        check_alpha(config_.alpha2_xy, "alpha2_xy");
        check_alpha(config_.alpha3_xy, "alpha3_xy");
        check_alpha(config_.alpha1_z, "alpha1_z");
        check_alpha(config_.alpha2_z, "alpha2_z");
        check_alpha(config_.alpha3_z, "alpha3_z");
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