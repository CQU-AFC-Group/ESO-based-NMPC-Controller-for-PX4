#include "MPCController.h"

// 构造函数
MPCController::MPCController() : solver_initialized_(false)
{

    // 初始化参数
    param_.horizon = 15;
    param_.dt = 0.01;
    // 设置权重矩阵
    // 水平通道
    param_.Q_p = Eigen::Matrix<double, 3, 3>::Identity() * 40;      
    param_.Q_v = Eigen::Matrix<double, 3, 3>::Identity() * 2.5;     
    param_.R = Eigen::Matrix<double, 4, 4>::Identity() * 10;
    // 垂直通道
    param_.Q_p(2, 2) = 100;
    param_.Q_v(2, 2) = 0.75;
    param_.R(0, 0) = 0.002;

    param_.mass = 1.62;
    param_.gravity = 9.81;
    // param_.thrust_limit = 30.0;
    // 初始化求解器
    initializeSolver();
}    

double MPCController::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc)
{
    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

double MPCController::fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}

bool MPCController::estimateThrustModel(const Eigen::Vector3d &est_a,const Parameter_t &param)
{
    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1)
    {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        if (time_passed > 0.045) // 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        // fflush(stdout);

        // debug_msg_.thr2acc = thr2acc_;
        return true;
    }
    return false;
}

void MPCController::resetThrustMapping(void)
{
    thr2acc_ = param_.gravity / 0.5;
    P_ = 1e6;
}

template <typename Derived>
casadi::SX MPCController::eigenToCasadi(const Eigen::MatrixBase<Derived> &mat)
{
    casadi::SX result = casadi::SX::zeros(mat.rows(), mat.cols());
    for (int i = 0; i < mat.rows(); ++i)
    {
        for (int j = 0; j < mat.cols(); ++j)
        {
            result(i, j) = mat(i, j);
        }
    }
    return result;
}

// 四元数乘法
casadi::SX MPCController::quaternionMultiply(const casadi::SX &q1, const casadi::SX &q2)
{
    casadi::SX w1 = q1(0);
    casadi::SX x1 = q1(1);
    casadi::SX y1 = q1(2);
    casadi::SX z1 = q1(3);

    casadi::SX w2 = q2(0);
    casadi::SX x2 = q2(1);
    casadi::SX y2 = q2(2);
    casadi::SX z2 = q2(3);

    casadi::SX w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    casadi::SX x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    casadi::SX y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    casadi::SX z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

    return casadi::SX::vertcat({w, x, y, z});
}

// 四元数求逆
casadi::SX MPCController::quaternionInverse(const casadi::SX &q)
{
    return casadi::SX::vertcat({q(0), -q(1), -q(2), -q(3)});
}

// 四元数转旋转矩阵
casadi::SX MPCController::quaternionToRotationMatrix(const casadi::SX &q)
{
    casadi::SX w = q(0);
    casadi::SX x = q(1);
    casadi::SX y = q(2);
    casadi::SX z = q(3);

    casadi::SX R = casadi::SX::zeros(3, 3);

    R(0, 0) = 1 - 2 * y * y - 2 * z * z;
    R(0, 1) = 2 * x * y - 2 * w * z;
    R(0, 2) = 2 * x * z + 2 * w * y;

    R(1, 0) = 2 * x * y + 2 * w * z;
    R(1, 1) = 1 - 2 * x * x - 2 * z * z;
    R(1, 2) = 2 * y * z - 2 * w * x;

    R(2, 0) = 2 * x * z - 2 * w * y;
    R(2, 1) = 2 * y * z + 2 * w * x;
    R(2, 2) = 1 - 2 * x * x - 2 * y * y;

    return R;
}

// 计算期望的姿态四元数
Eigen::Quaterniond MPCController::computeDesiredAttitude(const Eigen::Vector3d &des_acc, double des_yaw)
{
    // 期望的Z轴方向（推力方向）
    Eigen::Vector3d zd = des_acc.normalized();

    // 计算期望的X轴方向（垂直于Z轴且朝向期望偏航方向）
    Eigen::Vector3d xc(Eigen::Vector3d::UnitX());
    if (fabs(zd.dot(Eigen::Vector3d::UnitX())) > 0.95)
    {
        xc = Eigen::Vector3d::UnitY();
    }
    Eigen::Vector3d yd = zd.cross(xc).normalized();
    Eigen::Vector3d xd = yd.cross(zd).normalized();

    // 构建旋转矩阵
    Eigen::Matrix3d R;
    R.col(0) = xd;
    R.col(1) = yd;
    R.col(2) = zd;

    // 转换为四元数
    return Eigen::Quaterniond(R);
}

void MPCController::initializeSolver()
{
    // 状态变量: [位置, 速度]
    casadi::SX x = casadi::SX::sym("x", 6);
    // 控制输入: [总推力, 欧拉角]
    casadi::SX u = casadi::SX::sym("u", 4);

    // 四旋翼模型
    casadi::SX x_next = nonlinearQuadrotorEulerModel(x, u);

    // 定义离散时间动态函数
    casadi::Function f("f", {x, u}, {x_next}, {"x", "u"}, {"x_next"});

    // 定义优化问题变量
    casadi::SX U = casadi::SX::sym("U", 4, param_.horizon); // 控制序列
    casadi::SX X0 = casadi::SX::sym("X0", 6);               // 初始状态（参数）
    casadi::SX X_ref = casadi::SX::sym("X_ref", 6);         // 参考状态（参数）
    casadi::SX thr2acc = casadi::SX::sym("thr2acc", 1);
    // 目标函数
    casadi::SX obj = 0;

    // 约束条件
    std::vector<casadi::SX> g;

    // 初始状态
    casadi::SX x_current = X0;

    // 转换 Eigen 权重矩阵为 CasADi 格式
    casadi::SX Q_p_casadi = eigenToCasadi(param_.Q_p);
    casadi::SX Q_v_casadi = eigenToCasadi(param_.Q_v);
    casadi::SX R_casadi = eigenToCasadi(param_.R);
    casadi::SX u_k_last = casadi::SX::zeros(4);
    // 构建目标函数和约束
    for (int k = 0; k < param_.horizon; ++k)
    {
        // 当前控制输入
        casadi::SX u_k = U(casadi::Slice(), k);

        // 计算下一时刻状态
        casadi::SXDict args = {{"x", x_current}, {"u", u_k}};
        casadi::SX x_next = f(args).at("x_next");

        // 位置和速度误差
        casadi::SX e_p = x_current(casadi::Slice(0, 3)) - X_ref(casadi::Slice(0, 3));
        casadi::SX e_v = x_current(casadi::Slice(3, 6)) - X_ref(casadi::Slice(3, 6));

        // 目标函数
        obj += casadi::SX::mtimes(e_p.T(), casadi::SX::mtimes(Q_p_casadi, e_p));
        obj += casadi::SX::mtimes(e_v.T(), casadi::SX::mtimes(Q_v_casadi, e_v));
        obj += casadi::SX::mtimes(u_k.T(), casadi::SX::mtimes(R_casadi, u_k));
        obj += -u_k(0) * u_k(0) * R_casadi(0) + (u_k(0) - param_.mass * param_.gravity) * (u_k(0) - param_.mass * param_.gravity) * R_casadi(0);

        // 控制约束
        g.push_back(u_k(0) / param_.mass / thr2acc); // 推力
        g.push_back(u_k(1)); // roll
        g.push_back(u_k(2)); // pitch
        g.push_back(u_k(3)); // yaw
        // g.push_back(u_k(2) * u_k(2) + u_k(3) * u_k(3) + u_k(4) * u_k(4) - 1);
        if (k > 0){
            g.push_back(casadi::SX::abs(u_k_last(0) - u_k(0)));  // dot_thrust
            g.push_back(casadi::SX::abs(u_k_last(1) - u_k(1)));  // dot_phi
            g.push_back(casadi::SX::abs(u_k_last(2) - u_k(2)));  // dot_theta
        }

        u_k_last = u_k;
        // 更新当前状态
        x_current = x_next;
    }

    // 定义优化问题
    casadi::SXDict nlp = {
        {"x", casadi::SX::reshape(U, 4 * param_.horizon, 1)}, // 决策变量仅包含 U
        {"p", casadi::SX::vertcat({X0, X_ref, thr2acc})},              // 参数
        {"f", obj},
        {"g", casadi::SX::vertcat(g)}};

    // 设置求解器选项
    casadi::Dict solver_opts;
    solver_opts["ipopt.tol"] = 1e-5;
    solver_opts["ipopt.max_iter"] = 100;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;

    // 创建求解器
    solver_ = casadi::nlpsol("solver", "ipopt", nlp, solver_opts);
    solver_initialized_ = true;
}

casadi::SX MPCController::nonlinearQuadrotorTranslationQuaternionModel(const casadi::SX &x, const casadi::SX &u)
{
    // 提取状态变量
    casadi::SX p = x(casadi::Slice(0, 3)); // 位置
    casadi::SX v = x(casadi::Slice(3, 6)); // 速度

    // 提取控制输入
    casadi::SX thrust = u(0);              // 总推力
    casadi::SX q = u(casadi::Slice(1, 5)); // 四元数 [qw, qx, qy, qz]

    // 重力向量（明确为3x1列向量）
    casadi::SX g = casadi::SX::zeros(3, 1);
    g(2, 0) = -param_.gravity;

    // 旋转矩阵 (机体到惯性)
    casadi::SX R = quaternionToRotationMatrix(q);

    // 调试输出
    // std::cout << "R shape: " << R.size1() << "x" << R.size2() << std::endl;

    // 机体坐标系下的推力向量（明确为3x1列向量）
    casadi::SX F_body = casadi::SX::zeros(3, 1);
    F_body(2, 0) = thrust;

    // 调试输出
    // std::cout << "F_body shape: " << F_body.size1() << "x" << F_body.size2() << std::endl;

    // 将推力转换到惯性坐标系
    casadi::SX F_inertial = casadi::SX::mtimes(R, F_body);

    // 调试输
    // std::cout << "F_inertial shape: " << F_inertial.size1() << "x" << F_inertial.size2() << std::endl;

    // 位置导数（速度）
    casadi::SX p_dot = v;

    // 速度导数（加速度）
    casadi::SX v_dot = F_inertial / param_.mass + g;

    // 返回状态导数（离散化）
    casadi::SX x_next = casadi::SX::vertcat({p + param_.dt * p_dot,
                                             v + param_.dt * v_dot});

    return x_next;
}

casadi::SX MPCController::nonlinearQuadrotorEulerModel(const casadi::SX &x, const casadi::SX &u)
{
    // 提取状态变量
    casadi::SX p = x(casadi::Slice(0, 3)); // 位置
    casadi::SX v = x(casadi::Slice(3, 6)); // 速度

    // 提取控制输入
    casadi::SX thrust = u(0); // 总推力
    casadi::SX phi = u(1);    // 滚转角 (roll)
    casadi::SX theta = u(2);  // 俯仰角 (pitch)
    casadi::SX psi = u(3);    // 偏航角 (yaw)

    // 重力向量
    casadi::SX g = casadi::SX::zeros(3, 1);
    g(2, 0) = -param_.gravity;

    // 从欧拉角计算旋转矩阵 (机体到惯性)
    casadi::SX R = eulerAnglesToRotationMatrix(phi, theta, psi);

    // 机体坐标系下的推力向量
    casadi::SX F_body = casadi::SX::zeros(3, 1);
    F_body(2, 0) = thrust;

    // 将推力转换到惯性坐标系
    casadi::SX F_inertial = casadi::SX::mtimes(R, F_body);

    // 位置导数（速度）
    casadi::SX p_dot = v;

    // 速度导数（加速度）
    casadi::SX v_dot = F_inertial / param_.mass + g;

    // 返回状态导数（离散化）
    casadi::SX x_next = casadi::SX::vertcat({p + param_.dt * p_dot,
                                             v + param_.dt * v_dot});

    return x_next;
}

// 辅助函数：欧拉角到旋转矩阵转换
casadi::SX MPCController::eulerAnglesToRotationMatrix(const casadi::SX &phi,
                                                      const casadi::SX &theta,
                                                      const casadi::SX &psi)
{
    // 滚转矩阵 (绕X轴)
    casadi::SX R_x = casadi::SX::eye(3);
    R_x(1, 1) = casadi::SX::cos(phi);
    R_x(1, 2) = -casadi::SX::sin(phi);
    R_x(2, 1) = casadi::SX::sin(phi);
    R_x(2, 2) = casadi::SX::cos(phi);

    // 俯仰矩阵 (绕Y轴)
    casadi::SX R_y = casadi::SX::eye(3);
    R_y(0, 0) = casadi::SX::cos(theta);
    R_y(0, 2) = casadi::SX::sin(theta);
    R_y(2, 0) = -casadi::SX::sin(theta);
    R_y(2, 2) = casadi::SX::cos(theta);

    // 偏航矩阵 (绕Z轴)
    casadi::SX R_z = casadi::SX::eye(3);
    R_z(0, 0) = casadi::SX::cos(psi);
    R_z(0, 1) = -casadi::SX::sin(psi);
    R_z(1, 0) = casadi::SX::sin(psi);
    R_z(1, 1) = casadi::SX::cos(psi);

    // 组合旋转矩阵: R = R_z * R_y * R_x
    casadi::SX R = casadi::SX::mtimes(R_z, casadi::SX::mtimes(R_y, R_x));

    return R;
}

// 主控制计算函数
quadrotor_msgs::Px4ctrlDebug MPCController::calculateControl(const Desired_State_t &des,
                                                             const Odom_Data_t &odom,
                                                             const Imu_Data_t &imu,
                                                             Controller_Output_t &u)
{

    // 更新当前状态
    state_.segment(0, 3) = odom.p.cast<double>();          // 位置
    state_.segment(3, 3) = odom.v.cast<double>();          // 速度

    // 构建参考状态
    Eigen::Matrix<double, 6, 1> ref_state;
    ref_state.segment(0, 3) = des.p; // 期望位置
    ref_state.segment(3, 3) = des.v; // 期望速度
    casadi::DM thr2acc_dm = casadi::DM::zeros(1,1);
    thr2acc_dm(0, 0) = thr2acc_;
    // 设置求解器输入
    casadi::DM p = casadi::DM::vertcat({casadi::DM::reshape(
                                            casadi::DM(std::vector<double>(state_.data(), state_.data() + state_.size())),
                                            state_.size(), 1),
                                        casadi::DM::reshape(
                                            casadi::DM(std::vector<double>(ref_state.data(), ref_state.data() + ref_state.size())),
                                            ref_state.size(), 1),
                                        thr2acc_dm});

    // 求解优化问题
    casadi::DMDict arg = {{"p", p}};

    // 设置约束边界
    std::vector<double> lbg, ubg;

    for (int i = 0; i < param_.horizon; ++i)
    {
        // 控制约束
        // 推力范围
        // lbg.push_back(0.5 * param_.mass * param_.gravity);
        // ubg.push_back(param_.thrust_limit);

        lbg.push_back(0.2);
        ubg.push_back(0.9);

        // // 四元数约束
        // lbg.push_back(0);
        // ubg.push_back(0);

        // 欧拉角约束
        lbg.push_back(-10); //roll
        ubg.push_back(10);
        lbg.push_back(-10); //pitch
        ubg.push_back(10);
        lbg.push_back(0);   //yaw
        ubg.push_back(0);

        if (i > 0)
        {
            lbg.push_back(0); // dot_thrust
            ubg.push_back(0.05);
            lbg.push_back(0); // dot_pitch
            ubg.push_back(0.01);
            lbg.push_back(0); // dot_theta
            ubg.push_back(0.01);
        }
        
    }

    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    // 调试输出
    // std::cout << "实际参数维度: " << p.size1() << "x" << p.size2()
    //           << " (期望值:13x1)" << std::endl;
    // 求解
    casadi::DMDict res = solver_(arg);
    casadi::DM U_opt = casadi::DM::reshape(res.at("x"), 4, param_.horizon);

    // 提取最优控制输入（推力和期望姿态）
    Eigen::Matrix<double, 4, 1> u_opt;
    for (int i = 0; i < 4; ++i)
    {
        u_opt(i) = static_cast<double>(U_opt(i, 0)); // 类型转换
    }
    // Eigen::Quaterniond des_q(u_opt(1), u_opt(2), u_opt(3), u_opt(4));
    Eigen::Quaterniond des_q = Eigen::AngleAxisd(u_opt(3), Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(u_opt(2), Eigen::Vector3d::UnitY()) 
                            * Eigen::AngleAxisd(u_opt(1), Eigen::Vector3d::UnitX());
    // 更新控制器输出
    u.thrust = u_opt(0) / param_.mass / thr2acc_;
    u.q = imu.q * odom.q.inverse() * des_q;
    // std::cout << "thrust:" << u.thrust << std::endl;
    // 填充调试信息
    // debug_msg_.des_p_x = des.p(0);
    // debug_msg_.des_p_y = des.p(1);
    // debug_msg_.des_p_z = des.p(2);

    // debug_msg_.des_v_x = des.v(0);
    // debug_msg_.des_v_y = des.v(1);
    // debug_msg_.des_v_z = des.v(2);

    // debug_msg_.des_a_x = des_acc(0);
    // debug_msg_.des_a_y = des_acc(1);
    // debug_msg_.des_a_z = des_acc(2);

    debug_msg_.des_q_x = des_q.x();
    debug_msg_.des_q_y = des_q.y();
    debug_msg_.des_q_z = des_q.z();
    debug_msg_.des_q_w = des_q.w();

    debug_msg_.des_thr = u.thrust;

    return debug_msg_;
}