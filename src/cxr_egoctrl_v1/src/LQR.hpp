#include <Eigen/Dense>
#include <iostream>
class Parameter_t {
    public:
        float Q_xy;
        float Q_vel_xy;
        float Q_z;
        float Q_vel_z;
        float R_xy;
        float R_z;
        bool ignore_z;
        float dt;

        Parameter_t() {
            Q_xy = 2.0f;
            Q_vel_xy = 2.0f;
            Q_z = 1.0f;
            Q_vel_z = 0.5f;
            R_xy = 0.1f;
            R_z = 0.1f;
            ignore_z = true;
            dt = 0.01f;
        }
};

class LQR_Controller {
    public:
        LQR_Controller(Parameter_t &param);
        void computeControl(const Eigen::VectorXd &state, const Eigen::VectorXd &ref, Eigen::VectorXd &control_output);    
    private:
        Parameter_t param_;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd N;
        Eigen::MatrixXd K;
        void DARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                  const Eigen::MatrixXd &R, const Eigen::MatrixXd &N, Eigen::MatrixXd &K, const double tolerance,
                  const int max_num_iteration);
        double fromQuaternion2yaw(Eigen::Quaterniond q);
};

void LQR_Controller::computeControl(const Eigen::VectorXd &state, const Eigen::VectorXd &ref, Eigen::VectorXd &control_output) {
    // 这里实现根据当前状态state和参考状态ref，计算控制输出control_output的逻辑
    // 可以使用状态反馈控制律 u = -K * (state - ref) 来计算控制输入
    if (state.size() != ref.size() || K.cols() != state.size()) {
        std::cout << "Error: State dimension is " << state.size() << ", reference dimension is " << ref.size() << ", and gain matrix columns are " << K.cols() << std::endl;
        // 处理维度不匹配的情况，可以抛出异常或者返回错误
        throw std::invalid_argument("State, reference, and gain matrix dimensions do not match.");
    }
    control_output = -K * (state - ref);
}

LQR_Controller::LQR_Controller(Parameter_t &param) : param_(param) {
    if (param_.ignore_z)
    {
        A = Eigen::MatrixXd::Identity(4, 4);
        A(0, 2) = param_.dt;
        A(1, 3) = param_.dt;
        B = Eigen::MatrixXd::Zero(4, 2);
        B(2, 0) = param_.dt;
        B(3, 1) = param_.dt;
        // B(0, 0) = 0.5 * param_.dt * param_.dt;
        // B(1, 1) = 0.5 * param_.dt * param_.dt;
        Q = Eigen::MatrixXd::Zero(4,4);
        Q(0,0) = param_.Q_xy; // 位置的权重
        Q(1,1) = param_.Q_xy;
        Q(2,2) = param_.Q_vel_xy; // 位置和速度的权重可以不同
        Q(3,3) = param_.Q_vel_xy;
        R = param_.R_xy * Eigen::MatrixXd::Identity(2,2);
        N= Eigen::MatrixXd::Zero(4, 2);         // 状态与输入的耦合项权重，默认没有交叉项
        DARE(A, B, Q, R, N, K, 1e-15, 1000);    // 解离散时间Algebraic Riccati方程，得到状态反馈增益矩阵K
    }else
    {
        // 如果不忽略Z轴，则需要构建6维状态空间模型（位置和速度）
        A = Eigen::MatrixXd::Identity(6, 6);
        A(0, 3) = param_.dt;
        A(1, 4) = param_.dt;
        A(2, 5) = param_.dt;
        B = Eigen::MatrixXd::Zero(6, 3);
        B(3, 0) = param_.dt;
        B(4, 1) = param_.dt;
        B(5, 2) = param_.dt;
        Q = Eigen::MatrixXd::Zero(6,6);
        Q(0,0) = param_.Q_xy; // 位置的权重
        Q(1,1) = param_.Q_xy;
        Q(2,2) = param_.Q_z;
        Q(3,3) = param_.Q_vel_xy; // 速度的权重
        Q(4,4) = param_.Q_vel_xy;
        Q(5,5) = param_.Q_vel_z;
        R = param_.R_xy * Eigen::MatrixXd::Identity(3,3);
        N= Eigen::MatrixXd::Zero(6, 3);
        DARE(A, B, Q, R, N, K, 1e-15, 1000);
    }
}

void LQR_Controller::DARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                  const Eigen::MatrixXd &R, const Eigen::MatrixXd &N, Eigen::MatrixXd &K, const double tolerance,
                  const int max_num_iteration) {
    // 这里实现求解离散时间Algebraic Riccati方程的算法，得到状态反馈增益矩阵K
    // 可以使用迭代法或者其他数值方法来求解
    Eigen::MatrixXd Q_cal = Q - N * R.inverse() * N.transpose();
    Eigen::MatrixXd P = Q_cal;
    Eigen::MatrixXd P_next;
    Eigen::MatrixXd A_cal = A - B * R.inverse() * N.transpose();
    // std::cout << "A:\n" << A << std::endl;
    // std::cout << "B:\n" << B << std::endl;
    // std::cout << "Q:\n" << Q << std::endl;
    // std::cout << "R:\n" << R << std::endl;
    // std::cout << "A_cal:\n" << A_cal << std::endl;
    // std::cout << "Q_cal:\n" << Q_cal << std::endl;

    for (int i = 0; i < max_num_iteration; ++i) {
        P_next = Q_cal + A_cal.transpose() * P * A_cal - A_cal.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A_cal;
        if ((P_next - P).norm() < tolerance) {
            break;
        }
        P = P_next;
    }
    // std::cout <<  << std::endl;
    K = (R + B.transpose() * P_next * B).inverse() * B.transpose() * P_next * A_cal;
    // std::cout << K << std::endl;
}