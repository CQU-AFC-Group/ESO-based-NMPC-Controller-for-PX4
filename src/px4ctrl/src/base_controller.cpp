#include "base_controller.h"

using namespace std;

double BaseController::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

BaseController::BaseController(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/*
  compute throttle percentage 
*/
double BaseController::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc){
    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

bool BaseController::estimateThrustModel(const Eigen::Vector3d &est_a,
                                         const Parameter_t &param)
{
    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1){
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        // 剔除太远数据
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
        //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        //fflush(stdout);

        // debug_msg_.thr2acc = thr2acc_;
        return true;
    }
    return false;
}

void BaseController::resetThrustMapping(void)
{
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_ = 1e6;
}
