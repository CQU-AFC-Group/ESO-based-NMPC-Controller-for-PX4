/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后菜单选择，再切offborad模式即可运行
 ******************************************************************************************/

/* ============================== 依赖与头文件 ============================== */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include "NonlinearESO.hpp"
#include "LQR.hpp"
#include <cmath>

/* ============================== 控制掩码定义 ============================== */
// 掩码定义
// 加速度控制掩码: 0b100000111111 (使用AX/AY/AZ/YAW)
#define ACCELERATION_CONTROL 2048
#define POSITION_HOLD_MASK 2552

/* ============================== 全局变量与状态 ============================== */

visualization_msgs::Marker trackpoint;
ros::Publisher *pubMarkerPointer;
mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped target_pos;
mavros_msgs::State current_state;
float position_x, position_y, position_z,  current_yaw, targetpos_x, targetpos_y;
double current_roll = 0.0, current_pitch = 0.0;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate;
bool receive = false;
float pi = 3.14159265;
float offboard_hold_x = 0.0f, offboard_hold_y = 0.0f, offboard_hold_z = 0.0f;
bool offboard_hold_set = false;
static bool stop_hold_active = false;

/* 误差发布器 */
ros::Publisher pos_err_pub;
ros::Publisher pos_err_xy_pub;

/* ============================== 控制器参数（从参数服务器读取） ============================== */
float kp_pos_x = 1.0, kp_pos_y = 1.0, kp_pos_z = 1.0;
float kd_pos_x = 0.0, kd_pos_y = 0.0, kd_pos_z = 0.0;
float max_accel_x = 3.0, max_accel_y = 3.0, max_accel_z = 3.0;
float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
float vel_lpf_alpha = 1.0f;
float velocity_x_f = 0.0f, velocity_y_f = 0.0f, velocity_z_f = 0.0f;
float fixed_yaw = 0.0f;

/* ✅ NEW：Z速度限幅参数 */
float max_vel_z = 0.5f;

Parameter_t param_;
LQR_Controller lqr_controller(param_);
/* ============================== 回调函数：RC通道 ============================== */
void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
  rc = *msg;
  rc_value = rc.channels[4];
}

/* ============================== 回调函数：飞控状态 ============================== */
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

/* ============================== 回调函数：里程计与姿态（含速度滤波） ============================== */
void position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_msg = *msg;
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;

    velocity_x = position_msg.twist.twist.linear.x;
    velocity_y = position_msg.twist.twist.linear.y;
    velocity_z = position_msg.twist.twist.linear.z;

    velocity_x_f += vel_lpf_alpha * (velocity_x - velocity_x_f);
    velocity_y_f += vel_lpf_alpha * (velocity_y - velocity_y_f);
    velocity_z_f += vel_lpf_alpha * (velocity_z - velocity_z_f);

    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_roll = roll;
    current_pitch = pitch;
    current_yaw = yaw;
}

/* ============================== 回调函数：RViz航点 ============================== */
void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  target_pos = *msg;
  targetpos_x = target_pos.pose.position.x;
  targetpos_y = target_pos.pose.position.y;
}

/* ============================== 回调函数：EGO轨迹指令 ============================== */
quadrotor_msgs::PositionCommand ego;
void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    receive = true;
    ego = *msg;
    ego_pos_x = ego.position.x;
    ego_pos_y = ego.position.y;
    ego_pos_z = ego.position.z;
    ego_vel_x = ego.velocity.x;
    ego_vel_y = ego.velocity.y;
    ego_vel_z = ego.velocity.z;
    ego_a_x = ego.acceleration.x;
    ego_a_y = ego.acceleration.y;
    ego_a_z = ego.acceleration.z;
    ego_yaw = ego.yaw;
    ego_yaw_rate = ego.yaw_dot;
    ROS_INFO("接收到轨迹命令: pos=(%.2f,%.2f,%.2f)", ego_pos_x, ego_pos_y, ego_pos_z);
}

/* ============================== 回调函数：在线更新PD增益 ============================== */
void gains_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 6) {
        kp_pos_x = msg->data[0];
        kp_pos_y = msg->data[1];
        kp_pos_z = msg->data[2];
        kd_pos_x = msg->data[3];
        kd_pos_y = msg->data[4];
        kd_pos_z = msg->data[5];
        ROS_INFO("更新PD增益 kp=(%.3f,%.3f,%.3f) kd=(%.3f,%.3f,%.3f)",
                 kp_pos_x, kp_pos_y, kp_pos_z, kd_pos_x, kd_pos_y, kd_pos_z);
    }
}

/* ============================== 主程序入口 ============================== */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control_node");
    setlocale(LC_ALL,"");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    /* -------------------- 控制器参数加载 -------------------- */
    private_nh.param("controller/kp_pos_x", kp_pos_x, 3.0f);
    private_nh.param("controller/kp_pos_y", kp_pos_y, 3.0f);
    private_nh.param("controller/kp_pos_z", kp_pos_z, 4.0f);
    private_nh.param("controller/kd_pos_x", kd_pos_x, 0.8f);
    private_nh.param("controller/kd_pos_y", kd_pos_y, 0.8f);
    private_nh.param("controller/kd_pos_z", kd_pos_z, 1.0f);
    private_nh.param("controller/max_accel_x", max_accel_x, 2.0f);
    private_nh.param("controller/max_accel_y", max_accel_y, 2.0f);
    private_nh.param("controller/max_accel_z", max_accel_z, 2.0f);
    private_nh.param("controller/vel_lpf_alpha", vel_lpf_alpha, 0.2f);
    private_nh.param("controller/fixed_yaw", fixed_yaw, 0.0f);

    /* ✅ NEW：读取Z速度限幅（可选） */
    private_nh.param("controller/max_vel_z", max_vel_z, 0.5f);

    ROS_INFO("控制器参数加载完成:");
    ROS_INFO("位置比例增益: kp_x=%.2f, kp_y=%.2f, kp_z=%.2f", kp_pos_x, kp_pos_y, kp_pos_z);
    ROS_INFO("位置微分增益: kd_x=%.2f, kd_y=%.2f, kd_z=%.2f", kd_pos_x, kd_pos_y, kd_pos_z);
    ROS_INFO("最大加速度限制: max_a_x=%.2f, max_a_y=%.2f, max_a_z=%.2f", max_accel_x, max_accel_y, max_accel_z);
    ROS_INFO("Z速度限幅: max_vel_z=%.2f", max_vel_z);

    /* -------------------- 发布频率参数 -------------------- */
    float publish_rate = 100.0f;
    private_nh.param("simulation/publish_rate", publish_rate, 100.0f);
    ROS_INFO("控制器发布频率: %.1f Hz", publish_rate);

    /* -------------------- ESO配置与初始化 -------------------- */
    NonlinearESO::Config eso_cfg;
    double eso_omega_xy = 4.0;
    double eso_omega_z = 5.0;
    double eso_mass = 2.30;
    double eso_delta = 0.1;
    private_nh.param("eso/omega_xy", eso_omega_xy, 5.0);
    private_nh.param("eso/omega_z", eso_omega_z, 6.5);
    private_nh.param("eso/mass", eso_mass, 2.30);
    private_nh.param("eso/delta", eso_delta, 0.1);
    eso_cfg.beta1_xy = 3 * eso_omega_xy;
    eso_cfg.beta2_xy = 3 * eso_omega_xy * eso_omega_xy;
    eso_cfg.beta3_xy = eso_omega_xy * eso_omega_xy * eso_omega_xy;
    eso_cfg.alpha1_xy = 0.6;
    eso_cfg.alpha2_xy = 0.6;
    eso_cfg.alpha3_xy = 0.6;
    eso_cfg.beta1_z = 3 * eso_omega_z;
    eso_cfg.beta2_z = 3 * eso_omega_z * eso_omega_z;
    eso_cfg.beta3_z = eso_omega_z * eso_omega_z * eso_omega_z;
    eso_cfg.alpha1_z = 0.5;
    eso_cfg.alpha2_z = 0.5;
    eso_cfg.alpha3_z = 0.5;
    eso_cfg.delta = eso_delta;
    eso_cfg.mass = eso_mass;
    eso_cfg.dt = 1.0 / publish_rate;
    eso_cfg.gravity = 9.81;
    NonlinearESO eso(eso_cfg);

    /* -------------------- ROS通信（订阅/发布/服务） -------------------- */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

    ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
    pubMarkerPointer = &pubMarker;

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rc_cb);
    ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 10, twist_cb);
    ros::Subscriber gains_sub = nh.subscribe<std_msgs::Float32MultiArray>("/controller/gains", 1, gains_cb);
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 10, target_cb);
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, position_cb);
    
    /* 误差话题发布器 */
    pos_err_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/controller/pos_error_xyz", 10);
    pos_err_xy_pub = nh.advertise<std_msgs::Float32>("/controller/pos_error_xy", 10);

    ros::Rate rate(publish_rate);

    /* -------------------- 等待飞控连接 -------------------- */
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle connected");

    /* -------------------- 初始设定点发送 -------------------- */
    for (int i = 100; ros::ok() && i > 0; --i) {
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask = POSITION_HOLD_MASK;
        current_goal.position.x = position_x;
        current_goal.position.y = position_y;
        current_goal.position.z = position_z;
        current_goal.yaw = fixed_yaw;
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();
    ROS_INFO("等待人工切换OFFBOARD模式");
    std::string last_mode_state = current_state.mode;

    /* -------------------- 主控制循环 -------------------- */
    while (ros::ok())
    {
        /* ---- 运行状态周期打印 ---- */
        if (fmod((ros::Time::now() - last_request).toSec(), 2.0) < 0.1) {
            ROS_INFO("当前模式: %s, 电机状态: %s",
                     current_state.mode.c_str(),
                     current_state.armed ? "已解锁" : "已锁定");
        }

        if (current_state.mode == "OFFBOARD" && last_mode_state != "OFFBOARD") {
            double p0[3] = {position_x, position_y, position_z};
            double v0[3] = {velocity_x_f, velocity_y_f, velocity_z_f};
            double d0[3] = {0.0, 0.0, 0.0};
            eso.initialize(p0, v0, d0);

            offboard_hold_x = position_x;
            offboard_hold_y = position_y;
            offboard_hold_z = position_z;
            offboard_hold_set = true;
        }
        last_mode_state = current_state.mode;

        if (current_state.mode != "OFFBOARD" || !receive)
        {
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_goal.header.stamp = ros::Time::now();
            current_goal.type_mask = POSITION_HOLD_MASK;

            if (current_state.mode == "OFFBOARD" && !receive && offboard_hold_set) {
                current_goal.position.x = offboard_hold_x;
                current_goal.position.y = offboard_hold_y;
                current_goal.position.z = offboard_hold_z;
            } else {
                current_goal.position.x = position_x;
                current_goal.position.y = position_y;
                current_goal.position.z = position_z;
            }
            current_goal.yaw = fixed_yaw;
        }
        else if (current_state.mode == "OFFBOARD" && receive)
        {
            bool stop_cmd = std::fabs(ego_vel_x) < 1e-6 && std::fabs(ego_vel_y) < 1e-6 && std::fabs(ego_vel_z) < 1e-6 &&
                            std::fabs(ego_a_x) < 1e-6 && std::fabs(ego_a_y) < 1e-6 && std::fabs(ego_a_z) < 1e-6;
            if (stop_cmd) {
                if (!stop_hold_active) {
                    offboard_hold_x = position_x;
                    offboard_hold_y = position_y;
                    offboard_hold_z = position_z;
                    offboard_hold_set = true;
                    stop_hold_active = true;
                }
                current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                current_goal.header.stamp = ros::Time::now();
                current_goal.type_mask = POSITION_HOLD_MASK;
                current_goal.position.x = offboard_hold_x;
                current_goal.position.y = offboard_hold_y;
                current_goal.position.z = offboard_hold_z;
                current_goal.yaw = fixed_yaw;
            }
        }

        /* ---- OFFBOARD+EGO：加速度跟踪与ESO补偿 ---- */
        if (current_state.mode == "OFFBOARD" && receive && !stop_hold_active)
        {
            float yaw_erro;
            yaw_erro = (ego_yaw - current_yaw);
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_goal.header.stamp = ros::Time::now();

            // ✅ Z轴从位置控制改为速度控制：忽略PZ，不忽略VZ
            current_goal.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |  
                                     mavros_msgs::PositionTarget::IGNORE_VX |
                                     mavros_msgs::PositionTarget::IGNORE_VY |
                                     /* mavros_msgs::PositionTarget::IGNORE_VZ | */
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            /* ---- PD前馈/误差补偿计算 ---- */
            float pos_error_x = ego_pos_x - position_x;
            float acc_x = ego_a_x + kp_pos_x * pos_error_x + kd_pos_x * (ego_vel_x - velocity_x_f);

            float pos_error_y = ego_pos_y - position_y;
            float acc_y = ego_a_y + kp_pos_y * pos_error_y + kd_pos_y * (ego_vel_y - velocity_y_f);

            Eigen::VectorXd state_now(4), state_des(4), control_output(2);
            state_now << position_x, position_y, velocity_x_f, velocity_y_f;
            state_des << ego_pos_x, ego_pos_y, ego_vel_x, ego_vel_y;
            lqr_controller.computeControl(state_now, state_des, control_output);
            acc_x = control_output(0);
            acc_y = control_output(1);
            // std::cout << "LQR控制输出: ax=" << control_output(0) << ", ay=" << control_output(1) << std::endl;
            // std::cout << "PD控制输出: ax=" << acc_x << ", ay=" << acc_y << std::endl;

            // ✅ Z轴速度指令（由位置误差生成速度命令）
            float pos_error_z = ego_pos_z - position_z;
            float vel_z_cmd = ego_vel_z + kp_pos_z * pos_error_z + kd_pos_z * (ego_vel_z - velocity_z_f);
            
            // ✅ Z轴速度饱和限幅
            if (vel_z_cmd >  max_vel_z) vel_z_cmd =  max_vel_z;   
            if (vel_z_cmd < -max_vel_z) vel_z_cmd = -max_vel_z;  

            current_goal.velocity.z = vel_z_cmd;

            /* ---- ESO更新（推力与姿态映射） ---- */
            Eigen::Vector3d pos_meas(position_x, position_y, position_z);
            const double cr = std::cos(current_roll);
            const double sr = std::sin(current_roll);
            const double cp = std::cos(current_pitch);
            const double sp = std::sin(current_pitch);
            const double cy = std::cos(current_yaw);
            const double sy = std::sin(current_yaw);
            Eigen::Vector3d e3_world(cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr);
            Eigen::Vector3d a_world(acc_x, acc_y, 0.0);
            Eigen::Vector3d g_world(0.0, 0.0, -eso_cfg.gravity);
            double thrust_cmd = eso_cfg.mass * (a_world + g_world).dot(e3_world);
            Eigen::Vector3d euler(current_roll, current_pitch, current_yaw);
            eso.update(pos_meas, thrust_cmd, euler);
            double d_hat[3];
            eso.getDisturbanceEstimate(d_hat);

            /* ---- 扰动估计与加速度补偿 ---- */
            // acc_x -= d_hat[0] / eso_cfg.mass;
            // acc_y -= d_hat[1] / eso_cfg.mass;

            /* ---- 加速度限幅 ---- */
            if (acc_x > max_accel_x) acc_x = max_accel_x;
            if (acc_x < -max_accel_x) acc_x = -max_accel_x;
            if (acc_y > max_accel_y) acc_y = max_accel_y;
            if (acc_y < -max_accel_y) acc_y = -max_accel_y;

            current_goal.acceleration_or_force.x = acc_x;
            current_goal.acceleration_or_force.y = acc_y;
            current_goal.yaw = fixed_yaw;

            float acc_magnitude = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
            // ROS_INFO("ESO补偿: d_hat=(%.2f,%.2f,%.2f), cmd_a_xy=(%.2f,%.2f), |a_xy|=%.2f",
            //          (float)d_hat[0], (float)d_hat[1], (float)d_hat[2], acc_x, acc_y, acc_magnitude);
        }

        if (!(std::fabs(ego_vel_x) < 1e-6 && std::fabs(ego_vel_y) < 1e-6 && std::fabs(ego_vel_z) < 1e-6 &&
              std::fabs(ego_a_x) < 1e-6 && std::fabs(ego_a_y) < 1e-6 && std::fabs(ego_a_z) < 1e-6)) {
            stop_hold_active = false;
        }

        /* 发布位置误差（reference - actual） */
        {
            bool can_pub = (current_state.mode == "OFFBOARD") && (receive || offboard_hold_set);
            if (can_pub) {
                float ref_x, ref_y, ref_z;

                if (stop_hold_active && offboard_hold_set) {
                    ref_x = offboard_hold_x;
                    ref_y = offboard_hold_y;
                    ref_z = offboard_hold_z;
                } else if (receive) {
                    ref_x = ego_pos_x;
                    ref_y = ego_pos_y;
                    ref_z = ego_pos_z;
                } else {
                    ref_x = offboard_hold_x;
                    ref_y = offboard_hold_y;
                    ref_z = offboard_hold_z;
                }

                float ex = ref_x - position_x;
                float ey = ref_y - position_y;
                float ez = ref_z - position_z;

                geometry_msgs::Vector3Stamped err_msg;
                err_msg.header.stamp = ros::Time::now();
                err_msg.header.frame_id = position_msg.header.frame_id.empty() ? "world" : position_msg.header.frame_id;
                err_msg.vector.x = ex;
                err_msg.vector.y = ey;
                err_msg.vector.z = ez;
                pos_err_pub.publish(err_msg);

                std_msgs::Float32 exy_msg;
                exy_msg.data = std::sqrt(ex * ex + ey * ey);
                pos_err_xy_pub.publish(exy_msg);
            }
        }

        /* ---- 设定点发布与循环节拍 ---- */
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
