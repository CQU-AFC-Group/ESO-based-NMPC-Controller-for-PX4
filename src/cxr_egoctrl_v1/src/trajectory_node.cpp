/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>
#include <string>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

static mavros_msgs::State g_state;
static double g_pos_x = 0.0, g_pos_y = 0.0, g_pos_z = 0.0;

static void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    g_state = *msg;
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    g_pos_x = msg->pose.pose.position.x;
    g_pos_y = msg->pose.pose.position.y;
    g_pos_z = msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 私有命名空间节点句柄，用于参数服务器

    ros::Publisher trajectory_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>
            ("/position_cmd", 10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);
    
    // 从参数服务器读取轨迹参数
    std::string trajectory_type = "circle";
    //std::string trajectory_type = "square";
    float radius = 2.0f;
    float height = 2.0f;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float angular_velocity = 0.3f;
    float square_size = 4.0f;
    float hover_x = 0.0f;
    float hover_y = 0.0f;
    float hover_z = 2.0f;
    float publish_rate = 20.0f;
    
    // 加载轨迹参数
    private_nh.param("trajectory/type", trajectory_type, std::string("circle"));
    private_nh.param("trajectory/radius", radius, 0.5f);
    private_nh.param("trajectory/height", height, 2.0f);
    private_nh.param("trajectory/center_x", center_x, 0.0f);
    private_nh.param("trajectory/center_y", center_y, 0.0f);
    private_nh.param("trajectory/angular_velocity", angular_velocity, 0.3f);
    private_nh.param("trajectory/square_size", square_size, 4.0f);
    private_nh.param("trajectory/hover_position/x", hover_x, 0.0f);
    private_nh.param("trajectory/hover_position/y", hover_y, 0.0f);
    private_nh.param("trajectory/hover_position/z", hover_z, 2.0f);
    private_nh.param("simulation/trajectory_rate", publish_rate, 20.0f);
    
    ROS_INFO("轨迹生成节点已启动，准备发布 %s 轨迹命令", trajectory_type.c_str());
    
    if (trajectory_type == "circle") {
        ROS_INFO("圆形轨迹参数: 半径=%.2fm, 高度=%.2fm, 角速度=%.2frad/s", radius, height, angular_velocity);
    } else if (trajectory_type == "square") {
        ROS_INFO("方形轨迹参数: 边长=%.2fm, 高度=%.2fm, 角速度=%.2frad/s", square_size, height, angular_velocity);
    } else if (trajectory_type == "hover") {
        ROS_INFO("悬停轨迹参数: 位置=(%.2f, %.2f, %.2f)", hover_x, hover_y, hover_z);
    }
    
    // 设置发布频率
    ros::Rate rate(publish_rate);

    // 创建轨迹命令
    quadrotor_msgs::PositionCommand trajectory_cmd;
    trajectory_cmd.header.frame_id = "map";
    
    double start_time = 0.0;
    bool offboard_started = false;

    ROS_INFO("等待OFFBOARD模式切换以开始发布轨迹命令");

    while(ros::ok()){
            if (!offboard_started) {
                if (g_state.mode == "OFFBOARD") {
                    center_x = g_pos_x;
                    center_y = g_pos_y;
                    height = g_pos_z;
                    start_time = ros::Time::now().toSec();
                    offboard_started = true;
                    ROS_INFO("检测到OFFBOARD，设置圆心=(%.2f, %.2f)，高度=%.2f", center_x, center_y, height);
                }
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            double t = ros::Time::now().toSec() - start_time;
            
            if (trajectory_type == "circle") {
                // 圆形轨迹计算
                double theta = angular_velocity * t;
                
                // 计算圆上的位置
                trajectory_cmd.position.x = center_x + radius * cos(theta);
                trajectory_cmd.position.y = center_y + radius * sin(theta);
                trajectory_cmd.position.z = height;
                
                // 计算速度（位置对时间求导）
                trajectory_cmd.velocity.x = -radius * angular_velocity * sin(theta);
                trajectory_cmd.velocity.y = radius * angular_velocity * cos(theta);
                trajectory_cmd.velocity.z = 0;
                
                // 计算加速度（速度对时间求导）
                trajectory_cmd.acceleration.x = -radius * angular_velocity * angular_velocity * cos(theta);
                trajectory_cmd.acceleration.y = -radius * angular_velocity * angular_velocity * sin(theta);
                trajectory_cmd.acceleration.z = 0;
                
                // 计算方向向量，使机头指向圆心
                float dx = center_x - trajectory_cmd.position.x;
                float dy = center_y - trajectory_cmd.position.y;
                float yaw = atan2(dy, dx); // 计算偏航角
                trajectory_cmd.yaw = yaw;
                
                // 计算偏航角速度
                trajectory_cmd.yaw_dot = angular_velocity;
            } else if (trajectory_type == "square") {
                // 方形轨迹计算
                double total_distance = square_size * 4.0; // 周长
                double linear_velocity = angular_velocity * square_size / 2.0; // 线速度
                double phase = fmod(t * linear_velocity, total_distance); // 当前所在边的位置
                
                // 根据当前phase确定在方形的哪一条边
                if (phase < square_size) { // 右边
                    trajectory_cmd.position.x = center_x + square_size / 2.0;
                    trajectory_cmd.position.y = center_y - square_size / 2.0 + phase;
                    trajectory_cmd.velocity.x = 0;
                    trajectory_cmd.velocity.y = linear_velocity;
                    trajectory_cmd.acceleration.x = 0;
                    trajectory_cmd.acceleration.y = 0;
                    trajectory_cmd.yaw = M_PI / 2.0; // 向上
                } else if (phase < square_size * 2) { // 上边
                    phase -= square_size;
                    trajectory_cmd.position.x = center_x + square_size / 2.0 - phase;
                    trajectory_cmd.position.y = center_y + square_size / 2.0;
                    trajectory_cmd.velocity.x = -linear_velocity;
                    trajectory_cmd.velocity.y = 0;
                    trajectory_cmd.acceleration.x = 0;
                    trajectory_cmd.acceleration.y = 0;
                    trajectory_cmd.yaw = M_PI; // 向左
                } else if (phase < square_size * 3) { // 左边
                    phase -= square_size * 2;
                    trajectory_cmd.position.x = center_x - square_size / 2.0;
                    trajectory_cmd.position.y = center_y + square_size / 2.0 - phase;
                    trajectory_cmd.velocity.x = 0;
                    trajectory_cmd.velocity.y = -linear_velocity;
                    trajectory_cmd.acceleration.x = 0;
                    trajectory_cmd.acceleration.y = 0;
                    trajectory_cmd.yaw = 3 * M_PI / 2.0; // 向下
                } else { // 下边
                    phase -= square_size * 3;
                    trajectory_cmd.position.x = center_x - square_size / 2.0 + phase;
                    trajectory_cmd.position.y = center_y - square_size / 2.0;
                    trajectory_cmd.velocity.x = linear_velocity;
                    trajectory_cmd.velocity.y = 0;
                    trajectory_cmd.acceleration.x = 0;
                    trajectory_cmd.acceleration.y = 0;
                    trajectory_cmd.yaw = 0; // 向右
                }
                
                trajectory_cmd.position.z = height;
                trajectory_cmd.velocity.z = 0;
                trajectory_cmd.acceleration.z = 0;
                trajectory_cmd.yaw_dot = 0;
            } else if (trajectory_type == "hover") {
                // 悬停轨迹
                trajectory_cmd.position.x = hover_x;
                trajectory_cmd.position.y = hover_y;
                trajectory_cmd.position.z = hover_z;
                trajectory_cmd.velocity.x = 0;
                trajectory_cmd.velocity.y = 0;
                trajectory_cmd.velocity.z = 0;
                trajectory_cmd.acceleration.x = 0;
                trajectory_cmd.acceleration.y = 0;
                trajectory_cmd.acceleration.z = 0;
                trajectory_cmd.yaw = 0;
                trajectory_cmd.yaw_dot = 0;
            }
            
            // 设置消息头并发布
            trajectory_cmd.header.stamp = ros::Time::now();
            trajectory_cmd_pub.publish(trajectory_cmd);
            
            // 每秒打印一次轨迹信息
            if (fmod(t, 1.0) < 0.05) {
                ROS_INFO("轨迹位置: (%.2f, %.2f, %.2f)", 
                         trajectory_cmd.position.x, 
                         trajectory_cmd.position.y, 
                         trajectory_cmd.position.z);
            }
            ros::spinOnce();
            rate.sleep();
        }

    return 0;
}
