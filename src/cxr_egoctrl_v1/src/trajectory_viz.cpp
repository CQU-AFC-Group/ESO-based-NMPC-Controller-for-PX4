#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Empty.h>
#include <string>
#include <cmath>

static mavros_msgs::State g_state;
static bool g_have_odom = false;
static double g_x = 0.0, g_y = 0.0, g_z = 0.0;
static nav_msgs::Path g_actual_path;

static void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    g_state = *msg;
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    g_x = msg->pose.pose.position.x;
    g_y = msg->pose.pose.position.y;
    g_z = msg->pose.pose.position.z;
    geometry_msgs::PoseStamped p;
    p.header = msg->header;
    p.pose = msg->pose.pose;
    g_actual_path.poses.push_back(p);
    g_have_odom = true;
}

static void clear_cb(const std_msgs::Empty::ConstPtr& msg) {
    g_actual_path.poses.clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_viz");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double radius = 2.0;
    int circle_points = 360;
    double viz_rate = 20.0;
    std::string frame_id = "map";
    pnh.param("trajectory/radius", radius, 2.0);
    pnh.param("trajectory/circle_points", circle_points, 360);
    pnh.param("simulation/viz_rate", viz_rate, 20.0);
    pnh.param("frame_id", frame_id, std::string("map"));

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 50, odom_cb);
    ros::Subscriber clear_sub = nh.subscribe<std_msgs::Empty>("/trajectory_viz/clear", 1, clear_cb);

    ros::Publisher expected_pub = nh.advertise<nav_msgs::Path>("/expected_circle_path", 1, true);
    ros::Publisher actual_pub = nh.advertise<nav_msgs::Path>("/actual_flight_path", 10);

    ros::Rate rate(viz_rate);
    bool offboard_started = false;

    g_actual_path.header.frame_id = frame_id;

    while (ros::ok()) {
        if (!offboard_started && g_state.mode == "OFFBOARD" && g_have_odom) {
            offboard_started = true;
        }

        g_actual_path.header.stamp = ros::Time::now();
        actual_pub.publish(g_actual_path);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}