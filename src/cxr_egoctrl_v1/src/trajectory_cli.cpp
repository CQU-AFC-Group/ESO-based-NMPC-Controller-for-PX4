#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>

static std::atomic<bool> run_circle(false);
static std::atomic<bool> run_goto(false);
static std::atomic<bool> run_goto_seq(false);
static std::atomic<bool> stop_req(false);
static double cx = 0.0, cy = 0.0, cz = 0.0;
static double px = 0.0, py = 0.0, pz = 0.0;
static double vx = 0.0, vy = 0.0, vz = 0.0;
static double tx = 0.0, ty = 0.0, tz = 0.0;
static double tx1 = 0.0, ty1 = 0.0, tz1 = 0.0;
static double tx2 = 0.0, ty2 = 0.0, tz2 = 0.0;
static double dwell_seconds = 3.0;
static double arrival_tol = 0.3;
static int goto_stage = 0;
static double dwell_start_t = 0.0;
static double radius = 2.0;
static double angular_velocity = 1.0;
static double kp_x = 3.0, kp_y = 3.0, kp_z = 4.0;
static double kd_x = 1.5, kd_y = 1.5, kd_z = 2.0;
static double start_t = 0.0;
static std::mutex mtx;
static double stop_hold_x = 0.0, stop_hold_y = 0.0, stop_hold_z = 0.0;
static bool stop_hold_set = false;

static void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    pz = msg->pose.pose.position.z;
    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
    vz = msg->twist.twist.linear.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_cli");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("trajectory/radius", radius, 0.5);
    pnh.param("trajectory/angular_velocity", angular_velocity, 1.0);
    pnh.param("trajectory/target_position/x", tx, 0.0);
    pnh.param("trajectory/target_position/y", ty, 0.0);
    pnh.param("trajectory/target_position/z", tz, 2.0);
    pnh.param("trajectory/target_position_1/x", tx1, tx);
    pnh.param("trajectory/target_position_1/y", ty1, ty);
    pnh.param("trajectory/target_position_1/z", tz1, tz);
    pnh.param("trajectory/target_position_2/x", tx2, tx);
    pnh.param("trajectory/target_position_2/y", ty2, ty);
    pnh.param("trajectory/target_position_2/z", tz2, tz);
    pnh.param("trajectory/target_dwell_seconds", dwell_seconds, 3.0);
    pnh.param("trajectory/target_arrival_tolerance", arrival_tol, 0.3);
    pnh.param("controller/kp_pos_x", kp_x, 3.0);
    pnh.param("controller/kp_pos_y", kp_y, 3.0);
    pnh.param("controller/kp_pos_z", kp_z, 4.0);
    pnh.param("controller/kd_pos_x", kd_x, 1.5);
    pnh.param("controller/kd_pos_y", kd_y, 1.5);
    pnh.param("controller/kd_pos_z", kd_z, 2.0);

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 50, odom_cb);
    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    ros::Publisher gains_pub = nh.advertise<std_msgs::Float32MultiArray>("/controller/gains", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/expected_circle_path", 1, true);
    ros::Publisher clear_pub = nh.advertise<std_msgs::Empty>("/trajectory_viz/clear", 1);

    std::thread input_thr([&]{
        for(;;){
            double speed = radius * angular_velocity;
            double r_safe = radius <= 1e-6 ? 1e-6 : radius;
            double ac = (speed * speed) / r_safe;
            std::cout << "[params] kp=(" << kp_x << "," << kp_y << "," << kp_z << ") kd=("
                      << kd_x << "," << kd_y << "," << kd_z << ") radius=" << radius
                      << " omega=" << angular_velocity << " v=" << speed << " a_c=" << ac
                      << " target1=(" << tx1 << "," << ty1 << "," << tz1 << ")"
                      << " target2=(" << tx2 << "," << ty2 << "," << tz2 << ")"
                      << " dwell=" << dwell_seconds << "s tol=" << arrival_tol
                      << " status=" << (run_circle.load()?"circle":"idle") << (run_goto_seq.load()?"+goto2":"") << "\n";
            std::cout << "[1] start circle  [g] goto 2 points  [2] stop  [p] set kp  [d] set kd  [r] set radius  [w] set omega  [q] quit\n> " << std::flush;
            std::string s; if(!std::getline(std::cin, s)) break;
            if(s=="1"){
                std::lock_guard<std::mutex> lk(mtx);
                cx = px; cy = py; cz = pz;
                start_t = ros::Time::now().toSec();
                std_msgs::Empty e; clear_pub.publish(e);
                nav_msgs::Path path; path.header.frame_id = "map"; path.header.stamp = ros::Time::now();
                int N = 360;
                for(int i=0;i<N;i++){
                    double th = 2.0*M_PI*double(i)/double(N);
                    geometry_msgs::PoseStamped p; p.header.frame_id = "map"; p.header.stamp = ros::Time::now();
                    p.pose.position.x = cx + radius*std::cos(th);
                    p.pose.position.y = cy + radius*std::sin(th);
                    p.pose.position.z = cz;
                    p.pose.orientation.w = 1.0;
                    path.poses.push_back(p);
                }
                path_pub.publish(path);
                run_circle.store(true);
                run_goto.store(false);
                stop_req.store(false);
                stop_hold_set = false;
            } else if(s=="2"){
                run_circle.store(false);
                run_goto.store(false);
                stop_hold_x = px;
                stop_hold_y = py;
                stop_hold_z = pz;
                stop_hold_set = true;
                stop_req.store(true);
            } else if(s=="g"){
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    std_msgs::Empty e; clear_pub.publish(e);
                }
                run_goto.store(false);
                run_goto_seq.store(true);
                goto_stage = 0;
                dwell_start_t = 0.0;
                run_circle.store(false);
                stop_req.store(false);
                stop_hold_set = false;
            } else if(s=="p"){
                std::cout << "kp_x kp_y kp_z: " << std::flush;
                double a,b,c; if(!(std::cin>>a>>b>>c)){std::cin.clear(); std::cin.ignore(1024,'\n'); continue;} std::cin.ignore(1024,'\n');
                kp_x=a; kp_y=b; kp_z=c;
                std_msgs::Float32MultiArray m; m.data={float(kp_x),float(kp_y),float(kp_z),float(kd_x),float(kd_y),float(kd_z)}; gains_pub.publish(m);
            } else if(s=="d"){
                std::cout << "kd_x kd_y kd_z: " << std::flush;
                double a,b,c; if(!(std::cin>>a>>b>>c)){std::cin.clear(); std::cin.ignore(1024,'\n'); continue;} std::cin.ignore(1024,'\n');
                kd_x=a; kd_y=b; kd_z=c;
                std_msgs::Float32MultiArray m; m.data={float(kp_x),float(kp_y),float(kp_z),float(kd_x),float(kd_y),float(kd_z)}; gains_pub.publish(m);
            } else if(s=="r"){
                std::cout << "radius: " << std::flush;
                double a; if(!(std::cin>>a)){std::cin.clear(); std::cin.ignore(1024,'\n'); continue;} std::cin.ignore(1024,'\n'); radius=a;
            } else if(s=="w"){
                std::cout << "omega(rad/s): " << std::flush;
                double a; if(!(std::cin>>a)){std::cin.clear(); std::cin.ignore(1024,'\n'); continue;} std::cin.ignore(1024,'\n'); angular_velocity=a;
            } else if(s=="q"){
                run_circle.store(false); stop_req.store(false); break;
            }
        }
    });

    ros::Rate rate(50.0);
    while(ros::ok()){
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        if(run_circle.load()){
            double t = ros::Time::now().toSec() - start_t;
            double th = angular_velocity * t;
            cmd.position.x = cx + radius*std::cos(th);
            cmd.position.y = cy + radius*std::sin(th);
            cmd.position.z = cz;
            cmd.velocity.x = -radius*angular_velocity*std::sin(th);
            cmd.velocity.y =  radius*angular_velocity*std::cos(th);
            cmd.velocity.z = 0.0;
            cmd.acceleration.x = -radius*angular_velocity*angular_velocity*std::cos(th);
            cmd.acceleration.y = -radius*angular_velocity*angular_velocity*std::sin(th);
            cmd.acceleration.z = 0.0;
            double dx = cx - cmd.position.x;
            double dy = cy - cmd.position.y;
            cmd.yaw = std::atan2(dy, dx);
            cmd.yaw_dot = angular_velocity;
        } else if(run_goto_seq.load()){
            if(goto_stage == 0){
                cmd.position.x = tx1;
                cmd.position.y = ty1;
                cmd.position.z = tz1;
                cmd.velocity.x = 0.0;
                cmd.velocity.y = 0.0;
                cmd.velocity.z = 0.0;
                cmd.acceleration.x = 0.0;
                cmd.acceleration.y = 0.0;
                cmd.acceleration.z = 0.0;
                cmd.yaw = 0.0;
                cmd.yaw_dot = 0.0;
                double dx = px - tx1;
                double dy = py - ty1;
                double dz = pz - tz1;
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if(dist <= arrival_tol){
                    goto_stage = 1;
                    dwell_start_t = ros::Time::now().toSec();
                }
            } else if(goto_stage == 1){
                cmd.position.x = tx1;
                cmd.position.y = ty1;
                cmd.position.z = tz1;
                cmd.velocity.x = 0.0;
                cmd.velocity.y = 0.0;
                cmd.velocity.z = 0.0;
                cmd.acceleration.x = 0.0;
                cmd.acceleration.y = 0.0;
                cmd.acceleration.z = 0.0;
                cmd.yaw = 0.0;
                cmd.yaw_dot = 0.0;
                double now = ros::Time::now().toSec();
                if(now - dwell_start_t >= dwell_seconds){
                    goto_stage = 2;
                }
            } else {
                cmd.position.x = tx2;
                cmd.position.y = ty2;
                cmd.position.z = tz2;
                cmd.velocity.x = 0.0;
                cmd.velocity.y = 0.0;
                cmd.velocity.z = 0.0;
                cmd.acceleration.x = 0.0;
                cmd.acceleration.y = 0.0;
                cmd.acceleration.z = 0.0;
                cmd.yaw = 0.0;
                cmd.yaw_dot = 0.0;
            }
        } else if(stop_req.load()){
            if(!stop_hold_set){
                stop_hold_x = px;
                stop_hold_y = py;
                stop_hold_z = pz;
                stop_hold_set = true;
            }
            cmd.position.x = stop_hold_x;
            cmd.position.y = stop_hold_y;
            cmd.position.z = stop_hold_z;
            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;
            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;
            cmd.yaw = 0.0;
            cmd.yaw_dot = 0.0;
        }
        if(run_circle.load() || run_goto_seq.load() || stop_req.load()) cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
    input_thr.join();
    return 0;
}
