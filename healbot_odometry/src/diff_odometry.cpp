#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#define EPSILON 0.000001
class DiffOdometry{
public:
    void rightSubCB(const std_msgs::Float64ConstPtr& ptr){
        vr = ptr->data;
    }

    void leftSubCB(const std_msgs::Float64ConstPtr& ptr){
        vl = ptr->data;
    }

    void updateOdom(){
        //calc vel
        double x_dot = 0.5 * (vr + vl) * cos(theta);
        double y_dot = 0.5 * (vr + vl) * sin(theta);
        double theta_dot = 1/ wheel_seperation * (vr - vl);
        ros::Time cur_time = ros::Time::now();
        double dt;
        try{
            dt = (cur_time - last_time).toSec();
        }
        catch(std::exception& e){
            ROS_INFO("error : %s", e.what());
            ros::shutdown();
        }

        double dx = x_dot * dt;
        double dy = y_dot * dt;
        double dtheta = theta_dot * dt;
        ROS_INFO("theta_dot, dth : %lf %lf", theta_dot, dtheta);
        ROS_WARN("vr, vl : %lf %lf", vr, vl);

        x += dx;
        y += dy;
        theta += dtheta;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = cur_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        static tf::TransformBroadcaster odom_broadcaster;
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = cur_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = x_dot;
        odom.twist.twist.linear.y = y_dot;
        odom.twist.twist.angular.z = theta_dot;

        //publish the message
        odom_pub.publish(odom);
        last_time = cur_time;
    }

    DiffOdometry() : last_time(ros::Time::now()), x(EPSILON), y(EPSILON), theta(EPSILON)
    {
        std::string this_node_name = ros::this_node::getName();
        if(!nh.getParam(this_node_name + "/wheel_radius", radius)) throw std::runtime_error("insert radius!");
        if(!nh.getParam(this_node_name + "/wheel_seperation", wheel_seperation)) throw std::runtime_error("insert wheel_seperation!");
        std::string right_vel_topic_name, left_vel_topic_name;
        if(!nh.getParam(this_node_name + "/right_vel_topic_name", right_vel_topic_name)) throw std::runtime_error("insert right_vel_topic_name!");
        if(!nh.getParam(this_node_name + "/left_vel_topic_name", left_vel_topic_name)) throw std::runtime_error("insert left_vel_topic_name!");
        std::string odom_topic_name;
        if(!nh.getParam(this_node_name + "/odom_topic_name", odom_topic_name)) throw std::runtime_error("insert odom_topic_name!");

        right_vel_sub = nh.subscribe(right_vel_topic_name.c_str(), 100, &DiffOdometry::rightSubCB, this);
        left_vel_sub = nh.subscribe(left_vel_topic_name.c_str(), 100, &DiffOdometry::leftSubCB, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic_name.c_str(), 100);
        ROS_INFO("start DiffOdom end!");
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber right_vel_sub, left_vel_sub;
    ros::Publisher odom_pub;
    double radius;
    double wheel_seperation;
    double vr;
    double vl;
    double x, y, theta;
    ros::Time last_time;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "diff_odometry");
    DiffOdometry o;
    std::thread([&o](){
        ros::Rate loop_rate(20);
        while(ros::ok()){
            o.updateOdom();
            loop_rate.sleep();
        }
    }).detach();
    ros::spin();
}