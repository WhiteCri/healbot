#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define EPSILON 0.000001

class VelProcessor{
public:
    void cmdVelCB(const geometry_msgs::TwistConstPtr& ptr){
        if ((fabs(ptr->linear.x) < EPSILON) && (fabs(ptr->angular.z) < EPSILON)) {
            ROS_INFO("cmd_vel_noise");
            return;
        }
        
        double v = ptr->linear.x;
        double w = ptr->angular.z;

        double vr = (2*v + w*wheel_seperation)/2;
        double vl = (2*v - w*wheel_seperation)/2;

        std_msgs::Float64 vr_msg, vl_msg;
        vr_msg.data = vr;
        vl_msg.data = vl;
        right_pub.publish(vr_msg);
        left_pub.publish(vl_msg);
        ROS_INFO("v, w, vr, vl : %lf %lf %lf %lf",v, w, vr, vl);
    }

    VelProcessor(){
        std::string right_controller_topic_name, left_controller_topic_name;
        if (!nh.getParam("/right_controller_topic_name", right_controller_topic_name))
            throw std::runtime_error("pls set right controller topic name!");
        if (!nh.getParam("/left_controller_topic_name", left_controller_topic_name))
            throw std::runtime_error("pls set left controller topic name!");
        if (!nh.getParam("/wheel_seperation", wheel_seperation))
            throw std::runtime_error("pls set wheel_seperation!");

        right_pub = nh.advertise<std_msgs::Float64>(right_controller_topic_name.c_str(), 10);
        left_pub = nh.advertise<std_msgs::Float64>(left_controller_topic_name.c_str(), 10);
    }

    ~VelProcessor(){//stop robot
        for(int i = 0 ; i < 5; ++i){
            std_msgs::Float64 vr_msg, vl_msg;
            vr_msg.data = 0;
            vl_msg.data = 0;
            right_pub.publish(vr_msg);
            left_pub.publish(vl_msg);
            ros::Rate(10).sleep();
        }
    }
private:
    ros::NodeHandle nh;
    ros::Publisher right_pub, left_pub;
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, &VelProcessor::cmdVelCB, this);
    double wheel_seperation;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cmd_vel_diff_processor");

    VelProcessor v;
    ros::spin();
}