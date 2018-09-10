#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define EPSILON 0.000001

class VelProcessor{
public:
    void cmdVelCB(const geometry_msgs::TwistConstPtr& ptr){
        if ((abs(ptr->linear.x) < EPSILON) && (abs(ptr->angular.z) < EPSILON)) return;
        
        double v = ptr->linear.x;
        double w = ptr->angular.z;

        double vr = (2*v + w*wheel_seperation)/2;
        double vl = (2*v - w*wheel_seperation)/2;

        right_pub.publish(vr);
        left_pub.publish(vl);
        ROS_INFO("vr, vl : %lf %lf", vr, vl);
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
private:
    ros::NodeHandle nh;
    ros::Publisher right_pub, left_pub;
    double wheel_seperation;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cmd_vel_diff_processor");

    VelProcessor v;
    ros::spin();
}