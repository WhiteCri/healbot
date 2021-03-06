#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "fake_controller");
    ros::NodeHandle nh;

    double default_vel;
    nh.param("/fake_controller_default_vel", default_vel, 0.05);

    ros::Publisher r_pub, l_pub;
    r_pub = nh.advertise<std_msgs::Float64>("/right_motor_goal", 10);
    l_pub = nh.advertise<std_msgs::Float64>("/left_motor_goal", 10);

    char ch;
    ros::Rate loop_rate(20);
    while(ros::ok()){
        ch = getchar();
        std_msgs::Float64 r_msg, l_msg;

        switch(ch){
        case 'w':
            ROS_INFO("straight!");
            r_msg.data = default_vel;
            l_msg.data = default_vel;
            break;
        case 'a':
            ROS_INFO("left!");
            r_msg.data = default_vel;
            l_msg.data = 0;
            break;
        case 'd':
            ROS_INFO("right!");
            r_msg.data = 0;
            l_msg.data = default_vel;
            break;
        case 's':
            ROS_INFO("backward!");
            r_msg.data = -default_vel;
            l_msg.data = -default_vel;
            break; 
        case ' ':
            ROS_INFO("stop!");
            r_msg.data = 0;
            l_msg.data = 0;
            break;
        case '\n':
            continue;
        default:
            ROS_INFO("only asdf and space allowed");
            break;
        }

        r_pub.publish(r_msg);
        l_pub.publish(l_msg);
        loop_rate.sleep();
    }
}