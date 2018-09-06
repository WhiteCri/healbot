#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <string>
#include <memory>
#include <signal.h>

#define EPSILON 0.0000001

//pid model : x = c*u
class Motor{
public:
    void motorDriverCB(const dynamixel_msgs::JointStateConstPtr& ptr){
        //cut outliler and save velocity
        const double lower_bound = last_angular_velocity - outlier_gap_bound;
        const double upper_bound = last_angular_velocity + outlier_gap_bound;
        double new_velocity = (reverse_rotation) ?
                    -ptr->velocity : ptr->velocity;
        if ((lower_bound < new_velocity ) && (new_velocity < upper_bound))
            last_angular_velocity = new_velocity;
        
        //exception handling for that new_velocity was 0
        if (abs(last_angular_velocity) < EPSILON)
            if((-3 <= new_velocity) && (new_velocity <= 3))//exception handling for outlier
                last_angular_velocity = new_velocity;
        //ROS_INFO("last angular velocity : %lf", last_angular_velocity);
        //publish filtered velocity
        std_msgs::Float64 filtered_angular_velocity_msg;
        filtered_angular_velocity_msg.data = last_angular_velocity;
        filtered_angular_velocity_pub.publish(filtered_angular_velocity_msg);
        pidContol();
    }

    void goalVelocityCB(const std_msgs::Float64ConstPtr& ptr){
        //assume that goal is velocity.
        //input: velocity/2
        goal_angular_velocity = ptr->data/2;
        ROS_WARN("reference input, last_state, error : %lf %lf %lf", goal_angular_velocity, last_angular_velocity,
            goal_angular_velocity - last_angular_velocity);
    }

    void pidContol() {
        double error = goal_angular_velocity - last_angular_velocity;
        double e_dot = error - old_error;
        tot_error += error;

        std_msgs::Float64 msg;
        msg.data = last_angular_velocity + kp*error;// + ki*tot_error + kd*e_dot;
        msg.data *= 2;
        if (reverse_rotation)
            msg.data *= -1;
        angular_velocity_pub.publish(msg);

        old_error = error;
    }

    Motor() {
        //load param
        const std::string& this_node_name = ros::this_node::getName();
        if(!nh.getParam(this_node_name + "/driver_node_name", driver_node_name)) 
            throw std::runtime_error("PLZ insert right node name for dynamixel...");
        if(!nh.getParam(this_node_name + "/radius", radius))
            throw std::runtime_error("PLZ insert radius");
        if(!nh.getParam(this_node_name + "/kp", kp)) throw std::runtime_error("PLS insert kp");
        if(!nh.getParam(this_node_name + "/ki", ki)) throw std::runtime_error("PLS insert ki");
        if(!nh.getParam(this_node_name + "/kd", kd)) throw std::runtime_error("PLS insert kd");
        nh.getParam(this_node_name + "/reverse_rotation", reverse_rotation);
        nh.getParam(this_node_name + "/outlier_gap_bound", outlier_gap_bound);
        std::string goal_topic_name;
        if(!nh.getParam(this_node_name + "/goal_topic_name", goal_topic_name))
            throw std::runtime_error("pls insert goal_topic_name!");

        //set publisher, subscriber

        std::string motor_driver_sub_topic_name = driver_node_name + "/state";
        state_sub = nh.subscribe(motor_driver_sub_topic_name.c_str(), 10, &Motor::motorDriverCB, this);

        goal_sub = nh.subscribe(goal_topic_name.c_str(), 10, &Motor::goalVelocityCB, this);

        std::string pub_topic_name = driver_node_name + "/command";
        angular_velocity_pub = nh.advertise<std_msgs::Float64>(pub_topic_name.c_str(), 10);

        std::string debug_topic_name = driver_node_name + "/filtered_velocity";
        filtered_angular_velocity_pub = nh.advertise<std_msgs::Float64>(debug_topic_name.c_str(), 10);
    }

    ~Motor(){
        std_msgs::Float64 msg;
        msg.data = 0;
        for(int i = 0 ; i < 5; ++i){ //to ensure stop motor,publish topic 5 times.
            angular_velocity_pub.publish(msg);
            ros::Rate(20).sleep();
            ROS_WARN("stop dynamixel...");
        }
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber goal_sub;
    ros::Publisher angular_velocity_pub;
    ros::Publisher filtered_angular_velocity_pub;
    std::string driver_node_name;
    bool reverse_rotation;

    double goal_angular_velocity;
    double outlier_gap_bound;
    double last_angular_velocity;
    double tot_error;
    double radius;

    double old_error;
    double kp, ki, kd;
};

std::shared_ptr<Motor> motorPtr;

void mySigInt(int sig){
    motorPtr =  nullptr;
    ros::shutdown();
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "motor_controller");
    
    motorPtr = std::make_shared<Motor>();
    signal(SIGINT, mySigInt);
    ROS_INFO("start..");

    ros::spin();
}
