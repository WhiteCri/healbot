#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <algorithm>

#define EPSILON 0.000001
#define MISSING_TARGET -1000

struct Box{
    struct Pos{
        Pos() : x(0.0), y(0.0) {}
        Pos(double x, double y): x(x), y(y) {}
        double x, y;
    };

    Box() {}

    Box(std::vector<double>&& pos){
        if (pos.size() != 8) { //assert
            ROS_ERROR("Box() failed because of the lack pos information : %ld", pos.size());
            exit(-1);
        }

        for (int i = 0 ; i < 8 ; ++i){ //save
            if (i%2) posAry[(i/2)].y = pos[i];
            else posAry[(i/2)].x = pos[i];
        }

        std::sort(posAry, posAry+4, [](const Pos& f, const Pos& s){
            return f.x < s.x;
        }); //sort by x
        
        //sort by y for [0] [1], [2] [3]. 
        //then the result in posAry would be top-left, bottom-left, bottom-right, bottom-up
        if (posAry[0].y < posAry[1].y) std::swap(posAry[0], posAry[1]);
        if (posAry[2].y > posAry[3].y) std::swap(posAry[2], posAry[3]);

        for (int i = 0 ; i < 4; ++i)
            ROS_INFO("result of square : %lf, %lf", posAry[i].x, posAry[i].y);
    }
    
    bool in(double x, double y){
        //we can divide square to the triangle of two.
        //then, when the pos is in the triangles, we can say that the pos is in the triangle.
        //the condition of the point and triangle is based on the area of triangle
        static auto calc_triangle_area = [](Pos f, Pos s, Pos t)->double { //first, second, third
            double area = fabs((f.x*(s.y-t.y) + s.x*(t.y-f.y) + t.x*(f.y-s.y))/2);
            return area;
        };

        Pos p(x,y);
        //0,1,2 2,3,0
        double area_with_point1 = 
            calc_triangle_area(p, posAry[0], posAry[1]) +
            calc_triangle_area(p, posAry[1], posAry[2]) +
            calc_triangle_area(p, posAry[2], posAry[0]);
        double area_without_point1 = 
            calc_triangle_area(posAry[0], posAry[1], posAry[2]);
        if (fabs(area_with_point1 - area_without_point1) <= EPSILON) return true; //the point is in first triangle

        double area_with_point2 = 
            calc_triangle_area(p, posAry[2], posAry[3]) +
            calc_triangle_area(p, posAry[3], posAry[0]) +
            calc_triangle_area(p, posAry[0], posAry[2]);
        double area_without_point2 = 
            calc_triangle_area(posAry[2], posAry[3], posAry[0]);
        if (fabs(area_with_point2 - area_without_point2) <= EPSILON) return true; //the point is in first triangle
        return false;
    }
public:
    Pos posAry[4];
};

class ObjectTracker{
public:
    void detectClosestObject(const obstacle_detector::ObstaclesConstPtr& ptr){
        if (!ptr->segments.size())
            return;
        
        //ensure that this function works when the target is selected.
        double target_x, target_y;
        bool no_target=false;
        if(!nh.getParam("target_middle_point_x", target_x)) no_target = true;
        if(!nh.getParam("target_middle_point_y", target_y)) no_target = true;
        if (fabs((target_x - (MISSING_TARGET))) < EPSILON) no_target = true;
        if (fabs((target_y - (MISSING_TARGET))) < EPSILON) no_target = true; // when the obj distance is -1000(-1KM), the obj is missing.
        if (no_target){
            last_time = ros::Time::now();
            detectNewObject(ptr);
            return;
        }

        geometry_msgs::Point middle_point;
        middle_point.x = target_x;
        middle_point.y = target_y;

        //next target is closest thing.
        std::vector<double> dist_vec;
        std::for_each(ptr->segments.begin(), ptr->segments.end(), [&dist_vec, &middle_point](const obstacle_detector::SegmentObstacle& obj){
            geometry_msgs::Point mid;
            mid.x = (obj.first_point.x + obj.last_point.x) / 2;
            mid.y = (obj.first_point.y + obj.last_point.y) / 2;

            double dist = 
                std::sqrt(
                    std::pow(mid.x - middle_point.x, 2) + 
                    std::pow(mid.y - middle_point.y, 2)
                    );
            dist_vec.push_back(dist);
        });
        int target_idx = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();

        //cut when the difference is to big
        ros::Time cur_time = ros::Time::now();
        double distance_tolerace = target_distance_tolerance * (cur_time - last_time).toSec();
        
        if (dist_vec[target_idx] >= distance_tolerace) target_idx = -1;
        last_time = cur_time;

        //set next target
        obstacle_detector::SegmentObstacle target_obj = ptr->segments[target_idx];
        double next_x, next_y;
        if (target_idx == -1){
            ROS_WARN("MISSING OBJECT...");
            next_x = MISSING_TARGET;
            next_y = MISSING_TARGET;
        }
        else {
            next_x = (target_obj.first_point.x + target_obj.last_point.x) / 2;
            next_y = (target_obj.first_point.y + target_obj.last_point.y) / 2;
        }
        nh.setParam("target_middle_point_x", next_x);
        nh.setParam("target_middle_point_y", next_y);

        //pub for debugging
        obstacle_detector::Obstacles msg;
        msg.segments.push_back(target_obj);
        msg.header.frame_id = "laser";
        msg.header.stamp = ros::Time::now();
        msg.header.seq +=1;
        target_obj_pub.publish(msg);
        
    }

    void detectNewObject(const obstacle_detector::ObstaclesConstPtr& ptr){
        //find closest obstacle
        std::vector<double> dist_vec;
        std::for_each(ptr->segments.begin(), ptr->segments.end(), [&dist_vec](const obstacle_detector::SegmentObstacle& obj){
            double dist = std::sqrt(
                std::pow((obj.first_point.x + obj.last_point.x)/2, 2) +
                std::pow((obj.first_point.y + obj.last_point.y)/2, 2)
            );
            dist_vec.push_back(dist);
        });
        int target_idx = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
        auto target_obj = ptr->segments[target_idx];

        double cand_x = (target_obj.first_point.x + target_obj.last_point.x) / 2;
        double cand_y = (target_obj.first_point.y + target_obj.last_point.y) / 2;

        bool enroll_new_target;
        if(box.in(cand_x, cand_y)){
            if (new_target_detected_start.toSec() < EPSILON)  new_target_detected_start = ros::Time::now();
            else new_target_detected_end = ros::Time::now();
        } else return;
        
        if((new_target_detected_end - new_target_detected_start).toSec() >= target_select_duration){//new target
            nh.setParam("target_middle_point_x", cand_x);
            nh.setParam("target_middle_point_y", cand_y);
            new_target_detected_start = ros::Time(0.0);
            new_target_detected_start = ros::Time(0.0);
            ROS_WARN("new target selected!");
        }
    }
    ObjectTracker() : last_time(ros::Time::now()){
        if(!nh.getParam("target_distance_tolerance", target_distance_tolerance)) 
            throw std::runtime_error("plz set target_distance_tolerance!");
        double x_min, x_max, y_min, y_max;
        if(!nh.getParam("target_select_x_min", x_min)) 
            throw std::runtime_error("plz set target_select_x_min!");
        if(!nh.getParam("target_select_x_max", x_max)) 
            throw std::runtime_error("plz set target_select_x_max!");
        if(!nh.getParam("target_select_y_min", y_min)) 
            throw std::runtime_error("plz set target_select_y_min!");
        if(!nh.getParam("target_select_y_max", y_max)) 
            throw std::runtime_error("plz set target_select_y_max!");
        if(!nh.getParam("target_select_duration", target_select_duration))
            throw std::runtime_error("plz set target_select_duration!");
        box = Box({x_min, y_min, x_max, y_min, x_max, y_max, x_min, y_max});
        target_obj_pub = nh.advertise<obstacle_detector::Obstacles>("lidar2d_target_obj", 10);
        obj_tracker_sub = nh.subscribe("raw_obstacles", 10, &ObjectTracker::detectClosestObject, this);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher target_obj_pub;
    ros::Subscriber obj_tracker_sub;

    double target_distance_tolerance;
    ros::Time last_time;

    double closest_x;
    double closest_y;
    Box box;
    double target_select_duration;
    ros::Time new_target_detected_start, new_target_detected_end;
};


int main(int argc, char *argv[]){
    ros::init(argc, argv, "lidar2d_object_tracker");
    
    ObjectTracker o;
    ros::spin();
}