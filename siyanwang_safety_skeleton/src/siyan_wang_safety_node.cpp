#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <vector>
// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    float step;
    float min; // minimum angle
    float min_TTC;

    ackermann_msgs::AckermannDriveStamped current_speed;
    std_msgs::Bool stop;

    std::vector<float> distance ;
    std::vector<float> angle;
    std::vector<float> TTC;

    std::vector<float>::iterator it;

    ros::Subscriber sub_to_scan;
    ros::Subscriber sub_to_odom;
    ros::Publisher pub_to_brake;
    ros::Publisher pub_to_brakebool;
    // TODO: create ROS subscribers and publishers

public:
    Safety() {
        //n = ros::NodeHandle();
        speed = 0.0;

        pub_to_brake = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake",1000);
        pub_to_brakebool = n.advertise<std_msgs::Bool>("brake_bool",1000);
        sub_to_scan = n.subscribe("scan",1000,&Safety::scan_callback,this);
        sub_to_odom = n.subscribe("odom",1000,&Safety::odom_callback,this);

        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        
    }


    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        angle.clear();
        TTC.clear();

        distance = scan_msg->ranges;
        step = scan_msg->angle_increment;
        min = scan_msg->angle_min;

        for(int i =0; i<distance.size();i++){

            if(std::isnan(distance[i])||std::isinf(distance[i])){
                continue;
            } else{
                angle.push_back(min + i*step);
            }

        }

        for(it=distance.begin();it!=distance.end();it++){

            if(std::isnan(*it) ||std::isinf(*it)){
                distance.erase(it);
            }
        }


        for(int j = 0;j<distance.size();j++){

            float r_dot = std::max(0.0,speed*cos(angle[j]));
            TTC.push_back(distance[j]/r_dot);

        }

        min_TTC = TTC[0];

        for(int k = 1; k < TTC.size();k++){
                if(TTC[k]>min_TTC){
                    continue;
                }else{
                    min_TTC = TTC[k];
                }

        }


        if(min_TTC < 0.4){
            current_speed.drive.speed =0;
            pub_to_brake.publish(current_speed);

            stop.data = true;

            pub_to_brakebool.publish(stop);
        }




      //  ROS_INFO("TTC 2: %f",min_TTC);
        

    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "siyan_wang_safety_node");
    Safety sn;
    ros::spin();
    return 0;
}
