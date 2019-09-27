//
// Created by siyan on 9/2/19.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"
//#include "siyanwang_roslab/scan_range.h"
#include "scan_range.h"
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <vector>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_1 = n.advertise<std_msgs::Float64>("closest_point",1000);  //closest point topic
        pub_2 = n.advertise<std_msgs::Float64>("farthest_point",1000); // farthest point topic
        pub_3 = n.advertise<siyanwang_roslab::scan_range>("scan_range",1000); //scan range
        sub = n.subscribe("scan",1000,&SubscribeAndPublish::scanCallBack,this);
    }

    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        std::vector<float> arr = msg->ranges;
        std::vector<float>::iterator it;

        for(it=arr.begin();it!=arr.end();it++){
            if(std::isnan(*it) ||std::isinf(*it)){
                arr.erase(it);
            }
        }

        std::sort(arr.begin(),arr.end());
       // ROS_INFO("min: %f ",arr[0]);
        //ROS_INFO("max: %f",arr[arr.size()-1]);
        Closest.data = arr[0];
        Farthest.data = arr[arr.size()-1];
        pub_1.publish(Closest);
        pub_2.publish(Farthest);
        sr.Closest = arr[0];
        sr.Farthest = arr[arr.size()-1];
        pub_3.publish(sr);
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub_1; // closest
    ros::Publisher pub_2; // farthest
    ros::Publisher pub_3;
    ros::Subscriber sub;
    std_msgs::Float64 Closest;
    std_msgs::Float64 Farthest;
    siyanwang_roslab::scan_range sr;
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"lidar_processing");
    SubscribeAndPublish SubAndPub;

    ros::spin();
    return 0;

}