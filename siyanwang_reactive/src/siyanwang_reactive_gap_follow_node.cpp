/*
 * Author: Siyan Wang
 * Date: 2019-09-25
 * Description: Reactive Methods Lab
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <algorithm>
#include <vector>
#include<numeric>


class reactive_follow_gap {

private:
    ros::NodeHandle n;
    ros::Subscriber lidar_sub;
    ros::Publisher drive_pub;
    ackermann_msgs::AckermannDriveStamped ignite;


    int endIndex = 0;
    int startIndex = 0;
    const float PI = 3.14159265;

public:
    reactive_follow_gap() {

        lidar_sub = n.subscribe("scan", 1, &reactive_follow_gap::lidar_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
    }

    // this function is used to smooth the lidar data.
    static void filter(std::vector<float> &ranges) {

        int N = 5; //window size
        float sum = 0;

        for (int i = 0; i < ranges.size() - N; i++) {
            sum = std::accumulate(ranges.begin() + i, ranges.begin() + i + N, 0.0);
            ranges[i] = sum / N; // Moving mean filter
        }
    }


    static void preprocess_lidar(std::vector<float> &ranges) {

        /* 1. find closet point within 240 degree of lidar scan
         * 2. find the min value position
         * 3. set min and around 10 elements to be zero
         * */


        std::vector<float>::iterator minValue;
        minValue = std::min_element(ranges.begin() + 297, ranges.end() - 297); // min Value with in 240 degrees of scan
        int minPosition = std::distance(ranges.begin(), minValue);// position of min value

        for (int i = minPosition - 10; i <= minPosition + 10; i++) {
            ranges[i] = 0;
        }

    }

    void find_max_gap(std::vector<float> &free_space_ranges) {
        /*
         * 1. loop the ranges vector
         * 2. check whether the element is 0, inf or nan
         * 3. change the end index and start index when find the max gap.
         * 4. end index and start idnex are global variables.
         */

        int length = 0;
        int temp = 0;

        for (int i = 297; i < free_space_ranges.size() - 297; i++) {

            bool flag_inf = std::isinf(free_space_ranges[i]);
            bool flag_nan = std::isnan(free_space_ranges[i]);

            if (!flag_inf && !flag_nan && free_space_ranges[i] >= 1.5) {
                length++;

            }

            if ((free_space_ranges[i] < 1.5) || (i == free_space_ranges.size() - 298)) {
                if (length > temp) {

                    temp = length;
                    endIndex = i;
                    startIndex = endIndex - length;
                    length = 0;

                } else {

                    length = 0;

                }
            }
        }

    }


    static int find_best_point(int endPoint, int startPoint) {

        int maxIndex = 0;

        maxIndex = startPoint + (endPoint - startPoint) / 2;

        return maxIndex;

    }


    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {


        std::vector<float> ranges = scan_msg->ranges;

        filter(ranges); // moving mean filter
        preprocess_lidar(ranges);
        find_max_gap(ranges);

        int bestPoint = find_best_point(endIndex, startIndex);

        float angle = -((float) 540 - (float) bestPoint) * scan_msg->angle_increment;

        ignite.drive.steering_angle = angle;

        ignite.drive.speed = 2;

        ROS_INFO_STREAM(angle);

        drive_pub.publish(ignite);

    }


};


int main(int argc, char **argv) {
    ros::init(argc, argv, "siyanwang_reactive_gap_follow_node");
    reactive_follow_gap gf;
    ros::spin();
    return 0;
}
