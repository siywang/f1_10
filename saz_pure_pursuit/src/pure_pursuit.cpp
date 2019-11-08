#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <math.h>

#include "read_way_point_CSVfile.h"


using namespace std;
// TODO: include ROS msg type headers and libraries you need


class PurePursuit {
private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Publisher vis_pub;
    ros::Subscriber pose_sub;
    vector<vector<float>> waypoint_data;
    visualization_msgs::Marker marker;

    float angle_factor = 0.1;
    float look_ahead_distance = 0.7;
    float nominal_speed = 3;
    float angle_speed = 1.8;
    int first_time = 1;

    float steering_angle = 0.0;
    int last_index = -1;
    float last_distance = 0;
    int waypoint_length = 0;
    float rot_waypoint_x = 0;
    float rot_waypoint_y = 0;
    float steering_offset = -0.005;
    

public:
    PurePursuit() {
        n = ros::NodeHandle();
        // n.param<float>("/pure_pursuit/look_ahead_distance", look_ahead_distance, 1);
        // n.param<float>("/pure_pursuit/angle_factor", angle_factor, 0.05);
        // n.param<float>("/pure_pursuit/nominal_speed", nominal_speed, 0.5);
        // n.param<float>("/pure_pursuit/angle_speed", angle_speed, 0.3);

        //  waypoint_data = read_way_point_CSVfile("/home/anmol/anmol_kathail_ws/src/anmolk_pure_pursuit/gtpose.csv");
        // load the waypoints and simplify them
        vector<vector<float>> waypoint_data_long = read_way_point_CSVfile("/home/siyan/SiyanWang_ws/src/f1_10/saz_pure_pursuit/saz.csv");
        waypoint_length = waypoint_data_long[0].size();
        vector<float> waypoint_data1;
        vector<float> waypoint_data2;
        vector<float> waypoint_data3;
        for (int i = 0; i < waypoint_length; i+=10) {
            waypoint_data1.push_back(waypoint_data_long[0][i]);
            waypoint_data2.push_back(waypoint_data_long[1][i]);
            waypoint_data3.push_back(waypoint_data_long[2][i]);
        }
        waypoint_data.push_back(waypoint_data1);
        waypoint_data.push_back(waypoint_data2);
        waypoint_data.push_back(waypoint_data3);
        waypoint_length = waypoint_data[0].size();
        
         //pose_sub = n.subscribe("gt_pose", 1, &PurePursuit::pose_callback, this); //simulator topic that publishes ground truth pose of the car
        pose_sub = n.subscribe("pf/pose/odom", 1, &PurePursuit::pose_callback, this);
        // drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1", 1); //publishes steering angle and velocity
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1); //publishes steering angle and velocity
        vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

        marker.header.frame_id = "map";
        marker.ns = "my_namespace";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.id = 0;
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) { //geometry_msgs::PoseStamped
        // find the current waypoint to track using methods mentioned in lecture
	if(!checkInfOrNan(odom_msg->pose.pose) ) {
		const geometry_msgs::Pose pose_msg = odom_msg->pose.pose;
		double currentX = pose_msg.position.x; //vehicle pose X
		double currentY = pose_msg.position.y; //vehicle pose Y
		double currentTheta = convert_to_Theta(pose_msg.orientation); //vehicle orientation theta converted from quaternion to euler angle

		float csv_offset_x = 0;
		float csv_offset_y = 0;
		float csv_offset_theta = 0;
		
		float waypoint_x;
		float waypoint_y;
		float waypoint_distance = sqrt( pow(currentX - waypoint_x, 2) + pow(currentY - waypoint_y, 2) );
		float distance_min = 10000;
		float ind_min = 0;
        
		for(int i = 0; i < waypoint_length; i += 1){
		    float distance = sqrt( pow(currentX - (waypoint_data[0][i] - csv_offset_x), 2) + pow(currentY - (waypoint_data[1][i] - csv_offset_y), 2) );
		    rot_waypoint_x = (waypoint_data[0][i] - currentX) * cos(-currentTheta) - (waypoint_data[1][i] - currentY) * sin(-currentTheta);
		    rot_waypoint_y = (waypoint_data[0][i] - currentX) * sin(-currentTheta) + (waypoint_data[1][i] - currentY) * cos(-currentTheta);   
		    if (distance_min > distance && distance >= look_ahead_distance && rot_waypoint_x > 0) {
		        distance_min = distance;
		        ind_min = i;
		    }
		}
        
		last_index = ind_min;
		waypoint_x = waypoint_data[0][last_index];
		waypoint_y = waypoint_data[1][last_index];
        
		rot_waypoint_x = (waypoint_x - currentX) * cos(-currentTheta) - (waypoint_y - currentY) * sin(-currentTheta);
		rot_waypoint_y = (waypoint_x - currentX) * sin(-currentTheta) + (waypoint_y - currentY) * cos(-currentTheta);       
		steering_angle = angle_factor * (2 * rot_waypoint_y) / (pow(rot_waypoint_x, 2) + pow(rot_waypoint_y, 2));
		steering_angle += steering_offset;
		
		setAngleAndVelocity(steering_angle);
		
		marker.header.frame_id = "map";
		marker.pose.position.x = waypoint_x;
		marker.pose.position.y = waypoint_y;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.id += 1;
		marker.header.stamp = ros::Time();
		marker.lifetime = ros::Duration(0.1);
		vis_pub.publish(marker);

		// marker.header.frame_id = "laser";
		// marker.pose.position.x = rot_waypoint_x;
		// marker.pose.position.y = rot_waypoint_y;
		// marker.color.r = 0.0;
		// marker.color.g = 0.0;
		// marker.color.b = 1.0;
		// marker.id += 1;
		// marker.header.stamp = ros::Time();
		// marker.lifetime = ros::Duration(0.1);
		// vis_pub.publish(marker);
		// }
	}
  
    }

    bool checkInfOrNan(const geometry_msgs::Pose pose_msg){

        if(std::isnan(pose_msg.position.x) || std::isnan(pose_msg.position.y) || std::isnan(pose_msg.position.z) || std::isinf(pose_msg.position.x) || std::isinf(pose_msg.position.y) || std::isinf(pose_msg.position.z) ) {
	return true;
	}
	
	if(std::isnan(pose_msg.orientation.x) || std::isnan(pose_msg.orientation.y) || std::isnan(pose_msg.orientation.z) || std::isnan(pose_msg.orientation.w) || std::isinf(pose_msg.orientation.x) || std::isinf(pose_msg.orientation.y) || std::isinf(pose_msg.orientation.z) || std::isinf(pose_msg.orientation.w) ) {
		return true;
		}

	
	return false;
    }

    double convert_to_Theta(const geometry_msgs::Quaternion msg){
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        //theta is just the yaw angle
        return yaw;
    }

    void setAngleAndVelocity(float u) {

        ackermann_msgs::AckermannDriveStamped drive_msg;
        if(u < -0.4189){
            u = -0.4189;
        }
        if(u > 0.4189){
            u = 0.4189;
        }

        drive_msg.drive.steering_angle = u; //Sets steering angle
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.speed = nominal_speed - (nominal_speed - angle_speed) * fabs(u) / 0.4189;
        drive_pub.publish(drive_msg); //Sets velocity based on steering angle conditions

    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit");
    PurePursuit pp;
    ros::spin();
    return 0;
}
