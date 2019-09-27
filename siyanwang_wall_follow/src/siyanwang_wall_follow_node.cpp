#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <vector>

class WallFollow{
 //the class that handles wall following
private:
    ros::NodeHandle n;
    ros::Subscriber lidar_sub;
    ros::Publisher drive_pub;

    // Wall Follow Params
    const float DESIRED_DISTANCE_RIGHT = 0.9; // meters
    const float DESIRED_DISTANCE_LEFT = 0.95;
    const float CAR_LENGTH = 0.50; // Traxxas Rally is 20 inches or 0.5 meters
    const float PI = 3.14159265;
    float VELOCITY = 0.0; // meters per second



    //PID control params
    double kp = 8;
    double kd = 0.09;
    double ki = 0.0;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    double steering_angle = 0.0;
    ackermann_msgs::AckermannDriveStamped acker ;

public:

    WallFollow()// Implement Wall Following on the car
    {
        lidar_sub = n.subscribe("scan",1000,&WallFollow::lidar_callback,this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav",1000);
    }


    double getRange(std::vector<float> &data, double & angle)
    {
        //  make sure to take care of nans etc
        // 0 degree is along x axis.

        int index = (int)angle*270 / 90;
        double theta_rad = (angle * PI) / 180;

        float b = data[810];// directly to the right ;
        float a = data[810-index]; // a distance
        double Dt = 0.0;
        double Dt_future = 0.0;

        double alpha = atan((a*cos(theta_rad) - b)/(a*sin(theta_rad)));

        Dt = b * cos(alpha);
        Dt_future = Dt + CAR_LENGTH * sin(alpha);

        return Dt_future;
    }


    void pid_control(double & err, float & velocity)
    {
        integral+=err;
        servo_offset = kp * err + ki * integral + kd * (err-prev_error);
        prev_error = err;

        steering_angle =   -(servo_offset/0.1) * (5 *PI)/180;

        if (0 <= steering_angle < 10*PI/180){
            VELOCITY = 1.5;
        }else if (10*PI/180 <= steering_angle < 20*PI/180){
            VELOCITY = 1.0;
        }else{
            VELOCITY = 0.5;
        }

        acker.drive.steering_angle = steering_angle ;
        acker.drive.speed=VELOCITY;
        drive_pub.publish(acker);
    }


    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {

        std::vector<float> ranges = scan_msg->ranges;
        double angle = 50; // in degree to determine laser beam a;
        double distance =  getRange(ranges,angle);

        error =   DESIRED_DISTANCE_LEFT - distance ; // err
        pid_control(error,VELOCITY);

        ROS_INFO("steering angle: %f",steering_angle);

    }

};


int main(int argc, char ** argv)
{
    ros::init(argc,argv,"siyanwang_wall_follow_node");
    WallFollow wf;
    ros::spin();
    return 0;
}