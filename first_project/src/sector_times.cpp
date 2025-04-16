#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <cmath>
#include <vector>




float latest_speed = 0.0;                     // velocità in km/h
double latest_lat = 0.0, latest_lon = 0.0;    // coordinate GPS
bool received_gps = false;                    // per sapere se abbiamo già ricevuto almeno un messaggio

// Callback for /speedsteer
void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    latest_speed = msg->point.y;  // estraiamo la velocità dal campo y (x è lo sterzo)
    ROS_INFO("Velocity riceived: %.2f km/h", latest_speed);
}

// Callback for /swiftnav/front/gps_pose
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    latest_lat = msg->latitude;
    latest_lon = msg->longitude;
    received_gps = true;  // segniamo che abbiamo ricevuto un messaggio
    ROS_INFO("GPS received: lat=%.6f, lon=%.6f", latest_lat, latest_lon);
    //ROS_INFO("GPS received");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle nh;

    ros::Subscriber sub_speed = nh.subscribe("/speedsteer", 10, speedCallback);
    ros::Subscriber sub_gps = nh.subscribe("/swiftnav/front/gps_pose", 10, gpsCallback);

    ROS_INFO("Odometer node started and subscribed to /speedsteer and /swiftnav/front/gps_pose");


    ros::spin();
    return 0;
}
