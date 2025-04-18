#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <cmath>

// WGS84 constants
constexpr double a = 6378137.0;               // semi-major axis
constexpr double f = 1.0 / 298.257223563;     // flattening
constexpr double e_sq = f * (2 - f);          // eccentricity squared

// Reference GPS (set from first valid reading or launch params)
bool reference_set = false;
double lat_ref, lon_ref, alt_ref;
double Xr, Yr, Zr;  // Reference ECEF

ros::Publisher gps_odom_pub;
tf::TransformBroadcaster* tf_broadcaster;

// Last ENU position (for heading computation)
bool last_pose_set = false;
double last_x, last_y;

void gpsToECEF(double lat, double lon, double alt, double& X, double& Y, double& Z) {
    lat = lat * M_PI / 180.0;
    lon = lon * M_PI / 180.0;
    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double sin_lon = sin(lon);
    double cos_lon = cos(lon);
    double N = a / sqrt(1 - e_sq * sin_lat * sin_lat);

    X = (N + alt) * cos_lat * cos_lon;
    Y = (N + alt) * cos_lat * sin_lon;
    Z = ((1 - e_sq) * N + alt) * sin_lat;
}

void ecefToENU(double X, double Y, double Z, double& xEast, double& yNorth, double& zUp) {
    double lat = lat_ref * M_PI / 180.0;
    double lon = lon_ref * M_PI / 180.0;

    double dx = X - Xr;
    double dy = Y - Yr;
    double dz = Z - Zr;

    xEast  = -sin(lon) * dx + cos(lon) * dy;
    yNorth = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
    zUp    =  cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!reference_set) {
        if (msg->status.status < 0) return; // wait for valid GPS
        lat_ref = msg->latitude;
        lon_ref = msg->longitude;
        alt_ref = msg->altitude;
        gpsToECEF(lat_ref, lon_ref, alt_ref, Xr, Yr, Zr);
        reference_set = true;
        ROS_INFO("Reference GPS set: lat=%.8f, lon=%.8f, alt=%.2f", lat_ref, lon_ref, alt_ref);
        return;
    }

    double X, Y, Z;
    gpsToECEF(msg->latitude, msg->longitude, msg->altitude, X, Y, Z);

    double x, y, z;
    ecefToENU(X, Y, Z, x, y, z);

    // Heading computation (from consecutive ENU points)
    double heading = 0.0;
    if (last_pose_set) {
        double dx = x - last_x;
        double dy = y - last_y;
        if (fabs(dx) > 1e-6 || fabs(dy) > 1e-6) {
            heading = atan2(dy, dx);
        }
    }
    last_x = x;
    last_y = y;
    last_pose_set = true;

    ros::Time now = msg->header.stamp;

    // Create Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "gps";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(heading);
    odom.pose.pose.orientation = q;

    gps_odom_pub.publish(odom);

    // Broadcast TF from odom → gps
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion tf_q;
    tf_q.setRPY(0, 0, heading);
    transform.setRotation(tf_q);
    tf_broadcaster->sendTransform(tf::StampedTransform(transform, now, "odom", "gps"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle nh;

    ros::Time::waitForValid();

    ros::NodeHandle private_nh("~");
    double p_lat_r, p_lon_r, p_alt_r;
    if (private_nh.getParam("lat_r", p_lat_r) && private_nh.getParam("lon_r", p_lon_r) && private_nh.getParam("alt_r", p_alt_r)) {
        ROS_INFO("Set GPS references from params: lat_r=%f lon_r=%f alt_r=%f", p_lat_r, p_lon_r, p_alt_r);
        lat_ref = p_lat_r;
        lon_ref = p_lon_r;
        alt_ref = p_alt_r;
        reference_set = true;
    } else {
        ROS_INFO("GPS reference not found in params, going to use first message");
    }

    gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 50);
    tf_broadcaster = new tf::TransformBroadcaster();

    ros::Subscriber gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 1000, gpsCallback);

    ROS_INFO("GPS Odometer node started.");

    ros::spin();

    delete tf_broadcaster;
    return 0;
}
