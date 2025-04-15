#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double rear_wheel_base = 1.3;     // meters (130 cm)
double front_to_rear = 1.765;     // meters (176.5 cm)
double steering_factor = 32.0;    // steering column to wheel angle factor

ros::Publisher odom_pub;   
tf::TransformBroadcaster* tf_broadcaster;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

ros::Time last_time;

void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  ROS_INFO("Callback triggered! Speed = %.2f km/h, Steer = %.2f deg", msg->point.y, msg->point.x);

  ros::Time current_time = msg->header.stamp;
  if (last_time.isZero()) {
    last_time = current_time;
    return;
  }

  double dt = (current_time - last_time).toSec();
  last_time = current_time;

  double speed_kmh = msg->point.y;
  double steer_deg = msg->point.x;

  // Convert speed to m/s
  double speed = speed_kmh / 3.6;

  // Convert steering to radians at the wheel (Ackermann)
  double steering_rad = (steer_deg / steering_factor) * M_PI / 180.0;

  // Ackermann bicycle model
  double angular_velocity = speed * tan(steering_rad) / front_to_rear;
  double delta_theta = angular_velocity * dt;

  double delta_x = speed * cos(theta + delta_theta / 2) * dt;
  double delta_y = speed * sin(theta + delta_theta / 2) * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  // Normalize theta to [-pi, pi]
  theta = atan2(sin(theta), cos(theta));

  // Publish Odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "vehicle";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear.x = speed;
  odom.twist.twist.angular.z = angular_velocity;

  odom_pub.publish(odom);

  // Publish TF
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, theta);
  transform.setRotation(q);
  tf_broadcaster->sendTransform(tf::StampedTransform(transform, current_time, "odom", "vehicle"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");
  ros::NodeHandle nh;

  // Wait for /clock if using rosbag --clock
  ros::Time::waitForValid();

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
  tf_broadcaster = new tf::TransformBroadcaster();

  ros::Subscriber sub = nh.subscribe("/speedsteer", 1000, callback);

  ROS_INFO("Odometer node started and subscribed to /speedsteer");

  ros::spin();

  delete tf_broadcaster;
  return 0;
}
