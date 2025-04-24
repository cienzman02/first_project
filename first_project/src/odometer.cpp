#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double rear_wheel_base = 1.3;     // meters (130 cm)
double front_to_rear = 1.765;     // meters (176.5 cm)
double steering_bias = 0.0;       // to be subtracted from steer_deg
double steering_factor = 32.0;    // steering column to wheel angle factor

ros::Publisher odom_pub;   
tf::TransformBroadcaster* tf_broadcaster;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

ros::Time last_time;

void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // ROS_INFO("odometer: Callback triggered! Speed = %.2f km/h, Steer = %.2f deg", msg->point.y, msg->point.x);

  ros::Time current_time = msg->header.stamp;
  if (last_time.isZero()) {
    last_time = current_time;
    return;
  }

  double dt = (current_time - last_time).toSec();
  last_time = current_time;

  double speed_kmh = msg->point.y;
  double steer_deg = msg->point.x - steering_bias;

  // Convert speed to m/s
  double speed = speed_kmh / 3.6;

  // Convert steering to radians at the wheel (Ackermann)
  double steering_rad = (steer_deg / steering_factor) * M_PI / 180.0;

  // Ackermann bicycle model
  double angular_velocity = speed * tan(steering_rad) / front_to_rear;

  double delta_x = 0.0, delta_y = 0.0, delta_theta = angular_velocity * dt;
  if (abs(angular_velocity) <= 1e-6) {
    delta_x = speed * dt * cos(theta + (angular_velocity * dt) / 2);
    delta_y = speed * dt * sin(theta + (angular_velocity * dt) / 2);
  } else {
    double new_theta = theta + delta_theta;
    delta_x = (speed / angular_velocity) * (sin(new_theta) - sin(theta));
    delta_y = -(speed / angular_velocity) * (cos(new_theta) - cos(theta));
  }

  x += delta_x;
  y += delta_y;
  theta += delta_theta;

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
  
  ros::NodeHandle private_nh("~");
  double steer_p;
  if (private_nh.getParam("steer", steer_p)) {
    steering_factor = steer_p;
    ROS_INFO("odom: set steering factor %f", steering_factor);
  }
  
  double steer_bias_p;
  if (private_nh.getParam("steering_bias", steer_bias_p)) {
    steering_bias = steer_bias_p;
    ROS_INFO("odom: set steering bias %f", steering_bias);
  }

  double initial_theta_p;
  if (private_nh.getParam("initial_theta", initial_theta_p)) {
    theta = initial_theta_p;
    ROS_INFO("odom: set initial theta %f", theta);
  }
  
  // Wait for /clock if using rosbag --clock
  ros::Time::waitForValid();

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
  tf_broadcaster = new tf::TransformBroadcaster();

  ros::Subscriber sub = nh.subscribe("/speedsteer", 1000, callback);

  ROS_INFO("odomter: Odometer node started and subscribed to /speedsteer");

  ros::spin();

  delete tf_broadcaster;
  return 0;
}
