#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>

double distance_threshold;
double time_threshold;
int max_msg_discards;
nav_msgs::Odometry filtered_odom;
ros::Time last_odom_update;
bool odom_received = false;

void odomCallback(const nav_msgs::Odometry &msg) {
  last_odom_update = ros::Time::now();
  static int messages_discarded = 0;

  if (!odom_received) {
    filtered_odom = msg;
    odom_received = true;
    return;
  }

  double x_diff = filtered_odom.pose.pose.position.x - msg.pose.pose.position.x;
  double y_diff = filtered_odom.pose.pose.position.y - msg.pose.pose.position.y;
  double z_diff = filtered_odom.pose.pose.position.z - msg.pose.pose.position.z;

  double distance = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);

  if (distance < distance_threshold || messages_discarded > max_msg_discards) {
    filtered_odom = msg;
    messages_discarded = 0;
  } else {
    messages_discarded++;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_filter");
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
  std::string world_frame_, robot_frame_;
  nh_private.param<std::string>("world_frame", world_frame_, "odom");
  nh_private.param<std::string>("robot_frame", robot_frame_, "base_link");
  nh_private.param<double>("time_threshold", time_threshold, 2.0);
  nh_private.param<double>("distance_threshold", distance_threshold, 1.0);
  nh_private.param<int>("max_msgs_to_discard", max_msg_discards, 10);

  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = world_frame_; // The parent frame
  transform_stamped.child_frame_id = robot_frame_;  // The child frame
  ros::Rate loop_rate(50);

  while (ros::ok()) {
    ros::spinOnce();

    if (!odom_received ||
        (last_odom_update - ros::Time::now()).toSec() > time_threshold) {
      ROS_WARN("No odom received");
      continue;
    }

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.transform.translation.x =
        filtered_odom.pose.pose.position.x;
    transform_stamped.transform.translation.y =
        filtered_odom.pose.pose.position.y;
    transform_stamped.transform.translation.z =
        filtered_odom.pose.pose.position.z;
    transform_stamped.transform.rotation = filtered_odom.pose.pose.orientation;
    tf_broadcaster.sendTransform(transform_stamped);

    loop_rate.sleep();
  }

  return 0;
}