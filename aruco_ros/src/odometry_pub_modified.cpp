#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <iostream>

#include <aruco_ros/aruco_ros_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <unordered_map>

class ArucoOdometry {
public:
  ArucoOdometry(ros::NodeHandle nh) {
    nh.param<bool>("image_is_rectified", use_rectified_image_, true);
    nh.param<double>("marker_size", marker_size_, 0.05);
    XmlRpc::XmlRpcValue markers_list;

    nh.param<XmlRpc::XmlRpcValue>("markers", markers_list, markers_list);

    for (int i = 0; i < markers_list.size(); i++) {
      tf::Vector3 marker_translation(
          static_cast<double>(markers_list[i]["pose"][0]),
          static_cast<double>(markers_list[i]["pose"][1]),
          static_cast<double>(markers_list[i]["pose"][2]));

      tf::Quaternion marker_rotation(
          static_cast<double>(markers_list[i]["pose"][3]),
          static_cast<double>(markers_list[i]["pose"][4]),
          static_cast<double>(markers_list[i]["pose"][5]),
          static_cast<double>(markers_list[i]["pose"][6]));

      int id = static_cast<int>(markers_list[i]["id"]);

      tf::Transform marker_pose;
      marker_pose.setOrigin(marker_translation);
      marker_pose.setRotation(marker_rotation);
      marker_poses_.insert({id, marker_pose});
    }

    image_transport::ImageTransport it(nh);

    image_sub_ = it.subscribe("/image", 1, &ArucoOdometry::imageCallback, this);
    cam_info_sub_ =
        nh.subscribe("/camera_info", 1, &ArucoOdometry::camInfoCallback, this);
    image_pub_ = it.advertise("result", 1);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    cam_info_received_ = false;

    odom.header.frame_id = "camera_link";
    odom.child_frame_id = "odom";
    odom.header.seq = 0;

    m_detector_.setDetectionMode(aruco::DM_VIDEO_FAST, 0.02);
  }

  void camInfoCallback(const sensor_msgs::CameraInfo &msg) {
    cam_param_ =
        aruco_ros::rosCameraInfo2ArucoCamParams(msg, use_rectified_image_);
    cam_info_received_ = true;
    // cam_info_sub_.shutdown();
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    if (cam_info_received_) {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat in_image;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        in_image = cv_ptr->image;

      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat in_image_grey;
      std::vector<aruco::Marker> markers;

      // ok, let's detect
      m_detector_.detect(in_image, markers, cam_param_, marker_size_, false);
      // for each marker, draw info and its boundaries in the image
      tf::Transform camera_in_world_frame;
      ROS_INFO("Number of aruco markers detected: %d", int(markers.size()));
      printAllRelativePoses(markers);
      std::vector<int> unknown_ids;
      bool odom_calculated = false;
      for (unsigned int i = 0; i < markers.size(); i++) {

        markers[i].draw(in_image, cv::Scalar(0, 0, 255), 2);

        tf::Transform camera_in_aruco_frame =
            aruco_ros::arucoMarker2Tf(markers[i]).inverse();

        if (marker_poses_.find(markers[i].id) == marker_poses_.end()) {

          // ROS_WARN_STREAM("The pose of the marker with Aruco ID: "
          //                 << markers[i].id << "is unknown.");
          unknown_ids.push_back(i);
          continue;
        }

        tf::Transform aruco_in_world_frame = marker_poses_.at(markers[i].id);
        camera_in_world_frame = aruco_in_world_frame * camera_in_aruco_frame;
        tf::poseTFToMsg(camera_in_world_frame, odom.pose.pose);
        odom_calculated = true;
      }

      if (odom_calculated) {
        for (int index : unknown_ids) {
          tf::Transform aruco_in_camera_frame =
              aruco_ros::arucoMarker2Tf(markers[index]);

          tf::Transform aruco_in_world_frame =
              camera_in_world_frame * aruco_in_camera_frame;

          geometry_msgs::Pose aruco_pose;
          tf::poseTFToMsg(aruco_in_world_frame, aruco_pose);
          // ROS_WARN_STREAM(
          //     "Id: " << markers[index].id << ", pose is (x,y,z,qx,qy,qz,qw): "
          //            << aruco_pose.position.x << ", " << aruco_pose.position.y
          //            << ", " << aruco_pose.position.z << ", "
          //            << aruco_pose.orientation.x << ", "
          //            << aruco_pose.orientation.y << ", "
          //            << aruco_pose.orientation.z << ", "
          //            << aruco_pose.orientation.w);
        }
      }




      odom.header.stamp = ros::Time::now();
      odom.header.seq++;
      odom_pub_.publish(odom);

      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = ros::Time::now();
      out_msg.encoding = sensor_msgs::image_encodings::RGB8;
      out_msg.image = in_image;
      image_pub_.publish(out_msg.toImageMsg());
    }
  }

private:
  void printXMLRPCValueType(XmlRpc::XmlRpcValue val) {
    switch (val.getType()) {
    case XmlRpc::XmlRpcValue::TypeArray:
      ROS_INFO("Type is Array");
      break;

    case XmlRpc::XmlRpcValue::TypeBase64:
      ROS_INFO("Type is Base64");
      break;

    case XmlRpc::XmlRpcValue::TypeBoolean:
      ROS_INFO("Type is Boolen");
      break;

    case XmlRpc::XmlRpcValue::TypeDateTime:
      ROS_INFO("Type is DateTime");
      break;

    case XmlRpc::XmlRpcValue::TypeDouble:
      ROS_INFO("Type is Double");
      break;

    case XmlRpc::XmlRpcValue::TypeInt:
      ROS_INFO("Type is Int");
      break;

    case XmlRpc::XmlRpcValue::TypeInvalid:
      ROS_INFO("Type is Invalid");
      break;

    case XmlRpc::XmlRpcValue::TypeString:
      ROS_INFO("Type is String");
      break;

    case XmlRpc::XmlRpcValue::TypeStruct:
      ROS_INFO("Type is Struct");
      break;

    default:
      break;
    }
  }

  void printAllRelativePoses(std::vector<aruco::Marker>& markers) {
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
    for (int i = 0; i < markers.size(); i++) {
      for (int j = 0; j < markers.size(); j++) {
        tf::Transform camera_in_aruco1_frame = aruco_ros::arucoMarker2Tf(markers[i]).inverse();
        tf::Transform aruco2_in_camera_frame = aruco_ros::arucoMarker2Tf(markers[j]);
        tf::Transform aruco2_in_aruco1_frame = camera_in_aruco1_frame * aruco2_in_aruco1_frame;
        
        // print the relative pose between marker i and marker j in x,y,z, r,p,y
        tf::Vector3 translation = aruco2_in_aruco1_frame.getOrigin();
        tf::Quaternion rotation = aruco2_in_aruco1_frame.getRotation();

        double roll, pitch, yaw;
        tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

        std::cout << "Marker " << markers[i].id << " to Marker " << markers[j].id << " is (x,y,z,r,p,y): " << translation.getX() << ", " << translation.getY() << ", " << translation.getZ() << ", " << roll << ", " << pitch << ", " << yaw << "\n";
        
      }
    }
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
  }

  bool use_rectified_image_;
  aruco::MarkerDetector m_detector_;
  ros::Subscriber cam_info_sub_;
  image_transport::Subscriber image_sub_;
  bool cam_info_received_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_image_pub_;
  ros::Publisher odom_pub_;
  double marker_size_;
  aruco::CameraParameters cam_param_;
  nav_msgs::Odometry odom;

  std::unordered_map<int, tf::Transform> marker_poses_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");

  ArucoOdometry odom(nh);

  ros::spin();
  return 0;
}