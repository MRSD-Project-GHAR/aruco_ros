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
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unordered_map>

class ArucoPoseCalculator {
public:
  ArucoPoseCalculator(ros::NodeHandle nh) {
    nh.param<bool>("image_is_rectified", use_rectified_image_, true);
    nh.param<double>("known_marker_size", known_marker_size_, 0.05);
    nh.param<double>("unknown_marker_size", unknown_marker_size_, 0.05);
    nh.param<std::string>("world_frame", world_frame_, "odom");
    nh.param<std::string>("robot_frame", robot_frame_, "base_link");

    XmlRpc::XmlRpcValue known_markers_list;
    XmlRpc::XmlRpcValue unknown_markers_list;

    nh.param<XmlRpc::XmlRpcValue>("known_markers", known_markers_list,
                                  known_markers_list);

    nh.param<XmlRpc::XmlRpcValue>("unknown_markers", unknown_markers_list,
                                  unknown_markers_list);

    printXMLRPCValueType(known_markers_list);

    for (int i = 0; i < known_markers_list.size(); i++) {
      tf::Vector3 marker_translation(
          static_cast<double>(known_markers_list[i]["pose"][0]),
          static_cast<double>(known_markers_list[i]["pose"][1]),
          static_cast<double>(known_markers_list[i]["pose"][2]));

      tf::Quaternion marker_rotation(
          static_cast<double>(known_markers_list[i]["pose"][3]),
          static_cast<double>(known_markers_list[i]["pose"][4]),
          static_cast<double>(known_markers_list[i]["pose"][5]),
          static_cast<double>(known_markers_list[i]["pose"][6]));

      int id = static_cast<int>(known_markers_list[i]["id"]);

      tf::Transform marker_pose;
      marker_pose.setOrigin(marker_translation);
      marker_pose.setRotation(marker_rotation);
      known_marker_poses_.insert({id, marker_pose});
    }

    ROS_INFO("known markers initlaier");

    for (int i = 0; i < unknown_markers_list.size(); i++) {
      //   tf::Vector3 marker_translation(
      //       static_cast<double>(unknown_markers_list[i]["pose"][0]),
      //       static_cast<double>(unknown_markers_list[i]["pose"][1]),
      //       static_cast<double>(unknown_markers_list[i]["pose"][2]));

      //   tf::Quaternion marker_rotation(
      //       static_cast<double>(unknown_markers_list[i]["pose"][3]),
      //       static_cast<double>(unknown_markers_list[i]["pose"][4]),
      //       static_cast<double>(unknown_markers_list[i]["pose"][5]),
      //       static_cast<double>(unknown_markers_list[i]["pose"][6]));
      int id = static_cast<int>(unknown_markers_list[i]["id"]);

      //   tf::Transform marker_pose;
      //   marker_pose.setOrigin(marker_translation);
      //   marker_pose.setRotation(marker_rotation);
      //   marker_poses_.insert({id, marker_pose});
      unknown_marker_ids_.push_back(id);
    }

    image_transport::ImageTransport it(nh);

    image_sub_ =
        it.subscribe("/image", 1, &ArucoPoseCalculator::imageCallback, this);
    cam_info_sub_ = nh.subscribe("/camera_info", 1,
                                 &ArucoPoseCalculator::camInfoCallback, this);
    image_pub_ = it.advertise("result", 1);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    cam_info_received_ = false;

    odom.header.frame_id = world_frame_;
    odom.child_frame_id = robot_frame_;
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
      std::vector<aruco::Marker> known_markers;

      // ok, let's detect
      m_detector_.detect(in_image, known_markers, cam_param_,
                         known_marker_size_, false);
      // for each marker, draw info and its boundaries in the image
      tf::Transform camera_in_world_frame;
      ROS_INFO("Number of known aruco markers detected: %d",
               int(known_markers.size()));
      std::vector<int> unknown_ids;
      bool odom_calculated = false;
      for (unsigned int i = 0; i < known_markers.size(); i++) {

        known_markers[i].draw(in_image, cv::Scalar(0, 0, 255), 2);

        tf::Transform camera_in_aruco_frame =
            aruco_ros::arucoMarker2Tf(known_markers[i]).inverse();

        if (known_marker_poses_.find(known_markers[i].id) ==
            known_marker_poses_.end()) {

          // ROS_WARN_STREAM("The pose of the marker with Aruco ID: "
          //                 << known_markers[i].id << "is unknown.");
          // unknown_ids.push_back(i);
          continue;
        }

        tf::Transform aruco_in_world_frame =
            known_marker_poses_.at(known_markers[i].id);
        camera_in_world_frame = aruco_in_world_frame * camera_in_aruco_frame;
        tf::poseTFToMsg(camera_in_world_frame, odom.pose.pose);
        odom_calculated = true;
      }

      if (odom_calculated) {
        ROS_INFO("Number of unknown aruco markers detected: %d",
                 int(known_markers.size()));
        std::vector<aruco::Marker> unknown_markers;
        m_detector_.detect(in_image, unknown_markers, cam_param_,
                           unknown_marker_size_, false);

        for (unsigned int i = 0; i < unknown_markers.size(); i++) {

          //   unknown_markers[i].draw(in_image, cv::Scalar(255, 0, 0), 2);
          bool is_correct_unknown_marker =
              std::find(unknown_marker_ids_.begin(), unknown_marker_ids_.end(),
                        unknown_markers[i].id) != unknown_marker_ids_.end();

          if (!is_correct_unknown_marker) {

            // ROS_WARN_STREAM("The pose of the marker with Aruco ID: "
            //                 << markers[i].id << "is unknown.");
            // unknown_ids.push_back(i);
            // ROS_WARN("This ")
            ROS_WARN_STREAM("The marker with aruco id "
                            << unknown_markers[i].id
                            << ", is not in the unknown list");
            continue;
          }

          tf::Transform aruco_in_camera =
              aruco_ros::arucoMarker2Tf(unknown_markers[i]);

          tf::Transform aruco_in_world_frame =
              camera_in_world_frame * aruco_in_camera;

          geometry_msgs::Pose aruco_in_world_pose;
          tf::poseTFToMsg(aruco_in_world_frame, aruco_in_world_pose);

          std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
          std::cout << "  - id: " << unknown_markers[i].id << "\n";
          std::cout << "    pose: [" << aruco_in_world_pose.position.x << ", "
                    << aruco_in_world_pose.position.y << ", "
                    << aruco_in_world_pose.position.z << ", "
                    << aruco_in_world_pose.orientation.x << ", "
                    << aruco_in_world_pose.orientation.y << ", "
                    << aruco_in_world_pose.orientation.z << ", "
                    << aruco_in_world_pose.orientation.w << "]\n";
          std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
        }
      }

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

  bool use_rectified_image_;
  aruco::MarkerDetector m_detector_;
  ros::Subscriber cam_info_sub_;
  image_transport::Subscriber image_sub_;
  bool cam_info_received_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_image_pub_;
  ros::Publisher odom_pub_;
  double known_marker_size_;
  double unknown_marker_size_;
  aruco::CameraParameters cam_param_;
  nav_msgs::Odometry odom;

  std::unordered_map<int, tf::Transform> known_marker_poses_;
  std::vector<int> unknown_marker_ids_;
  // tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string robot_frame_;
  std::string world_frame_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_pose_calculator");
  ros::NodeHandle nh("~");

  ArucoPoseCalculator pose_calculator(nh);

  ros::spin();
  return 0;
}