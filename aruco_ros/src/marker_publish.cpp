/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <unordered_map>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>

#include <aruco_msgs/AprilTagDetection.h>
#include <aruco_msgs/AprilTagDetectionArray.h>

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;
  std::vector<aruco::Marker> markers_;

  // node params
  bool useRectifiedImages_;
  std::string marker_frame_;
  std::string camera_frame_;
  std::string reference_frame_;
  double marker_size_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_list_pub_;
  tf::TransformListener tfListener_;

  ros::Subscriber cam_info_sub_;
  aruco_msgs::MarkerArray::Ptr marker_msg_;
  cv::Mat inImage_;
  bool useCamInfo_;
  std_msgs::UInt32MultiArray marker_list_msg_;

  aruco_msgs::AprilTagDetection apriltag_detection_;
  aruco_msgs::AprilTagDetectionArray::Ptr apriltags_msg_;
  ros::Publisher apriltag_pub_;

public:
  XmlRpc::XmlRpcValue markers_list;
  std::unordered_map<int, tf::Transform> marker_poses_;

  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);

    nh_.param<bool>("use_camera_info", useCamInfo_, true);
    if (useCamInfo_)
    {
      sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", nh_); //, 10.0);

      nh_.param<double>("marker_size", marker_size_, 0.05);
      nh_.param<bool>("image_is_rectified", useRectifiedImages_, true);
      nh_.param<std::string>("reference_frame", reference_frame_, "");
      nh_.param<std::string>("camera_frame", camera_frame_, "");
      camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);
      ROS_ASSERT(not (camera_frame_.empty() and not reference_frame_.empty()));
      if (reference_frame_.empty())
        reference_frame_ = camera_frame_;
    }
    else
    {
      camParam_ = aruco::CameraParameters();
    }

    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    marker_pub_ = nh_.advertise<aruco_msgs::MarkerArray>("markers", 100);
    marker_list_pub_ = nh_.advertise<std_msgs::UInt32MultiArray>("markers_list", 10);

    marker_msg_ = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());
    marker_msg_->header.frame_id = reference_frame_;
    marker_msg_->header.seq = 0;

    apriltag_pub_ = nh_.advertise<aruco_msgs::AprilTagDetectionArray>("/tag_detections", 100);
    apriltags_msg_ = aruco_msgs::AprilTagDetectionArray::Ptr(new aruco_msgs::AprilTagDetectionArray());
    apriltags_msg_->header.frame_id = reference_frame_;
    apriltags_msg_->header.seq = 0;

    nh_.param<XmlRpc::XmlRpcValue>("markers", markers_list, markers_list);
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
  }

  bool marker_exists(int id) {
    return marker_poses_.find(id) != marker_poses_.end();
  }

  bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
  {
    std::string errMsg;

    if (!tfListener_.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                      &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        tfListener_.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
                                    transform);
      }
      catch (const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishMarkers = marker_pub_.getNumSubscribers() > 0;
    bool publishMarkersList = marker_list_pub_.getNumSubscribers() > 0;
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;
    bool publishApriltags = apriltag_pub_.getNumSubscribers() > 0;

    if (!publishMarkers && !publishMarkersList && !publishImage && !publishDebug && !publishApriltags)
      return;
    ROS_DEBUG("receiving image");

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
      ROS_DEBUG("number of markers detected: %d", (int)markers_.size());
      // marker array publish
      if (publishMarkers || publishApriltags)
      {
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;
        marker_msg_->header.seq++;

        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          aruco_msgs::Marker & marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = markers_.at(i).id;
          marker_i.confidence = 1.0;
        }

        apriltags_msg_->detections.clear();
        apriltags_msg_->detections.resize(markers_.size());
        apriltags_msg_->header.stamp = curr_stamp;
        apriltags_msg_->header.seq++;

        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          aruco_msgs::AprilTagDetection & apriltag_i = apriltags_msg_->detections.at(i);
        //   // apriltag_i.header.stamp = curr_stamp;
          apriltag_i.id.push_back(markers_.at(i).id);
          // apriltag_i.size = marker_size_;
        //   // apriltag_i.pose.pose.position.x = markers_.at(i).Tvec.at<float>(0, 0);
        //   // apriltag_i.pose.pose.position.y = markers_.at(i).Tvec.at<float>(1, 0);
        //   // apriltag_i.pose.pose.position.z = markers_.at(i).Tvec.at<float>(2, 0);
        //   // apriltag_i.pose.pose.orientation.x = markers_.at(i).Rvec.at<float>(0, 0);
        //   // apriltag_i.pose.pose.orientation.y = markers_.at(i).Rvec.at<float>(1, 0);
        //   // apriltag_i.pose.pose.orientation.z = markers_.at(i).Rvec.at<float>(2, 0);
        //   // apriltag_i.pose.pose.orientation.w = 1.0;
        }

        // if there is camera info let's do 3D stuff
        if (useCamInfo_)
        {
          // get the current transform from the camera frame to output ref frame
          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame_ != camera_frame_)
          {
            getTransform(reference_frame_, camera_frame_, cameraToReference);
          }

          // now find the transform for each detected marker
          for (std::size_t i = 0; i < markers_.size(); ++i)
          {
            aruco_msgs::Marker & marker_i = marker_msg_->markers.at(i);
            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers_[i]);
            transform = static_cast<tf::Transform>(cameraToReference) * transform;
            // tf::poseTFToMsg(transform, marker_i.pose.pose);
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(transform, pose);
            marker_i.pose.pose = pose;
            marker_i.header.frame_id = reference_frame_;
            
            aruco_msgs::AprilTagDetection & apriltag_i = apriltags_msg_->detections.at(i);
            // apriltag_i.id.push_back(markers_.at(i).id);
            geometry_msgs::PoseWithCovarianceStamped poseCovStamped;
            poseCovStamped.header.stamp = curr_stamp;
            poseCovStamped.header.frame_id = reference_frame_;
            poseCovStamped.pose.pose = pose;
            apriltag_i.pose = poseCovStamped;
          }
        }

        // publish marker array
        if (marker_msg_->markers.size() > 0)
        {
          marker_pub_.publish(marker_msg_);
          if (marker_exists(marker_msg_->markers.at(0).id))
          {
            ROS_INFO_STREAM("Marker ID: " << marker_msg_->markers.at(0).id << " detected.");
            apriltag_pub_.publish(apriltags_msg_);
          }
        }
      }

      if (publishMarkersList)
      {
        marker_list_msg_.data.resize(markers_.size());
        for (std::size_t i = 0; i < markers_.size(); ++i)
          marker_list_msg_.data[i] = markers_[i].id;

        marker_list_pub_.publish(marker_list_msg_);

        
      }

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }

      // draw a 3D cube in each marker if there is 3D info
      if (camParam_.isValid() && marker_size_ != -1)
      {
        for (std::size_t i = 0; i < markers_.size(); ++i)
          aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
      }

      // publish input image with markers drawn on it
      if (publishImage)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}
