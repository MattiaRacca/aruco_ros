#ifndef MARKER_PUBLISHER_H
#define MARKER_PUBLISHER_H

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt32MultiArray.h>

class ArucoMarkerPublisher
{
private:
  // aruco stuff
  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;
  vector<aruco::Marker> markers_;

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
  tf::TransformBroadcaster tfBroadcaster_;

  ros::Subscriber cam_info_sub_;
  aruco_msgs::MarkerArray::Ptr marker_msg_;
  cv::Mat inImage_;
  bool useCamInfo_;
  std_msgs::UInt32MultiArray marker_list_msg_;

public:
  ArucoMarkerPublisher();
  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform);
  void image_callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif // MARKER_PUBLISHER_H
