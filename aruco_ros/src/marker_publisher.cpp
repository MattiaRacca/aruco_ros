#include <aruco_ros/marker_publisher.h>

ArucoMarkerPublisher::ArucoMarkerPublisher()
  : nh_("~")
  , it_(nh_)
  , useCamInfo_(true)
{
  image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);

  nh_.param<bool>("use_camera_info", useCamInfo_, true);
  if (useCamInfo_)
  {
    sensor_msgs::CameraInfoConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", nh_);
    camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);
    nh_.param<double>("marker_size", marker_size_, 0.05);
    nh_.param<bool>("image_is_rectified", useRectifiedImages_, true);
    nh_.param<std::string>("reference_frame", reference_frame_, "");
    nh_.param<std::string>("camera_frame", camera_frame_, "");
    ROS_ASSERT(not(camera_frame_.empty() and not reference_frame_.empty()));
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
}

bool ArucoMarkerPublisher::getTransform(const std::string& refFrame,
                                        const std::string& childFrame,
                                        tf::StampedTransform& transform)
{
  std::string errMsg;

  if (!tfListener_.waitForTransform(refFrame,
                                    childFrame,
                                    ros::Time(0),
                                    ros::Duration(0.5),
                                    ros::Duration(0.01),
                                    &errMsg))
  {
    ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
    return false;
  }
  else
  {
    try
    {
      tfListener_.lookupTransform(refFrame, childFrame,
                                  ros::Time(0),        //get latest available
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

void ArucoMarkerPublisher::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  bool publishMarkers = marker_pub_.getNumSubscribers() > 0;
  bool publishMarkersList = marker_list_pub_.getNumSubscribers() > 0;
  bool publishImage = image_pub_.getNumSubscribers() > 0;
  bool publishDebug = debug_pub_.getNumSubscribers() > 0;

  if (!publishMarkers && !publishMarkersList && !publishImage && !publishDebug)
    return;

  ros::Time curr_stamp(ros::Time::now());
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    inImage_ = cv_ptr->image;

    //clear out previous detection results
    markers_.clear();

    //Ok, let's detect
    mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

    // marker array publish
    marker_msg_->markers.clear();
    marker_msg_->markers.resize(markers_.size());
    marker_msg_->header.stamp = curr_stamp;
    marker_msg_->header.seq++;

    for (size_t i = 0; i < markers_.size(); ++i)
    {
      aruco_msgs::Marker & marker_i = marker_msg_->markers.at(i);
      marker_i.header.stamp = curr_stamp;
      marker_i.id = markers_.at(i).id;
      marker_i.confidence = 1.0;
    }

    // if there is camera info let's do 3D stuff
    if (useCamInfo_)
    {
      //get the current transform from the camera frame to output ref frame
      tf::StampedTransform cameraToReference;
      cameraToReference.setIdentity();

      if (reference_frame_ != camera_frame_)
      {
        getTransform(reference_frame_,
                     camera_frame_,
                     cameraToReference);
      }

      //Now find the transform for each detected marker
      for (size_t i = 0; i < markers_.size(); ++i)
      {
        aruco_msgs::Marker & marker_i = marker_msg_->markers.at(i);
        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers_[i]);
        transform = static_cast<tf::Transform>(cameraToReference) * transform;
        tf::poseTFToMsg(transform, marker_i.pose.pose);
        marker_i.header.frame_id = reference_frame_;

        /*Trick 'cause c++ doesn't like sprint on std::string*/
        char buff[100];
        std::snprintf(buff, sizeof(buff), "%02d", marker_i.id);
        std::string buffAsStdStr = buff;

        std::string tmp_marker_name = "marker_";
        tmp_marker_name.append(buffAsStdStr);

        tfBroadcaster_.sendTransform(tf::StampedTransform(transform,
                                     ros::Time::now(), reference_frame_, tmp_marker_name));
      }
    }

    //publish marker array
    if (marker_msg_->markers.size() > 0)
      marker_pub_.publish(marker_msg_);

    if (publishMarkersList)
    {
      marker_list_msg_.data.resize(markers_.size());
      for (size_t i = 0; i < markers_.size(); ++i)
        marker_list_msg_.data[i] = markers_[i].id;

      marker_list_pub_.publish(marker_list_msg_);
    }

    // Draw detected markers on the image for visualization
    for (size_t i = 0; i < markers_.size(); ++i)
    {
      markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
    }

    //draw a 3d cube in each marker if there is 3d info
    if (camParam_.isValid() && marker_size_ != -1)
    {
      for (size_t i = 0; i < markers_.size(); ++i)
        aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
    }

    // publish input image with markers drawn on it
    if (publishImage)
    {
      //show input with augmented information
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = curr_stamp;
      out_msg.encoding = sensor_msgs::image_encodings::RGB8;
      out_msg.image = inImage_;
      image_pub_.publish(out_msg.toImageMsg());
    }

    // publish image after internal image processing
    if (publishDebug)
    {
      //show also the internal image resulting from the threshold operation
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
