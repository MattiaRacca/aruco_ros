#include <aruco_ros/marker_publisher.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}
