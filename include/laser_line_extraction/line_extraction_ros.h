#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"
#include <std_srvs/SetBool.h>

namespace line_extraction
{

class LineExtractionROS
{

public:
  // Constructor / destructor
  LineExtractionROS(ros::NodeHandle&, ros::NodeHandle&);
  ~LineExtractionROS();
  // Running
  void run();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::Subscriber scan_subscriber_;
  ros::Publisher line_publisher_;
  ros::Publisher marker_publisher_;

  ros::ServiceServer enable_detector_service_;

  // Parameters
  std::string frame_id_;
  std::string scan_topic_;
  bool pub_markers_;
  // Line extraction
  LineExtraction line_extraction_;
  bool data_cached_; // true after first scan used to cache data
  // Enable detector:
  bool enable_detector_;
  // Members
  void loadParameters();
  void populateLineSegListMsg(const std::vector<Line>&, laser_line_extraction::LineSegmentList&);
  void populateMarkerMsg(const std::vector<Line>&, visualization_msgs::Marker&);
  void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
  bool enableDetectorCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
};

} // namespace line_extraction

#endif
