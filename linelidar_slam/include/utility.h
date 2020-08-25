#ifndef __utility_h_
#define __utility_h_


//#include "LineLidarSlam.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Float64.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "laser_geometry/laser_geometry.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <boost/thread.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdint.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <linelidar_slam/line_segment.h>

#define PI 3.1415926

#endif