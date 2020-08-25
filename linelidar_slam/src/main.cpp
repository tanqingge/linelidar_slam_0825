/*Author:Junjie Xu */

#include <ros/ros.h>

#include "PointProcess.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "linelidar_slam");

  PointProcess pp;
  pp.startExtract();
  ros::spin();

  return(0);
}