#ifndef __LineSegment_h_
#define __LineSegment_h_

#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#define MAXSEGMENTS 200
#define connect_th_ 0.03
#define min_distance_ 0.03
#define count_threhold_ 10
#define length_limit_ 0.1

struct SegmentStats {
  float x_start_,y_start_,x_end_,y_end_,xMean_,yMean_;
  int count_; //the horizontal length of the line
  double grad_;//gradient, dy/dx
  double invgrad_; //inv gradient, dx/dy
  float x_,y_,xy_,xx_,yy_; //for gradient stats
};

struct Valid_SegmentStats{
  float x_start_,y_start_,x_end_,y_end_;
};

class LineSegment{
    public:
        SegmentStats init_segs_[MAXSEGMENTS];
        std::vector <Valid_SegmentStats> valid_segs_;
    
        LineSegment();
        ~LineSegment();
        void segment_init();
        void segment_extraction(pcl::PointCloud<pcl::PointXY>::Ptr LaserPoint);
        void segment_valid();      

};

#endif
