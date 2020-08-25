
#include "utility.h"
#include "LineSegment.h"
#include <linelidar_slam/line_segment.h>



//#define NDEBUG 0
//#define NDEBUG_1 0
///using namespace DataPoint;

class PointProcess
{
      private:

        ros::NodeHandle private_nh_;
        ros::NodeHandle node_;
        ros::Subscriber scan_sub_;
        ros::Publisher scan_line_pub_;
        ros::Publisher scan_point_pub_;
        tf::TransformListener tf_;
        laser_geometry::LaserProjection projector_;
        sensor_msgs::PointCloud cloud_;
        sensor_msgs::PointCloud2 cloud_two_;
        pcl::PointCloud<pcl::PointXY>::Ptr laserCloudIn;
        int laser_count_;

    public:
        PointProcess():
        laser_count_(0), private_nh_("~")
        {
          startExtract();
          allocateMemory();

        }

        void startExtract(){

          scan_sub_=private_nh_.subscribe("/scan",1000,&PointProcess::laserCallback,this);
          //scan_point_pub_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);
          //scan_point_pub_ = node_.advertise<sensor_msgs::PointCloud2>("slam_cloud",1,false);
          scan_line_pub_= node_.advertise<linelidar_slam::line_segment>("line_segs",1000);

        }

        void allocateMemory(){
           laserCloudIn.reset(new pcl::PointCloud<pcl::PointXY>());    
        }

        /*resetParameters(){
          laserCloudIn->clear();
        }*/

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
          std::cout<<laser_count_<<std::endl;  
         // ROS_INFO("I heard:[%d]",scan->header.stamp);
          laser_count_++;
          /*projector_.projectLaser(*scan, cloud_);
          sensor_msgs::convertPointCloudToPointCloud2(cloud_,cloud_two_);
          pcl::fromROSMsg(cloud_two_,*laserCloudIn);*/
          laserCloudIn->clear();
          pcl::PointXY newPoint;
          double newPointAngle;
          int beamNum = scan->ranges.size();
          for (int i = 0; i < beamNum; i++)
          {
            double R=scan->ranges[i];
            const double& range_max = scan->range_max;
            const double& range_min = scan->range_min;
            if(R > range_max || R < range_min)
            continue;
            if (R>0.5)
            {
              newPointAngle=scan->angle_min + scan->angle_increment * i;
              newPoint.x=R*cos(newPointAngle);
              newPoint.y=R*sin(newPointAngle);
              laserCloudIn->push_back(newPoint);
            }
            
          }
          
          //std::cout<<"The number of points:"<<laserCloudIn->points.size()<<std::endl;
          //scan_point_pub_.publish(cloud_);
          //scan_point_pub_.publish(cloud_two_);
          //copyPOintCloud();
          #ifndef NDEBUG
          if (laser_count_==700){
            std::ofstream outfile;
            outfile.open("/home/tanqingge/Desktop/lidat.txt", std::ios::out|std::ios::app);
            for(int i=0;i<laserCloudIn->points.size();i++){
              outfile<<laserCloudIn->points[i].x<<" "<<laserCloudIn->points[i].y<<"\n";
            }
            outfile.close();
          }
          
          #endif
          LineSegment ls;
          ls.segment_extraction(laserCloudIn);
          ls.segment_valid();
          #ifndef NDEBUG_1
          /*for (int i=0;i<ls.valid_segs_.size();i++)
          {
            std::cout<<"points number"<<i<<std::endl;
            std::cout<<"start_point"<<ls.valid_segs_[i].x_start_<<" "<<ls.valid_segs_[i].y_start_<<std::endl;
            std::cout<<"end_point"<<ls.valid_segs_[i].x_end_<<" "<<ls.valid_segs_[i].y_end_<<std::endl;
          }*/
          if (laser_count_==700){
            std::ofstream outfile;
            outfile.open("/home/tanqingge/Desktop/line.txt", std::ios::out|std::ios::app);
            for(int i=0;i<ls.valid_segs_.size();i++){
              outfile<<ls.valid_segs_[i].x_start_<<" "<<ls.valid_segs_[i].y_start_<<" "<<ls.valid_segs_[i].x_end_<<" "<<ls.valid_segs_[i].y_end_<<"\n";
            }
            outfile<<"THis is  700"<<"\n";
            outfile.close();
          }
          std::cout<<"points number"<<ls.valid_segs_.size()<<std::endl;

          if (laser_count_==710){
            std::ofstream outfile;
            outfile.open("/home/tanqingge/Desktop/line.txt", std::ios::out|std::ios::app);
            for(int i=0;i<ls.valid_segs_.size();i++){
              outfile<<ls.valid_segs_[i].x_start_<<" "<<ls.valid_segs_[i].y_start_<<" "<<ls.valid_segs_[i].x_end_<<" "<<ls.valid_segs_[i].y_end_<<"\n";
            }
            outfile<<"THis is  710"<<"\n";
            outfile.close();
          }
          #endif
          int num_lines_=ls.valid_segs_.size();
          linelidar_slam::line_segment msg_;
          msg_.header=scan->header;
          msg_.num_linesegments=num_lines_;
          msg_.start.resize(num_lines_);
          msg_.end.resize(num_lines_);
          for (int i=0;i<num_lines_;i++)
          {
            msg_.start[i].x=ls.valid_segs_[i].x_start_;
            msg_.start[i].y=ls.valid_segs_[i].y_start_;
            msg_.end[i].x=ls.valid_segs_[i].x_end_;
            msg_.end[i].y=ls.valid_segs_[i].y_end_;
          }
          scan_line_pub_.publish(msg_);
          //ROS_INFO("line num is %d",num_lines_);
          std::cout << "It's frame:" << laser_count_<<std::endl;
        }

        ~PointProcess(){
          
        };

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Pointprocess");
  ROS_INFO("PointProcess start.");     
  PointProcess pp;
  //pp.startExtract();
  ros::spin();

  return(0);
}
