#include "utility.h"
#include "Pose2d.h"
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <linelidar_slam/line_segment.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <visualization_msgs/Marker.h>

#define PI 3.1415926
#define PT_NUMS 5
struct LocalLineSegs
{
    float x_start_,y_start_,x_end_,y_end_;
    bool isnew_;//whether a new segment for map
    int count_;// the number of calclate as valid segments
    //double A_,B_,C_;//AX+BY+C=0;
    double k_,b_;//y=kx+b;
};

std::vector <LocalLineSegs> GLOBALLINES_;
Pose2d T_R_L_;
Pose2d robot_pose_;
std::ofstream outfile;
ros::Publisher marker_pub ;
//ros::Rate r;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_);
void lineSegsCallback(const linelidar_slam::line_segment::ConstPtr& LineSegs_);
void SaveImg(std::string save_dir);

int main(int argc, char **argv)
{
    ros::init(argc,argv,"GlobalLines");
    ros::NodeHandle nh_;
    double x, y, theta;
    std::string map_image_save_dir;
    //lauch test, multiple node test:
    /*nh_.getParam ( "/GlobalLines/robot_laser/x", x );
    nh_.getParam ( "/GlobalLines/robot_laser/y", y );
    nh_.getParam ( "/GlobalLines/robot_laser/theta", theta );
    nh_.getParam ( "/GlobalLines/map_image_save_dir", map_image_save_dir );*/
    //single node debug:
    nh_.getParam ( "/robot_laser/x", x );
    nh_.getParam ( "/robot_laser/y", y );
    nh_.getParam ( "/robot_laser/theta", theta );
    nh_.getParam ( "/map_image_save_dir", map_image_save_dir );
    T_R_L_ = Pose2d ( x, y, theta );
    std::cout<<" g_x=:"<< x<<std::endl;
    std::cout<<" g_y=:"<< y<<std::endl;
    std::cout<<" g_theta=:"<< theta<<std::endl;
    //outfile.open("/home/tanqingge/Desktop/line_and_pose.txt", std::ios::out|std::ios::app);
    ros::Subscriber sub_tf_=nh_.subscribe("/mbot/odometry",1000,odomCallback);
    ros::Subscriber sub_line_=nh_.subscribe("/line_segs",1000,lineSegsCallback);
    
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    ros::Rate r(30);
    ros::spin();
    SaveImg(map_image_save_dir);
    //outfile.close();
    return 1;
}

//get the pose 
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_)
{
    double x1=odom_->pose.pose.position.x;
    double y1=odom_->pose.pose.position.y;
    double theta1 = tf::getYaw ( odom_->pose.pose.orientation);
    robot_pose_=Pose2d(x1,y1,theta1);

    ROS_INFO("odometry success");
};

void lineSegsCallback(const linelidar_slam::line_segment::ConstPtr& LineSegs_)
{
    static double pre_time=0.0;
    static int time_cnt=0;
    static int id_=0;
    static Pose2d pre_pose,curr_pose;
    int n= LineSegs_->num_linesegments;
    double cur_time = LineSegs_->header.stamp.toSec();
    Pose2d laser_pose = robot_pose_*T_R_L_;
    LocalLineSegs ls_;
    visualization_msgs::Marker points,line_list;
    //outfile.open("/home/tanqingge/Desktop/line_and_pose.txt", std::ios::out|std::ios::app);
    //outfile<<"pose_w_x:"<<" "<<laser_pose.x_<<" pose_w_y"<<" "<<laser_pose.y_<<" pose_w_theta"<<" "<<laser_pose.theta_<<"\n";
    //if (pre_time==0)
    //if (pre_pose==0)
    //{
       /* std::cout<<"The first time"<<std::endl;
        std::cout<<"The number of LineSegs_:"<<n<<std::endl;
        pre_time=cur_time;*/
        for (int i=0; i<n;i++)
        {
            //LocalLineSegs ls_;
            Eigen::Vector2d start_p_l(LineSegs_->start[i].x,LineSegs_->start[i].y);
            Eigen::Vector2d end_p_l(LineSegs_->end[i].x,LineSegs_->end[i].y);
            Eigen::Vector2d start_laser_coord=laser_pose*start_p_l;
            Eigen::Vector2d end_laser_coord=laser_pose*end_p_l;
            ls_.x_start_=start_laser_coord(0);
            ls_.y_start_=start_laser_coord(1);
            ls_.x_end_=end_laser_coord(0);
            ls_.y_end_=end_laser_coord(1);
            /*ls_.A_=ls_.y_end_-ls_.y_start_;//Y2-Y1
            ls_.B_=ls_.x_start_-ls_.x_end_;//X1-X2
            ls_.C_=ls_.x_end_*ls_.y_start_-ls_.x_start_*ls_.y_end_;//X2*Y1-X1*Y2*/
            ls_.k_=(ls_.y_end_-ls_.y_start_)/(ls_.x_start_-ls_.x_end_);
            ls_.b_=ls_.y_start_-ls_.k_*ls_.x_start_;
            GLOBALLINES_.push_back(ls_);
        }
            //outfile<<"i:"<<" "<<i<<" line_xstart_ "<<ls_.x_start_<<" line_ystart_ "<<ls_.y_start_<<" line_xend_ "<<ls_.x_end_<<" line_yend_ "<<ls_.y_end_<<" line_k_ "<<ls_.k_<<" line_b_ "<<ls_.b_<<"\n";
        //}
        //int num_=GLOBALLINES_.size();
        //std::cout<<"Num="<<num_<<std::endl;
        //outfile.close();
        /*time_cnt++;
    }
    else
    {
        pre_time = cur_time;
        std::cout<<"It's"<<time_cnt<<"Time"<<std::endl;
        time_cnt++;
        std::cout<<"The number of LineSegs_:"<<n<<std::endl;
        for (int i=0; i<n;i++)
        {
            //LocalLineSegs ls_;
            Eigen::Vector2d start_p_l(LineSegs_->start[i].x,LineSegs_->start[i].y);
            Eigen::Vector2d end_p_l(LineSegs_->end[i].x,LineSegs_->end[i].y);
            Eigen::Vector2d start_laser_coord=laser_pose*start_p_l;
            Eigen::Vector2d end_laser_coord=laser_pose*end_p_l;
            ls_.x_start_=start_laser_coord(0);
            ls_.y_start_=start_laser_coord(1);
            ls_.x_end_=end_laser_coord(0);
            ls_.y_end_=end_laser_coord(1);
            //std::cout<<"x_s:"<<ls_.x_start_<<"y_s:"<<ls_.y_start_<<"x_e:"<<ls_.x_end_<<"y_e:"<<ls_.y_end_<<std::endl;
            /*ls_.A_=ls_.y_end_-ls_.y_start_;//Y2-Y1
            ls_.B_=ls_.x_start_-ls_.x_end_;//X1-X2
            ls_.C_=ls_.x_end_*ls_.y_start_-ls_.x_start_*ls_.y_end_;//X2*Y1-X1*Y2*/
            /*ls_.k_=(ls_.y_end_-ls_.y_start_)/(ls_.x_start_-ls_.x_end_);
            ls_.b_=ls_.y_start_-ls_.k_*ls_.x_start_;
            //outfile<<"i:"<<" "<<i<<" line_xstart_ "<<ls_.x_start_<<" line_ystart_ "<<ls_.y_start_<<" line_xend_ "<<ls_.x_end_<<" line_yend_ "<<ls_.y_end_<<" line_k_ "<<ls_.k_<<" line_b_ "<<ls_.b_<<"\n";
            int flag_isnew=0;//flag to determine whether the ls_ line is a new line;
            int num_=GLOBALLINES_.size();
            for (int k=0;k<num_;k++)
            {
                /*double A1_=GLOBALLINES_[k].y_end_-GLOBALLINES_[k].y_start_;
                double B1_=GLOBALLINES_[k].x_start_-GLOBALLINES_[k].x_end_;
                double C1_=GLOBALLINES_[k].y_end_*GLOBALLINES_[k].x_start_-GLOBALLINES_[k].y_start_*GLOBALLINES_[k].x_end_;*/
                /*double k1_=GLOBALLINES_[k].k_;
                double b1_=GLOBALLINES_[k].b_;
                double theta_thisframe_=atan(ls_.k_)*180.f/PI;
                double theta_lastframe_=atan(k1_)*180.f/PI;
                //calculate theta error:
                double theta_err=fabs(theta_thisframe_-theta_lastframe_);
                //calculate distance from line to origin error:
                double dist_lastframe_=fabs(b1_)/sqrt(k1_*k1_+1);
                double dist_thisframe_=fabs(ls_.b_)/sqrt(ls_.k_*ls_.k_+1);
                double dist_err=fabs(dist_lastframe_-dist_thisframe_);
                int flag1_=1;
                int flag2_=1;
                int flag3_=1;
                if (theta_err<2)
                    flag1_=0;
                if (dist_err<0.3)
                    flag2_=0;
                //compute the middle point and the length
                double length_1=sqrt(pow(ls_.y_end_-ls_.y_start_,2)+pow(ls_.x_end_-ls_.x_start_,2));
                double length_2=sqrt(pow(GLOBALLINES_[k].y_end_-GLOBALLINES_[k].y_start_,2)+pow(GLOBALLINES_[k].x_end_-GLOBALLINES_[k].x_start_,2));
                Eigen::Vector2d Midpoint_1;
                Midpoint_1<<(ls_.x_start_+ls_.x_end_)/2.0,(ls_.y_start_+ls_.y_end_)/2.0;
                Eigen::Vector2d Midpoint_2;
                Midpoint_2<<(GLOBALLINES_[k].x_start_+GLOBALLINES_[k].x_end_)/2.0,(GLOBALLINES_[k].y_start_+GLOBALLINES_[k].y_end_)/2.0;
                double length_3=sqrt(pow(Midpoint_1(0)-Midpoint_1(0),2)+pow(Midpoint_1(1)-Midpoint_1(1),2));
                double max_dist=std::max(length_1,length_2);
                if (length_3<=max_dist/2.0)
                    flag3_=0;
                int sum= flag1_+flag2_+flag3_;
                
                if (sum==0)
                {
                    //std::cout<<i<<"Line "<<k<<"Global Line"<<std::endl;
                   //std::cout<<"theta_thisframe_: "<<theta_thisframe_<<" theta_lastframe_: "<<theta_lastframe_<<" dist_lastframe_: "<<dist_lastframe_<<" dist_thisframe_: "<<dist_thisframe_<<std::endl;
                    /*std::vector <cv::Point2d> points;
                    cv::Vec4f line;
                    cv::Point2d  single_pt_;
                    //interval of points
                    double dis_lsx_=(ls_.y_end_-ls_.y_start_)/PT_NUMS;
                    double dis_lsy_=(ls_.x_end_-ls_.x_start_)/PT_NUMS;
                    double dis_globalx_=(GLOBALLINES_[k].y_end_-GLOBALLINES_[k].y_start_)/PT_NUMS;
                    double dis_globaly_=(GLOBALLINES_[k].x_end_-GLOBALLINES_[k].x_start_)/PT_NUMS;
                    for (int nump_=0;nump_<(PT_NUMS+1);nump_++)
                    {
                        double x__=ls_.x_start_+nump_*dis_lsx_;
                        double y__=ls_.y_start_+nump_*dis_lsy_;
                        single_pt_=cv::Point2d(x__,y__);
                        points.push_back(single_pt_);
                    }
                    for (int nump_=0;nump_<(PT_NUMS+1);nump_++)
                    {
                        single_pt_=cv::Point2d(GLOBALLINES_[k].x_start_+nump_*dis_globalx_,GLOBALLINES_[k].y_start_+nump_*dis_globaly_);
                        points.push_back(single_pt_);
                    }

                    cv::fitLine(points,line,cv::DIST_L2,0,1e-2,1e-2);
                    double temp_x_=line[2];
                    double temp_y_=line[3];
                    double k=line[1]/line[0];
                    cv::Point2d point_0_,point_1_,point_2_,point_3_;//projection of ls_ and GLOBAL
                    point_0_.x=ls_.x_start_;
                    //point_0_.y=k*(point_0_.x-temp_x_)+temp_y_;
                    point_1_.x=ls_.x_end_;
                    //point_1_.y=k*(point_1_.x-temp_x_)+temp_y_;
                    point_2_.x=GLOBALLINES_[k].x_start_;
                    //point_2_.y=k*(point_2_.x-temp_x_)+temp_y_;
                    point_3_.x=GLOBALLINES_[k].x_end_;
                    //point_3_.y=k*(point_3_.x-temp_x_)+temp_y_;
                    double x_[4]={point_0_.x,point_1_.x,point_2_.x,point_3_.x};
                    int MaxPos=std::max_element(x_,x_+4)-x_;
                    int MinPos=std::min_element(x_,x_+4)-x_;
                    LocalLineSegs New_seg;
                    New_seg.x_start_=x_[MaxPos-1];
                    New_seg.y_start_=k*(New_seg.x_start_-temp_x_)+temp_y_;
                    New_seg.x_end_=x_[MinPos-1];
                    New_seg.y_end_=k*(New_seg.x_end_-temp_x_)+temp_y_;
                    New_seg.k_=k;
                    New_seg.b_=temp_y_-k*temp_x_;
                    GLOBALLINES_.push_back(New_seg);
                    GLOBALLINES_.erase(GLOBALLINES_.begin()+k-1);
                    std::cout<<"The ith time"<<i<<std::endl;*/
                    //use least square Fitting to get the new line:  
                   /* Eigen::Vector2d p_os_,p_oe_,p_ns_,p_ne_;
                    p_os_<<ls_.x_start_,ls_.y_start_;
                    p_oe_<<ls_.x_end_,ls_.y_end_;
                    p_ns_<<GLOBALLINES_[k].x_start_,GLOBALLINES_[k].y_start_;
                    p_ne_<<GLOBALLINES_[k].x_end_,GLOBALLINES_[k].y_end_;
                    //Ak+Bb=C;Bk+nb=D;
                    double A_= p_os_(0)*p_os_(0)+p_oe_(0)*p_oe_(0)+p_ns_(0)*p_ns_(0)+p_ne_(0)*p_ne_(0);
                    double B_=p_os_(0)+p_oe_(0)+p_ns_(0)+p_ne_(0);
                    double C_=p_os_(0)*p_os_(1)+p_oe_(0)*p_oe_(1)+p_ns_(0)*p_ns_(1)+p_ne_(0)*p_ne_(1);
                    double D_=p_os_(1)+p_oe_(1)+p_ns_(1)+p_ne_(1);
                    int np_=4;
                    double k_new_=(C_*np_-B_*D_)/(A_*np_-B_*B_);
                    double b_new_=(A_*D_-C_*B_)/(A_*np_-B_*B_);
                    double x1_,y1_,x2_,y2_,x3_,y3_,x4_,y4_;
                    x1_=(k_new_*p_os_(1)+p_os_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                    y1_=(k_new_*k_new_*p_os_(1)+k_new_*p_os_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1);
                    x2_=(k_new_*p_oe_(1)+p_oe_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                    y2_=(k_new_*k_new_*p_oe_(1)+k_new_*p_oe_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1);
                    x3_=(k_new_*p_ns_(1)+p_ns_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                    y3_=(k_new_*k_new_*p_ns_(1)+k_new_*p_ns_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1);
                    x4_=(k_new_*p_ne_(1)+p_ne_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                    y4_=(k_new_*k_new_*p_ne_(1)+k_new_*p_ne_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1);
                    double px_proj_[4]={x1_,x2_,x3_,x4_};
                    double py_proj_[4]={y1_,y2_,y3_,y4_};
                    double max_x_=px_proj_[0];
                    double min_x_=px_proj_[0];
                    int max_th_=0;
                    int min_th_=0;
                    for(int i=1;i<4;i++)
                    {
                        if(max_x_<px_proj_[i]){
                            max_x_=px_proj_[i];
                            max_th_=i;
                        }
                        if (min_x_>px_proj_[i]){
                            min_x_=px_proj_[i];
                            min_th_=i;
                        }
                    }
                    //std::cout<<"max_th_: "<<max_th_<<" min_th_: "<<min_th_<<std::endl;
                    LocalLineSegs Newseg_;
                    Newseg_.x_start_=px_proj_[min_th_];
                    Newseg_.y_start_=py_proj_[min_th_];
                    Newseg_.x_end_=px_proj_[max_th_];
                    Newseg_.y_end_=py_proj_[max_th_];
                    Newseg_.k_=k_new_;
                    Newseg_.b_=b_new_;
                    GLOBALLINES_.push_back(Newseg_);
                   // std::cout<<"Num_before="<<GLOBALLINES_.size()<<std::endl;
                    GLOBALLINES_.erase(GLOBALLINES_.begin()+k);
                    int num_=GLOBALLINES_.size();
                   // std::cout<<"Num_after="<<GLOBALLINES_.size()<<std::endl;
                    flag_isnew++;
                    //std::cout<<"The "<<i<<" Line"<<"is Compare already lines finished"<<std::endl;
                    break;
                }
            }
            
            //add new segment in global lines:
            if(flag_isnew==0)
                {
                    LocalLineSegs Newseg_;
                    Newseg_.x_start_=ls_.x_start_;
                    Newseg_.y_start_=ls_.y_start_;
                    Newseg_.x_end_=ls_.x_end_;
                    Newseg_.y_end_=ls_.y_end_;
                    Newseg_.k_=ls_.k_;
                    Newseg_.b_=ls_.b_;
                    GLOBALLINES_.push_back(Newseg_);
                    //std::cout<<"The number in GLOBALLINES:"<<GLOBALLINES_.size()<<std::endl;
                }
               // std::cout<<"The capacity of Vector"<<GLOBALLINES_.capacity()<<std::endl;
        }
        //outfile.close();
    }*/
   
    
    //ROS_INFO("I heard:[%f]",LineSegs_->header.stamp.toSec());
    std::cout<<"Total Line Nums"<<GLOBALLINES_.size()<<std::endl;
    points.header.frame_id=line_list.header.frame_id="/odom";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "map_lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    /*points.pose.position.x = line_list.pose.position.x = 0;
    points.pose.position.y = line_list.pose.position.y = 0;
    points.pose.orientation.w = line_list.pose.orientation.w = 1;*/
    points.pose.position.x = line_list.pose.position.x = robot_pose_.x_;
    points.pose.position.y = line_list.pose.position.y = robot_pose_.y_;
    geometry_msgs::Quaternion q_ = tf::createQuaternionMsgFromYaw (robot_pose_.theta_);
    points.pose.orientation = line_list.pose.orientation = q_;
    //add new pos
    Eigen::Vector2d T_;
    T_<<robot_pose_.x_,robot_pose_.y_;
    Eigen::Matrix2d R_;
    R_<<cos(robot_pose_.theta_),sin(robot_pose_.theta_),-sin(robot_pose_.theta_),cos(robot_pose_.theta_);
    
    points.id = 0;
    line_list.id = id_;
    id_++;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.05;
    line_list.color.r = 0;
    line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for (int i=0;i<GLOBALLINES_.size();i++)
    {
        geometry_msgs::Point p1;
        /*p1.x=GLOBALLINES_[i].x_start_;
        p1.y=GLOBALLINES_[i].y_start_;
        p1.z=0;*/
        Eigen::Vector2d p_st_w;
        p_st_w<<GLOBALLINES_[i].x_start_,GLOBALLINES_[i].y_start_;
        Eigen::Vector2d p_st_r;
        p_st_r=R_*(p_st_w-T_);
        p1.x=p_st_r(0);
        p1.y=p_st_r(1);
        p1.z=0;
        line_list.points.push_back(p1);
        geometry_msgs::Point p2;
        /*p2.x=GLOBALLINES_[i].x_end_;
        p2.y=GLOBALLINES_[i].y_end_;
        p2.z=0;*/
        Eigen::Vector2d p_ed_w;
        p_ed_w<<GLOBALLINES_[i].x_end_,GLOBALLINES_[i].y_end_;
        Eigen::Vector2d p_ed_r;
        p_ed_r=R_*(p_ed_w-T_);
        p2.x=p_ed_r(0);
        p2.y=p_ed_r(1);
        p2.z=0;
        line_list.points.push_back(p2);

    }
    marker_pub.publish(line_list);
    GLOBALLINES_.clear();
   // r.sleep();

       //std::cout<<"It's success"<<std::endl;
    //Map to opencv
       /* cv::Mat image = cv::Mat::zeros(1000, 1000, CV_8UC3);
        for(int i_=0;i_<GLOBALLINES_.size();i_++)
        {
            cv::Point beginPoint_;
            cv::Point endPoint_;
            beginPoint_=cv::Point(GLOBALLINES_[i_].x_start_,GLOBALLINES_[i_].y_start_);
            endPoint_=cv::Point(GLOBALLINES_[i_].x_end_,GLOBALLINES_[i_].y_end_);
            line(image, beginPoint_, endPoint_, cv::Scalar(0, 0, 255), 2);
            cv::imshow("Lines",image);
            
        }
       
        cv::waitKey ( 15);*/
}

void SaveImg(std::string save_dir)
{
    cv::Mat image = cv::Mat::zeros(2000, 2000, CV_8UC3);
    for(int i_=0;i_<GLOBALLINES_.size();i_++)
    {
            cv::Point beginPoint_;
            cv::Point endPoint_;
            beginPoint_=cv::Point(GLOBALLINES_[i_].x_start_,GLOBALLINES_[i_].y_start_);
            endPoint_=cv::Point(GLOBALLINES_[i_].x_end_,GLOBALLINES_[i_].y_end_);
            line(image, beginPoint_, endPoint_, cv::Scalar(0, 0, 255), 2);
    }
     cv::imwrite(save_dir, image);
};
