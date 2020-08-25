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


struct Submap
{
    double map_count;
    double time; 
    Pose2d sub_map_pose;
    std::vector <LocalLineSegs> sublines_;
};

std::vector <LocalLineSegs> GLOBALLINES_;
std::vector <LocalLineSegs> COMPARELINES_;
std::vector <Submap> SUBMAPS_;
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
    ros::init(argc,argv,"MapLines");
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
    
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker_one", 1000);
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
    static int id_=0;
    static double pre_time=0.0;
    static int time_count=0;
    static Pose2d pre_pose;
    Pose2d curr_pose=robot_pose_;
    int n= LineSegs_->num_linesegments;
    double curr_time = LineSegs_->header.stamp.toSec();
    Pose2d laser_pose = robot_pose_ * T_R_L_;
    LocalLineSegs ls_;
    Submap sbmap_;
    visualization_msgs::Marker points,line_list;
    //outfile.open("/home/tanqingge/Desktop/line_and_pose.txt", std::ios::out|std::ios::app);
    //outfile<<"pose_w_x:"<<" "<<laser_pose.x_<<" pose_w_y"<<" "<<laser_pose.y_<<" pose_w_theta"<<" "<<laser_pose.theta_<<"\n";
    if(time_count==0)
    {
        std::cout<<"It's first time"<<std::endl;
        pre_pose=robot_pose_;
        std::cout<<"pre_pose"<<pre_pose.x_<<pre_pose.y_<<pre_pose.theta_<<std::endl;
        id_++;
    for (int i=0; i<n;i++)
        {
         Eigen::Vector2d start_p_l(LineSegs_->start[i].x,LineSegs_->start[i].y);
         Eigen::Vector2d end_p_l(LineSegs_->end[i].x,LineSegs_->end[i].y);
         Eigen::Vector2d start_laser_coord=laser_pose*start_p_l;
         Eigen::Vector2d end_laser_coord=laser_pose*end_p_l;
         ls_.x_start_=start_laser_coord(0);
         ls_.y_start_=start_laser_coord(1);
         ls_.x_end_=end_laser_coord(0);
         ls_.y_end_=end_laser_coord(1);
         ls_.k_=(ls_.y_end_-ls_.y_start_)/(ls_.x_start_-ls_.x_end_);
         ls_.b_=ls_.y_start_-ls_.k_*ls_.x_start_;
         COMPARELINES_.push_back(ls_);
        }
        std::cout<<"init compare num:"<<COMPARELINES_.size()<<std::endl;
        time_count++;
    }
    else
    {
        std::cout<<"time_count: "<<time_count<<std::endl;
        //std::cout<<"pre_pose"<<pre_pose.x_<<" "<<pre_pose.y_<<" "<<pre_pose.theta_<<std::endl;
        //std::cout<<"curr_pose"<<curr_pose.x_<<" "<<curr_pose.y_<<" "<<curr_pose.theta_<<std::endl;
        double delta_x=fabs(curr_pose.x_-pre_pose.x_);
        double delta_y=fabs(curr_pose.y_-pre_pose.y_);
        //std::cout<<"delta_x: "<<delta_x<<"deltay: "<<delta_y<<std::endl;
        //belong to one submap
        if((delta_x<0.3)&&(delta_y<0.3))
        {
            pre_time=curr_time;
            
            for (int i=0; i<n;i++)
            {
                //std::cout<<"i:"<<i<<std::endl;
                Eigen::Vector2d start_p_l(LineSegs_->start[i].x,LineSegs_->start[i].y);
                Eigen::Vector2d end_p_l(LineSegs_->end[i].x,LineSegs_->end[i].y);
                
                Eigen::Vector2d start_laser_coord=laser_pose*start_p_l;
                Eigen::Vector2d end_laser_coord=laser_pose*end_p_l;
                ls_.x_start_=start_laser_coord(0);
                ls_.y_start_=start_laser_coord(1);
                ls_.x_end_=end_laser_coord(0);
                ls_.y_end_=end_laser_coord(1);
                ls_.k_=(ls_.y_end_-ls_.y_start_)/(ls_.x_start_-ls_.x_end_);
                ls_.b_=ls_.y_start_-ls_.k_*ls_.x_start_;
                
                int flag_isnew=0;//flag to determine whether the ls_ line is a new line;
                int num_=COMPARELINES_.size();
                
                for (int k=0;k<num_;k++)
                {
                    //std::cout<<"k:="<<k<<std::endl;
                    
                    double k1_=COMPARELINES_[k].k_;
                    double b1_=COMPARELINES_[k].b_;
                    double theta_thisframe_=atan(ls_.k_)*180.f/PI;
                    double theta_lastframe_=atan(k1_)*180.f/PI;
                    //calculate theta error:
                    double theta_err=fabs(theta_thisframe_-theta_lastframe_);
                    //calculate distance from line to origin error:
                    double dist_lastframe_=fabs(b1_)/sqrt(k1_*k1_+1);
                    double dist_thisframe_=fabs(ls_.b_)/sqrt(ls_.k_*ls_.k_+1);
                    double dist_err_3_=fabs(dist_lastframe_-dist_thisframe_);
                    Eigen::Vector2d Midpoint_1;
                    Midpoint_1<<(ls_.x_start_+ls_.x_end_)/2.0,(ls_.y_start_+ls_.y_end_)/2.0;
                    Eigen::Vector2d Midpoint_2;
                    Midpoint_2<<(COMPARELINES_[k].x_start_+COMPARELINES_[k].x_end_)/2.0,(COMPARELINES_[k].y_start_+COMPARELINES_[k].y_end_)/2.0;
                    //the distance between Midpoint1 & line in last frame
                    double dist_err_1_=fabs(k1_*Midpoint_1(0)-Midpoint_1(1)+b1_)/sqrt(k1_*k1_+1);
                    //the distance between Midpoint2 & line in new frame
                    double dist_err_2_=fabs(ls_.k_*Midpoint_2(0)-Midpoint_2(1)+ls_.b_)/sqrt(ls_.k_*ls_.k_+1);
                    //std::cout<<"dist_err:"<<dist_err<<std::endl;
                    //std::cout<<"theta_err:"<<theta_err<<std::endl;
                    int flag1_=1;
                    int flag2_=1;
                    int flag3_=1;
                    if (theta_err<3)
                        flag1_=0;
                    if ((dist_err_1_<0.2)&&(dist_err_2_<0.2)&&(dist_err_3_<0.3))
                        flag2_=0;
                    //std::cout<<"flag: "<<flag1_<<" "<<flag2_ <<std::endl;                   //compute the middle point and the length
                    double length_1=sqrt(pow(ls_.y_end_-ls_.y_start_,2)+pow(ls_.x_end_-ls_.x_start_,2));
                    //std::cout<<"length_1: "<<length_1<<std::endl;
                    double length_2=sqrt(pow(COMPARELINES_[k].y_end_-COMPARELINES_[k].y_start_,2)+pow(COMPARELINES_[k].x_end_-COMPARELINES_[k].x_start_,2));
                    //std::cout<<"length_2: "<<length_2<<std::endl;
                    double length_line_=(length_1+length_2)/2;
                    double length_3=sqrt(pow(Midpoint_1(0)-Midpoint_2(0),2)+pow(Midpoint_1(1)-Midpoint_2(1),2));
                    //double max_dist=std::max(length_1,length_2);
                    //std::cout<<"max_dist:"<<max_dist<<std::endl;
                    if (length_3<=length_line_)
                        flag3_=0;
                    int sum= flag1_+flag2_+flag3_;
                    //std::cout<<"sun_:"<<sum<<std::endl;

                    if (sum==0)
                    {
                    //use least square Fitting to get the new line:  
                        Eigen::Vector2d p_os_,p_oe_,p_ns_,p_ne_;
                        p_os_<<ls_.x_start_,ls_.y_start_;
                        p_oe_<<ls_.x_end_,ls_.y_end_;
                        p_ns_<<COMPARELINES_[k].x_start_,COMPARELINES_[k].y_start_;
                        p_ne_<<COMPARELINES_[k].x_end_,COMPARELINES_[k].y_end_;
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
                        y1_=(k_new_*k_new_*p_os_(1)+k_new_*p_os_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1)+b_new_;
                        x2_=(k_new_*p_oe_(1)+p_oe_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                        y2_=(k_new_*k_new_*p_oe_(1)+k_new_*p_oe_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1)+b_new_;
                        x3_=(k_new_*p_ns_(1)+p_ns_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                        y3_=(k_new_*k_new_*p_ns_(1)+k_new_*p_ns_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1)+b_new_;
                        x4_=(k_new_*p_ne_(1)+p_ne_(0)-k_new_*b_new_)/(k_new_*k_new_+1);
                        y4_=(k_new_*k_new_*p_ne_(1)+k_new_*p_ne_(0)-k_new_*k_new_*b_new_)/(k_new_*k_new_+1)+b_new_;
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
                    COMPARELINES_.push_back(Newseg_);
                    //std::cout<<"Num_before="<<COMPARELINES_.size()<<std::endl;
                    COMPARELINES_.erase(COMPARELINES_.begin()+k);
                    int num_=COMPARELINES_.size();
                    //std::cout<<"Num_after="<<COMPARELINES_.size()<<std::endl;
                    flag_isnew++;
                    //std::cout<<"The "<<i<<" Line"<<"is Compare already lines finished"<<std::endl;
                    break;
                    }
                    //std::cout<<"flag_isnew: "<<flag_isnew<<std::endl;
                }
                if(flag_isnew==0)
                {
                    LocalLineSegs Newseg_;
                    Newseg_.x_start_=ls_.x_start_;
                    Newseg_.y_start_=ls_.y_start_;
                    Newseg_.x_end_=ls_.x_end_;
                    Newseg_.y_end_=ls_.y_end_;
                    Newseg_.k_=ls_.k_;
                    Newseg_.b_=ls_.b_;
                    COMPARELINES_.push_back(Newseg_);
                    
                }
            }
            time_count++;
            std::cout<<"The number in COMPARELINES:"<<COMPARELINES_.size()<<std::endl;
            std::cout<<"A turn is over"<<std::endl;
            
        }
        //publish a submap
        else
        {
            std::cout<<"A new submap occurred!"<<std::endl;
            Submap temp_map;
            temp_map.map_count=id_;
            temp_map.time=curr_time;
            temp_map.sub_map_pose=pre_pose;
            Eigen::Vector2d T_;
            T_<<pre_pose.x_,pre_pose.y_;
            Eigen::Matrix2d R_;
            R_<<cos(pre_pose.theta_),sin(pre_pose.theta_),-sin(pre_pose.theta_),cos(pre_pose.theta_);
            std::cout<<"R_"<<R_(0)<<" "<<R_(1)<<" "<<R_(2)<<" "<<R_(3)<<std::endl;
            for(int j=0;j<COMPARELINES_.size();j++)
            {
                Eigen::Vector2d p_st_w;
                p_st_w<<COMPARELINES_[j].x_start_,COMPARELINES_[j].y_start_;
                Eigen::Vector2d p_ed_w;
                p_ed_w<<COMPARELINES_[j].x_end_,COMPARELINES_[j].y_end_;
                Eigen::Vector2d p_st_r,p_ed_r;
                p_st_r=R_*(p_st_w-T_);
                p_ed_r=R_*(p_ed_w-T_);
                LocalLineSegs temp_ls_;
                temp_ls_.x_start_=p_st_r(0);
                temp_ls_.y_start_=p_st_r(1);
                temp_ls_.x_end_=p_ed_r(0);
                temp_ls_.y_end_=p_ed_r(1);
                GLOBALLINES_.push_back(temp_ls_);
            }
            temp_map.sublines_=GLOBALLINES_;
            //std::cout<<"Total Line Nums"<<GLOBALLINES_.size()<<std::endl;
            points.header.frame_id=line_list.header.frame_id="/odom";
            points.header.stamp = line_list.header.stamp = ros::Time::now();
            points.ns = line_list.ns = "map_lines";
            points.action = line_list.action = visualization_msgs::Marker::ADD;
            //std::cout<<"robotx"<<robot_pose_.x_<<"roboty"<<robot_pose_.y_<<std::endl;
            points.pose.position.x = line_list.pose.position.x = temp_map.sub_map_pose.x_;
            points.pose.position.y = line_list.pose.position.y = temp_map.sub_map_pose.y_;
            //std::cout<<"theta:"<<robot_pose_.theta_<<std::endl;
            geometry_msgs::Quaternion q_ = tf::createQuaternionMsgFromYaw (temp_map.sub_map_pose.theta_);
            points.pose.orientation = line_list.pose.orientation = q_;
            //std::cout<<"x:"<<q_.x<<"y:"<<q_.y<<"z:"<<q_.z<<"w"<<q_.w<<std::endl;
            //points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
            points.id = 0;
            line_list.id = id_;
            points.type = visualization_msgs::Marker::POINTS;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            line_list.scale.x = 0.05;
            line_list.color.r = 1.0;
            line_list.color.a = 0.5;
            line_list.lifetime=ros::Duration(0,0);
    // Create the vertices for the points and lines
            for (int i=0;i<GLOBALLINES_.size();i++)
            {
                geometry_msgs::Point p1;
                p1.x=GLOBALLINES_[i].x_start_;
                p1.y=GLOBALLINES_[i].y_start_;
                p1.z=0;
                line_list.points.push_back(p1);
                geometry_msgs::Point p2;
                p2.x=GLOBALLINES_[i].x_end_;
                p2.y=GLOBALLINES_[i].y_end_;
                p2.z=0;
                line_list.points.push_back(p2);
            }
            marker_pub.publish(line_list);
            GLOBALLINES_.clear();  
            COMPARELINES_.clear();
            SUBMAPS_.push_back(temp_map);
            time_count=0;
            pre_pose=robot_pose_;
        }
    }
        
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
