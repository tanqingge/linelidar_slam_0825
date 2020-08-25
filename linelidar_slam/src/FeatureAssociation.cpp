#include "utility.h"
#include "Frame.h"
#include <linelidar_slam/line_segment.h>

class Frame;
class FeatureAssociation
{
    private:
    ros::NodeHandle nh_pa_;
    ros::NodeHandle node_;
    //message_filters::Subscriber<linelidar_slam::line_segment> line_sub_;
    ros::Subscriber line_sub_;
    ros::Publisher odometry_pub_;
    tf::TransformListener tf_;
    //tf::MessageFilter<linelidar_slam::line_segment> * tf_filter_;
    //ros::NodeHandle n_;
    std::string target_frame_;
    
    //Odemetry::Ptr odom_;

    public:
    Frame::Ptr frame_;

    FeatureAssociation():nh_pa_("~"),node_("~"),target_frame_("/odom"),frame_(nullptr)
    {
        /*line_sub_.subscribe(node_,"line_segs",10);
        tf_filter_=new tf::MessageFilter<linelidar_slam::line_segment>(line_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback(boost::bind(&FeatureAssociation::msgCallback, this, _1));*/
        line_sub_=node_.subscribe("/line_segs",1000,&FeatureAssociation::msgCallback,this);
        odometry_pub_=node_.advertise<nav_msgs::Odometry>("/odometry",1000);
        allocatememory();
        //std::cout<<"gappy"<<std::endl;
        
    };

    //void msgCallback(const linelidar_slam::line_segment::ConstPtr& line_ptr_,const geometry_msgs::TransformStamped::ConstPtr& tf_ptr_ boost::shared_ptr<const linelidar_slam::line_segment>&)
    void msgCallback(const  linelidar_slam::line_segment::ConstPtr& line_ptr_)
    {
        std::cout<<"error"<<std::endl;
        tf::StampedTransform transform;
        ros::Time::init();
        try
        {
            
            tf_.waitForTransform(target_frame_,line_ptr_->header.frame_id, line_ptr_->header.stamp,ros::Duration(5));
            tf_.lookupTransform(target_frame_,line_ptr_->header.frame_id, line_ptr_->header.stamp, transform);
            
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR(" %s ", ex.what());
            ros::Duration(1.0).sleep();
           
            return;
        }
          
        frame_=Frame::createFrame();
        frame_->seq_=line_ptr_->header.seq;
        /*frame_->position_<<tf_ptr_->transform.translation.x,tf_ptr_->transform.translation.y,tf_ptr_->transform.translation.z;
        double x_=tf_ptr_->transform.rotation.x;
        double y_=tf_ptr_->transform.rotation.y;
        double z_=tf_ptr_->transform.rotation.z;
        double w_=tf_ptr_->transform.rotation.w;
        Eigen::Quaterniond quaternion_(w_,x_,y_,z_);
        frame_->orientation_=quaternion_;*/
        
        
        double x_=transform.getOrigin().x();
        double y_=transform.getOrigin().y();
        double z_=transform.getOrigin().z();
        std::cout<<"x:"<<x_<<std::endl;
        std::cout<<"y:"<<y_<<std::endl;
        std::cout<<"z:"<<z_<<std::endl;
        //T_(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());;
        Eigen::Vector3d T_(x_,y_,z_);
        //tf::vectorTFToEigen(T_,frame_->position_);
        frame_->position_=T_;
        Eigen::Quaterniond Q_(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());
        //Eigen::Quaterniond quaternion_(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());
        //frame_->orientation_=quaternion_;
        //tf::quaternionTFToEigen(Q_,frame_->orientation_);
        frame_->orientation_=Q_;
        ROS_INFO("transition is [%f,%f,%f]",frame_->position_[0],frame_->position_[1],frame_->position_[2]);
        frame_->local_lines_.resize(line_ptr_->num_linesegments);
        for (int i=0;i<line_ptr_->num_linesegments;i++)
        {
            frame_->local_lines_[i].x_start_=line_ptr_->start[i].x;
            frame_->local_lines_[i].y_start_=line_ptr_->start[i].x;
            frame_->local_lines_[i].x_end_=line_ptr_->end[i].x;
            frame_->local_lines_[i].y_end_=line_ptr_->end[i].y;
            frame_->local_lines_[i].isnew_=true;
            frame_->local_lines_[i].count_=1;
        };
        ROS_INFO("frame_id is [%d]",frame_->frame_id_);
        ROS_INFO("seq is [%d]",frame_->seq_);
        
        nav_msgs::Odometry odom;
        odom.header.seq=frame_->frame_id_;
        odom.header.stamp=line_ptr_->header.stamp;
        odom.header.frame_id="odom";
        odom.child_frame_id="odom.child_frame_id";
        odom.pose.pose.position.x=x_;
        odom.pose.pose.position.y=y_;
        odom.pose.pose.orientation.x=transform.getRotation().getX();
        odom.pose.pose.orientation.y=transform.getRotation().getY();
        odom.pose.pose.orientation.z=transform.getRotation().getZ();
        odom.pose.pose.orientation.w=transform.getRotation().getW();
        odometry_pub_.publish(odom);
       
    };

    void allocatememory()
    {
        //Odometry::Ptr odom_(new Odometry::Ptr);
        
    };

    /*void runFeatureAssociation()
    void runFeatureAssociation()
    {
        odom_->addFrame(frame_);
        addKeyframe();
        publishmap();
        std::cout<<"correct"<<std::endl;
        ROS_INFO("frame_id is [%s]",frame_->frame_id_);
        std::cout<<"celebrate"<<std::endl;
        /*ROS_INFO("frame_id is [%d]",frame_->frame_id_);
        ROS_INFO("seq is [%d]",frame_->seq_);
        ROS_INFO("It's the second time");
                       
    };*/

    ~FeatureAssociation()
    {

    };



};

void MsgCallback(const  linelidar_slam::line_segment::ConstPtr& line_ptr_)
{
    std::cout<<"it's out"<<std::endl;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "FeatureAssociation");
    ROS_INFO(" Feature Association start.");  
    //ros::Rate rate(200);
    
    FeatureAssociation FA; 
    //FA.runFeatureAssociation();
    /*while (ros::ok())
    {
        ros::spinOnce();
        FeatureAssociation FA; 
        rate.sleep();

    };    */  
    //FA.runFeatureAssociation(FA.frame_);
    /*ros::Subscriber line_sub1_;
    ros::NodeHandle n_;
    line_sub1_=n_.subscribe("line_segs",1000,&MsgCallback);*/

    ros::spin();
    return(0);
}