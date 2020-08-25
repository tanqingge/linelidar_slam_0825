// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)
#include "utility.h"
#include "grid_mapper.h"
#include <linelidar_slam/line_segment.h>

GridMapper::GridMapper ( GridMap* map, Pose2d& T_r_l, double& P_occ, double& P_free, double& P_prior):
map_(map), T_r_l_(T_r_l), P_occ_(P_occ), P_free_(P_free), P_prior_(P_prior)
{
    
}

void GridMapper::updateMap ( const linelidar_slam::line_segment::ConstPtr& line_segs,  Pose2d& robot_pose )
{
    /* 获取激光的信息 */
    /*const double& ang_min = scan->angle_min;
    const double& ang_max = scan->angle_max;
    const double& ang_inc = scan->angle_increment;
    const double& range_max = scan->range_max;
    const double& range_min = scan->range_min;
    
    /* 设置遍历的步长，沿着一条激光线遍历 */
    /*const double& cell_size = map_->getCellSize();
    const double inc_step = 1.0 * cell_size;

    /* for every laser beam */
    /*for(size_t i = 0; i < scan->ranges.size(); i ++)
    {
        /* 获取当前beam的距离 */
        /*double R = scan->ranges.at(i); 
        if(R > range_max || R < range_min)
            continue;
        
        /* 沿着激光射线以inc_step步进，更新地图*/
       /* double angle = ang_inc * i + ang_min;
        double cangle = cos(angle);
        double sangle = sin(angle);
        Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity); //上一步更新的grid位置，防止重复更新
        for(double r = 0; r < R + cell_size; r += inc_step)
        {
            Eigen::Vector2d p_l(
                r * cangle,
                r * sangle
            ); //在激光雷达坐标系下的坐标
            
            /* 转换到世界坐标系下 */
            /*Pose2d laser_pose = robot_pose * T_r_l_;
            Eigen::Vector2d p_w = laser_pose * p_l;

            /* 更新这个grid */
            /*if(p_w == last_grid) //避免重复更新
                continue;
            
            updateGrid(p_w, laserInvModel(r, R, cell_size));
            	    
            last_grid = p_w;
        }//for each step
    }// for each beam*/
    const double& cell_size = map_->getCellSize();
    const double inc_step = 1.0 * cell_size;
    int num_lines=line_segs->num_linesegments;
    Eigen::MatrixXd lines_;
    lines_.resize(num_lines,3);
    Eigen::VectorXd l_dist;
    l_dist.resize(num_lines,1);
    Pose2d laser_pose = robot_pose * T_r_l_;
    Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity);
    //std::cout<<"last_grid:="<<last_grid(0)<<last_grid(1)<<std::endl;
    /*for(int i=0;i<num_lines;i++)
    {
        Eigen::Vector2d start_p_l(line_segs->start[i].x,line_segs->start[i].y);
        Eigen::Vector2d end_p_l(line_segs->end[i].x,line_segs->end[i].y);
        
        Eigen::Vector2d start_laser_coord=laser_pose*start_p_l;
        Eigen::Vector2d end_laser_coord=laser_pose*end_p_l;
        double A_,B_,C_;
        A_=-start_laser_coord(1,0)+end_laser_coord(1,0);
        B_=start_laser_coord(0,0)-end_laser_coord(0,0);
        C_=start_laser_coord(1,0)*end_laser_coord(0,0)-start_laser_coord(0,0)*end_laser_coord(1,0);
        lines_(i,0)=A_;
        lines_(i,1)=B_;
        lines_(i,2)=C_;
        //std::cout<<"A_,B_,C_"<<A_<<" "<<B_<<" "<<C_<<std::endl;
    }*/
    Eigen::MatrixXd lines_segs_;
    lines_segs_.resize(num_lines,6);
    for(int i=0;i<num_lines;i++)
    {
        Eigen::Vector2d start_p_l(line_segs->start[i].x,line_segs->start[i].y);
        Eigen::Vector2d end_p_l(line_segs->end[i].x,line_segs->end[i].y);
        
        Eigen::Vector2d start_laser_coord=laser_pose*start_p_l;
        Eigen::Vector2d end_laser_coord=laser_pose*end_p_l;
        /*double A_,B_,C_;
        A_=-start_laser_coord(1,0)+end_laser_coord(1,0);
        B_=start_laser_coord(0,0)-end_laser_coord(0,0);
        C_=start_laser_coord(1,0)*end_laser_coord(0,0)-start_laser_coord(0,0)*end_laser_coord(1,0);
        lines_(i,0)=A_;
        lines_(i,1)=B_;
        lines_(i,2)=C_;*/
        lines_segs_(i,0)=start_laser_coord(0);
        lines_segs_(i,1)=start_laser_coord(1);
        lines_segs_(i,2)=end_laser_coord(0);
        lines_segs_(i,3)=end_laser_coord(1);
        Eigen::Vector2d P_(
            lines_segs_(i,0)-lines_segs_(i,2),
            lines_segs_(i,1)-lines_segs_(i,3)
        );
        lines_segs_(i,4)=P_.squaredNorm();
        lines_segs_(i,5)=atan2(end_laser_coord(1)-start_laser_coord(1),end_laser_coord(0)-start_laser_coord(0));
        //std::cout<<"A_,B_,C_"<<A_<<" "<<B_<<" "<<C_<<std::endl;
    }

    /*for (int i=0;i<map_->size_x_;i++)
    {
        for(int j=0;j<map_->size_y_;j++)
        {
            double p_xl=(i-map_->init_x_)*inc_step;
            double p_yl=(j-map_->init_y_)*inc_step;
            Eigen::Vector2d point_l(
                p_xl,
                p_yl
            );
            Eigen::Vector2d point_w(
                robot_pose.x_+p_xl,
                robot_pose.y_+p_yl
            );
            double p_w_x=point_w(0,0);
            double p_w_y=point_w(1,0);
            for (int k=0;k<num_lines;k++)
            {
                l_dist[k]=fabs(lines_(k,0)*p_w_x+lines_(k,1)*p_w_y+lines_(k,2))/sqrt(pow(lines_(k,0),2)+pow(lines_(k,1),2));//|AX+BY+C|/sqrt(a2+b2)
            }
            Eigen::MatrixXd::Index minRow, minCol;
            double d_min_=l_dist.minCoeff(&minRow, &minCol);
            Eigen::Vector2d start_minp_l(line_segs->start[minRow].x,line_segs->start[minRow].y);
            Eigen::Vector2d end_minp_l(line_segs->end[minRow].x,line_segs->end[minRow].y);
            Eigen::Vector2d start_minp_coord=laser_pose*start_minp_l;
            Eigen::Vector2d end_minp_coord=laser_pose*end_minp_l;
            double dist_=sqrt(pow(start_minp_coord(0)-end_minp_coord(0),2)+pow(start_minp_coord(1)-end_minp_coord(1),2));
            double num_seg=floor(dist_/cell_size) ;
            double angle_=atan2(end_minp_coord(1)-start_minp_coord(1),end_minp_coord(0)-start_minp_coord(0));
            Eigen::VectorXd ml_dist;
            ml_dist.resize(num_seg+1,1);
            double step_=dist_/(num_seg+1);//buchang
            double x_origin_=(start_minp_coord(0)+end_minp_coord(0))/2-dist_*cos(angle_)/2;
            double y_origin_=(start_minp_coord(1)+end_minp_coord(1))/2-dist_*sin(angle_)/2;
            for(int i=0;i<num_seg+1;i++)
            {                
                double temp_x_=x_origin_+i*step_*cos(angle_);
                double temp_y_=y_origin_+i*step_*sin(angle_);
                Eigen::Vector2d err(
                    temp_x_-point_w(0),
                    temp_y_-point_w(1)
                );
                ml_dist(i)=err.squaredNorm(); 
            }
            double d_min=ml_dist.minCoeff();
            //std::cout<<"minRow:"<<minRow<<" "<<"minCol:"<<minCol<<std::endl;
            //std::cout<<"robot_pose"<<robot_pose.x_<<" "<<robot_pose.y_<<std::endl;
            std::cout<<"d_min:"<<d_min<<std::endl;
            if(point_w == last_grid) //避免重复更新
                continue;

            updateGrid(point_w,laserInvModel(d_min,cell_size));

            last_grid=point_w;
        }
    }*/
    Eigen::VectorXd NUM_Point_;
    NUM_Point_.resize(num_lines,1  );
    for (int i=0;i<num_lines;i++)
    {
        NUM_Point_(i)=floor(lines_segs_(i,4)/cell_size)+1;
    }
    for (int k=0;k<num_lines;k++)
    {
        for (int i=0;i<NUM_Point_(k);i++)
        {
            for(int j=0;j<3;j++)
            {
                double step_=lines_segs_(k,4)/NUM_Point_(k);
                double x_origin_=(lines_segs_(k,0)+lines_segs_(k,2))/2-lines_segs_(k,4)*cos(lines_segs_(k,5))/2;
                double y_origin_=(lines_segs_(k,1)+lines_segs_(k,3))/2-lines_segs_(k,4)*sin(lines_segs_(k,5))/2;  
                double temp_x_=x_origin_+i*step_*cos(lines_segs_(k,5));
                double temp_y_=y_origin_+i*step_*sin(lines_segs_(k,5));
                double p_xl=(j-1.5)*inc_step;
                double p_yl=(j-1.5)*inc_step;
                Eigen::Vector2d P_C_W(
                    temp_x_-p_xl*cos(PI/2-lines_segs_(k,5)),
                    temp_y_+p_yl*sin(PI/2-lines_segs_(k,5))
                );
                if(P_C_W == last_grid) //避免重复更新
                continue;
                updateGrid(P_C_W,laserInvModel(fabs(j-7.5)));
                last_grid=P_C_W;

            }
        }
    }
    
   
            /*for (int k=0;k<num_lines;k++)
            {
                l_dist[k]=fabs(lines_(k,0)*p_w_x+lines_(k,1)*p_w_y+lines_(k,2))/sqrt(pow(lines_(k,0),2)+pow(lines_(k,1),2));//|AX+BY+C|/sqrt(a2+b2)
            }*/
            /*Eigen::MatrixXd::Index minRow, minCol;
            double d_min_=l_dist.minCoeff(&minRow, &minCol);
            Eigen::Vector2d start_minp_l(line_segs->start[minRow].x,line_segs->start[minRow].y);
            Eigen::Vector2d end_minp_l(line_segs->end[minRow].x,line_segs->end[minRow].y);
            Eigen::Vector2d start_minp_coord=laser_pose*start_minp_l;
            Eigen::Vector2d end_minp_coord=laser_pose*end_minp_l;
            double dist_=sqrt(pow(start_minp_coord(0)-end_minp_coord(0),2)+pow(start_minp_coord(1)-end_minp_coord(1),2));
            double num_seg=floor(dist_/cell_size) ;
            double angle_=atan2(end_minp_coord(1)-start_minp_coord(1),end_minp_coord(0)-start_minp_coord(0));*/
}

void GridMapper::updateGrid ( const Eigen::Vector2d& grid, const double& pmzx )
{
    /* TODO 这个过程写的太低效了 */
    double log_bel;
    if(  ! map_->getGridLogBel( grid(0), grid(1), log_bel )  ) //获取log的bel
        return;
    //double new_log_bel= log( pmzx / (1.0 - pmzx) ); //更新
    map_->setGridLogBel( grid(0), grid(1), pmzx  );
    /*if (log_bel<=pmzx)
        map_->setGridLogBel( grid(0), grid(1), new_log_bel  ); 
    else
        map_->setGridLogBel( grid(0), grid(1), log_bel  ); */
}

double GridMapper::laserInvModel ( const double& j )
{
    /*if(d_min <=2*cell_size )
        return 0.6;
    else if ((2*cell_size<d_min) && (d_min<=(4*cell_size)))
        return 0.5;
    else
        return 0.4;

    /*else if (((2*cell_size)<d_min)&&(d_min<=(4*cell_size)))
        return 0.85;
    else if (((4*cell_size)<d_min)&&(d_min<=(8*cell_size)))
        return 0.8;
    else if (((8*cell_size)<d_min)&&(d_min<=(12*cell_size)))
        return 0.75;
    else if (((12*cell_size)<d_min)&&(d_min<=(20*cell_size)))
        return 0.6;
    else if (((20*cell_size)<d_min)&&(d_min<=(30*cell_size)))
        return 0.5;
    else if (((30*cell_size)<d_min)&&(d_min<=(50*cell_size)))
        return 0.4;
    else if (((50*cell_size)<d_min)&&(d_min<=(70*cell_size)))
        return 0.3;
    else if (((70*cell_size)<d_min)&&(d_min<=(90*cell_size)))
        return 0.2;
    else
        return 0.1;*/
    double p_=1-j/7.5;
    return p_;
}

