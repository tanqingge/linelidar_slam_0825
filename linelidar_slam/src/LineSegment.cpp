#include "LineSegment.h"



LineSegment::LineSegment(){
    segment_init();
};
        
LineSegment::~LineSegment(){

};
        
void LineSegment::segment_init(){
    for (int i=0;i<MAXSEGMENTS;i++)
    {
        init_segs_[i].count_=0;
        init_segs_[i].grad_=0;
        init_segs_[i].invgrad_=0;
        init_segs_[i].x_=0;
        init_segs_[i].y_=0;
        init_segs_[i].xx_=0;
        init_segs_[i].xy_=0;
        init_segs_[i].yy_=0;
        init_segs_[i].x_start_=0;
        init_segs_[i].y_start_=0;
        init_segs_[i].x_end_=0;
        init_segs_[i].y_end_=0;
        init_segs_[i].xMean_=0;
        init_segs_[i].yMean_=0;
    }
};
        
void LineSegment::segment_extraction(pcl::PointCloud<pcl::PointXY>::Ptr LaserPoint)
{
    int line_num_=0;
    for (int i=0;i<LaserPoint->points.size();i++)
    {
        int seg_state_=0;
        float x_temp_=LaserPoint->points[i].x;
        float y_temp_=LaserPoint->points[i].y;
        for (int j=0; j<line_num_; j++)
        {
            double d_,err_;
            if (init_segs_[j].grad_<__DBL_MAX__)
            {
                d_=init_segs_[j].grad_*x_temp_-y_temp_+init_segs_[j].yMean_-init_segs_[j].grad_*init_segs_[j].xMean_;
                err_=abs(d_)/sqrt(init_segs_[j].grad_*init_segs_[j].grad_+1);
            }
            else
            {
                d_=init_segs_[j].invgrad_*y_temp_-x_temp_+init_segs_[j].xMean_-init_segs_[j].invgrad_*init_segs_[j].yMean_;
                err_=abs(d_)/sqrt(init_segs_[j].invgrad_*init_segs_[j].invgrad_+1);
            }
            double d1_=sqrt(pow(x_temp_-init_segs_[j].x_end_,2)+pow(y_temp_-init_segs_[j].y_end_,2));
            double d2_=sqrt(pow(x_temp_-init_segs_[j].x_start_,2)+pow(y_temp_-init_segs_[j].y_start_,2));
            if ((err_<connect_th_) && ((d1_<min_distance_)||(d2_<min_distance_)))
            {
                init_segs_[j].xMean_=(init_segs_[j].count_*init_segs_[j].xMean_+x_temp_)/(init_segs_[j].count_+1);
                init_segs_[j].yMean_=(init_segs_[j].count_*init_segs_[j].yMean_+y_temp_)/(init_segs_[j].count_+1);
                init_segs_[j].x_ += x_temp_;
                init_segs_[j].y_ += y_temp_;
                init_segs_[j].xx_ += pow(x_temp_,2);
                init_segs_[j].yy_ += pow(y_temp_,2);
                init_segs_[j].xy_ += x_temp_*y_temp_;
                init_segs_[j].count_ += 1;
                seg_state_ +=1;
                init_segs_[j].grad_=(init_segs_[j].xy_-init_segs_[j].x_*init_segs_[j].y_/init_segs_[j].count_)/(init_segs_[j].xx_-init_segs_[j].x_*init_segs_[j].x_/init_segs_[j].count_);
                if (init_segs_[j].grad_==__DBL_MAX__)
                    init_segs_[j].invgrad_=(init_segs_[j].xy_-init_segs_[j].x_*init_segs_[j].y_/init_segs_[j].count_)/(init_segs_[j].yy_-init_segs_[j].y_*init_segs_[j].y_/init_segs_[j].count_);
                init_segs_[j].x_end_=x_temp_;
                init_segs_[j].y_end_=y_temp_;

            }
        }
        if ((seg_state_==0) && (line_num_<MAXSEGMENTS))
        {
            init_segs_[line_num_].x_start_=x_temp_;
            init_segs_[line_num_].y_start_=y_temp_;
            init_segs_[line_num_].x_end_=x_temp_;
            init_segs_[line_num_].y_end_=y_temp_;
            init_segs_[line_num_].xMean_=x_temp_;
            init_segs_[line_num_].yMean_=y_temp_;
            init_segs_[line_num_].xx_=pow(x_temp_,2);
            init_segs_[line_num_].yy_=pow(y_temp_,2);
            init_segs_[line_num_].xy_=x_temp_*y_temp_;
            init_segs_[line_num_].count_=1;
            init_segs_[line_num_].x_=x_temp_;
            init_segs_[line_num_].y_=y_temp_;
            line_num_ += 1;
        }
    }
};   

void LineSegment::segment_valid()
{   struct Valid_SegmentStats segs_temp_;
    //int num__=0;
    for (int i=0;i<MAXSEGMENTS;i++)
    {
        double length_=sqrt(pow(init_segs_[i].x_end_-init_segs_[i].x_start_,2)+pow(init_segs_[i].y_end_-init_segs_[i].y_start_,2));
        //std::cout<< "i:"<<i<<"Length"<<length_<<std::endl;
        if((init_segs_[i].count_>count_threhold_)&&(length_>=length_limit_))
        {
            if(init_segs_[i].grad_!=__DBL_MAX__)
            {
                segs_temp_.x_start_=init_segs_[i].xMean_-length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                segs_temp_.y_start_=init_segs_[i].yMean_-init_segs_[i].grad_*length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                segs_temp_.x_end_=init_segs_[i].xMean_+length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                segs_temp_.y_end_=init_segs_[i].yMean_+init_segs_[i].grad_*length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));

            }
            else if ((init_segs_[i].grad_==__DBL_MAX__)&&(init_segs_[i].invgrad_!=0))
            {
                segs_temp_.x_start_=init_segs_[i].xMean_-init_segs_[i].invgrad_*length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                segs_temp_.y_start_=init_segs_[i].yMean_-length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                segs_temp_.x_end_=init_segs_[i].xMean_+init_segs_[i].invgrad_*length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                segs_temp_.y_end_=init_segs_[i].yMean_+length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
            }
            valid_segs_.push_back(segs_temp_);
           // num__+=1;
            //std::cout<<num__<<" "<<length_<<std::endl;
        }

    }
};