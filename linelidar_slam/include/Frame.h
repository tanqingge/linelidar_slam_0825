#ifndef __FRAME_H_
#define __FRAME_H_

#include "utility.h"


struct LocalLineSegs
{
    float x_start_,y_start_,x_end_,y_end_;
    bool isnew_;//whether a new segment for map
    int count_;// the number of calclate as valid segments
};

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long frame_id_;
    uint32_t seq_; 
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
    std::vector <LocalLineSegs> local_lines_;
    bool is_key_frame_;

public:
    Frame(long id);
    ~Frame();
    static Frame::Ptr createFrame(); 
    void Setpose(const Eigen::Vector3d& Trans_, const Eigen::Quaterniond& orien_ );
    void updateFrame();


};
#endif