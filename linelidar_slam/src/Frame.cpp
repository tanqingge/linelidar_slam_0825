#include "Frame.h"

Frame::Frame(long id)
{
    
    this->frame_id_=id;
    std::cout<<"id"<<id<<std::endl;
    this->is_key_frame_=false;
};

Frame::Ptr Frame::createFrame()
{
    static long id=0;
    return Frame::Ptr(new Frame(id++));
};

void Frame::Setpose(const Eigen::Vector3d& Trans_, const Eigen::Quaterniond& orien_ )
{

};

void Frame::updateFrame()
{

};

Frame::~Frame()
{

};
