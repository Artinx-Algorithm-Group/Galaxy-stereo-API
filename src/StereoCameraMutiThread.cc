#include <mutex>

#include <opencv2/opencv.hpp>

#include "StereoCamera/StereoCameraMutiThread.hpp"

using std::mutex;
using std::lock_guard;

using cv::Mat;

using StereoCamera::StereoMultithread;
using StereoCamera::StereoFrame;
using StereoCamera::ThreadCaptureStatus;

void StereoMultithread::MutithreadCaptureTask(){
    while(!stop_flag_){
        StereoFrame stereo_frame;
        this->SendSoftTrigger();
        this->GetColorImgStereoRectified(stereo_frame);
        this->buffer_queue_.Push(stereo_frame);
    }
    
}

void StereoMultithread::AcquireStereoFrameFromThread(StereoFrame &stereo_frame_out){
    this->buffer_queue_.WaitAndPop(stereo_frame_out);
}

void StereoMultithread::TerminateTask(){
    this->stop_flag_ = true;
}
