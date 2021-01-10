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
        {
            lock_guard<mutex> guard(this->buffer_mutex_);
            while (this->buffer_.size() >= this->buffer_size_){
                this->buffer_.pop();
            }
            this->buffer_.push(stereo_frame);
        }
    }
    
}

bool StereoMultithread::IsBufferEmpty(){
    lock_guard<mutex> guard(this->buffer_mutex_);
    return this->buffer_.empty();
}

void StereoMultithread::AcquireStereoFrameFromThread(StereoFrame &stereo_frame){
    lock_guard<mutex> guard(this->buffer_mutex_);

    stereo_frame = this->buffer_.front();
    this->buffer_.pop();
}

void StereoMultithread::TerminateTask(){
    this->stop_flag_ = true;
}
