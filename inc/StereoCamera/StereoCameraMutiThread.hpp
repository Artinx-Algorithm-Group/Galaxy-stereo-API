#ifndef _STEREO_CAMERA_MUTITHREAD_HPP_
#define _STEREO_CAMERA_MUTITHREAD_HPP_

#include <queue>
#include <mutex>

#include <opencv2/opencv.hpp>

#include "StereoCamera/StereoCamera.hpp"

namespace StereoCamera{

enum class ThreadCaptureStatus{
    kCaptureSuccess = 0,
    kBufferEmpty
};

class StereoMultithread : public Stereo{
public:
    void MutithreadCaptureTask();
    void TerminateTask();

    bool IsBufferEmpty();

    void AcquireStereoFrameFromThread(StereoFrame &stereo_frame);

private:
    std::mutex buffer_mutex_;
    std::queue<StereoFrame> buffer_;

    uint8_t buffer_size_ = 2;

    bool stop_flag_ = false;
};

}

#endif