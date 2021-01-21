#ifndef _STEREO_CAMERA_MUTITHREAD_HPP_
#define _STEREO_CAMERA_MUTITHREAD_HPP_

#include <queue>
#include <mutex>

#include <opencv2/opencv.hpp>

#include "StereoCamera/StereoCamera.hpp"
#include "ThreadSafeQueue/ThreadSafeQueue.hpp"

namespace StereoCamera{

enum class ThreadCaptureStatus{
    kCaptureSuccess = 0,
    kBufferEmpty
};

class StereoMultithread : public Stereo{
public:
    StereoMultithread() = default;

    void MutithreadCaptureTask();
    void TerminateTask();

    void AcquireStereoFrameFromThread(StereoFrame &stereo_frame);

private:

    ThreadSafe::ThreadSafeQueue<StereoFrame> buffer_queue_;

    bool stop_flag_ = false;
};

}

#endif