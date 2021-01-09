#include <csignal>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>

#include "StereoCamera/StereoCamera.hpp"

using std::signal;
using std::cout;
using std::cerr;
using std::endl;
using std::to_string;
using std::thread;
using std::queue;
using std::mutex;
using std::lock_guard;

using std::chrono::steady_clock;
using std::chrono::microseconds;
using std::chrono::duration_cast;

using cv::Mat;
using cv::hconcat;
using cv::imshow;
using cv::cvtColor;
using cv::waitKey;

using StereoCamera::StereoStatus;

namespace {
    StereoCamera::Stereo stereo;
    
    const double kExposureTime = 20.0;
    const double kFrameRate = 150.0;

    // c-style string serial number for code compatibility
    char left_cam_serial_num[] = "KE0200080465";
    char right_cam_serial_num[] = "KE0200080462";

    bool stop_flag = false;

    mutex img_queue_mutex;
    //mutex img_dequeue_mutex;

    const int kImgQueueSize = 2;
    queue<Mat> img_queue;
}

void SigintHandler(int sig) {
    stop_flag = true;
    cout << "SIGINT received, exiting" << endl;
}

void EnqueueImg(Mat& img){
    lock_guard<mutex> guard(img_queue_mutex);
    while (img_queue.size() >= kImgQueueSize){
        img_queue.pop();
    }
    
    img_queue.push(img);
}

void ImgAcquireTask(){

    Mat left_rect, right_rect, combined_img;
    double timestamp = 0.0;
    while (!stop_flag){
        stereo.SendSoftTrigger();
        stereo.GetColorImgStereoRectified(left_rect, right_rect, timestamp);
        hconcat(left_rect, right_rect, combined_img);
        EnqueueImg(combined_img);
    }
    
}

int main(int argc, char **argv) {
    stereo.StereoInitSerialNumber(left_cam_serial_num, right_cam_serial_num);

    stereo.SetStereoCamExposureTime(kExposureTime);
    stereo.SetStereoCamFrameRate(kFrameRate);

    if(argc > 1){
        cout << "Stereo calibration data path: " << argv[1] << endl;
        StereoStatus status = stereo.LoadStereoCaliData(argv[1]);
        if(status != StereoStatus::kStereoSuccess) {
            cerr << "Load stereo calibration data failed" << endl;
            return EXIT_FAILURE;
        }
    }

    stereo.StartStereoStream();

    signal(SIGINT, SigintHandler);
    thread img_acquire_thread(ImgAcquireTask);

    Mat current_img;
    while (!stop_flag){
        // steady_clock::time_point cap_start = steady_clock::now();
        {   // Automatically destoryed when out of scope
            lock_guard<mutex> guard(img_queue_mutex);

            if(img_queue.empty()){
                continue;
            }
            current_img = img_queue.front();
            img_queue.pop();
        }
        // steady_clock::time_point right_cam_cap_end = steady_clock::now();
        // microseconds cap_time = duration_cast<microseconds>(right_cam_cap_end - cap_start);
        // cout << "Capture time: " << cap_time.count() << " us" << endl;

        cvtColor(current_img, current_img, cv::COLOR_RGB2BGR);
        imshow("out", current_img);
        waitKey(10);
    }

    img_acquire_thread.join();

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}