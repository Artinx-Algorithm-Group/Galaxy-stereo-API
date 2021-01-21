#include <csignal>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>

#include "StereoCamera/StereoCameraMutiThread.hpp"

using std::signal;
using std::cout;
using std::cerr;
using std::endl;
using std::thread;

using std::chrono::steady_clock;
using std::chrono::microseconds;
using std::chrono::duration_cast;

using cv::Mat;
using cv::hconcat;
using cv::imshow;
using cv::cvtColor;
using cv::waitKey;

using StereoCamera::StereoStatus;
using StereoCamera::StereoMultithread;
using StereoCamera::StereoFrame;

namespace {
    const double kExposureTime = 15000.0;
    const double kFrameRate = 200.0;

    // c-style string serial number for code compatibility
    char left_cam_serial_num[] = "KE0200080465";
    char right_cam_serial_num[] = "KE0200080462";

    bool stop_flag = false;
}

void SigintHandler(int sig) {
    stop_flag = true;
    cout << "SIGINT received, exiting" << endl;
}

int main(int argc, char **argv) {
    StereoCamera::StereoMultithread stereo;

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

    stereo.ComputeRectParam();
    stereo.SetFrameDownSampleFactor(0.5);

    stereo.StartStereoStream();

    signal(SIGINT, SigintHandler);
    thread img_acquire_thread(&StereoMultithread::MutithreadCaptureTask, &stereo);

    while (!stop_flag){
        StereoFrame stereo_frame;
        Mat combined_img;

        // steady_clock::time_point cap_start = steady_clock::now();
        stereo.AcquireStereoFrameFromThread(stereo_frame);
        // steady_clock::time_point right_cam_cap_end = steady_clock::now();
        // microseconds cap_time = duration_cast<microseconds>(right_cam_cap_end - cap_start);
        // cout << "Capture time: " << cap_time.count() << " us" << endl;

        hconcat(stereo_frame.left_img(), stereo_frame.right_img(), combined_img);
        cvtColor(combined_img, combined_img, cv::COLOR_RGB2BGR);
        imshow("out", combined_img);
        waitKey(1);
    }

    stereo.TerminateTask();

    img_acquire_thread.join();

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}