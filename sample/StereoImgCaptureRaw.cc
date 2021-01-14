#include <csignal>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

#include "StereoCamera/StereoCamera.hpp"

using std::signal;
using std::cout;
using std::cerr;
using std::endl;
using std::to_string;
using std::chrono::steady_clock;
using std::chrono::microseconds;
using std::chrono::duration_cast;

using cv::Mat;
using cv::hconcat;
using cv::imshow;
using cv::waitKey;
using cv::imwrite;
using cv::cvtColor;
using cv::line;
using cv::Point;
using cv::Scalar;

using StereoCamera::StereoStatus;
using StereoCamera::StereoFrame;

namespace {
    const double kExposureTime = 20.0;
    const double kFrameRate = 150.0;

    // c-style string serial number for code compatibility
    char left_cam_serial_num[] = "KE0200080465";
    char right_cam_serial_num[] = "KE0200080462";

    bool stop_flag = false;

    int save_frame_index = 0;
}

void SigintHandler(int sig) {
    stop_flag = true;
    cout << "SIGINT received, exiting" << endl;
}

int main() {

    StereoCamera::Stereo stereo;

    stereo.StereoInitSerialNumber(left_cam_serial_num, right_cam_serial_num);

    stereo.SetStereoCamExposureTime(kExposureTime);
    stereo.SetStereoCamFrameRate(kFrameRate);

    stereo.StartStereoStream();

    Mat combined_img;
    StereoFrame stereo_frame;

    signal(SIGINT, SigintHandler);
    while (!stop_flag){

        // steady_clock::time_point cap_start = steady_clock::now();
        stereo.SendSoftTrigger();
        stereo.GetColorImgStereo(stereo_frame);
        // steady_clock::time_point right_cam_cap_end = steady_clock::now();
        // microseconds cap_time = duration_cast<microseconds>(right_cam_cap_end - cap_start);
        // cout << "Capture time: " << cap_time.count() << endl;
        // cout << "Timestamp in second: " << stereo_frame.timestamp() << endl;

        hconcat(stereo_frame.left_img(), stereo_frame.right_img(), combined_img);
        cvtColor(combined_img, combined_img, cv::COLOR_RGB2BGR);

        imshow("Stereo output", combined_img);
        
        if(waitKey(10) == 'c') {
            imwrite("cap/left_" + to_string(save_frame_index) + ".png", stereo_frame.left_img());
            imwrite("cap/right_" + to_string(save_frame_index) + ".png", stereo_frame.right_img());
            ++save_frame_index;
            cout << "Stereo frame saved" << endl;
        }
    }

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}
