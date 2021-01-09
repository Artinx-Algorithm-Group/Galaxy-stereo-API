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
using cv::resize;
using cv::imwrite;
using cv::cvtColor;
using cv::line;
using cv::Point;
using cv::Scalar;

using StereoCamera::StereoStatus;

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

int main(int argc, char **argv) {

    StereoCamera::Stereo stereo;

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

    Mat left_rect, right_rect, combined_img;
    double timestamp = 0.0;

    signal(SIGINT, SigintHandler);
    while (!stop_flag){
        steady_clock::time_point cap_start = steady_clock::now();
        stereo.SendSoftTrigger();
        stereo.GetColorImgStereoRectified(left_rect, right_rect, timestamp);
        steady_clock::time_point right_cam_cap_end = steady_clock::now();
        microseconds cap_time = duration_cast<microseconds>(right_cam_cap_end - cap_start);
        cout << "Capture time: " << cap_time.count() << endl;
        cout << "Timestamp in second: " << timestamp << endl;

        cvtColor(left_rect, left_rect, cv::COLOR_RGB2BGR);
        cvtColor(right_rect, right_rect, cv::COLOR_RGB2BGR);

        hconcat(left_rect, right_rect, combined_img);

        // Resize image for better display
        // resize(combined_img, combined_img, cv::Size(), 0.6, 0.6);

        // Display horizontal line for verifying rectification result 
        Point pt1(0, combined_img.size().height / 2);
        Point pt2(combined_img.size().width, combined_img.size().height / 2);
        Scalar line_color(0, 0, 255);
        line(combined_img, pt1, pt2, line_color);

        imshow("Stereo output", combined_img);
        
        if(waitKey(10) == 'c') {
            imwrite("left_" + to_string(save_frame_index) + ".png", left_rect);
            imwrite("right_" + to_string(save_frame_index) + ".png", right_rect);
            ++save_frame_index;
            cout << "Stereo frame saved" << endl;
        }
    }

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}
