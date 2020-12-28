#include <csignal>
#include <opencv2/opencv.hpp>
#include <string>

#include "StereoCamera/StereoCamera.hpp"

using std::signal;
using std::cout;
using std::endl;
using std::to_string;

using cv::Mat;
using cv::hconcat;
using cv::imshow;
using cv::waitKey;
using cv::resize;
using cv::imwrite;

namespace {
    const uint32_t kExposureTime = 15000;
    const uint16_t kFrameRate = 120;

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

    Mat left_color_img, right_color_img, combined_img;
    double timestamp = 0.0;

    signal(SIGINT, SigintHandler);
    while (!stop_flag){
        stereo.GetColorImgStereo(left_color_img, right_color_img, timestamp);
        cout << "Timestamp in second: " << timestamp << endl;

        hconcat(left_color_img, right_color_img, combined_img);
        // Resize image for better display
        // resize(combined_img, combined_img, cv::Size(), 0.6, 0.6);
        imshow("Stereo output", combined_img);
        
        if(waitKey(10) == 'c') {
            imwrite("left_" + to_string(save_frame_index) + ".png", left_color_img);
            imwrite("right_" + to_string(save_frame_index) + ".png", right_color_img);
            ++save_frame_index;
            cout << "Stereo frame saved" << endl;
        }
    }

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}
