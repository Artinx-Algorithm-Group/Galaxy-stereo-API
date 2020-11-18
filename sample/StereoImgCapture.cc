#include <csignal>
#include <opencv2/opencv.hpp>

#include "StereoCamera/StereoCamera.hpp"

using std::signal;
using std::cout;
using std::endl;

using cv::Mat;
using cv::hconcat;
using cv::imshow;
using cv::waitKey;
using cv::resize;

namespace {
    const uint32_t kExposureTime = 7500;
    const uint16_t kFrameRate = 150;

    // c-style string serial number for code compatibility
    char left_cam_serial_num[] = "KE0200080462";
    char right_cam_serial_num[] = "KE0200080465";

    bool stop_flag = false;
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

    signal(SIGINT, SigintHandler);
    while (!stop_flag){
        stereo.GetColorImgStereo(left_color_img, right_color_img);

        hconcat(left_color_img, right_color_img, combined_img);
        // Resize image for better display
        resize(combined_img, combined_img, cv::Size(), 0.6, 0.6);
        imshow("Stereo output", combined_img);
        waitKey(1);
    }

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}
