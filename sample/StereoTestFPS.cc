#include <csignal>
#include <opencv2/opencv.hpp>
#include <string>

#include "StereoCamera/StereoCamera.hpp"

using std::signal;
using std::cout;
using std::cerr;
using std::endl;
using std::to_string;

using cv::Mat;

using StereoCamera::StereoStatus;

namespace {
    const uint32_t kExposureTime = 10000;
    const uint16_t kFrameRate = 150;

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
    double start_point = 0.0;
    int frame_count = 0;

    signal(SIGINT, SigintHandler);
    stereo.GetColorImgStereoRectified(left_rect, right_rect, start_point);
    while (!stop_flag){
        stereo.GetColorImgStereoRectified(left_rect, right_rect, timestamp);
        frame_count += 1;
        if(timestamp - start_point > 1.0){
            start_point = timestamp;
            cout << frame_count << " frames in last second" << endl;
            frame_count = 0;
        }
    }

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}
