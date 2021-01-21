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
using StereoCamera::StereoFrame;

namespace {
    const uint32_t kExposureTime = 15000.0;
    const uint16_t kFrameRate = 180.0;

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

    stereo.ComputeRectParam();
    stereo.SetFrameDownSampleFactor(0.5);

    stereo.StartStereoStream();

    Mat left_rect, right_rect, combined_img;
    StereoFrame stereo_frame;
    
    int frame_count = 0;

    signal(SIGINT, SigintHandler);
    stereo.SendSoftTrigger();
    stereo.GetColorImgStereoRectified(stereo_frame);

    double start_point = stereo_frame.timestamp();
    double timestamp = 0.0;
    while (!stop_flag){
        stereo.SendSoftTrigger();
        stereo.GetColorImgStereoRectified(stereo_frame);
        frame_count += 1;
        timestamp = stereo_frame.timestamp();
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
