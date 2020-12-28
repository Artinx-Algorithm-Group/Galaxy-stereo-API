#include <iostream>
#include <type_traits>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "StereoCamera/StereoCamera.hpp"
#include "GxCamera/GxCamera.hpp"

using std::string;
using std::remove_const;
using std::cout;
using std::cerr;
using std::endl;

// using std::chrono::milliseconds;
using std::chrono::duration;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

using cv::Mat;
using cv::cvtColor;

using StereoCamera::Stereo;
using StereoCamera::StereoStatus;

namespace {
    const uint32_t kExposureTimeUpperLimit = 1000000;
    const uint32_t kExposureTimeLowerLimit = 20;
    const uint8_t kFrameRateUpperLimit = 200;
    const uint8_t kFrameRateLowerLimit = 1;
}

StereoStatus Stereo::StereoInitSerialNumber(char *left_cam_serial_num, 
                                            char *right_cam_serial_num) {
    this->left_cam_.camStatus = 
    this->left_cam_.CameraInitOpsSerialNumber(left_cam_serial_num);

    this->right_cam_.camStatus = 
    this->right_cam_.CameraInitOpsSerialNumber(right_cam_serial_num);

    if(this->left_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Left camera initialization fail, error code " 
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    } else if (this->right_cam_.camStatus != GX_STATUS_SUCCESS){
        cerr << "Right camera initialization fail, error code " 
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    } else {
        cout << "Stereo camera initialization success" << endl;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetLeftCamExposureTime(const uint32_t exposure_time) {
    if(exposure_time < kExposureTimeLowerLimit || exposure_time > kExposureTimeUpperLimit) {
        cerr << "Exposure time out Of bound" << endl;
        return StereoStatus::kExposureTimeOutOfBound;
    }

    this->left_cam_.camStatus = this->left_cam_.SetExposureTime(exposure_time);

    if(this->left_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Set left camera exposure time fail, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Left camera exposure time set to " << exposure_time << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetRightCamExposureTime(const uint32_t exposure_time) {
    if(exposure_time < kExposureTimeLowerLimit || exposure_time > kExposureTimeUpperLimit) {
        cerr << "Exposure time out Of bound" << endl;
        return StereoStatus::kExposureTimeOutOfBound;
    }

    this->right_cam_.camStatus = this->right_cam_.SetExposureTime(exposure_time);

    if(this->right_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Set right camera exposure time fail, error code "
             << this->right_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Right camera exposure time set to " << exposure_time << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetStereoCamExposureTime(const uint32_t exposure_time) {
    StereoStatus left_ret = this->SetLeftCamExposureTime(exposure_time);
    StereoStatus right_ret = this->SetRightCamExposureTime(exposure_time);

    if (left_ret != StereoStatus::kStereoSuccess || right_ret != StereoStatus::kStereoSuccess) {
        return StereoStatus::kSetStereoExposureFail;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetLeftCamFrameRate(const uint16_t frame_rate) {
    if(frame_rate < kFrameRateLowerLimit || frame_rate > kFrameRateUpperLimit) {
        cerr << "Frame rate out Of bound" << endl;
        return StereoStatus::kFrameRateOutOfBound;
    }

    this->left_cam_.camStatus = this->left_cam_.SetFrameRate(frame_rate);
    if(this->left_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Set left camera frame rate fail, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Left camera frame rate set to " << frame_rate << endl;
    return StereoStatus::kStereoSuccess;

}

StereoStatus Stereo::SetRightCamFrameRate(const uint16_t frame_rate) {
    if(frame_rate < kFrameRateLowerLimit || frame_rate > kFrameRateUpperLimit) {
        cerr << "Frame rate out Of bound" << endl;
        return StereoStatus::kFrameRateOutOfBound;
    }

    this->right_cam_.camStatus = this->right_cam_.SetFrameRate(frame_rate);
    if(this->right_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Set right camera frame rate fail, error code "
             << this->right_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Right camera frame rate set to " << frame_rate << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetStereoCamFrameRate(const uint16_t frame_rate) {
    StereoStatus left_ret = this->SetLeftCamFrameRate(frame_rate);
    StereoStatus right_ret = this->SetRightCamFrameRate(frame_rate);

    if (left_ret != StereoStatus::kStereoSuccess || right_ret != StereoStatus::kStereoSuccess) {
        return StereoStatus::kSetStereoFrameRateFail;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StartLeftCamStream() {
    this->left_cam_.camStatus = this->left_cam_.StreamOn();

    if(this->left_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Start left camera stream fail, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Left camera stream start" << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StartRightCamStream() {
    this->right_cam_.camStatus = this->right_cam_.StreamOn();

    if(this->right_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Start right camera stream fail, error code "
             << this->right_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Right camera stream start" << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StartStereoStream() {
    StereoStatus left_ret = this->StartLeftCamStream();
    StereoStatus right_ret = this->StartRightCamStream();

    if (left_ret != StereoStatus::kStereoSuccess || right_ret != StereoStatus::kStereoSuccess) {
        return StereoStatus::kSetStereoFrameRateFail;
    }

    return StereoStatus::kStreamStartFail;
}

StereoStatus Stereo::GetColorImgStereo(Mat &left_img, Mat &right_img, double &timestamp) {

    // steady_clock::time_point left_cam_cap_start = steady_clock::now();
    this->left_cam_.GetColorImg(left_img);
    // steady_clock::time_point left_cam_cap_end = steady_clock::now();
    // milliseconds left_cap_time = duration_cast<milliseconds>(left_cam_cap_end - left_cam_cap_start);

    // steady_clock::time_point right_cam_cap_start = steady_clock::now();
    this->right_cam_.GetColorImg(right_img);
    // steady_clock::time_point right_cam_cap_end = steady_clock::now();
    // milliseconds right_cap_time = duration_cast<milliseconds>(right_cam_cap_end - right_cam_cap_start);

    // cout << "Left cam capture time: " << left_cap_time.count() << " ms" << endl;
    // cout << "Right cam capture time: " << right_cap_time.count() << " ms" << endl;

    if(is_timestamp_init_ == false) {
        capture_start_time_ = std::chrono::steady_clock::now();
        timestamp = 0.0;
        is_timestamp_init_ = true;
    } else {
        steady_clock::time_point cap_time = steady_clock::now();
        duration<double> time_from_start = duration_cast<duration<double>>(cap_time - capture_start_time_);
        timestamp = time_from_start.count();
    }

    if (left_img.empty()) {
        cerr << "No data captured from left camera" << endl;
        return StereoStatus::kGetLeftColorImgFail;
    } else if (right_img.empty()) {
        cerr << "No data captured from right camera" << endl;
        return StereoStatus::kGetRightColorImgFail;
    }

    cvtColor(left_img, left_img, cv::COLOR_RGB2BGR);
    cvtColor(right_img, right_img, cv::COLOR_RGB2BGR);

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StereoStreamOff() {
    this->left_cam_.camStatus = this->left_cam_.StreamOff();
    this->right_cam_.camStatus = this->right_cam_.StreamOff();

    if (this->left_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Fail to turn off left camera stream, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    } else if (this->right_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Fail to turn off right camera stream, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Stereo stream turned off" << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StereoClose() {
    this->left_cam_.camStatus = this->left_cam_.CameraCloseOps();
    this->right_cam_.camStatus = this->right_cam_.CameraCloseOps();

    if (this->left_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Fail to turn off left camera stream, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    } else if (this->right_cam_.camStatus != GX_STATUS_SUCCESS) {
        cerr << "Fail to turn off right camera stream, error code "
             << this->left_cam_.camStatus << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "Stereo camera successfully closed" << endl;
    return StereoStatus::kStereoSuccess;
}
