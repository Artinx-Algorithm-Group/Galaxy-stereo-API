/**
 * @file    StereoCamera.cc
 * @brief   Class implementation for stereo driver
 * @author  Jing Yonglin
 * @mail:   11712605@mail.sustech.edu.cn
 *          yonglinjing7@gmail.com
*/

#include <iostream>
#include <type_traits>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "StereoCamera/StereoCamera.hpp"
#include "GxCamera/GxCamera.hpp"

using std::string;
using std::cout;
using std::cerr;
using std::endl;

// using std::chrono::milliseconds;
using std::chrono::duration;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

using cv::Mat;
using cv::Mat_;
using cv::cvtColor;
using cv::stereoRectify;
using cv::Size;
using cv::initUndistortRectifyMap;
using cv::remap;
using cv::resize;

using StereoCamera::Stereo;
using StereoCamera::StereoStatus;
using StereoCamera::StereoFrame;

// namespace {
//     const double kExposureTimeUpperLimit = 1000000;
//     const double kExposureTimeLowerLimit = 20;
//     const uint8_t kFrameRateUpperLimit = 400;
//     const uint8_t kFrameRateLowerLimit = 1;
// }

StereoStatus Stereo::StereoInitSerialNumber(char *left_cam_serial_num, 
                                            char *right_cam_serial_num) {
    GX_STATUS status = GX_STATUS_SUCCESS;

    // Open with soft trigger mode by default

    status = this->left_cam_.CameraInit(left_cam_serial_num, true);

    status = this->right_cam_.CameraInit(right_cam_serial_num, true);

    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Left camera initialization fail, error code: " 
             << status << endl;
        return StereoStatus::kCameraDriverError;
    } else if (status != GX_STATUS_SUCCESS){
        cerr << "[StereoCam] Right camera initialization fail, error code: " 
             << status << endl;
        return StereoStatus::kCameraDriverError;
    } else {
        cout << "[StereoCam] Stereo camera initialization success" << endl;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetLeftCamExposureTime(const double exposure_time) {
    // if(exposure_time < kExposureTimeLowerLimit || exposure_time > kExposureTimeUpperLimit) {
    //     cerr << "[StereoCam] Exposure time out Of bound" << endl;
    //     return StereoStatus::kExposureTimeOutOfBound;
    // }

    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->left_cam_.SetExposureTime(exposure_time);

    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Set left camera exposure time fail, error code: "
             << status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Left camera exposure time set to " << exposure_time << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetRightCamExposureTime(const double exposure_time) {
    // if(exposure_time < kExposureTimeLowerLimit || exposure_time > kExposureTimeUpperLimit) {
    //     cerr << "[StereoCam] Exposure time out Of bound" << endl;
    //     return StereoStatus::kExposureTimeOutOfBound;
    // }

    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->right_cam_.SetExposureTime(exposure_time);

    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Set right camera exposure time fail, error code "
             << status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Right camera exposure time set to " << exposure_time << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetStereoCamExposureTime(const double exposure_time) {
    StereoStatus left_ret = this->SetLeftCamExposureTime(exposure_time);
    StereoStatus right_ret = this->SetRightCamExposureTime(exposure_time);

    if (left_ret != StereoStatus::kStereoSuccess || right_ret != StereoStatus::kStereoSuccess) {
        return StereoStatus::kSetStereoExposureFail;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetLeftCamFrameRate(const double frame_rate) {
    // if(frame_rate < kFrameRateLowerLimit || frame_rate > kFrameRateUpperLimit) {
    //     cerr << "[StereoCam] Frame rate out Of bound" << endl;
    //     return StereoStatus::kFrameRateOutOfBound;
    // }

    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->left_cam_.SetFrameRate(frame_rate);
    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Set left camera frame rate fail, error code "
             << status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Left camera frame rate set to " << frame_rate << endl;
    return StereoStatus::kStereoSuccess;

}

StereoStatus Stereo::SetRightCamFrameRate(const double frame_rate) {
    // if(frame_rate < kFrameRateLowerLimit || frame_rate > kFrameRateUpperLimit) {
    //     cerr << "[StereoCam] Frame rate out Of bound" << endl;
    //     return StereoStatus::kFrameRateOutOfBound;
    // }

    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->right_cam_.SetFrameRate(frame_rate);
    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Set right camera frame rate fail, error code "
             << status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Right camera frame rate set to " << frame_rate << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SetStereoCamFrameRate(const double frame_rate) {
    StereoStatus left_ret = this->SetLeftCamFrameRate(frame_rate);
    StereoStatus right_ret = this->SetRightCamFrameRate(frame_rate);

    if (left_ret != StereoStatus::kStereoSuccess || right_ret != StereoStatus::kStereoSuccess) {
        return StereoStatus::kSetStereoFrameRateFail;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StartLeftCamStream() {
    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->left_cam_.CameraStreamOn();

    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Start left camera stream fail, error code "
             << status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Left camera stream start" << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StartRightCamStream() {
    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->right_cam_.CameraStreamOn();

    if(status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Start right camera stream fail, error code "
             << status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Right camera stream start" << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StartStereoStream() {
    StereoStatus left_ret = this->StartLeftCamStream();
    StereoStatus right_ret = this->StartRightCamStream();

    if (left_ret != StereoStatus::kStereoSuccess || right_ret != StereoStatus::kStereoSuccess) {
        return StereoStatus::kSetStereoFrameRateFail;
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::LoadStereoCaliData(const std::string cali_data_path) {
    cv::FileStorage cali_data(cali_data_path, cv::FileStorage::READ);

    if(!cali_data.isOpened()){
        cerr << "[StereoCam] ERROR: Wrong path to calibration data" << endl;
        return StereoStatus::kLoadStereoCaliDataFail;
    }

    cali_data["LEFT.K"] >> this->left_cam_intrinsic_mat_;
    cali_data["LEFT.D"] >> this->left_cam_dist_param_;
    cali_data["RIGHT.K"] >> this->right_cam_intrinsic_mat_;
    cali_data["RIGHT.D"] >> this->right_cam_dist_param_;

    cali_data["R"] >> this->rot_mat_;
    cali_data["TRANS"] >> this->trans_vec_;

    // cout << "Left camera intrinsic:" << endl << this->left_cam_intrinsic_mat_ << endl;
    // cout << "Right camera intrinsic:" << endl << this->right_cam_intrinsic_mat_ << endl;
    // cout << "Left camera distortion:" << endl << this->left_cam_dist_param_ << endl;
    // cout << "Right camera distortion:" << endl << this->right_cam_dist_param_ << endl;

    // cout << "Rotation:" << endl << this->rot_mat_ << endl;
    // cout << "Translation:" << endl << this->trans_vec_ << endl;

    this->frame_width_ = cali_data["Frame.width"];
    this->frame_height_ = cali_data["Frame.height"];

    cout << "[StereoCam] calibration data loaded" << endl;

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::ComputeRectParam(){
    Mat R1, R2, P1, P2, Q;
    stereoRectify(this->left_cam_intrinsic_mat_, this->left_cam_dist_param_, 
                  this->right_cam_intrinsic_mat_, this->right_cam_dist_param_,
                  Size(this->frame_width_, this->frame_height_),
                  this->rot_mat_, this->trans_vec_,
                  R1, R2, P1, P2, Q);

    cout << "[StereoCam] New intrinsics matrix after rectify P1:" << endl << P1 << endl;
    cout << "[StereoCam] New intrinsics matrix after rectify P2:" << endl << P2 << endl;

    initUndistortRectifyMap(this->left_cam_intrinsic_mat_, this->left_cam_dist_param_, 
                            R1, P1, cv::Size(this->frame_width_, this->frame_height_),
                            CV_32F, this->left_map1_, this->left_map2_);

    initUndistortRectifyMap(this->right_cam_intrinsic_mat_, this->right_cam_dist_param_,
                            R2, P2, cv::Size(this->frame_width_, this->frame_height_),
                            CV_32F, this->right_map1_, this->right_map2_);

    // cout << "Left_map1: " << endl << this->left_map1_ << endl;
    // cout << "Left_map2: " << endl << this->left_map2_ << endl;
    // cout << "Right_map1: " << endl << this->right_map1_ << endl;
    // cout << "Right_map2: " << endl << this->right_map2_ << endl;

    this->is_rectified_img_available = true;

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::SendSoftTrigger(){
    GX_STATUS status = GX_STATUS_SUCCESS;

    status = this->left_cam_.SendSoftTrigger();
    status = this->right_cam_.SendSoftTrigger();

    if(status != GX_STATUS_SUCCESS){
        cerr << "[StereoCam] Send soft trigger fail" << endl;
        return StereoStatus::kSendSoftTriggerFail;
    }

    if(!is_timestamp_init_) {
        this->capture_start_time_ = std::chrono::steady_clock::now();
        this->last_trigger_time_ = this->capture_start_time_;
        is_timestamp_init_ = true;
    } else {
        this->last_trigger_time_ = steady_clock::now();
    }

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::GetColorImgStereo(StereoFrame &stereo_frame) {

    Mat left_img, right_img;
    double timestamp;
    // steady_clock::time_point left_cam_cap_start = steady_clock::now();
    this->left_cam_.GetLatestColorImg(left_img);
    // steady_clock::time_point left_cam_cap_end = steady_clock::now();
    // microseconds left_cap_time = duration_cast<microseconds>(left_cam_cap_end - left_cam_cap_start);

    // steady_clock::time_point right_cam_cap_start = steady_clock::now();
    this->right_cam_.GetLatestColorImg(right_img);
    // steady_clock::time_point right_cam_cap_end = steady_clock::now();
    // microseconds right_cap_time = duration_cast<microseconds>(right_cam_cap_end - right_cam_cap_start);

    // cout << "[StereoCam] Left cam capture time: " << left_cap_time.count() << " us" << endl;
    // cout << "[StereoCam] Right cam capture time: " << right_cap_time.count() << " us" << endl;

    duration<double> time_from_start = duration_cast<duration<double>>(this->last_trigger_time_ - this->capture_start_time_);
    timestamp = time_from_start.count();

    if(this->downsample_factor_ < 1.0){
        resize(left_img, left_img, Size(), this->downsample_factor_, this->downsample_factor_);
        resize(right_img, right_img, Size(), this->downsample_factor_, this->downsample_factor_);
    }

    StereoFrame _stereo_frame(left_img, right_img, timestamp);
    stereo_frame = _stereo_frame;

    if (left_img.empty()) {
        cerr << "[StereoCam] No data captured from left camera" << endl;
        return StereoStatus::kGetLeftColorImgFail;
    } else if (right_img.empty()) {
        cerr << "[StereoCam] No data captured from right camera" << endl;
        return StereoStatus::kGetRightColorImgFail;
    }

    // cvtColor(left_img, left_img, cv::COLOR_RGB2BGR);
    // cvtColor(right_img, right_img, cv::COLOR_RGB2BGR);

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::GetColorImgStereoRectified(StereoFrame& stereo_frame) {

    this->GetColorImgStereo(stereo_frame);

    if(! this->is_rectified_img_available) {
        cerr << "[StereoCam] ERROR: Cannot get rectified stereo image pair, calibration data not available." 
             << " ORIGINAL IMAGE RETURNED" << endl;
        return StereoStatus::kStereoCaliDataNotAvailable;
    }

    remap(stereo_frame.left_img(), stereo_frame.left_img(), this->left_map1_, this->left_map2_, cv::INTER_LINEAR);
    remap(stereo_frame.right_img(), stereo_frame.right_img(), this->right_map1_, this->right_map2_, cv::INTER_LINEAR);

    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StereoStreamOff() {
    GX_STATUS left_status = GX_STATUS_SUCCESS;
    GX_STATUS right_status = GX_STATUS_SUCCESS;

    left_status = this->left_cam_.CameraStreamOff();
    right_status = this->right_cam_.CameraStreamOff();

    if (left_status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Fail to turn off left camera stream, error code "
             << left_status << endl;
        return StereoStatus::kCameraDriverError;
    } else if (right_status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Fail to turn off right camera stream, error code "
             << right_status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Stereo stream turned off" << endl;
    return StereoStatus::kStereoSuccess;
}

StereoStatus Stereo::StereoClose() {
    GX_STATUS left_status = GX_STATUS_SUCCESS;
    GX_STATUS right_status = GX_STATUS_SUCCESS;

    left_status = this->left_cam_.CameraClose();
    right_status = this->right_cam_.CameraClose();

    if (left_status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Fail to turn off left camera stream, error code "
             << left_status << endl;
        return StereoStatus::kCameraDriverError;
    } else if (right_status != GX_STATUS_SUCCESS) {
        cerr << "[StereoCam] Fail to turn off right camera stream, error code "
             << right_status << endl;
        return StereoStatus::kCameraDriverError;
    }

    cout << "[StereoCam] Stereo camera successfully closed" << endl;
    return StereoStatus::kStereoSuccess;
}

int Stereo::FrameWidth(){
    return this->frame_width_;
}

int Stereo::FrameHeight(){
    return this->frame_height_;
}

void Stereo::SetFrameDownSampleFactor(double f){
    this->downsample_factor_ = f;

    this->left_cam_intrinsic_mat_.at<double>(0, 0) *= f;
    this->left_cam_intrinsic_mat_.at<double>(0, 2) *= f;
    this->left_cam_intrinsic_mat_.at<double>(1, 1) *= f;
    this->left_cam_intrinsic_mat_.at<double>(1, 2) *= f;

    this->right_cam_intrinsic_mat_.at<double>(0, 0) *= f;
    this->right_cam_intrinsic_mat_.at<double>(0, 2) *= f;
    this->right_cam_intrinsic_mat_.at<double>(1, 1) *= f;
    this->right_cam_intrinsic_mat_.at<double>(1, 2) *= f;

    this->frame_width_ *= f;
    this->frame_height_ *= f;

    cout << "[StereoCam] New left intrinsics after down sample: " << endl
         << this->left_cam_intrinsic_mat_ << endl;

    cout << "[StereoCam] New right intrinsics after down sample: " << endl
         << this->right_cam_intrinsic_mat_ << endl;

    this->ComputeRectParam();
}

cv::Mat Stereo::LeftCamIntrinsicMat(){
    return this->left_cam_intrinsic_mat_;
}

cv::Mat Stereo::RightCamIntrinsicMat(){
    return this->left_cam_intrinsic_mat_;
}

StereoFrame::StereoFrame(cv::Mat& left_img, cv::Mat& right_img, double timestamp){
    this->left_img_ = left_img.clone();
    this->right_img_ = right_img.clone();
    this->timestamp_ = timestamp;
}

Mat StereoFrame::left_img(){
    return this->left_img_;
}

Mat StereoFrame::right_img(){
    return this->right_img_;
}

double StereoFrame::timestamp(){
    return this->timestamp_;
}
