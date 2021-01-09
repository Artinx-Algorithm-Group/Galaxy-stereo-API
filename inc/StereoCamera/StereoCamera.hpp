/**
 * @file    StereoCamera.hpp
 * @brief   Class decleration for stereo driver
 * @author  Jing Yonglin
 * @mail:   11712605@mail.sustech.edu.cn
 *          yonglinjing7@gmail.com
*/

#ifndef _STEREO_CAMERA_HPP_
#define _STEREO_CAMERA_HPP_

#include <chrono>

#include <opencv2/opencv.hpp>

#include "GxCamera/GxCamera.hpp"

namespace StereoCamera{

enum class StereoStatus{
    kStereoSuccess              = 0,
    kCameraDriverError          = 1,
    kSetStereoExposureFail      = 2,
    kSetStereoFrameRateFail     = 3,
    kStreamStartFail            = 4,
    kExposureTimeOutOfBound     = 5,
    kFrameRateOutOfBound        = 6,
    kGetLeftColorImgFail        = 7,
    kGetRightColorImgFail       = 8,
    kLoadStereoCaliDataFail     = 9,
    kStereoCaliDataNotAvailable = 10,
    kSendSoftTriggerFail        = 11
};

enum class FrameFormat{
    kRGB = 0
};

class Stereo{

public:
    ~Stereo() {};

    StereoStatus StereoInitSerialNumber(char *left_cam_serial_num, 
                                        char *right_cam_serial_num);

    StereoStatus SetLeftCamExposureTime(const double exposure_time);
    StereoStatus SetRightCamExposureTime(const double exposure_time);
    StereoStatus SetStereoCamExposureTime(const double exposure_time);

    StereoStatus SetLeftCamFrameRate(const double frame_rate);
    StereoStatus SetRightCamFrameRate(const double frame_rate);
    StereoStatus SetStereoCamFrameRate(const double frame_rate);

    StereoStatus StartLeftCamStream();
    StereoStatus StartRightCamStream();
    StereoStatus StartStereoStream();

    StereoStatus LoadStereoCaliData(const std::string cali_data_path);

    StereoStatus SendSoftTrigger();

    StereoStatus GetColorImgStereo(cv::Mat &left_img, cv::Mat &right_img, double &timestamp);
    StereoStatus GetColorImgStereoRectified(cv::Mat &left_img, cv::Mat &right_img, double &timestamp);

    StereoStatus StereoStreamOff();
    StereoStatus StereoClose();

    FrameFormat frame_format = FrameFormat::kRGB;

private:
    GxCamera::Camera left_cam_;
    GxCamera::Camera right_cam_;

    int frame_width_;
    int frame_height_;

    cv::Mat left_cam_intrinsic_mat_;
    cv::Mat right_cam_intrinsic_mat_;

    cv::Mat left_cam_dist_param_;
    cv::Mat right_cam_dist_param_;

    cv::Mat rot_mat_; // Rotation matrix from left camera to right camera
    cv::Mat trans_vec_; // Translation vector from left camera to right camera

    // Map for undistortion and rectification
    cv::Mat left_map1_, left_map2_, right_map1_, right_map2_;

    std::chrono::steady_clock::time_point capture_start_time_;
    std::chrono::steady_clock::time_point last_trigger_time_;

    bool is_timestamp_init_ = false;
    bool is_rectified_img_available = false;
};

}

#endif