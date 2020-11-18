/**
 * @file    GxCamera.hpp
 * @brief   This file is specific for the case that only one Daheng-color-camera connected
 * @author  Lin Zijun
 *              mail:   11810710@mail.sustech.edu.cn
 *                      1774525013@qq.com
*/


#ifndef _GXCAMERA_HPP_
#define _GXCAMERA_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <opencv2/opencv.hpp>

#include "GxCamera/GxIAPI.h"
#include "GxCamera/DxImageProc.h"

/// These are not recommended to modify
#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success

#define EXPOSURE_TIME_MAX 1000000.0
#define EXPOSURE_TIME_MIN 20.0

#define FRAME_RATE_MAX 10000.0                  /// Max frame rate (not practical)
#define FRAME_RATE_MIN 0.1                      /// Min frame rate



/// @brief For users, in other functions or in main, show error and return
/// @return GX_STATUS , the return code of main
#define GX_CHECK(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)      \
    {                                       \
        printf("<GX_CHECK>\n");            \
        return emStatus;                    \
    }                                       \
    
/// @brief  Not for users. Used in this class, in init steps , show error and do exit ops
///         For functions before PreForAcquisition()
/// @return GX_STATUS
#define GX_INIT_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)          \
    {                                           \
        printf("<GX_INIT_VERIFY_EXIT>\n");   \
        GetErrorString(emStatus);               \
        GXCloseDevice(this->hDevice);           \
        this->hDevice = NULL;                   \
        GXCloseLib();                           \
        printf("<App Exit!>\n");                \
        return emStatus;                        \
    }

/// @brief  Not for users. Used in this class, in running steps , show error
///         For functions after PreForAcquisition()
/// @return GX_STATUS
#define GX_RUNNING_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)              \
    {                                               \
        printf("<GX_RUNNING_VERIFY>\n");            \
        GetErrorString(emStatus);                   \
        return emStatus;                            \
    }

/// @brief  Not for Users. Used in this class, in running steps , show error and do exit ops
///         For functions after PreForAcquisition()
/// @return GX_STATUS
#define GX_RUNNING_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)              \
    {                                               \
        printf("<GX_RUNNING_VERIFY_EXIT>\n");       \
        GetErrorString(emStatus);                   \
        this->UnPreForAcquisition();                \
        GXCloseDevice(this->hDevice);               \
        this->hDevice = NULL;                       \
        GXCloseLib();                               \
        printf("<App Exit!>\n");                    \
        return emStatus;                            \
    }






void GetErrorString(GX_STATUS emErrorStatus);

class GxCamera
{
    private:
    GX_DEV_HANDLE hDevice = NULL;
    uint32_t nDeviceNum = 0;
    bool bColorFilter = 0;
    int64_t i64ColorFilter = GX_COLOR_FILTER_NONE;
    int64_t nPayloadSize = 0;

    PGX_FRAME_BUFFER pFrameBuffer = NULL;
    unsigned char* pRaw8ImgBuff = NULL;
    unsigned char* pRgb24ImgBuff = NULL;

    /// Init functions
    int InitCameraDefault();        // Used in the case that only one device connected
    int InitCameraSerialNumber(char *serial_num);
    int GetMyCameraInfo();
    int CheckBasicProperties();
    int SetWorkingProperties();
    void PreForAcquisition();       // Allocate memory for buff pointers

    /// Close functions    
    void UnPreForAcquisition();     // DeAllocate memory for buff ponters
    int UnInitCameraDefault();

    /// Operation functions
    int PixelFormatConvert(); 



    public:
    GX_STATUS camStatus = GX_STATUS_SUCCESS;

    GxCamera(){}
    ~GxCamera(){}

    /*************************************************
        Function: 
        Description: 
        Input: 
        Output: 
        Others: 
    *************************************************/

    /*************************************************
        Function: CameraInitOps
        Description: Operations to init camera. A pack of init functions.
        Input: None
        Output: status
        Others: This function is specific for the case that only one camera connected.
    *************************************************/
    int CameraInitOps();
    int CameraInitOpsSerialNumber(char *serial_num);

    /*************************************************
        Function: CameraCloseOps
        Description: Operations to close camera. A pack of close functions.
        Input: None
        Output: status
        Others: None
    *************************************************/
    int CameraCloseOps();
      
    /*************************************************
        Function: StreamOn
        Description: Start the acquisition stream
        Input: None
        Output: status
        Others: None
    *************************************************/
    int StreamOn();

    /*************************************************
        Function: StreamOff
        Description: Close the acquisition stream
        Input: None
        Output: status
        Others: None
    *************************************************/
    int StreamOff();
    
    /*************************************************
        Function: GetColorImg
        Description: To get a color frame
        Input: cv::Mat to store frame data
        Output: None
        Others: None
    *************************************************/          
    void GetColorImg(cv::Mat &pframe);      // Get color image

    /*************************************************
        Function: SetExposureTime
        Description: Set exposure time
        Input: exTime - exposure time
        Output: status
        Others: None
    *************************************************/
    int SetExposureTime(int exTime); 

    /*************************************************
        Function: SetFrameRate
        Description: Set frame rate
        Input: frRate - frame rate
        Output: status
        Others: None
    *************************************************/
    int SetFrameRate(int frRate); 
    
    // set gain

    // set white balance

    // set resolution

    // ...
};


#endif
