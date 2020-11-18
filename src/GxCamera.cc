#include "GxCamera/GxCamera.hpp"

using namespace cv;

namespace {
    bool is_lib_closed = false;
}

/// For users to init
int GxCamera::CameraInitOps()
{
     /// Init
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = this->InitCameraDefault();
    GX_CHECK(emStatus);

    /// GetInfo
    emStatus = this->GetMyCameraInfo();
    GX_CHECK(emStatus);

    /// CheckCameraType
    emStatus = this->CheckBasicProperties();
    GX_CHECK(emStatus);

    /// SetWorkingProperties
    emStatus = this->SetWorkingProperties();
    GX_CHECK(emStatus);

     /// PreForAquisition
    this->PreForAcquisition();

    return emStatus;
}

int GxCamera::CameraInitOpsSerialNumber(char *serial_num)
{
     /// Init
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = this->InitCameraSerialNumber(serial_num);
    GX_CHECK(emStatus);

    /// GetInfo
    emStatus = this->GetMyCameraInfo();
    GX_CHECK(emStatus);

    /// CheckCameraType
    emStatus = this->CheckBasicProperties();
    GX_CHECK(emStatus);

    /// SetWorkingProperties
    emStatus = this->SetWorkingProperties();
    GX_CHECK(emStatus);

     /// PreForAquisition
    this->PreForAcquisition();

    return emStatus;
}

/// For users to close
int GxCamera::CameraCloseOps()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
     /// UnPreForAquisition
    this->UnPreForAcquisition();

    /// ~Gx_camera(); // UnInit
    this->camStatus = this->UnInitCameraDefault();
    GX_CHECK(emStatus);

    return emStatus;
}

void GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    
    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }
    
    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
    
    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}


// Functions in class
int GxCamera::InitCameraDefault()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXInitLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    emStatus = GXUpdateDeviceList(&(this->nDeviceNum),1000);
    if(emStatus != GX_STATUS_SUCCESS)
    { 
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }
    if(nDeviceNum <= 0)
    {
        printf("<No device found>\n");
        emStatus = GX_STATUS_NOT_FOUND_DEVICE;
        GXCloseLib();
        return emStatus;
    }

    emStatus = GXOpenDeviceByIndex(1,&this->hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;           
    }

    return emStatus;
}

int GxCamera::InitCameraSerialNumber(char *serial_num)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXInitLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    emStatus = GXUpdateDeviceList(&(this->nDeviceNum),1000);
    if(emStatus != GX_STATUS_SUCCESS)
    { 
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }
    if(nDeviceNum <= 0)
    {
        printf("<No device found>\n");
        emStatus = GX_STATUS_NOT_FOUND_DEVICE;
        GXCloseLib();
        return emStatus;
    }

    // emStatus = GXOpenDeviceByIndex(1,&this->hDevice);
    GX_OPEN_PARAM open_param;
    open_param.accessMode = GX_ACCESS_CONTROL;
    open_param.openMode = GX_OPEN_SN;
    open_param.pszContent = serial_num;
    emStatus = GXOpenDevice(&open_param,&this->hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;           
    }

    return emStatus;
}

int GxCamera::UnInitCameraDefault()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    emStatus = GXCloseDevice(this->hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        this->hDevice = NULL;
        GXCloseLib();
        return emStatus;
    }

    if (is_lib_closed == false){
        emStatus = GXCloseLib();
        is_lib_closed = true;
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            return emStatus;
        }
    }

    return emStatus;
}

int GxCamera::GetMyCameraInfo()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Get Device Info
    printf("***********************************************\n");
    printf("Get device info:\n");
    //Get libary version
    printf("<Libary Version : %s>\n", GXGetLibVersion());
    size_t nSize = 0;
    //Get string length of Vendor name
    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    GX_INIT_VERIFY_EXIT(emStatus);
    //Alloc memory for Vendor name
    char *pszVendorName = new char[nSize];
    //Get Vendor name
    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = NULL;
        GX_INIT_VERIFY_EXIT(emStatus);
    }

    printf("<Vendor Name : %s>\n", pszVendorName);
    //Release memory for Vendor name
    delete[] pszVendorName;
    pszVendorName = NULL;

    //Get string length of Model name
    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_INIT_VERIFY_EXIT(emStatus);
    //Alloc memory for Model name
    char *pszModelName = new char[nSize];
    //Get Model name
    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = NULL;
        GX_INIT_VERIFY_EXIT(emStatus);
    }

    printf("<Model Name : %s>\n", pszModelName);
    //Release memory for Model name
    delete[] pszModelName;
    pszModelName = NULL;

    //Get string length of Serial number
    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_INIT_VERIFY_EXIT(emStatus);
    //Alloc memory for Serial number
    char *pszSerialNumber = new char[nSize];
    //Get Serial Number
    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;
        GX_INIT_VERIFY_EXIT(emStatus);
    }

    printf("<Serial Number : %s>\n", pszSerialNumber);
    //Release memory for Serial number
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    //Get string length of Device version
    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_VERSION, &nSize);
    GX_INIT_VERIFY_EXIT(emStatus);
    char *pszDeviceVersion = new char[nSize];
    //Get Device Version
    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        GX_INIT_VERIFY_EXIT(emStatus);
    }

    printf("<Device Version : %s>\n", pszDeviceVersion);
    //Release memory for Device version
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
    printf("***********************************************\n");
    return emStatus;
}


int GxCamera::CheckBasicProperties()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Get the type of Bayer conversion. whether is a color camera.
    emStatus = GXIsImplemented(this->hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &this->bColorFilter);
    GX_INIT_VERIFY_EXIT(emStatus);

    //This app only support color cameras
    if (!this->bColorFilter)
    {
        printf("<This app only support color cameras! App Exit!>\n");
        GXCloseDevice(this->hDevice);
        this->hDevice = NULL;
        GXCloseLib();
        return 0;
    }
    else
    {
        emStatus = GXGetEnum(this->hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &this->i64ColorFilter);
        GX_INIT_VERIFY_EXIT(emStatus);
    }
    
    emStatus = GXGetInt(this->hDevice, GX_INT_PAYLOAD_SIZE, &this->nPayloadSize);
    GX_INIT_VERIFY_EXIT(emStatus);

    return emStatus;
}

int GxCamera::SetWorkingProperties()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Set acquisition mode
    emStatus = GXSetEnum(this->hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_INIT_VERIFY_EXIT(emStatus);

    //Set trigger mode
    emStatus = GXSetEnum(this->hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_INIT_VERIFY_EXIT(emStatus);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(this->hDevice, nBufferNum);
    GX_INIT_VERIFY_EXIT(emStatus);

    bool bStreamTransferSize = false;
    emStatus = GXIsImplemented(this->hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_INIT_VERIFY_EXIT(emStatus);

    if(bStreamTransferSize)
    {
        //Set size of data transfer block
        emStatus = GXSetInt(this->hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_INIT_VERIFY_EXIT(emStatus);
    }

    bool bStreamTransferNumberUrb = false;
    emStatus = GXIsImplemented(this->hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_INIT_VERIFY_EXIT(emStatus);

    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        emStatus = GXSetInt(this->hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_INIT_VERIFY_EXIT(emStatus);
    }

    //Set Balance White Mode : Continuous
    emStatus = GXSetEnum(this->hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    GX_INIT_VERIFY_EXIT(emStatus);

    return emStatus;
}

void GxCamera::PreForAcquisition()
{
    this->pRaw8ImgBuff = new unsigned char[this->nPayloadSize];
    this->pRgb24ImgBuff = new unsigned char[this->nPayloadSize*3];
}
void GxCamera::UnPreForAcquisition()
{
     //Release resources
    if (pRaw8ImgBuff != NULL)
    {
        delete[] pRaw8ImgBuff;
        pRaw8ImgBuff = NULL;
    }
    if (pRgb24ImgBuff != NULL)
    {
        delete[] pRgb24ImgBuff;
        pRgb24ImgBuff = NULL;
    }
}


int GxCamera::StreamOn()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Device start acquisition
    emStatus = GXStreamOn(hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        this->UnPreForAcquisition();
        GX_RUNNING_VERIFY_EXIT(emStatus);
    }
    return emStatus;
}
int GxCamera::StreamOff()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Device stop acquisition
    emStatus = GXStreamOff(hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_RUNNING_VERIFY_EXIT(emStatus);
    }
    return emStatus;
}

int GxCamera::PixelFormatConvert()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    switch (pFrameBuffer->nPixelFormat)
    {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image
            // emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, this->pRgb24ImgBuff, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
            //                   RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            DX_PIXEL_COLOR_FILTER ctype = BAYERRG;
            emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, this->pRgb24ImgBuff, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, ctype, false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, this->pRaw8ImgBuff, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24((unsigned char*)this->pRaw8ImgBuff, this->pRgb24ImgBuff, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(this->i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return PIXFMT_CVT_FAIL;
        }
    }
    return PIXFMT_CVT_SUCCESS;
}

void GxCamera::GetColorImg(Mat &pframe)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    // GXDQ
    emStatus = GXDQBuf(this->hDevice,&this->pFrameBuffer,1000);

    // Convert to rgb buffer
    if (emStatus==GX_STATUS_SUCCESS)
    {
        if(pFrameBuffer->nStatus==GX_FRAME_STATUS_SUCCESS)
        {
            // Img Process
            // Convert to rgb buffer
            this->PixelFormatConvert();
            // Convert to Mat
            pframe = cv::Mat(Size(this->pFrameBuffer->nWidth,this->pFrameBuffer->nHeight),\
            CV_8UC3,pRgb24ImgBuff); 
        }
        emStatus = GXQBuf(this->hDevice,pFrameBuffer);
    }

}

int GxCamera::SetExposureTime(int exTime)
{
    //Set Expose Time
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // GX_FLOAT_RANGE shutterRange;
    // emStatus = GXGetFloatRange(this->hDevice,GX_FLOAT_EXPOSURE_TIME,&shutterRange);
    // GX_RUNNING_VERIFY_EXIT(emStatus);
    // if( !(exTime <= shutterRange.dMax) || !(exTime>=shutterRange.dMin) )
    // {
    //     printf("<Invalid exposure time. Exposure time should be in (%d,%d)>\n",
    //     shutterRange.dMin,shutterRange.dMax);
    //     emStatus = GX_STATUS_ERROR;
    //     GX_RUNNING_VERIFY_EXIT(emStatus);
    // }

    emStatus = GXSetFloat(this->hDevice,GX_FLOAT_EXPOSURE_TIME,exTime);
    GX_RUNNING_VERIFY(emStatus);

    return emStatus;
}

int GxCamera::SetFrameRate(int frRate)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    emStatus = GXSetEnum(this->hDevice,GX_ENUM_ACQUISITION_FRAME_RATE_MODE,GX_ACQUISITION_FRAME_RATE_MODE_ON);
    GX_RUNNING_VERIFY(emStatus);

    emStatus = GXSetFloat(this->hDevice,GX_FLOAT_ACQUISITION_FRAME_RATE,frRate);
    GX_RUNNING_VERIFY(emStatus);
    
    return emStatus;
}
