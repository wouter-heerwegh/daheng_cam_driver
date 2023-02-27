#include "cameras.hpp"


/**Initialize the library for the cameras.
        @return The error code
        */
int Stereocamera::initCameras()
{

    status = GXInitLib();

    if (status != GX_STATUS_SUCCESS)
    {
        std::cout << "init library failed" << std::endl;
    }

    return status;
}


/**Search for available cameras and give them and ID.
        @return The error code
        */
int Stereocamera::updateDeviceList()
{

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status != GX_STATUS_SUCCESS || (nDeviceNum <= 0))
    {
        std::cout << "updating device list failed, are the cameras plugged in correctly or is another program using the cameras?" << std::endl;
        
    }
    return status;
}


/**Open the camera.
        @param camera_ID The ID of the camera you want to open
        @return The error code
        */
int Stereocamera::openCamera(int camera_ID)
{
    status = GXOpenDeviceByIndex(camera_ID, &hDevice);
    if (status != GX_STATUS_SUCCESS)
    {
        std::cout << "opening cameras failed, are the cameras plugged in correctly or is another program using the cameras?" << std::endl;
        
    }
    return status;
}


/**Initialize the settings on the camera
        @param gain Gain applied on the image after capture
        @param exposure Time the shutter is open in microseconds
        @param red_ratio Gain on the red channel (for whitebalance)
        @param green_ratio Gain on the green channel (for whitebalance)
        @param blue_ratio Gain on the blue channel (for whitebalance)
        @param triggermode Trigger manually (GX_TRIGGER_MODE_ON) or trigger automatically at set fps (GX_TRIGGER_MODE_OFF)
        
        */
int Stereocamera::initCameraSettings(float gain, float exposure, float red_ratio, float green_ratio, float blue_ratio, int64_t triggermode)
{
    GXSetFloat(hDevice, GX_FLOAT_GAIN, gain);
    GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, red_ratio);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, green_ratio);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, blue_ratio);
    GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, triggermode);
    GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
}

/**Set the callback function to handle the frames
        @param callback pointer to the callback function
        @return The error code
        */
int Stereocamera::setCallback(GXCaptureCallBack callback)
{
    status = GXRegisterCaptureCallback(hDevice, NULL, callback);
    return status;
}

/**Start capturing frames
        @return The error code
        */
int Stereocamera::startCamera()
{
    status = GXStreamOn(hDevice);
    if(status != GX_STATUS_SUCCESS)
    {
        std::cout << "starting cameras failed" << std::endl;
    }
    return status;
}

/**trigger the camera
        @return The error code
        */
int Stereocamera::trigger()
{
    status = GXSendCommand(hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
    return status;
}

/**change the exposure
        @param exposure the exposure in microseconds
        @return The error code
        */
int Stereocamera::setExposure(float exposure)
{
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
    return status;
}
