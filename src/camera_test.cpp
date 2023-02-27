#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <opencv2/opencv.hpp> // C++
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"

sensor_msgs::CameraInfo cam_info_left;
sensor_msgs::CameraInfo cam_info_right;


bool setCameraInfoLeftCb(sensor_msgs::SetCameraInfo::Request & req, sensor_msgs::SetCameraInfo::Response & resp){
	cam_info_left = req.camera_info;
	resp.status_message = "Success";
	resp.success = true;
	return resp.success;
}

bool setCameraInfoRightCb(sensor_msgs::SetCameraInfo::Request & req, sensor_msgs::SetCameraInfo::Response & resp){
	cam_info_right = req.camera_info;
	resp.status_message = "Success";
	resp.success = true;
	return resp.success;
}

sensor_msgs::CameraInfo get_cam_info(std::string name){
    sensor_msgs::CameraInfo cam_info;
	try
	{
		ROS_INFO_STREAM("Opening:" << ("caminfo" + name + ".xml"));
		cv::FileStorage storage;
		/// TODO: move this file into package and use relative path 
		std::string filename = "/home/wouterheerwegh/ros1_ws/caminfo" + name + ".xml";
		if(storage.open(filename, cv::FileStorage::READ)){
			ROS_INFO("GOT FILE");
		}else{
			ROS_WARN("Could not open cam info file");
		}

		int height, width;
		storage["height"] >> height;
		storage["width"] >> width;

		cv::Mat D, K, R, P;
		storage["distortion"] >> D;
		storage["rectification"] >> R;
		storage["cameraMatrix"] >> K;
		storage["projection"] >> P;

		cam_info.distortion_model = "plumb_bob";
		cam_info.height = height;
		cam_info.width = width;
		cam_info.D = D;
		for (int i = 0; i < 9; i++) cam_info.K[i] = K.at<double>(i);
		for (int i = 0; i < 9; i++) cam_info.R[i] = R.at<double>(i);
		for (int i = 0; i < 12; i++) cam_info.P[i] = P.at<double>(i);
	}
	catch (cv::Exception &e)
	{
		ROS_INFO_STREAM("No cam info found, not publishing" << e.what());
	}
    return cam_info;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo");
    ros::NodeHandle handle("~");
    image_transport::ImageTransport it(handle);
    image_transport::Publisher left_pub = it.advertise("left/image_raw", 5);
    image_transport::Publisher right_pub = it.advertise("right/image_raw", 5);
    ros::ServiceServer set_caminfo_service_left = handle.advertiseService("/stereo/left/set_camera_info", setCameraInfoLeftCb);
    ros::ServiceServer set_caminfo_service_right = handle.advertiseService("/stereo/right/set_camera_info", setCameraInfoRightCb);

    cam_info_left = get_cam_info("FL");
    cam_info_right = get_cam_info("FR");


    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = NULL;
    uint32_t nDeviceNum = 0;
    // Initializes the library.
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        return 0;
    }
    // Updates the enumeration list for the devices.
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return 0;
    }
    // Opens the device.
    status = GXOpenDeviceByIndex(1, &hDevice);
    status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, 16);
    status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, 1.4);
    status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, 1.5);

    ros::Publisher camera_info_pub_left = handle.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 10, true);
    ros::Publisher camera_info_pub_right = handle.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 10, true);

    if (status == GX_STATUS_SUCCESS)
    {
        // Define the incoming parameters of GXDQBuf.
        PGX_FRAME_BUFFER pFrameBuffer;
        // Stream On.
        status = GXStreamOn(hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            sensor_msgs::ImagePtr image_msg;
            // Calls GXDQBuf to get a frame of image.
            while (1)
            {
                status = GXDQBuf(hDevice, &pFrameBuffer, 20);
                if (status == GX_STATUS_SUCCESS)
                {
                    if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                    {
                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                        cv::Mat yeet;
                        yeet.create(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC1);
                        VxInt32 DxStatus = DxBrightness(pFrameBuffer->pImgBuf, pFrameBuffer->pImgBuf, pFrameBuffer->nWidth * pFrameBuffer->nHeight, 0);
                        memcpy(yeet.data, pFrameBuffer->pImgBuf, pFrameBuffer->nWidth * pFrameBuffer->nHeight);
                        cv::cvtColor(yeet, yeet, cv::COLOR_BayerRG2GRAY);
                        cv::resize(yeet, yeet, cv::Size(1280, 512));
                        cv::flip(yeet, yeet, 0);
                        
                        cv::Mat left, right;
                        int width = yeet.cols;
                        left = yeet(cv::Range(0, yeet.rows - 1), cv::Range(0, (width/2) - 1));
                        right = yeet(cv::Range(0, yeet.rows - 1), cv::Range(width/2, width - 1));

                        cv::imshow("yeet", yeet);
                        cv::imshow("left", left);
                        cv::imshow("right", right);
                        sensor_msgs::Image msg_left, msg_right;
                        std_msgs::Header header;
                        header.stamp = ros::Time::now();
                        header.frame_id = "camera_link";
                        image_msg = cv_bridge::CvImage(header, "mono8", left).toImageMsg();
                        left_pub.publish(image_msg);
                        image_msg = cv_bridge::CvImage(header, "mono8", right).toImageMsg();
                        right_pub.publish(image_msg);
                        cam_info_left.header.stamp = header.stamp;
                        cam_info_left.header.seq = header.seq;
                        camera_info_pub_left.publish(cam_info_left);
                        cam_info_right.header.stamp = header.stamp;
                        cam_info_right.header.seq = header.seq;
                        camera_info_pub_right.publish(cam_info_right);
                        cv::waitKey(1);

                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
                    }
                    // Calls GXQBuf to put the image buffer back into the library
                    // and continue acquiring.
                    status = GXQBuf(hDevice, pFrameBuffer);
                }
            }
        }
        // Sends a stop acquisition command.
        status = GXStreamOff(hDevice);
    }
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();

    return 0;
}