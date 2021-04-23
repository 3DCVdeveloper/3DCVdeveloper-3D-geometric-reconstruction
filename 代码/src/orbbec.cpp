#include "orbbec.h"

Orbbec::Orbbec()
{
    openni::Status rc = openni::STATUS_OK;
    const char *deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();
    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = mDevice.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return;
    }
    rc = mDepthStream.create(mDevice, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = mDepthStream.start();
        if (rc != openni::STATUS_OK)
        {
            printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            mDepthStream.destroy();
        }
    }
    else
    {
        printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!mDepthStream.isValid())
    {
        printf("No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return;
    }

    mCapture.open(2); // 电脑没有摄像头填0
    if (mCapture.isOpened())
    {
        mCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        mCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    }
    else
    {
        printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
        return;
    }

    // 3D相机D2C对齐
    rc = mDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    OBCameraParams cameraParam;
    int param_size = sizeof(OBCameraParams);
    rc = mDevice.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&cameraParam, &param_size);

    
}

void Orbbec::getFrames(cv::Mat &rgb, cv::Mat &depth)
{
    // 循环读取数据流信息并保存在VideoFrameRef中
    openni::VideoFrameRef frameDepth;
    // 读取彩色图
    mDepthStream.readFrame(&frameDepth);
    mCapture >> rgb;

    // 将深度数据转换成OpenCV格式
    depth = cv::Mat(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void *)frameDepth.getData());
    cv::flip(depth, depth, 1);
}