#include "camera.hpp"

#include <stdexcept>
#include <unordered_map>

Camera::Camera()
{
    // 打开相机，构造失败时释放资源并抛出异常，后续由main统一捕获并打印错误。
    int ret;
    MV_CC_DEVICE_INFO_LIST device_list{};
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        throw std::runtime_error("MV_CC_EnumDevices failed");
    }

    if (device_list.nDeviceNum == 0) {
        throw std::runtime_error("No USB camera found");
    }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        throw std::runtime_error("MV_CC_CreateHandle failed");
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        MV_CC_DestroyHandle(handle_);
        throw std::runtime_error("MV_CC_OpenDevice failed");
    }

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    ret = MV_CC_SetFloatValue(handle_, "ExposureTime", 5000);
    if (ret != MV_OK) {
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        throw std::runtime_error("MV_CC_SetFloatValue ExposureTime failed");
    }

    ret = MV_CC_SetFloatValue(handle_, "Gain", 16.9); //0~16.9
    if (ret != MV_OK) {
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        throw std::runtime_error("MV_CC_SetFloatValue Gain failed");
    }

    MV_CC_SetFrameRate(handle_, 60);

    // 启动图像采集，之后可以反复调用read。
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        throw std::runtime_error("MV_CC_StartGrabbing failed");
    }
}

Camera::~Camera()
{
    // 关闭相机，释放资源。
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
}

// 读取一帧图像；通过img返回结果，用bool表示本次读取是否成功，方便处理读取失败。
bool Camera::read(cv::Mat & img)
{
    // 读取前先清空旧图片
    img.release();
    int ret;
    MV_FRAME_OUT raw;
    unsigned int nMsec = 100;

    ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
    if (ret != MV_OK) {
        return false;
    }

    try {
        img = transfer(raw);
    } catch (...) {
        // 图像转换异常时先归还相机缓存，再将异常继续抛给main统一处理。
        MV_CC_FreeImageBuffer(handle_, &raw);
        throw;
    }

    ret = MV_CC_FreeImageBuffer(handle_, &raw);
    if (ret != MV_OK) {
        img.release();
        return false;
    }

    return true;
}

cv::Mat Camera::transfer(MV_FRAME_OUT & raw)
{
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;

    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));

    return img;
}
