#ifndef HK_CAMERA_HPP
#define HK_CAMERA_HPP
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera
{
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
    cv::Mat frame;
    bool frame_empty = 0;
    pthread_mutex_t mutex;
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线
    };

    class Camera
    {
    public:
        Camera(rclcpp::Node &node);
        ~Camera();
        //********** 原始信息转换线程 **********************/
        static void *HKWorkThread(void *p_handle);

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        bool set(camera::CamerProperties type, float value);
        bool reset();
        void ReadImg(cv::Mat &image);

    private:
        void *handle;
        pthread_t nThreadID;
        //yaml config 
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;
        int BurstFrameCount;
        int ExposureTime;
        bool GammaEnable;
        float Gamma;
        int GainAuto;
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;
    };

    Camera::Camera(rclcpp::Node &node)
    {
        handle = NULL;

        //相机初始化的时候，从配置文件yaml中读取参数

        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        node.get_parameter_or("width", width, 1920);
        node.get_parameter_or("height", height, 1200);
        node.get_parameter_or("FrameRateEnable", FrameRateEnable, true);
        node.get_parameter_or("FrameRate", FrameRate, 30);
        node.get_parameter_or("BurstFrameCount", BurstFrameCount, 10); // 一次触发采集的次数
        node.get_parameter_or("ExposureTime", ExposureTime, 50000);
        node.get_parameter_or("GammaEnable", GammaEnable, false);
        node.get_parameter_or("Gamma", Gamma, (float)0.7);
        node.get_parameter_or("GainAuto", GainAuto, 2);
        node.get_parameter_or("SaturationEnable", SaturationEnable,true);
        node.get_parameter_or("Saturation", Saturation, 128);
        node.get_parameter_or("Offset_x", Offset_x, 0);
        node.get_parameter_or("Offset_y", Offset_y, 0);
        node.get_parameter_or("TriggerMode", TriggerMode, 1);
        node.get_parameter_or("TriggerSource", TriggerSource, 2);
        node.get_parameter_or("LineSelector", LineSelector, 2);

        // 枚举设备 
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }
        //选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        // 打开设备
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        //设置 yaml 文件里面的配置
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAP_PROP_FRAMERATE, FrameRate);
        // this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount);
        this->set(CAP_PROP_HEIGHT, height);
        this->set(CAP_PROP_WIDTH, width);
        this->set(CAP_PROP_OFFSETX, Offset_x);
        this->set(CAP_PROP_OFFSETY, Offset_y);
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
        this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
        if (GammaEnable)
            this->set(CAP_PROP_GAMMA, Gamma);
        this->set(CAP_PROP_GAINAUTO, GainAuto);
        // this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);
        // this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);
        // this->set(CAP_PROP_LINE_SELECTOR, LineSelector);
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n",0.0 );
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }
        this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
        if (SaturationEnable)
            this->set(CAP_PROP_SATURATION, Saturation);
        //软件触发
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK == nRet)
        {
            printf("set TriggerMode OK!\n");
        }
        else
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        }

        //********** 图像格式 **********/
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = RGB\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
        }
        MVCC_ENUMVALUE t = {0};
        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);
        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }
        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        //初始化互斥量
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        nRet = pthread_create(&nThreadID, NULL, HKWorkThread, handle);//创建线程。
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            exit(-1);
        }
    }

    Camera::~Camera()
    {
        int nRet;
        pthread_join(nThreadID, NULL);
        nRet = MV_CC_StopGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    //根据参数类型设置相机参数
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {
        case CAP_PROP_FRAMERATE_ENABLE:
        {
            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);
            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);
            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);
            if (MV_OK == nRet)
            {
                printf("set AcquisitionBurstFrameCount OK!\n");
            }
            else
            {
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            nRet = MV_CC_SetIntValue(handle, "Height", value); 
            if (MV_OK == nRet)
            {
                printf("set Height OK!\n");
            }
            else
            {
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_WIDTH:
        {
            nRet = MV_CC_SetIntValue(handle, "Width", value); 
            if (MV_OK == nRet)
            {
                printf("set Width OK!\n");
            }
            else
            {
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            nRet = MV_CC_SetIntValue(handle, "OffsetX", value); 

            if (MV_OK == nRet)
            {
                printf("set Offset X OK!\n");
            }
            else
            {
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            nRet = MV_CC_SetIntValue(handle, "OffsetY", value); 
            if (MV_OK == nRet)
            {
                printf("set Offset Y OK!\n");
            }
            else
            {
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间
            if (MV_OK == nRet)
            {
                printf("set ExposureTime OK! value=%f\n",value);
            }
            else
            {
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）
            if (MV_OK == nRet)
            {
                printf("set GammaEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA:
        {
            nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大
            if (MV_OK == nRet)
            {
                printf("set Gamma OK! value=%f\n",value);
            }
            else
            {
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAINAUTO:
        {
            nRet = MV_CC_SetEnumValue(handle, "GainAuto", value); //亮度 越大越亮
            if (MV_OK == nRet)
            {
                printf("set GainAuto OK! value=%f\n",value);
            }
            else
            {
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)
            if (MV_OK == nRet)
            {
                printf("set SaturationEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION:
        {
            nRet = MV_CC_SetIntValue(handle, "Saturation", value); 
            if (MV_OK == nRet)
            {
                printf("set Saturation OK! value=%f\n",value);
            }
            else
            {
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_MODE:
        {
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value); 
            if (MV_OK == nRet)
            {
                printf("set TriggerMode OK!\n");
            }
            else
            {
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value);
            if (MV_OK == nRet)
            {
                printf("set TriggerSource OK!\n");
            }
            else
            {
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {
            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value); 
            if (MV_OK == nRet)
            {
                printf("set LineSelector OK!\n");
            }
            else
            {
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        default:
            return 0;
        }
        return nRet;
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
    }

    //^ ********************************** PrintDeviceInfo ************************************ //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ ********************************** Camera contsructor************************************ //
    void Camera::ReadImg(cv::Mat &image)
    {

        pthread_mutex_lock(&mutex);
        if (frame_empty)
        {
            image = cv::Mat();
        }
        else
        {
            image = camera::frame.clone();//这里是用clone的方法，深拷贝。不太好！
            frame_empty = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    //^ ********************************** HKWorkThread1 ************************************ //
    void *Camera::HKWorkThread(void *p_handle)
    {
        double start;
        int nRet;
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0; //空图帧数
        while (rclcpp::ok())
        {
            start = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    // RCLCPP_INFO("The Number of Faild Reading Exceed The Set Value!\n");s
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数
            //转换图像格式为BGR8

            stConvertParam.nWidth = 1920;                               //ch:图像宽 | en:image width
            stConvertParam.nHeight = 1200;                              //ch:图像高 | en:image height
            stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format                      //! 输出格式 RGB
            stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format                       //! 输入格式 RGB
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);
            //创一个线程，读取图像数据frame。（互斥锁）
            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone(); //tmp.clone();
            frame_empty = 0;
            pthread_mutex_unlock(&mutex);

            double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            //*************************************testing img********************************//
            std::cout << "HK_camera,Time: " << time  << "s, " << "\tFPS: " << 1 / time << "\twidth: " << frame.cols << "\theight: " << frame.rows << std::endl;
            // imshow("HK vision",frame);
            // cv::waitKey(1);
        }
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);
        return 0;
    }

} // namespace camera
#endif
