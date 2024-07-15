#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "../include/my_hk_ros2/hk_camera.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto hk_camera = std::make_shared<rclcpp::Node>("hk_camera_node");
    RCLCPP_INFO(hk_camera->get_logger(), " ----------------Starting hk_camera_node-----------------\n");
    camera::Camera MVS_cap(*hk_camera);//传递节点过去，然后它的构造函数里面获取相机配置文件的参数

    //创建图像发布器
    image_transport::ImageTransport it(hk_camera);
    auto image_pub = it.advertiseCamera("/hk_camera/rgb", 1);
    //设置循环频率
    rclcpp::Rate loop_rate(60);//与曝光时间有关：如曝光时间设置为20ms，则最大发布频率为50。（最大发布频率为60）

    sensor_msgs::msg::Image ros_img_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; 

    int image_empty_count = 0;//空图帧数
    int nRet;

    while (rclcpp::ok())
    {
        loop_rate.sleep();
        rclcpp::spin_some(hk_camera);
        // 从相机中获取一帧图像，存放到pData里，图像信息放在stImageInfo，超时时间20毫秒
        nRet = MV_CC_GetOneFrameTimeout(MVS_cap.get_handle(), MVS_cap.get_pData(), MVS_cap.get_nDataSize(), MVS_cap.get_stImageInfo(), 20);
        if (nRet != MV_OK){
            if (++image_empty_count > 100){
                printf( "The Number of Faild Reading Exceed The Set Value: %d", image_empty_count);
                exit(-1);
            }
            continue;
        }
        image_empty_count = 0; 
        cv_ptr->image = cv::Mat(MVS_cap.get_stImageInfo()->nHeight, MVS_cap.get_stImageInfo()->nWidth, CV_8UC3, MVS_cap.get_pData()); 
        ros_img_msg = *(cv_ptr->toImageMsg());
        camera_info_msg.header.frame_id = "hk_camera";
        camera_info_msg.header.stamp = hk_camera->get_clock()->now(); // ros发出的时间不是快门时间
        image_pub.publish(ros_img_msg,camera_info_msg);//发出图像和信息的话题
        RCLCPP_INFO(hk_camera->get_logger(), "/hk_camera/rgb publishing...");
    }
    rclcpp::shutdown();
    return 0;
}
