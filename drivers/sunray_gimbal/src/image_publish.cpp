#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <string>
#include <std_msgs/String.h>
#include <sunray_gimbal/GimbalParams.h>

std::string current_codec = "h264";  // 默认h265
bool codec_changed = false;

void gimbalParamCallback(const sunray_gimbal::GimbalParams::ConstPtr& msg) {
    std::string new_codec;
    if (msg->encoding_type == 2)
        new_codec = "h265";
    else if (msg->encoding_type == 1)
        new_codec = "h264";
    else
        new_codec = "h264";
    if (new_codec != current_codec) {
        ROS_WARN("检测到编码类型切换: %s", new_codec.c_str());
        current_codec = new_codec;
        codec_changed = true;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_pub;

    image_transport::ImageTransport it(nh_pub);  // 用全局句柄
    image_transport::Publisher img_pub = it.advertise("/sunray/gimbal_cam/image_raw", 1);
    ros::Subscriber gimbal_param_sub = nh_pub.subscribe("/sunray/gimbal_param", 1, gimbalParamCallback);

    std::string rtsp_url;
    nh.param<std::string>("rtsp_url", rtsp_url, "rtsp://192.168.2.25:8554/main.264");

    std::string depay, decoder;
    if (current_codec == "h265" || current_codec == "H265" || current_codec == "hevc") {
        depay = "rtph265depay";
        decoder = "avdec_h265";
        ROS_INFO("使用H265解码pipeline");
    } else {
        depay = "rtph264depay";
        decoder = "avdec_h264";
        ROS_INFO("使用H264解码pipeline");
    }
    std::string pipeline = "rtspsrc location=" + rtsp_url + " latency=50 ! " + depay + " ! " + decoder + " ! videoconvert ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        ROS_ERROR("RTSP流无法打开: %s", rtsp_url.c_str());
        return -1;
    }

    cv::Mat frame;
    cv_bridge::CvImage cv_img;
    ros::Rate loop_rate(30);

    while (ros::ok()) {

        if (codec_changed) {

            cap.release();
            if (current_codec == "h265" || current_codec == "H265" || current_codec == "hevc") {
                depay = "rtph265depay";
                decoder = "avdec_h265";
                ROS_WARN("切换到H265解码pipeline");
            } else {
                depay = "rtph264depay";
                decoder = "avdec_h264";
                ROS_WARN("切换到H264解码pipeline");
            }
            pipeline = "rtspsrc location=" + rtsp_url + " latency=50 ! " + depay + " ! " + decoder + " ! videoconvert ! appsink";
            cap.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap.isOpened()) {
                ROS_ERROR("切换后RTSP流无法打开: %s", rtsp_url.c_str());
                return -1;
            }
            codec_changed = false;
        }
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("未能读取到图像帧");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        cv_img.header.stamp = ros::Time::now();
        cv_img.header.frame_id = "usb_cam";
        cv_img.encoding = "bgr8";
        cv_img.image = frame;
        sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
        img_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    cap.release();
    return 0;
}
