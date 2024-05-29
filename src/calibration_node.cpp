#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Header.h"

void image_publisher(cv::Mat cam_image, image_transport::Publisher cam_pub, std::string topic_name, std_msgs::Header image_header){
    /*
    @brief: Publish image to ROS topic
    @param: Camera image, Camera publisher, Topic name, Image header
    */
    if (!cam_image.empty()){
        std::string frame_id = topic_name + "_frame";
        image_header.frame_id = frame_id;
        sensor_msgs::ImagePtr cam_msg = cv_bridge::CvImage(image_header, "bgr8", cam_image).toImageMsg();
        cam_pub.publish(cam_msg);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    int deviceNode;
    n.getParam("device", deviceNode);
    cv::Size frameSize(1920, 1080); // Default frame size
    int fps = 30; // Default camera fps

    // Get the topic name as a parameter
    std::string topic_name;
    n.param<std::string>("topic_name", topic_name, "camera");

    ROS_INFO("Opening camera device: %d", deviceNode);
    cv::VideoCapture cam(deviceNode, cv::CAP_V4L2);

    if(!cam.isOpened())
        exit(EXIT_FAILURE);

    cam.set(cv::CAP_PROP_FRAME_WIDTH, frameSize.width);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, frameSize.height);
    cam.set(cv::CAP_PROP_FPS, fps);

    usleep(500000); // Wait for camera to initialize

    image_transport::ImageTransport it(nh);
    image_transport::Publisher cam_pub = it.advertise(topic_name + "/image_raw", 1);

    std_msgs::Header image_header;

    ros::Rate loop_rate(fps);
    while(ros::ok() && cam.isOpened()){
        cv::Mat cam_image;
        cam >> cam_image;

        image_header.stamp = ros::Time::now();
        image_publisher(cam_image, cam_pub, topic_name, image_header);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cam.release(); // Stop camera capturing

    return 0;
}
