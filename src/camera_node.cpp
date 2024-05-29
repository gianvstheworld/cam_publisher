#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "camera_functions.h"

// ROS stuff
#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Header.h"

void get_calib_params(const std::string& yaml_file, sensor_msgs::CameraInfoPtr cam_info) {
    /*
    @brief: Get calibration parameters for the camera
    @param: Camera object, CameraInfo message
    */
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);

        if (config["camera_matrix"] && config["camera_matrix"]["data"]) {
            std::vector<double> K = config["camera_matrix"]["data"].as<std::vector<double>>();
            if (K.size() == 9) {
                for (int i = 0; i < 9; i++) {
                    cam_info->K[i] = K[i];
                }
            } else {
                ROS_ERROR("Invalid camera_matrix size");
            }
        } else {
            ROS_ERROR("Failed to read camera_matrix from YAML");
        }

        if (config["distortion_model"]) {
            cam_info->distortion_model = config["distortion_model"].as<std::string>();
        } else {
            ROS_ERROR("Failed to read distortion_model from YAML");
        }

        if (config["distortion_coefficients"] && config["distortion_coefficients"]["data"]) {
            std::vector<double> D = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            cam_info->D.resize(D.size());
            if (D.size() > 0) {
                for (size_t i = 0; i < D.size(); i++) {
                    cam_info->D[i] = D[i];
                }
            } else {
                ROS_ERROR("Invalid distortion_coefficients size");
            }
        } else {
            ROS_ERROR("Failed to read distortion_coefficients from YAML");
        }

        if (config["rectification_matrix"] && config["rectification_matrix"]["data"]) {
            std::vector<double> R = config["rectification_matrix"]["data"].as<std::vector<double>>();
            if (R.size() == 9) {
                for (int i = 0; i < 9; i++) {
                    cam_info->R[i] = R[i];
                }
            } else {
                ROS_ERROR("Invalid rectification_matrix size");
            }
        } else {
            ROS_ERROR("Failed to read rectification_matrix from YAML");
        }

        if (config["projection_matrix"] && config["projection_matrix"]["data"]) {
            std::vector<double> P = config["projection_matrix"]["data"].as<std::vector<double>>();
            if (P.size() == 12) {
                for (int i = 0; i < 12; i++) {
                    cam_info->P[i] = P[i];
                }
            } else {
                ROS_ERROR("Invalid projection_matrix size");
            }
        } else {
            ROS_ERROR("Failed to read projection_matrix from YAML");
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML Exception: %s", e.what());
    }
}

void image_publisher(cv::Mat cam_image, image_transport::CameraPublisher cam_pub, std::string topic_name, std_msgs::Header image_header, sensor_msgs::ImagePtr cam_msg, sensor_msgs::CameraInfoPtr cam_info) {
    /*
    @brief: Publish image to ROS topic
    @param: Camera image, Camera publisher, Topic name, Image header, Image message, CameraInfo message
    */
    if (!cam_image.empty()) {
        std::string frame_id = topic_name + "_frame";
        image_header.frame_id = frame_id;

        cam_msg = cv_bridge::CvImage(image_header, "bgr8", cam_image).toImageMsg();

        cam_info->height = cam_image.rows;
        cam_info->width = cam_image.cols;
        cam_info->header = image_header;

        // Publish via image_transport
        cam_pub.publish(cam_msg, cam_info);
    } else {
        ROS_WARN("Captured empty frame, skipping...");
    }
}

int main(int argc, char **argv) {
    // Start ROS stuff
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    std::string yaml_path;
    n.getParam("camera_info_yaml", yaml_path);

    int deviceNode;
    n.getParam("device", deviceNode);
    cv::Size frameSize(1920, 1080); ///< default frame size 1920x1080
    int fps = 30; ///< default camera fps: 30

    // Get the topic name as a parameter
    std::string topic_name;
    n.param<std::string>("topic_name", topic_name, "camera");

    if (deviceNode < 0) {
        ROS_ERROR("Invalid camera device node: %d", deviceNode);
        return 1;
    }

    Camera cam(nh, deviceNode, cv::CAP_V4L2);
    
    if (!cam.isOpened()) {
        ROS_ERROR("Failed to open the camera");
        return 1;
    }

    cam.setRawFrameSize(frameSize);
    cam.setRawFrameRate(fps);

    cam.startCapture();
    usleep(500000);

    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher cam_pub = it.advertiseCamera(topic_name + "/image_raw", 1);

    std_msgs::Header image_header;
    sensor_msgs::ImagePtr cam_msg;
    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());

    // Get calibration parameters for the camera
    get_calib_params(yaml_path, cam_info);

    ros::Rate loop_rate(fps);
    while (ros::ok() && cam.isOpened()) {
        cv::Mat cam_image;
        cam.cap >> cam_image;

        if (cam_image.empty()) {
            ROS_WARN("Captured empty frame");
            continue;
        }

        image_header.stamp = ros::Time::now();
        image_publisher(cam_image, cam_pub, topic_name, image_header, cam_msg, cam_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cam.stopCapture();

    return 0;
}
