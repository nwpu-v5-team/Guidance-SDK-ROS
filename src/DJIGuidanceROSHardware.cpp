//
// Created by kehan on 19-6-18.
//

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "DJIGuidanceROSHardware.h"

using namespace vwpp;
using namespace cv;

DJIGuidanceROSHardware::DJIGuidanceROSHardware():
    key(0),
    show_images(false),
    verbosity(0),
    error_code(0),
    dji_lock(DJI_SDK::DJI_lock()),
    dji_event(DJI_SDK::DJI_event()),            //TODO
    greyscale_image_left(HEIGHT, WIDTH, CV_8UC1),
    greyscale_image_right(HEIGHT, WIDTH, CV_8UC1),
    depth8_image(HEIGHT, WIDTH, CV_8UC1),
    depth16_image(HEIGHT, WIDTH, CV_16UC1),
    disparity_image(HEIGHT, WIDTH, CV_16SC1),   //TODO
    front_vbus1(false),
    left_vbus2(false),
    back_vbus3(false),
    right_vbus4(false),
    down_vbus5(false),
    nh(ros::NodeHandle("~")),
    frame_id("guidance")
{

    this->node_name = ros::this_node::getName();
    this->body_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("body_velocity", 1);    // TODO with confidence
    this->global_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    this->obstacles_dis_pub = nh.advertise<sensor_msgs::LaserScan>("obstacle_distance", 1);
    this->body_imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1);
    this->ultrasonic_dis_pub = nh.advertise<sensor_msgs::LaserScan>("ultrasonic_data", 1);


    if (nh.hasParam("frame_id"))
    {
        nh.getParam("frame_id", this->frame_id);
        ROS_INFO("Use frame_id: %s", this->frame_id.c_str());
    }
    else
    {
        ROS_INFO("Use default frame_id: %s", this->frame_id.c_str());
    }

    // If not true, didn't creat the topic and send data.
    // Reduce the pressure of communication.
    if (nh.hasParam("front_vbus1"))
    {
        nh.getParam("front_vbus1", front_vbus1);
        if (front_vbus1)
        {
            this->front_greyscale_image_left_pub = nh.advertise<sensor_msgs::Image>("front_vbus1/left_image", 1);
            this->front_greyscale_image_right_pub = nh.advertise<sensor_msgs::Image>("front_vbus1/right_image", 1);
            this->front_depth_image_pub = nh.advertise<sensor_msgs::Image>("front_vbus1/depth_image", 1);
            this->front_disparity_image_pub = nh.advertise<sensor_msgs::Image>("front_vbus1/disparity_image", 1);
            ROS_INFO("%s, Open front_vbus1!", this->node_name.c_str());
        }
        else
        {
            ROS_INFO("%s, Didn't use the front_vbus1!", this->node_name.c_str());
        }
    }
    else
    {
        ROS_WARN("%s, Default didn't use the front_vbus1", this->node_name.c_str());
    }

    if (nh.hasParam("left_vbus2"))
    {
        nh.getParam("left_vbus2", left_vbus2);
        if (left_vbus2)
        {
            this->left_greyscale_image_left_pub = nh.advertise<sensor_msgs::Image>("left_vbus2/left_image", 1);
            this->left_greyscale_image_right_pub = nh.advertise<sensor_msgs::Image>("left_vbus2/right_image", 1);
            this->left_depth_image_pub = nh.advertise<sensor_msgs::Image>("left_vbus2/depth_image", 1);
            this->left_disparity_image_pub = nh.advertise<sensor_msgs::Image>("left_vbus2/disparity_image", 1);
            ROS_INFO("%s, Open left_vbus2!", this->node_name.c_str());
        }
        else
        {
            ROS_INFO("%s, Didn't use the left_vbus2!", this->node_name.c_str());
        }
    }
    else
    {
        ROS_WARN("%s, Default didn't use the left_vbus2", this->node_name.c_str());
    }

    if (nh.hasParam("back_vbus3"))
    {
        nh.getParam("back_vbus3", back_vbus3);
        if (back_vbus3)
        {
            this->back_greyscale_image_left_pub = nh.advertise<sensor_msgs::Image>("back_vbus3/left_image", 1);
            this->back_greyscale_image_right_pub = nh.advertise<sensor_msgs::Image>("back_vbus3/right_image", 1);
            this->back_depth_image_pub = nh.advertise<sensor_msgs::Image>("back_vbus3/depth_image", 1);
            this->back_disparity_image_pub = nh.advertise<sensor_msgs::Image>("back_vbus3/disparity_image", 1);
            ROS_INFO("%s, Open back_vbus3!", this->node_name.c_str());
        }
        else
        {
            ROS_INFO("%s, Didn't use the back_vbus3!", this->node_name.c_str());
        }
    }
    else
    {
        ROS_WARN("%s, Default didn't use the back_vbus3", this->node_name.c_str());
    }

    if (nh.hasParam("right_vbus4"))
    {
        nh.getParam("right_vbus4", right_vbus4);
        if (right_vbus4)
        {
            this->right_greyscale_image_left_pub = nh.advertise<sensor_msgs::Image>("right_vbus4/left_image", 1);
            this->right_greyscale_image_right_pub = nh.advertise<sensor_msgs::Image>("right_vbus4/right_image", 1);
            this->right_depth_image_pub = nh.advertise<sensor_msgs::Image>("right_vbus4/depth_image", 1);
            this->right_disparity_image_pub = nh.advertise<sensor_msgs::Image>("right_vbus4/disparity_image", 1);
            ROS_INFO("%s, Open right_vbus4!", this->node_name.c_str());
        }
        else
        {
            ROS_INFO("%s, Didn't use the right_vbus4!", this->node_name.c_str());
        }
    }
    else
    {
        ROS_WARN("%s, Default didn't use the right_vbus4", this->node_name.c_str());
    }

    if (nh.hasParam("down_vbus5"))
    {
        nh.getParam("down_vbus5", down_vbus5);
        if (down_vbus5)
        {
            this->down_greyscale_image_left_pub = nh.advertise<sensor_msgs::Image>("down_vbus5/left_image", 1);
            this->down_greyscale_image_right_pub = nh.advertise<sensor_msgs::Image>("down_vbus5/down_image", 1);
            this->down_depth_image_pub = nh.advertise<sensor_msgs::Image>("down_vbus5/depth_image", 1);
            this->down_disparity_image_pub = nh.advertise<sensor_msgs::Image>("down_vbus5/disparity_image", 1);
            ROS_INFO("%s, Open down_vbus5!", this->node_name.c_str());
        }
        else
        {
            ROS_INFO("%s, Didn't use the down_vbus5!", this->node_name.c_str());
        }
    }
    else
    {
        ROS_WARN("%s, Default didn't use the down_vbus5", this->node_name.c_str());
    }

    this->map_vbus_status.insert(std::make_pair(e_vbus_index::e_vbus1, front_vbus1));
    this->map_vbus_status.insert(std::make_pair(e_vbus_index::e_vbus2, left_vbus2));
    this->map_vbus_status.insert(std::make_pair(e_vbus_index::e_vbus3, back_vbus3));
    this->map_vbus_status.insert(std::make_pair(e_vbus_index::e_vbus4, right_vbus4));
    this->map_vbus_status.insert(std::make_pair(e_vbus_index::e_vbus5, down_vbus5));

    this->map_vbus_leftpub.insert(std::make_pair(e_vbus_index::e_vbus1, front_greyscale_image_left_pub));
    this->map_vbus_leftpub.insert(std::make_pair(e_vbus_index::e_vbus2, left_greyscale_image_left_pub));
    this->map_vbus_leftpub.insert(std::make_pair(e_vbus_index::e_vbus3, back_greyscale_image_left_pub));
    this->map_vbus_leftpub.insert(std::make_pair(e_vbus_index::e_vbus4, right_greyscale_image_left_pub));
    this->map_vbus_leftpub.insert(std::make_pair(e_vbus_index::e_vbus5, down_greyscale_image_left_pub));

    this->map_vbus_rightpub.insert(std::make_pair(e_vbus_index::e_vbus1, front_greyscale_image_right_pub));
    this->map_vbus_rightpub.insert(std::make_pair(e_vbus_index::e_vbus2, left_greyscale_image_right_pub));
    this->map_vbus_rightpub.insert(std::make_pair(e_vbus_index::e_vbus3, back_greyscale_image_right_pub));
    this->map_vbus_rightpub.insert(std::make_pair(e_vbus_index::e_vbus4, right_greyscale_image_right_pub));
    this->map_vbus_rightpub.insert(std::make_pair(e_vbus_index::e_vbus5, down_greyscale_image_right_pub));

    this->map_vbus_depthpub.insert(std::make_pair(e_vbus_index::e_vbus1, front_depth_image_pub));
    this->map_vbus_depthpub.insert(std::make_pair(e_vbus_index::e_vbus2, left_depth_image_pub));
    this->map_vbus_depthpub.insert(std::make_pair(e_vbus_index::e_vbus3, back_depth_image_pub));
    this->map_vbus_depthpub.insert(std::make_pair(e_vbus_index::e_vbus4, right_depth_image_pub));
    this->map_vbus_depthpub.insert(std::make_pair(e_vbus_index::e_vbus5, down_depth_image_pub));

    this->map_vbus_disparitypub.insert(std::make_pair(e_vbus_index::e_vbus1, front_disparity_image_pub));
    this->map_vbus_disparitypub.insert(std::make_pair(e_vbus_index::e_vbus2, left_disparity_image_pub));
    this->map_vbus_disparitypub.insert(std::make_pair(e_vbus_index::e_vbus3, back_disparity_image_pub));
    this->map_vbus_disparitypub.insert(std::make_pair(e_vbus_index::e_vbus4, right_disparity_image_pub));
    this->map_vbus_disparitypub.insert(std::make_pair(e_vbus_index::e_vbus5, down_disparity_image_pub));
}


DJIGuidanceROSHardware::~DJIGuidanceROSHardware()
= default;


std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value)
{
    const char* stream = nullptr;
    static char str[100]={0};
#define PROCESS_VAL(p) case(p): stream = #p; break;
    switch(value){
        PROCESS_VAL(e_OK);
        PROCESS_VAL(e_load_libusb_err);
        PROCESS_VAL(e_sdk_not_inited);
        PROCESS_VAL(e_disparity_not_allowed);
        PROCESS_VAL(e_image_frequency_not_allowed);
        PROCESS_VAL(e_config_not_ready);
        PROCESS_VAL(e_online_flag_not_ready);
        PROCESS_VAL(e_stereo_cali_not_ready);
        PROCESS_VAL(e_libusb_io_err);
        PROCESS_VAL(e_timeout);
        default:
            strcpy(str, "Unknown error");
            stream = str;
            break;
    }
#undef PROCESS_VAL

    return out << stream;
}


int DJIGuidanceROSHardware::publish_images(e_vbus_index vbus_index, image_data* data)
{
    if (this->map_vbus_status.at(vbus_index))
    {
        // TODO Share the same greyscale field in class?
        if (data->m_greyscale_image_left[vbus_index])
        {
            memcpy(this->greyscale_image_left.data, data->m_greyscale_image_left[vbus_index], IMAGE_SIZE);

            // Publish left greyscale image.
            cv_bridge::CvImage left_8;
            this->greyscale_image_left.copyTo(left_8.image);
            left_8.header.frame_id = this->frame_id;
            left_8.header.stamp = ros::Time::now();
            left_8.encoding = sensor_msgs::image_encodings::MONO8;
            map_vbus_leftpub.at(vbus_index).publish(left_8.toImageMsg());
        }

        if (data->m_greyscale_image_right[vbus_index])
        {
            memcpy(this->greyscale_image_right.data, data->m_greyscale_image_right[vbus_index], IMAGE_SIZE);

            // Publish right greyscale image.
            cv_bridge::CvImage right_8;
            this->greyscale_image_right.copyTo(right_8.image);
            right_8.header.frame_id = this->frame_id;
            right_8.header.stamp = ros::Time::now();
            right_8.encoding = sensor_msgs::image_encodings::MONO8;
            map_vbus_rightpub.at(vbus_index).publish(right_8.toImageMsg());
        }

        if (data->m_depth_image[vbus_index])
        {
            memcpy(this->depth16_image.data, data->m_depth_image[vbus_index], IMAGE_SIZE*2);

            // Publish depth image.
            cv_bridge::CvImage depth_16;
            this->depth16_image.copyTo(depth_16.image);
            depth_16.header.frame_id = this->frame_id;
            depth_16.header.stamp = ros::Time::now();
            depth_16.encoding = sensor_msgs::image_encodings::MONO16;
            map_vbus_depthpub.at(vbus_index).publish(depth_16.toImageMsg());
        }

        if (data->m_disparity_image[vbus_index])
        {
            // TODO
        }

        // TODO
        // key = waitKey(1);
    }

}


int DJIGuidanceROSHardware::datastream_callback(int data_type, int data_len, char *content)
{
    dji_lock.enter();

    // If velocity data
    if (e_guidance_event::e_velocity == data_type && content != nullptr)
    {
        auto* velocity_data = (velocity*)content;

        geometry_msgs::TwistStamped vo_msg;
        // vo_msg.header.seq
        vo_msg.header.frame_id = this->frame_id;
        vo_msg.header.stamp = ros::Time::now();
        vo_msg.twist.linear.x = 0.001f * velocity_data->vx;
        vo_msg.twist.linear.y = 0.001f * velocity_data->vy;
        vo_msg.twist.linear.z = 0.001f * velocity_data->vz;
        // vo_msg.twist.angular
        this->body_velocity_pub.publish(vo_msg);
    }

    // If motion data
    if (e_guidance_event::e_motion == data_type && content != nullptr)
    {
        // TODO NEXT
    }

    // If Image data
    if (e_guidance_event::e_image == data_type && content != nullptr)
    {
        auto* data = (image_data*)content;

        publish_images(e_vbus_index::e_vbus1, data);
        publish_images(e_vbus_index::e_vbus2, data);
        publish_images(e_vbus_index::e_vbus3, data);
        publish_images(e_vbus_index::e_vbus4, data);
        publish_images(e_vbus_index::e_vbus5, data);
    }

    // If IMU data
    if (e_guidance_event::e_imu == data_type && content != nullptr)
    {
        imu* body_imu_data = (imu*)content;

        // Publish IMU data
        sensor_msgs::Imu imu_msg;
        // imu_msg.header.seq
        imu_msg.header.frame_id = this->frame_id;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.linear_acceleration.x = body_imu_data->acc_x;
        imu_msg.linear_acceleration.y = body_imu_data->acc_y;
        imu_msg.linear_acceleration.z = body_imu_data->acc_z;
        imu_msg.orientation.w = body_imu_data->q[0];
        imu_msg.orientation.x = body_imu_data->q[1];
        imu_msg.orientation.y = body_imu_data->q[2];
        imu_msg.orientation.z = body_imu_data->q[3];
        this->body_imu_pub.publish(imu_msg);
    }


    return 0;
}

int DJIGuidanceROSHardware::Run()
{

}

int DJIGuidanceROSHardware::Release() {

    // Release data transfer.
    this->error_code = stop_transfer();
    RETURN_IF_ERR(error_code);

    // Make sure ACK packet from GUIDANCE is received.
    sleep(10);
    ROS_INFO("Release Guidance data transfer.");
    error_code = release_transfer();
    RETURN_IF_ERR(error_code);

    return 0;
}

