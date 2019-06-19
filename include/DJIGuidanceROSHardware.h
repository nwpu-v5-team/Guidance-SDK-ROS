//
// Created by kehan on 19-6-18.
//

#ifndef GUIDANCE_DJIGUIDANCEROSHARDWARE_H_
#define GUIDANCE_DJIGUIDANCEROSHARDWARE_H_


#include <ros/ros.h>
#include <opencv2/core/mat.hpp>
#include "DJI_utility.h"
#include "DJI_guidance.h"

namespace vwpp
{

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}


    class DJIGuidanceROSHardware
    {
    public:
        DJIGuidanceROSHardware();
        virtual ~DJIGuidanceROSHardware();
        int Prepare();
        int Run();
        int Release();
    protected:
    private:

        friend std::ostream& operator<<(std::ostream& out, e_sdk_err_code value); //TODO friend
        int publish_images(e_vbus_index vbus_index, image_data* data);
        int try_to_select_images(e_vbus_index vbus_index);

        static int transfer_callback(int data_type, int data_len, char* content);
        int datastream_callback(int data_type, int data_len, char* content);
        static DJIGuidanceROSHardware* pThis;

        char key{};
        bool show_images;
        uint8_t verbosity;                  //TODO

        int error_code;

        DJI_SDK::DJI_lock dji_lock;
        DJI_SDK::DJI_event dji_event;

        cv::Mat greyscale_image_left;
        cv::Mat greyscale_image_right;

        cv::Mat depth16_image;
        // cv::Mat depth8_image;

        cv::Mat disparity_image;

        std::string node_name;
        std::string frame_id;

        bool front_vbus1;
        bool left_vbus2;
        bool back_vbus3;
        bool right_vbus4;
        bool down_vbus5;

        std::map<e_vbus_index, bool> map_vbus_status;

        ros::NodeHandle nh;
        ros::Publisher body_velocity_pub;   // No angular velocity outputs.
        ros::Publisher pose_pub;
        ros::Publisher global_odom_pub;     // No angular velocity outputs.
        ros::Publisher obstacles_dis_pub;
        ros::Publisher body_imu_pub;
        ros::Publisher ultrasonic_dis_pub;

        // TODO put the pubs in a struct!
        ros::Publisher front_greyscale_image_left_pub;
        ros::Publisher front_greyscale_image_right_pub;
        ros::Publisher front_depth_image_pub;
        ros::Publisher front_disparity_image_pub;

        ros::Publisher left_greyscale_image_left_pub;
        ros::Publisher left_greyscale_image_right_pub;
        ros::Publisher left_depth_image_pub;
        ros::Publisher left_disparity_image_pub;

        ros::Publisher back_greyscale_image_left_pub;
        ros::Publisher back_greyscale_image_right_pub;
        ros::Publisher back_depth_image_pub;
        ros::Publisher back_disparity_image_pub;

        ros::Publisher right_greyscale_image_left_pub;
        ros::Publisher right_greyscale_image_right_pub;
        ros::Publisher right_depth_image_pub;
        ros::Publisher right_disparity_image_pub;

        ros::Publisher down_greyscale_image_left_pub;
        ros::Publisher down_greyscale_image_right_pub;
        ros::Publisher down_depth_image_pub;
        ros::Publisher down_disparity_image_pub;

        std::map<e_vbus_index, ros::Publisher> map_vbus_leftpub;
        std::map<e_vbus_index, ros::Publisher> map_vbus_rightpub;
        std::map<e_vbus_index, ros::Publisher> map_vbus_depthpub;
        std::map<e_vbus_index, ros::Publisher> map_vbus_disparitypub;

    };

}


#endif //GUIDANCE_DJIGUIDANCEHARDWARE_H_
