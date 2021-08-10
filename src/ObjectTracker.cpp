#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point32.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include <stdio.h>

#include "tf/transform_broadcaster.h"
#include "boost/foreach.hpp"
#include "sensor_msgs/image_encodings.h"

#include "librealsense2/rsutil.h"

#include <cmath>

/*
void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    ROS_INFO("%s, %d, %d\n", image_msg->encoding.c_str(), image_msg->height, image_msg->width);
    
}
*/

class ObjectDrawer
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_box_;
    ros::Subscriber sub_state_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher pub_;
    tf::TransformBroadcaster tf_publisher_;
    ros::Publisher obj_name_pub;

    darknet_ros_msgs::BoundingBoxes boxes;
    bool is_moving;
    rs2_intrinsics intrinsic_;

public:
    ObjectDrawer()
        : it_(nh_)
    {
        //printf("call\n");
        std::string image_topic = nh_.resolveName("camera/aligned_depth_to_color/image_raw");
        sub_ = it_.subscribeCamera(image_topic, 16, &ObjectDrawer::imageCb, this);
        sub_box_ = nh_.subscribe("darknet_ros/bounding_boxes", 1, &ObjectDrawer::boundingCallback, this);
        sub_state_ = nh_.subscribe("move_state", 1, &ObjectDrawer::moveStateCallback, this);
        obj_name_pub = nh_.advertise<std_msgs::String>("object_tracker/object_name", 100);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;

        try
        {
          input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
          image = input_bridge->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }

        intrinsic_.width = info_msg->width;
        intrinsic_.height = info_msg->height;
        intrinsic_.ppx = info_msg->K[2];
        intrinsic_.ppy = info_msg->K[5];
        intrinsic_.fx = info_msg->K[0];
        intrinsic_.fy = info_msg->K[4];
        intrinsic_.model = RS2_DISTORTION_NONE;

        for (int i = 0; i < info_msg->D.size(); i++)
        {
            intrinsic_.coeffs[i] = info_msg->D[i];
        }
        

        for(int i = 0; i < boxes.bounding_boxes.size(); i++)
        {
            //if (boxes.bounding_boxes[i].probability > 0.5 && boxes.bounding_boxes[i].Class == "person")
            if (boxes.bounding_boxes[i].probability > 0.2)
            {
                int center_x, center_y;
                std::string obj_name = boxes.bounding_boxes[i].Class;
                int obj_id = boxes.bounding_boxes[i].id;
                center_x = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin) / 2;
                center_y = (boxes.bounding_boxes[i].ymax + boxes.bounding_boxes[i].ymin) / 2;

                
                int depth_in_mm = image.at<short int>(cv::Point(center_x, center_y));
                float depth = (float)depth_in_mm / 1000;
                printf("depth: %f\n", depth);//depth_in_mm);
                
                printf("center: %d, %d\n", center_x, center_y);
                //cv::Point3d project_vec = cam_model_.projectPixelTo3dRay(cv::Point2d(center_x, center_y));
                float uv[2] = {(float)center_x, (float)center_y};
                float xyz[3];
                rs2_deproject_pixel_to_point(xyz, &intrinsic_, uv, depth);

                
                cv::Point3d obj_xyz(xyz[2], -xyz[0], -xyz[1]);

                ros::Time current_time = ros::Time::now();
            
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
                
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                //odom_trans.header.frame_id = obj_name;
                odom_trans.header.frame_id = "camera_color_frame";
                odom_trans.child_frame_id = obj_name; //+ std::to_string(i);
                //printf("%s: %f, %f, %f\n", (obj_name + std::to_string(i)).c_str(), obj_xyz.x, obj_xyz.y, obj_xyz.z);

                odom_trans.transform.translation.x = obj_xyz.x;
                odom_trans.transform.translation.y = obj_xyz.y;
                odom_trans.transform.translation.z = obj_xyz.z;
                odom_trans.transform.rotation = odom_quat;

                geometry_msgs::Point32 point;
                point.x = (float)obj_xyz.x;
                point.y = (float)obj_xyz.y;
                point.z = 0;

                if (!is_moving)
                {
                    ROS_INFO("%s published in %f, %f, %f\n", obj_name.c_str(), obj_xyz.x, obj_xyz.y, obj_xyz.z);
                    obj_name_pub.publish(odom_trans.child_frame_id);
                    tf_publisher_.sendTransform(odom_trans);
                }
                
            }
        }
    }
    void boundingCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &boxmsg)
    {
        boxes = *boxmsg;
    }

    void moveStateCallback(const std_msgs::BoolConstPtr &msg)
    {
        if (msg->data)
        {
            is_moving = true;
        }
        else
        {
            is_moving = false;
        }
    }
};




/*
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr pcl)
{
    printf("%d, %d\n", pcl->width, pcl->height);
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectTracker");

    //ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("darknet_ros/bounding_boxes", 256, &boundingCallback);
    //ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 256, &cloudCallback);
    /*
    image_transport::ImageTransport it(nh);
    image_transport::CameraSubscriber sub = it.subscribeCamera("camera/aligned_depth_to_color/image_raw", 1, &imageCb);
    image_transport::Publisher pub;
    */
    ObjectDrawer a;

    
    ros::spin();

}