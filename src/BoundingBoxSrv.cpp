#include "ros/ros.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "object_detection_msgs/DetectedBoxes_srv.h"


darknet_ros_msgs::BoundingBoxes boxes;

void boundingCallback(darknet_ros_msgs::BoundingBoxesConstPtr &boxmsg)
{
    boxes.header = boxmsg->header;
    boxes.image_header = boxmsg->image_header;
    boxes.bounding_boxes.resize(boxmsg->bounding_boxes.size());
    
    for (int i = 0; i < boxmsg->bounding_boxes.size(); i++)
    {
        boxes.bounding_boxes[i] = boxmsg->bounding_boxes[i];
    }

}

bool serviceCallback(object_detection_msgs::DetectedBoxes_srv::Request &req,
                        object_detection_msgs::DetectedBoxes_srv::Response &res)
{   
    res.res.header.frame_id = boxes.header.frame_id;
    res.res.header.seq = boxes.header.seq;
    res.res.header.stamp = boxes.header.stamp;
    res.res.image_header.frame_id = boxes.image_header.frame_id;
    res.res.image_header.seq = boxes.image_header.seq;
    res.res.image_header.stamp = boxes.image_header.stamp;

    for (int i = 0; i < boxes.bounding_boxes.size(); i++)
    {
        res.res.Detected_boxes[i].Class = boxes.bounding_boxes[i].Class;
        res.res.Detected_boxes[i].id = boxes.bounding_boxes[i].id;
        res.res.Detected_boxes[i].probability = boxes.bounding_boxes[i].probability;
        res.res.Detected_boxes[i].xmax = boxes.bounding_boxes[i].xmax;
        res.res.Detected_boxes[i].xmin = boxes.bounding_boxes[i].xmin;
        res.res.Detected_boxes[i].ymax = boxes.bounding_boxes[i].ymax;
        res.res.Detected_boxes[i].ymin = boxes.bounding_boxes[i].ymin;
    }

    return req.req;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "darknet_boundingboxes_server");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("darknet_ros/bounding_boxes", 256, &boundingCallback);

    ros::ServiceServer service = nh.advertiseService("bounding_boxes", serviceCallback);
    ros::spin();
    return 0;
}

