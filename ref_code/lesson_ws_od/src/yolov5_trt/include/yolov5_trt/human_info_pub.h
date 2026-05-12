#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "ros_base/ros_base.h"
#include <object_detection_msgs/RealsenseTargets.h>
#include <object_detection_msgs/TargetStatus.h>

class HumanInfo : public RosBase
{
public:
    HumanInfo(const ros::NodeHandle& _nh, double _period);
    ~HumanInfo();
    void LoopTaskWithoutVirtual(void);
private:
    void YoloHumanDetectCallback(const object_detection_msgs::RealsenseTargets::ConstPtr& _msg);
    virtual void LoopTask(void);
    object_detection_msgs::TargetStatus human_status;
    ros::Subscriber yolo_human_sub;
    ros::Publisher yolo_human_pub;  
};
