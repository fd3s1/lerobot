#include "yolov5_trt/human_info_pub.h"

void HumanInfo::LoopTaskWithoutVirtual(void)
{

}

void HumanInfo::LoopTask(void)
{

}

/**
 * @brief 做一个处理，只有检测到目标了才会把消息转发出去,当检测不到目标时，保留上一帧的检测数据
 * 
 * @param _msg 
 */
void HumanInfo::YoloHumanDetectCallback(const object_detection_msgs::RealsenseTargets::ConstPtr& _msg)
{
    const object_detection_msgs::RealsenseTarget *person = nullptr;
    object_detection_msgs::RealsenseTarget::_conf_type conf = 0;
    for(auto& item : _msg->realsense_targets)
    {
        if(item.conf > conf)
        {
            conf = item.conf;
            person = &item;
        }
    }
    if(person != nullptr)
    {
        this->human_status.xmax = person->xmax;
        this->human_status.xmin = person->xmin;
        this->human_status.ymax = person->ymax;
        this->human_status.ymin = person->ymin;
        if(person->position_from_realsense.z != 0)
        {
            this->human_status.raw_pcl_position = person->position_from_realsense;
        }
        else
        {
            std::cout << "depth error! " << this->human_status.raw_pcl_position << std::endl;
        }
    }
    this->human_status.update = (_msg->realsense_targets.size() != 0 && person->is_normal == true);
    this->yolo_human_pub.publish(this->human_status);
}


HumanInfo::HumanInfo(const ros::NodeHandle& _nh, double _period): RosBase(_nh, _period)
{
    this->yolo_human_pub = this->nh.advertise<object_detection_msgs::TargetStatus>("detection_status/human", 5);
    this->yolo_human_sub = this->nh.subscribe<object_detection_msgs::RealsenseTargets>("yolo_detector/realsense_targets",
                                                                                1,
                                                                                 &HumanInfo::YoloHumanDetectCallback,
                                                                                 this,
                                                                                   ros::TransportHints().tcpNoDelay());
}

HumanInfo::~HumanInfo()
{

}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"human_info_pub");
    ros::NodeHandle nh;

    HumanInfo HumanInfo(nh,1);
    ros::spin();
    
    return 0;
}