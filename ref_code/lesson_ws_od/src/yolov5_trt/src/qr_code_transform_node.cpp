#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <object_detection_msgs/RealsenseTargets.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class QRCodeTransform
{
public:
    QRCodeTransform()
    {
        // 初始化 message_filters 订阅器
        pose_subscriber.subscribe(nh, "/drone6/mavros/vision_pose/pose", 1);
        targets_subscriber.subscribe(nh, "/drone6/yolov8_detector/realsense_targets", 1);

        // 使用 ApproximateTime 策略进行时间同步
        sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), pose_subscriber, targets_subscriber));
        sync->registerCallback(boost::bind(&QRCodeTransform::callback, this, _1, _2));

        // 初始化旋转矩阵和平移向量
        camera_to_body_rotation << 0, -1, 0,  // x_camera -> -y_body
                                   -1, 0, 0,  // y_camera -> x_body
                                   0, 0, -1; // z_camera -> -z_body
        camera_to_body_translation << 0.13, 0, 0;  // 相机前方0.15米
    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const object_detection_msgs::RealsenseTargets::ConstPtr& targets_msg)
    {
        Eigen::Vector3d body_position(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
        Eigen::Quaterniond body_orientation(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);

        for (const auto& target : targets_msg->realsense_targets)
        {
            Eigen::Vector3d target_position_camera(target.position_from_realsense.x, target.position_from_realsense.y, 
            target.position_from_realsense.z);
            Eigen::Vector3d target_position_body = camera_to_body_rotation * target_position_camera + camera_to_body_translation;
            Eigen::Vector3d target_position_world = body_orientation * target_position_body + body_position;

            ROS_INFO("Target in world coordinates: [%f, %f, %f]", 
                     target_position_world.x(), 
                     target_position_world.y(), 
                     target_position_world.z());
        }
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_subscriber;
    message_filters::Subscriber<object_detection_msgs::RealsenseTargets> targets_subscriber;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, object_detection_msgs::RealsenseTargets> MySyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
    Eigen::Matrix3d camera_to_body_rotation;
    Eigen::Vector3d camera_to_body_translation;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_code_transform_node");
    QRCodeTransform qr_transform;
    ros::spin();
    return 0;
}
