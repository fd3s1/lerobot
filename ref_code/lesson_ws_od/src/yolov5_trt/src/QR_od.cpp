#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <your_custom_msgs/RealsenseTargets.h>
#include <Eigen/Geometry>

class QRCodeTransform
{
public:
    QRCodeTransform()
    {
        pose_subscriber = nh.subscribe("/drone6/mavros/vision_pose/pose", 10, &QRCodeTransform::poseCallback, this);
        targets_subscriber = nh.subscribe("/drone0/yolov8_detector/realsense_targets", 10, &QRCodeTransform::targetsCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber, targets_subscriber;
    Eigen::Quaterniond body_orientation;
    Eigen::Vector3d body_position;

    // 相机坐标系到机体坐标系的旋转
    Eigen::Matrix3d camera_to_body_rotation;

    // 相机到机体的平移向量
    Eigen::Vector3d camera_to_body_translation;

    QRCodeTransform()
    {
        // 初始化旋转矩阵：相机坐标轴 -> 机体坐标轴
        camera_to_body_rotation << 0, -1, 0,
                                   1, 0, 0,
                                   0, 0, -1;

        // 初始化平移：相机在机体中心前方0.15米
        camera_to_body_translation << 0.15, 0, 0;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        body_position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        body_orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }

    void targetsCallback(const your_custom_msgs::RealsenseTargets::ConstPtr& msg)
    {
        for (const auto& target : msg->targets)
        {
            // 从相机坐标系转换到机体坐标系
            Eigen::Vector3d target_position_camera(target.x, target.y, target.z);
            Eigen::Vector3d target_position_body = camera_to_body_rotation * target_position_camera + camera_to_body_translation;

            // 从机体坐标系转换到世界坐标系
            Eigen::Vector3d target_position_world = body_orientation * target_position_body + body_position;

            ROS_INFO("Target in world coordinates: [%f, %f, %f]", 
                     target_position_world.x(), 
                     target_position_world.y(), 
                     target_position_world.z());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_code_transform_node");
    QRCodeTransform qr_transform;
    ros::spin();
    return 0;
}
