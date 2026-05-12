#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

class Deal_test_cmd
{
    public:
    Deal_test_cmd();
    ros::Publisher cmd_pub;
    ros::Timer cmd_loop;
    quadrotor_msgs::PositionCommand test_cmd_pose;
    void cmd_loop_cb(const ros::TimerEvent&);
};

Deal_test_cmd::Deal_test_cmd()
{
    test_cmd_pose.acceleration.x=0.5;
    test_cmd_pose.acceleration.y=0.0;
    test_cmd_pose.acceleration.z=1e-6;
}    
    
void Deal_test_cmd::cmd_loop_cb(const ros::TimerEvent&)
{
    cmd_pub.publish(test_cmd_pose);
    ROS_INFO("4546677778");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_pub_test");
    ros::NodeHandle nh("~");
    Deal_test_cmd test_cmd;
    test_cmd.cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/drone6/position_cmd",100);
    test_cmd.cmd_loop = nh.createTimer(ros::Duration(0.008), &Deal_test_cmd::cmd_loop_cb,&test_cmd);

    ros::spin();
    return 0;
}