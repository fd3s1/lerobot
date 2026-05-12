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
    geometry_msgs::PoseStamped test_cmd_pose;
    void cmd_loop_cb(const ros::TimerEvent&);
};

Deal_test_cmd::Deal_test_cmd()
{
    test_cmd_pose.header.frame_id="map";
    test_cmd_pose.header.stamp=ros::Time::now();
    test_cmd_pose.pose.position.x=0.5;
    test_cmd_pose.pose.position.y=0.5;
    test_cmd_pose.pose.position.z=1.0;
}    
    
void Deal_test_cmd::cmd_loop_cb(const ros::TimerEvent&)
{
    cmd_pub.publish(test_cmd_pose);
    std::cout << "cmd_pose=" << test_cmd_pose.pose.position.x << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_pub_test");
    ros::NodeHandle nh("~");
    Deal_test_cmd test_cmd;
    test_cmd.cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone6/position_cmd",100);
    test_cmd.cmd_loop = nh.createTimer(ros::Duration(0.05), &Deal_test_cmd::cmd_loop_cb,&test_cmd);

    ros::spin();
    return 0;
}