#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class Deal_test_odom
{
    public:
    ros::Publisher odom_pub;
    ros::Timer odom_loop;
    ros::Time t_takeoff;
    //= ros::Time::now();
    geometry_msgs::PoseStamped test_odom_pose;
    void odom_loop_cb(const ros::TimerEvent&);
};
    
void Deal_test_odom::odom_loop_cb(const ros::TimerEvent&)
{
    if ((ros::Time::now()-t_takeoff).toSec()>=1)
    {
        test_odom_pose.pose.position.x=0; 
        test_odom_pose.pose.position.y=0;
        test_odom_pose.pose.position.z=1.5;
    }
    else{
        test_odom_pose.pose.position.x=0; 
        test_odom_pose.pose.position.y=0;
        test_odom_pose.pose.position.z=0;
    }
    
    odom_pub.publish(test_odom_pose);
    ROS_INFO("%f",test_odom_pose.pose.position.z);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_pub_test");
    ros::NodeHandle nh("~");
    Deal_test_odom test_odom;
    test_odom.t_takeoff = ros::Time::now();
    test_odom.odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone6/mavros/vision_pose/pose",10);
    test_odom.odom_loop = nh.createTimer(ros::Duration(0.007), &Deal_test_odom::odom_loop_cb,&test_odom);

    ros::spin();
    return 0;
}