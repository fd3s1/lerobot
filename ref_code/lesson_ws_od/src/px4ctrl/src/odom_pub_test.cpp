#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class Deal_test_odom
{
    public:
    ros::Publisher odom_pub0;
    ros::Publisher odom_pub1;
    ros::Publisher odom_pub2;
    ros::Publisher odom_pub3;
    ros::Publisher odom_pub4;
    ros::Timer odom_loop;
    ros::Time t_takeoff;
    //= ros::Time::now();
    geometry_msgs::PoseStamped test_odom_pose0;
    geometry_msgs::PoseStamped test_odom_pose1;
    geometry_msgs::PoseStamped test_odom_pose2;
    geometry_msgs::PoseStamped test_odom_pose3;
    geometry_msgs::PoseStamped test_odom_pose4;
    void odom_loop_cb(const ros::TimerEvent&);
};
    
void Deal_test_odom::odom_loop_cb(const ros::TimerEvent&)
{
    // if ((ros::Time::now()-t_takeoff).toSec()>=1)
    // {
    //     test_odom_pose.header.stamp=ros::Time::now();
    //     test_odom_pose.pose.position.x=0; 
    //     test_odom_pose.pose.position.y=0;
    //     test_odom_pose.pose.position.z=1.5;
    // }
    // else{
        test_odom_pose0.header.stamp=ros::Time::now();
        test_odom_pose0.pose.position.x=0; 
        test_odom_pose0.pose.position.y=-1;
        test_odom_pose0.pose.position.z=0.2;

        test_odom_pose1.header.stamp=ros::Time::now();
        test_odom_pose1.pose.position.x=0; 
        test_odom_pose1.pose.position.y=1;
        test_odom_pose1.pose.position.z=-0.2;

        test_odom_pose2.header.stamp=ros::Time::now();
        test_odom_pose2.pose.position.x=-1; 
        test_odom_pose2.pose.position.y=0;
        test_odom_pose2.pose.position.z=1;

        test_odom_pose3.header.stamp=ros::Time::now();
        test_odom_pose3.pose.position.x=0; 
        test_odom_pose3.pose.position.y=0;
        test_odom_pose3.pose.position.z=0;

        test_odom_pose4.header.stamp=ros::Time::now();
        test_odom_pose4.pose.position.x=0; 
        test_odom_pose4.pose.position.y=0;
        test_odom_pose4.pose.position.z=0;

    odom_pub0.publish(test_odom_pose0);
    odom_pub1.publish(test_odom_pose1);
    odom_pub2.publish(test_odom_pose2);
    odom_pub3.publish(test_odom_pose3);
    //odom_pub4.publish(test_odom_pose4);
    //ROS_INFO("%f",test_odom_pose.pose.position.z);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_pub_test");
    ros::NodeHandle nh("~");
    Deal_test_odom test_odom;
    test_odom.t_takeoff = ros::Time::now();
    
    test_odom.odom_pub0 = nh.advertise<geometry_msgs::PoseStamped>("/drone0/mavros/vision_pose/pose",10);
    test_odom.odom_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/vision_pose/pose",10);
    test_odom.odom_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/drone2/mavros/vision_pose/pose",10);
    test_odom.odom_pub3 = nh.advertise<geometry_msgs::PoseStamped>("/drone3/mavros/vision_pose/pose",10);
    //test_odom.odom_pub4 = nh.advertise<geometry_msgs::PoseStamped>("/drone4/mavros/vision_pose/pose",1);

    test_odom.odom_loop = nh.createTimer(ros::Duration(0.1), &Deal_test_odom::odom_loop_cb,&test_odom);

    ros::spin();
    return 0;
}