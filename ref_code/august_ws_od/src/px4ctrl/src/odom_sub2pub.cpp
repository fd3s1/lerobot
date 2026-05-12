#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//using namespace sensor_msgs;
//using namespace message_filters;

class Real_odom
{
    public:
    ros::Publisher odom_real_pub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pos_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub;
    nav_msgs::Odometry odom_for_pub;
    void callback(const geometry_msgs::PoseStampedConstPtr& pos, const geometry_msgs::TwistStampedConstPtr& vel);
};

void Real_odom::callback(const geometry_msgs::PoseStampedConstPtr& pos, const geometry_msgs::TwistStampedConstPtr& vel)
{
  //odom_for_pub.header.stamp=pos->header.stamp;
  odom_for_pub.header.stamp=ros::Time::now();
  odom_for_pub.header.frame_id = "world";
  odom_for_pub.pose.pose=pos->pose;
  odom_for_pub.twist.twist=vel->twist;
  odom_real_pub.publish(odom_for_pub);
  //ROS_INFO("odom");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odom_real_pub6");

  ros::NodeHandle nh("~");

  Real_odom real_odom;
  real_odom.pos_sub.subscribe(nh, "/vrpn_client_node/drone6/pose", 50);
  real_odom.vel_sub.subscribe(nh, "/vrpn_client_node/drone6/twist",50);
  real_odom.odom_real_pub = nh.advertise<nav_msgs::Odometry> ("/drone6/odom",50);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> mysync;
  message_filters::Synchronizer<mysync>sync(mysync(30),real_odom.pos_sub, real_odom.vel_sub);
  //mysync(real_odom.pos_sub, real_odom.vel_sub, 30);
  sync.registerCallback(boost::bind(&Real_odom::callback, &real_odom ,_1, _2));
  while (ros::ok())
  {
    ros::Rate rate(100);
    rate.sleep();
    ros::spinOnce();
  }  
  return 0;
}