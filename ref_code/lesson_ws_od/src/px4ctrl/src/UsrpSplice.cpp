#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <quadrotor_msgs/Sgn_Pose.h>
#include <quadrotor_msgs/sgn_stamp.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//using namespace sensor_msgs;
//using namespace message_filters;

class Usrp_Splice
{
    public:
    ros::Publisher usrp_pub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pos_sub;
    message_filters::Subscriber<quadrotor_msgs::sgn_stamp> usrp_sub;
    quadrotor_msgs::Sgn_Pose drone_state;
    void callback(const geometry_msgs::PoseStampedConstPtr& pos, const quadrotor_msgs::sgn_stampConstPtr& sgn);
};

void Usrp_Splice::callback(const geometry_msgs::PoseStampedConstPtr& pos, const quadrotor_msgs::sgn_stampConstPtr& sgn)
{
  //odom_for_pub.header.stamp=pos->header.stamp;
  drone_state.header.stamp=ros::Time::now();
  drone_state.header.frame_id = "world";
  drone_state.x=pos->pose.position.x;
  drone_state.y=pos->pose.position.y;
  drone_state.z=pos->pose.position.z;
  drone_state.rssi=sgn->rssi;
  usrp_pub.publish(drone_state);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "usrp_splice_6");

  ros::NodeHandle nh("~");

  Usrp_Splice usrp;
  usrp.pos_sub.subscribe(nh, "/drone6/mavros/vision_pose/pose", 50);
  usrp.usrp_sub.subscribe(nh, "/usrp_power_001",50);
  usrp.usrp_pub = nh.advertise<quadrotor_msgs::Sgn_Pose> ("/drone6/signal_status",50);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, quadrotor_msgs::sgn_stamp> mysync;
  message_filters::Synchronizer<mysync>sync(mysync(30),usrp.pos_sub, usrp.usrp_sub);
  //mysync(real_odom.pos_sub, real_odom.vel_sub, 30);
  sync.registerCallback(boost::bind(&Usrp_Splice::callback, &usrp ,_1, _2));
  while (ros::ok())
  {
    ros::Rate rate(50);
    rate.sleep();
    ros::spinOnce();
  }  
  return 0;
}