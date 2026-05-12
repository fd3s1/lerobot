#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nlink_parser/LinktrackNodeframe3.h>//这消息里竟然没header
#include <nlink_parser/LinktrackNode2.h>
#include <quadrotor_msgs/Uwb_Locate.h>
#include <quadrotor_msgs/Pose_Collect.h>
#include <quadrotor_msgs/Uwb_Stamp.h>
#include "PX4CtrlParam.h"
// template<typename T, typename... Targs>
// void make_n_drone_align(T value, Targs... drone)
// {
//     //void setVector(T arg) 
//     //vector.emplace_back(arg);
//     //template<typename T, typename... Targs>
//     typedef message_filters::sync_policies::ApproximateTime<quadrotor_msgs::Uwb_Stamp, Targs...> 
//                                         auto_sync;
// }
typedef message_filters::sync_policies::ApproximateTime<quadrotor_msgs::Uwb_Stamp, geometry_msgs::PoseStamped> auto_sync2;

typedef message_filters::sync_policies::ApproximateTime<quadrotor_msgs::Uwb_Stamp, geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped> auto_sync3;

typedef message_filters::sync_policies::ApproximateTime<quadrotor_msgs::Uwb_Stamp, geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped> auto_sync4;
                                                                        
typedef message_filters::sync_policies::ApproximateTime<quadrotor_msgs::Uwb_Stamp, geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped> auto_sync5;


int number;

struct UWBProblem {
    // 测量值
    std::vector<double> ranges;
    // anchor坐标
    std::vector<std::vector<double>> anchors;
    // 优化变量
    std::vector<double> x;
    // 重载()运算符，计算误差
    int operator()(const double* x, double* fvec)const;
};

template<typename TName, typename TVal>
	TVal read_essential_param(const ros::NodeHandle &nh, const TName &name ,TVal &val)
	{
		if (nh.getParam(name, val))
		{
			ROS_INFO_STREAM("Read param: "<< name << val << " succeed.");
            
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
        return val;
    }

class Uwb_Cal
{
private:

    inline void extract_raw_uwb(nlink_parser::LinktrackNodeframe3ConstPtr& raw_uwb, quadrotor_msgs::Uwb_Stamp& stamp_uwb)
    {   
        int node_num=raw_uwb->nodes.size();
        stamp_uwb.header.stamp=ros::Time::now();
        for (int i=0 ; i< 3 ; i++){
            stamp_uwb.dis.emplace_back(raw_uwb->nodes[i].dis);
            stamp_uwb.id.emplace_back(raw_uwb->nodes[i].id);
        }
    }

    
public:
    ros::Subscriber uwb_raw_sub;
    message_filters::Subscriber <quadrotor_msgs::Uwb_Stamp> uwb_stamp_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  drone0_vin_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  drone1_vin_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  drone2_vin_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  drone3_vin_sub;
    
    ros::Publisher uwb_stamp_pub;
    ros::Publisher cal_pos_pub;

    void levenbergMarquardt(UWBProblem& problem, double lambda, double tol, int max_iter);
    void uwb_raw_feed(nlink_parser::LinktrackNodeframe3ConstPtr msg);

    void uwb_cal_callback2(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0);

    void uwb_cal_callback3(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1);

    void uwb_cal_callback4(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1,
                           const geometry_msgs::PoseStampedConstPtr& pos2);

    void uwb_cal_callback5(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1,
                           const geometry_msgs::PoseStampedConstPtr& pos2,
                           const geometry_msgs::PoseStampedConstPtr& pos3);                       
    //void make_uwb_cal_callback(int num);
    // void make_n_drone_align(int number);   
};

// Uwb_Cal::Uwb_Cal(){
//     message_filters::Synchronizer<auto_sync>uwbsync(auto_sync(30), uwb_stamp_sub, drone4_vin_sub , drone5_vin_sub , drone7_vin_sub);
//     uwbsync.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback, this ,_1, _2, _3, _4));}




