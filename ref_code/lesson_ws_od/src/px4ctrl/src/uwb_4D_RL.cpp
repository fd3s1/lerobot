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
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackNode2.h>
#include <quadrotor_msgs/Uwb_Locate.h>
#include <quadrotor_msgs/Pose_Collect.h>
#include <quadrotor_msgs/Uwb_Stamp.h>
#include "PX4CtrlParam.h"

typedef message_filters::sync_policies::ApproximateTime<quadrotor_msgs::Uwb_Stamp, geometry_msgs::PoseStamped,
                                                                                   geometry_msgs::PoseStamped> auto_sync2;

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


struct UWBProblem {
    std::vector<double> ranges;  // 测量的距离
    std::vector<std::vector<double>> anchors;  // 锚点坐标
    std::vector<double> x;  // 优化变量：相对位置和旋转角度
    std::vector<double> last_x; 
    int operator()(const double* x, double* fvec) const;
};

int UWBProblem::operator()(const double* x, double* fvec) const {
    double dx = x[0];
    double dy = x[1];
    double dz = x[2];
    double theta = x[3];

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    for (size_t i = 0; i < ranges.size(); ++i) {
        Eigen::Vector3d anchor(anchors[i][0], anchors[i][1], anchors[i][2]);
        Eigen::Vector3d relative_position = R * anchor + Eigen::Vector3d(dx, dy, dz);
        double dist = relative_position.norm();
        fvec[i] = dist - ranges[i];
    }
    return 0;
}

class Uwb_Cal
{
private:

    // 数据缓存
    std::vector<quadrotor_msgs::Uwb_StampConstPtr> range_buffer;
    std::vector<geometry_msgs::PoseStampedConstPtr> pos0_buffer;
    std::vector<geometry_msgs::PoseStampedConstPtr> pos1_buffer;

    void extract_raw_uwb(nlink_parser::LinktrackNodeframe3ConstPtr& raw_uwb, quadrotor_msgs::Uwb_Stamp& stamp_uwb)
    {   
        int node_num=raw_uwb->nodes.size();
        stamp_uwb.header.stamp=ros::Time::now();
        for (int i=0 ; i< node_num ; i++){
            stamp_uwb.dis.emplace_back(raw_uwb->nodes[i].dis);
            stamp_uwb.id.emplace_back(raw_uwb->nodes[i].id);
        }
    }
    
public:
    ros::Subscriber uwb_raw_sub;
    message_filters::Subscriber <quadrotor_msgs::Uwb_Stamp> uwb_stamp_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  drone0_vin_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  drone1_vin_sub;
    
    ros::Publisher uwb_stamp_pub;
    ros::Publisher cal_pos_pub;

    void levenbergMarquardt(UWBProblem& problem, double lambda, double tol, int max_iter);
    void uwb_raw_feed(nlink_parser::LinktrackNodeframe3ConstPtr msg);

    void uwb_cal_callback2(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1);
                  
    //void make_uwb_cal_callback(int num);
    // void make_n_drone_align(int number);   
};

void Uwb_Cal::uwb_raw_feed(nlink_parser::LinktrackNodeframe3ConstPtr msg){
    quadrotor_msgs::Uwb_Stamp uwb_stamp;
    extract_raw_uwb(msg,uwb_stamp);
    uwb_stamp_pub.publish(uwb_stamp);
}

// void Uwb_Cal::uwb_cal_callback2_od(const quadrotor_msgs::Uwb_StampConstPtr& range, 
//                            const geometry_msgs::PoseStampedConstPtr& pos0,
//                            const geometry_msgs::PoseStampedConstPtr& pos1)
// {   UWBProblem problem;
//     problem.ranges.assign(range->dis.begin(),range->dis.end());
//     std::vector<double> collect0;
//     std::vector<double> collect1;
//     std::vector<double> collect2;
//     collect0.emplace_back(pos0->pose.position.x);
//     collect0.emplace_back(pos0->pose.position.y);
//     collect0.emplace_back(pos0->pose.position.z);
//     problem.anchors.emplace_back(collect0);

//     collect1.emplace_back(pos1->pose.position.x);
//     collect1.emplace_back(pos1->pose.position.y);
//     collect1.emplace_back(pos1->pose.position.z);
//     problem.anchors.emplace_back(collect1);

//     std::cout << "a = " << problem.anchors[0][1] << ", a = " << problem.anchors[1][1] << ", a=" << problem.anchors[2][1] << std::endl;
//     std::vector<double> x_last = problem.x;
//     if(problem.x.size()==0)
//     {problem.x={0,0,0};}
//     else{problem.x=x_last;}     
//     double lambda = 0.1;
//     double tol = 1e-5;
//     int max_iter = 1000;

//     levenbergMarquardt(problem, lambda, tol, max_iter);//这也在回调里
//     std::cout << "x = " << problem.x[0] << ", y = " << problem.x[1] << ", z = " << problem.x[2] << std::endl;
//     ROS_INFO("solved2");}


void Uwb_Cal::uwb_cal_callback2(const quadrotor_msgs::Uwb_StampConstPtr& range,
                               const geometry_msgs::PoseStampedConstPtr& pos0,
                               const geometry_msgs::PoseStampedConstPtr& pos1)
{
    // 收集数据
    range_buffer.push_back(range);
    pos0_buffer.push_back(pos0);
    pos1_buffer.push_back(pos1);

    // 保证缓存大小为20帧
    if (range_buffer.size() > 20) {
        range_buffer.erase(range_buffer.begin());
        pos0_buffer.erase(pos0_buffer.begin());
        pos1_buffer.erase(pos1_buffer.begin());
    }

    // 当缓存满时，执行估算
    if (range_buffer.size() == 20) {
        // 创建UWB问题实例
        UWBProblem problem;
        for (const auto& range_data : range_buffer) {
            problem.ranges.insert(problem.ranges.end(), range_data->dis.begin(), range_data->dis.end());
        }

        // 处理位置数据，提取坐标
        for (const auto& pos_data : pos0_buffer) {
            std::vector<double> anchor = {pos_data->pose.position.x, pos_data->pose.position.y, pos_data->pose.position.z};
            problem.anchors.push_back(anchor);
        }
        for (const auto& pos_data : pos1_buffer) {
            std::vector<double> anchor = {pos_data->pose.position.x, pos_data->pose.position.y, pos_data->pose.position.z};
            problem.anchors.push_back(anchor);
        }

        // // 初始化优化变量（当前假设位置为0,0,0，实际应用中可以设置为上次的优化结果）
        // if (problem.x.empty()) {
        //     problem.x = {0, 0, 0};
        // }
        // 使用上次的优化结果作为初始值
        if (!problem.last_x.empty()) {
            problem.x = problem.last_x;
        } else {
            // 如果没有上次的结果，则使用默认值
            problem.x = {0, 0, 0, 0};
        }
        // 调用Levenberg-Marquardt优化方法
        double lambda = 0.1;
        double tol = 1e-5;
        int max_iter = 1000;
        levenbergMarquardt(problem, lambda, tol, max_iter);
        problem.last_x = problem.x;
        // 打印计算结果
        ROS_INFO("Estimated Position: x = %f, y = %f, z = %f", problem.x[0], problem.x[1], problem.x[2]);
    }
}

void Uwb_Cal::levenbergMarquardt(UWBProblem& problem, double lambda, double tol, int max_iter) {
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(problem.x.data(), problem.x.size());
    int n = x.size();
    int m = problem.ranges.size();
    Eigen::VectorXd fvec(m);
    Eigen::MatrixXd J(m, n);

    for (int iter = 0; iter < max_iter; ++iter) {
        problem(x.data(), fvec.data());

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                double h = 1e-6;
                Eigen::VectorXd x1 = x;
                x1(j) += h;
                Eigen::VectorXd fvec1(m);
                problem(x1.data(), fvec1.data());
                J(i, j) = (fvec1(i) - fvec(i)) / h;
            }
        }

        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::VectorXd Jtf = J.transpose() * fvec;
        JtJ.diagonal().array() += lambda;
        Eigen::VectorXd dx = -JtJ.ldlt().solve(Jtf);

        Eigen::VectorXd x_new = x + dx;

        double err = fvec.squaredNorm();
        if (err < tol) {
            ROS_INFO("Optimization converged.");
            break;
        }

        Eigen::VectorXd fvec_new(m);
        problem(x_new.data(), fvec_new.data());
        double err_new = fvec_new.squaredNorm();
        if (err_new < err) {
            lambda /= 10;
            x = x_new;
            fvec = fvec_new;
        } else {
            lambda *= 10;
        }
    }

    problem.x = std::vector<double>(x.data(), x.data() + x.size());
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_cal");
    ros::NodeHandle nh("~");
    int number;
    read_essential_param(nh,"number",number);
    
    // if (!nh.getParam("number", number)) {
    //     ROS_ERROR("Failed to get 'number' parameter.");
    //     return -1;
    // }
    Uwb_Cal cal;
    ROS_INFO("solver is ok");
    cal.uwb_raw_sub = nh.subscribe<nlink_parser::LinktrackNodeframe3ConstPtr>("/nlink_linktrack_nodeframe3", 1, &Uwb_Cal::uwb_raw_feed, &cal);
    cal.uwb_stamp_pub = nh.advertise<quadrotor_msgs::Uwb_Stamp>("/uwb_stamp", 1);
    cal.uwb_stamp_sub.subscribe(nh, "/uwb_stamp", 1);
    cal.drone0_vin_sub.subscribe(nh, "/drone0/mavros/vision_pose/pose", 1);
    cal.drone1_vin_sub.subscribe(nh, "/drone1/mavros/vision_pose/pose", 1);

    if (number == 2) {
        // message_filters::Synchronizer<auto_sync2> uwbsync2(auto_sync2(5), cal.uwb_stamp_sub, cal.drone0_vin_sub);
        // uwbsync2.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback2, &cal, _1, _2));
        // ros::spin();
    
        message_filters::Synchronizer<auto_sync2>uwbsync2(auto_sync2(5), cal.uwb_stamp_sub, cal.drone0_vin_sub , cal.drone1_vin_sub);
        uwbsync2.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback2, &cal, _1, _2, _3));
        while (ros::ok()){
        ros::Rate rate(10);
        rate.sleep();
        ros::spinOnce();
        } 
    
    } else {
        ROS_ERROR("Unsupported number of drones.");
        return -1;
    }

    return 0;
}

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "uwb_cal");
//     ros::NodeHandle nh("~");

//     read_essential_param(nh,"number",number);
//     Uwb_Cal cal;
//     ROS_INFO("solver is ok");
//     cal.uwb_raw_sub = nh.subscribe<nlink_parser::LinktrackNodeframe3ConstPtr>("/nlink_linktrack_nodeframe3", 1, &Uwb_Cal::uwb_raw_feed , &cal);
//     cal.uwb_stamp_pub = nh.advertise<quadrotor_msgs::Uwb_Stamp>("/uwb_stamp",1);
//     cal.uwb_stamp_sub.subscribe(nh, "/uwb_stamp", 1);
//     cal.drone0_vin_sub.subscribe(nh, "/drone4/mavros/vision_pose/pose", 1);
//     cal.drone1_vin_sub.subscribe(nh, "/drone5/mavros/vision_pose/pose", 1);
//     cal.drone2_vin_sub.subscribe(nh, "/drone7/mavros/vision_pose/pose", 1);
//     cal.drone3_vin_sub.subscribe(nh, "/drone3/mavros/vision_pose/pose", 1);
//     if(number==2)
//     {
//         message_filters::Synchronizer<auto_sync2> uwbsync2(auto_sync2(5), cal.uwb_stamp_sub, cal.drone0_vin_sub);
//         uwbsync2.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback2, &cal, _1, _2));
//         while (ros::ok()){
//         ros::Rate rate(2);
//         rate.sleep();
//         ros::spinOnce();
//         }  
//     }
//     else if(number==3)
//     {
//         message_filters::Synchronizer<auto_sync3>uwbsync3(auto_sync3(5), cal.uwb_stamp_sub, cal.drone0_vin_sub , cal.drone1_vin_sub);
//         uwbsync3.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback3, &cal, _1, _2, _3));
//         while (ros::ok()){
//         ros::Rate rate(2);
//         rate.sleep();
//         ros::spinOnce();
//         } 
//     }