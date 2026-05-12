#include"uwb_cal.h"

// 定义非线性优化问题

    int UWBProblem::operator()(const double* x, double* fvec) const {
        for (int i = 0; i < ranges.size(); ++i) {
            double r = ranges[i];
            std::vector<double> a = anchors[i];   
            double dist = sqrt(pow(x[0] - a[0], 2) + pow(x[1] - a[1], 2) + pow(x[2] - a[2], 2));//这句话出问题了,因为x长度是0
            fvec[i] = dist - r;
        }
        return 0;
    }



// 非线性最小二乘法求解器
void Uwb_Cal::levenbergMarquardt(UWBProblem& problem, double lambda, double tol, int max_iter) {
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(problem.x.data(), problem.x.size());
    int n = x.size();//x=0?
    int m = problem.ranges.size();
    Eigen::VectorXd fvec(m);
    Eigen::MatrixXd J(m, n);
    for (int iter = 0; iter < max_iter; ++iter) {
        // 计算误差向量和雅可比矩阵
        problem(x.data(), fvec.data());//这句话出问题了
        
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
        // 计算增量
        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::VectorXd Jtf = J.transpose() * fvec;
        JtJ.diagonal().array() += lambda;
        Eigen::VectorXd dx = -JtJ.ldlt().solve(Jtf);
        // 更新变量
        Eigen::VectorXd x_new = x + dx;
        // 判断是否收敛
        double err = fvec.squaredNorm();
        if (err < tol) {
            ROS_INFO("finish");
            break;
        }
        // 更新lambda
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
    // 更新优化变量
    problem.x = std::vector<double>(x.data(), x.data() + x.size());
}

void Uwb_Cal::uwb_raw_feed(nlink_parser::LinktrackNodeframe3ConstPtr msg){
    quadrotor_msgs::Uwb_Stamp uwb_stamp;
    extract_raw_uwb(msg,uwb_stamp);
    uwb_stamp_pub.publish(uwb_stamp);
}
    
void Uwb_Cal::uwb_cal_callback2(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0)
{   UWBProblem problem;//这一堆应该在回调里进行
    problem.ranges.assign(range->dis.begin(),range->dis.end());
    std::vector<double> collect0;
    collect0.emplace_back(pos0->pose.position.x);
    collect0.emplace_back(pos0->pose.position.y);
    collect0.emplace_back(pos0->pose.position.z);
    problem.anchors.emplace_back(collect0);

    //problem.x = x0;//x0不应该是000，而是上一次的值
    double lambda = 0.1;
    double tol = 1e-5;
    int max_iter = 1000;
    levenbergMarquardt(problem, lambda, tol, max_iter);
    std::cout << "x = " << problem.x[0] << ", y = " << problem.x[1] << ", z = " << problem.x[2] << std::endl;
    std::vector<double> x0 = problem.x;
    ROS_INFO("solved2");}//在这一堆callback里把数据填了

void Uwb_Cal::uwb_cal_callback3(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1)
{   UWBProblem problem;
    problem.ranges.assign(range->dis.begin(),range->dis.end());
    std::vector<double> collect0;
    std::vector<double> collect1;
    std::vector<double> collect2;
    collect0.emplace_back(pos0->pose.position.x);
    collect0.emplace_back(pos0->pose.position.y);
    collect0.emplace_back(pos0->pose.position.z);
    problem.anchors.emplace_back(collect0);

    collect1.emplace_back(pos1->pose.position.x);
    collect1.emplace_back(pos1->pose.position.y);
    collect1.emplace_back(pos1->pose.position.z);
    problem.anchors.emplace_back(collect1);

    std::cout << "a = " << problem.anchors[0][1] << ", a = " << problem.anchors[1][1] << ", a=" << problem.anchors[2][1] << std::endl;
    std::vector<double> x_last = problem.x;
    if(problem.x.size()==0)
    {problem.x={0,0,0};}
    else{problem.x=x_last;}     
    double lambda = 0.1;
    double tol = 1e-5;
    int max_iter = 1000;

    levenbergMarquardt(problem, lambda, tol, max_iter);//这也在回调里
    std::cout << "x = " << problem.x[0] << ", y = " << problem.x[1] << ", z = " << problem.x[2] << std::endl;
    ROS_INFO("solved3");}

void Uwb_Cal::uwb_cal_callback4(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1,
                           const geometry_msgs::PoseStampedConstPtr& pos2)
{   ROS_INFO("I'm in callback4");
    UWBProblem problem;
    // for (int i = 0; i < number-1; i++)
    // {
    //    problem.ranges.emplace_back(range->dis[i]);
    // }
    problem.ranges.assign(range->dis.begin(),range->dis.end());
    std::vector<double> collect0;
    std::vector<double> collect1;
    std::vector<double> collect2;
    collect0.emplace_back(pos0->pose.position.x);
    collect0.emplace_back(pos0->pose.position.y);
    collect0.emplace_back(pos0->pose.position.z);
    problem.anchors.emplace_back(collect0);

    collect1.emplace_back(pos1->pose.position.x);
    collect1.emplace_back(pos1->pose.position.y);
    collect1.emplace_back(pos1->pose.position.z);
    problem.anchors.emplace_back(collect1);

    collect2.emplace_back(pos2->pose.position.x);
    collect2.emplace_back(pos2->pose.position.y);
    collect2.emplace_back(pos2->pose.position.z);
    std::cout << "pos2x = " << collect1[0] << ", pos2y = " << collect1[1] << ", pos2y" << collect1[2] << std::endl;
    problem.anchors.emplace_back(collect2);

    std::cout << "a = " << problem.anchors[0][1] << ", a = " << problem.anchors[1][1] << ", a=" << problem.anchors[2][1] << std::endl;
    std::vector<double> x_last = problem.x;
    if(problem.x.size()==0)
    {problem.x={0,0,0};}
    else{problem.x=x_last;}     
    double lambda = 0.1;
    double tol = 1e-5;
    int max_iter = 1000;

    levenbergMarquardt(problem, lambda, tol, max_iter);//这也在回调里
    std::cout << "x = " << problem.x[0] << ", y = " << problem.x[1] << ", z = " << problem.x[2] << std::endl;
    ROS_INFO("solved4");}

void Uwb_Cal::uwb_cal_callback5(const quadrotor_msgs::Uwb_StampConstPtr& range, 
                           const geometry_msgs::PoseStampedConstPtr& pos0,
                           const geometry_msgs::PoseStampedConstPtr& pos1,
                           const geometry_msgs::PoseStampedConstPtr& pos2,
                           const geometry_msgs::PoseStampedConstPtr& pos3)
{
    ROS_INFO("I'm in callback5");
    UWBProblem problem;
    problem.ranges.assign(range->dis.begin(),range->dis.end());
    std::vector<double> collect0;
    std::vector<double> collect1;
    std::vector<double> collect2;
    std::vector<double> collect3;
    collect0.emplace_back(pos0->pose.position.x);
    collect0.emplace_back(pos0->pose.position.y);
    collect0.emplace_back(pos0->pose.position.z);
    problem.anchors.emplace_back(collect0);

    collect1.emplace_back(pos1->pose.position.x);
    collect1.emplace_back(pos1->pose.position.y);
    collect1.emplace_back(pos1->pose.position.z);
    problem.anchors.emplace_back(collect1);

    collect2.emplace_back(pos2->pose.position.x);
    collect2.emplace_back(pos2->pose.position.y);
    collect2.emplace_back(pos2->pose.position.z);
    problem.anchors.emplace_back(collect2);

    collect3.emplace_back(pos3->pose.position.x);
    collect3.emplace_back(pos3->pose.position.y);
    collect3.emplace_back(pos3->pose.position.z);
    problem.anchors.emplace_back(collect3);

    //problem.x = x0;//x0不应该是000，而是上一次的值
    double lambda = 0.1;
    double tol = 1e-5;
    int max_iter = 1000;
    levenbergMarquardt(problem, lambda, tol, max_iter);//这也在回调里
    std::cout << "x = " << problem.x[0] << ", y = " << problem.x[1] << ", z = " << problem.x[2] << std::endl;
    std::vector<double> x0 = problem.x;
    ROS_INFO("solved5");}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_cal");
    ros::NodeHandle nh("~");

    read_essential_param(nh,"number",number);
    Uwb_Cal cal;
    ROS_INFO("solver is ok");
    cal.uwb_raw_sub = nh.subscribe<nlink_parser::LinktrackNodeframe3ConstPtr>("/nlink_linktrack_nodeframe3", 1, &Uwb_Cal::uwb_raw_feed , &cal);
    cal.uwb_stamp_pub = nh.advertise<quadrotor_msgs::Uwb_Stamp>("/uwb_stamp",1);
    cal.uwb_stamp_sub.subscribe(nh, "/uwb_stamp", 1);
    cal.drone0_vin_sub.subscribe(nh, "/drone4/mavros/vision_pose/pose", 1);
    cal.drone1_vin_sub.subscribe(nh, "/drone5/mavros/vision_pose/pose", 1);
    cal.drone2_vin_sub.subscribe(nh, "/drone7/mavros/vision_pose/pose", 1);
    cal.drone3_vin_sub.subscribe(nh, "/drone3/mavros/vision_pose/pose", 1);
    if(number==2)
    {
        message_filters::Synchronizer<auto_sync2> uwbsync2(auto_sync2(5), cal.uwb_stamp_sub, cal.drone0_vin_sub);
        uwbsync2.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback2, &cal, _1, _2));
        while (ros::ok()){
        ros::Rate rate(2);
        rate.sleep();
        ros::spinOnce();
        }  
    }
    else if(number==3)
    {
        message_filters::Synchronizer<auto_sync3>uwbsync3(auto_sync3(5), cal.uwb_stamp_sub, cal.drone0_vin_sub , cal.drone1_vin_sub);
        uwbsync3.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback3, &cal, _1, _2, _3));
        while (ros::ok()){
        ros::Rate rate(2);
        rate.sleep();
        ros::spinOnce();
        } 
    }
    else if(number==4)
    {
        message_filters::Synchronizer<auto_sync4>uwbsync4(auto_sync4(5), cal.uwb_stamp_sub, cal.drone0_vin_sub , cal.drone1_vin_sub , cal.drone2_vin_sub);
        uwbsync4.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback4, &cal, _1, _2, _3, _4));
        while (ros::ok()){
        ros::Rate rate(2);
        rate.sleep();
        ros::spinOnce();
        } 

    }
    else if(number==5)
    {
        message_filters::Synchronizer<auto_sync5>uwbsync5(auto_sync5(5), cal.uwb_stamp_sub, cal.drone0_vin_sub , cal.drone1_vin_sub , cal.drone2_vin_sub , cal.drone3_vin_sub);
        uwbsync5.registerCallback(boost::bind(&Uwb_Cal::uwb_cal_callback5, &cal, _1, _2, _3, _4, _5));
        while (ros::ok()){
        ros::Rate rate(2);
        rate.sleep();
        ros::spinOnce();
        } 
    }
    else
    {ROS_INFO("wrong number");}



  return 0;
}

