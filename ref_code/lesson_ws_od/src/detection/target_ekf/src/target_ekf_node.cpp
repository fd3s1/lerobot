#include <ros/ros.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h> //相近对齐
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <object_detection_msgs/BoundingBoxes.h>
#include <object_detection_msgs/TargetStatus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef message_filters::sync_policies::ApproximateTime<object_detection_msgs::TargetStatus, nav_msgs::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
    YoloOdomSynchronizer;
ros::Publisher target_odom_pub_, yolo_odom_pub_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_;
ros::Time last_update_stamp_;
double pitch_thr_ = 30;

Eigen::Vector3d cam_p;//相机系下自身的位置p
Eigen::Quaterniond cam_q;//相机系下自身的四元数q
double add_yaw;  //根据图像调整的yaw角

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 3);
    C.setZero(3, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    K = C;
    Qt.setIdentity(3, 3);
    Rt.setIdentity(3, 3);
    Qt(0, 0) = 4;
    Qt(1, 1) = 4;
    Qt(2, 2) = 1;
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    x.setZero(6);
  }
  inline void predict() {
    x = A * x;//更新x
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();//更新协方差P'
    return;
  }
  inline void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }
  inline bool checkValid(const Eigen::Vector3d& z) const {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }
  inline void update(const Eigen::Vector3d& z) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();//更新 K
    x = x + K * (z - C * x);//更新x
    Sigma = Sigma - K * C * Sigma;//更新协方差 P
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
};

std::shared_ptr<Ekf> ekfPtr_;

//这是定时器回调函数的一般格式
void predict_state_callback(const ros::TimerEvent& event) {
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    ROS_WARN("too long time no update!");
    return;
  }
  // publish target odom
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom.pose.pose.orientation.y = add_yaw;
  target_odom_pub_.publish(target_odom);//发布/real_target_ekf_node/target_odom话题  包含位置和线速度 固定的四元数  
}

/**
 * @brief 真机实验中 ekf更新函数,发布/target_ekf/yolo_odom话题
 * 
 * @param bboxes_msg 输入参数，来自yolov5的目标检测输出结果
 * @param odom_msg 输入参数，追踪无人机自身的odom
 */
void update_state_callback(const object_detection_msgs::BoundingBoxesConstPtr &bboxes_msg, const nav_msgs::OdometryConstPtr &odom_msg) {
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  Eigen::Vector3d odom_p;
  Eigen::Quaterniond odom_q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  // NOTE check pitch
  // Eigen::Vector3d eulerAngle = odom_q.matrix().eulerAngles(2, 1, 0);
  // double pitch = fabs( eulerAngle[1] / M_PI * 180 );
  // pitch = pitch > 90 ? 180 - pitch : pitch;
  // if (pitch > pitch_thr_) {
  //   ROS_ERROR("pitch too large!");
  //   return;
  // }

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;//相机系下自身的位置p
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);//相机系下自身的四元数q

  auto yolo_bbox = bboxes_msg->bounding_boxes.front();//数组的第一个框
  double xmin, ymin, xmax, ymax;//像素坐标系中的坐标
  xmin = yolo_bbox.xmin;
  xmax = yolo_bbox.xmax;
  ymin = yolo_bbox.ymin;
  ymax = yolo_bbox.ymax;
  // NOTE check ymin ymax
  /**
  double pixel_thr = 0;
  if (ymin < pixel_thr || ymax > 480 - pixel_thr) {
    ROS_ERROR("pitch out of range!");
    return;
  }
  */

  // calculate target odom
  double height = ymax - ymin;
  //depth只跟检测框的高度相关，高度height越高，说明目标越近，depth越小;height越小，说明目标离得越远,depth越大。
  double depth = 0.8 / height * fy_;//这个depth，不是靠深度相机计算出来的。是一个估计值 
  double y = ((ymin + ymax) * 0.5 - cy_) * depth / fy_; //计算出x y坐标  在相机坐标系下
  double x = ((xmin + xmax) * 0.5 - cx_) * depth / fx_;
  std::cout << " x = " << x << " y = " << y << " depth = " << depth << std::endl; 
  Eigen::Vector3d p(x, y, depth); //3行1列 的向量 代表x y z
  // std::cout << "p cam frame: " << p.transpose() << std::endl;
  p = cam_q * p + cam_p;//坐标转换 变换在世界坐标系下  更新这里的时候，
  // publish yolo odom
  nav_msgs::Odometry yolo_odom;
  yolo_odom.header.stamp = bboxes_msg->header.stamp;
  yolo_odom.header.frame_id = "world";
  yolo_odom.pose.pose.orientation.w = 1.0;
  yolo_odom.pose.pose.position.x = p.x();
  yolo_odom.pose.pose.position.y = p.y();
  yolo_odom.pose.pose.position.z = p.z();
  yolo_odom_pub_.publish(yolo_odom);//发布目标在世界坐标系的坐标  yolo_odom话题

  // update target odom
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 3.0) {
    ekfPtr_->reset(p);
    ROS_WARN("ekf reset!");
  } else if (ekfPtr_->checkValid(p)) {
    ekfPtr_->update(p);
  } else {
    ROS_ERROR("update invalid!");
    return;
  }
  last_update_stamp_ = ros::Time::now();
}

/**
 * @brief 真机实验中 ekf更新函数,发布/target_ekf/yolo_odom话题
 * 
 * @param detection_msg 输入参数，来自yolov5的目标检测输出结果
 * @param odom_msg 输入参数，追踪无人机自身的odom
 */
void update_realsense_state_callback(const object_detection_msgs::TargetStatusConstPtr &detection_msg, const nav_msgs::OdometryConstPtr &odom_msg) {
  Eigen::Vector3d odom_p;
  Eigen::Quaterniond odom_q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;//相机系下自身的位置p
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);//相机系下自身的四元数q

  // calculate target odom
  double depth = detection_msg->raw_pcl_position.z;//这个depth，是靠深度相机计算出来的
  double x = detection_msg->raw_pcl_position.x; //获取相机坐标系下xy坐标
  double y = detection_msg->raw_pcl_position.y;
  Eigen::Vector3d p(x, y, depth); //3行1列 的向量 代表x y z
  p = cam_q * p + cam_p;//坐标转换 变换在世界坐标系下  更新这里的时候，
  // publish yolo odom
  nav_msgs::Odometry yolo_odom;
  yolo_odom.header.stamp = detection_msg->header.stamp;
  yolo_odom.header.frame_id = "world";
  yolo_odom.pose.pose.orientation.w = 1.0;
  yolo_odom.pose.pose.position.x = p.x();
  yolo_odom.pose.pose.position.y = p.y();
  yolo_odom.pose.pose.position.z = p.z();
  yolo_odom_pub_.publish(yolo_odom);//发布目标在世界坐标系的坐标  yolo_odom话题

  // update target odom
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 3.0) {
    ekfPtr_->reset(p);
    ROS_WARN("ekf reset!");
  } else if (ekfPtr_->checkValid(p)) {
    ekfPtr_->update(p);
  } else {
    ROS_ERROR("update invalid!");
    return;
  }
  last_update_stamp_ = ros::Time::now();
}

void single_odom_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
  // std::cout << "single odom callback" << std::endl;
  Eigen::Vector3d odom_p;
  Eigen::Quaterniond odom_q;
  // tf::TransformBroadcaster tf_broadcaster;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;
  // tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
  //                     tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
  //                                   odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w),
  //                     tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z)),
  //                     ros::Time::now(), "world", "drone0_body")); // world->body
  // tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform( 
  //                   tf::inverse(tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
  //                           odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w)), // 欧拉角为w->b的表征，故取逆转化成b->w
  //                   tf::Vector3(-0.1, 0.0, 0.1)),  //(航模协会飞机 检测相机相对于飞机定位相机的偏差)
  //                   ros::Time::now(), "drone0_body","drone0_camera")); // body->camera
  // std::cout << "tannsform ok" << std::endl;

  //cam_p 一直在递增 是因为odom一直在递增  是因为仿真中无人机在螺旋上升，所以odom在增加
  // cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;//更新相机系下自身的位置p
  cam_p = odom_p;
  cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);//更新相机系下自身的四元数q

}

/**
 * @brief 
 * 
 * @param detection_msg 输入参数，来自realsense相机yolov5的目标检测输出结果
 */
void single_yolo_callback(const object_detection_msgs::TargetStatusConstPtr &detection_msg) {
  // calculate target odom
  if(detection_msg->update)
  {
    double image_x_pos = (detection_msg->xmin + detection_msg->xmax) / 2.0;
    if(image_x_pos <= 50) add_yaw = 0.523598;
    if(image_x_pos > 50 && image_x_pos <= 125) add_yaw = 0.349065;
    if(image_x_pos > 125 && image_x_pos <= 250) add_yaw = 0.174532;
    if(image_x_pos > 400 && image_x_pos <= 450) add_yaw = -0.261799;
    if(image_x_pos > 450 && image_x_pos <= 530) add_yaw = -0.349065;
    if(image_x_pos > 540 && image_x_pos <= 600) add_yaw = -0.523598;
    double depth = detection_msg->raw_pcl_position.z;//这个depth，是靠深度相机计算出来的
    double x = detection_msg->raw_pcl_position.x; //获取相机坐标系下xy坐标
    double y = detection_msg->raw_pcl_position.y;
    Eigen::Vector3d p(x, y, depth); //3行1列 的向量 代表x y z
    // std::cout << "camera pos:  " << "p.x(): " << p.x() << " " << "p.y(): " << p.y() <<  " " << "p.z(): " << p.z() << std::endl;
    p = cam_q * p + cam_p;//坐标转换 变换在世界坐标系下(即机体系VINS坐标+相对坐标)
    
    // p = cam2body_R_ * p + cam_p;//坐标转换 变换在世界坐标系下(即机体系VINS坐标+相对坐标) 
    p.z() = 1.8;
    //  !!!!更新这里的时候，需要odom_callback的数据。
    // std::cout << "world pos:  " << "p.x(): " << p.x() << " " <<  "p.y(): " << p.y() << " " <<  "p.z(): " << p.z() << std::endl;
    // publish yolo odom
    nav_msgs::Odometry yolo_odom;
    yolo_odom.header.stamp = detection_msg->header.stamp;
    yolo_odom.header.frame_id = "world";
    yolo_odom.pose.pose.orientation.w = 1.0;

    yolo_odom.pose.pose.position.x = p.x();
    yolo_odom.pose.pose.position.y = p.y();
    yolo_odom.pose.pose.position.z = p.z();
    // yolo_odom.pose.pose.position.z = 1.8;
    yolo_odom_pub_.publish(yolo_odom);//发布目标在世界坐标系的坐标  yolo_odom话题  只有位置数据，没有速度
    // std::cout << "/yolo_odom pub" << std::endl;
    // update target odom
    double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
    if (update_dt > 3.0) {
      ekfPtr_->reset(p);
      ROS_WARN("ekf reset!");
    } else if (ekfPtr_->checkValid(p)) {
      ekfPtr_->update(p);
    } else {
      ROS_ERROR("update invalid!");
      return;
    }
    last_update_stamp_ = ros::Time::now();
  }
}


void tf_yolo_callback(const object_detection_msgs::TargetStatusConstPtr &detection_msg) {
    object_detection_msgs::TargetStatus human_status; // camera坐标系 
    geometry_msgs::PoseStamped tf_human_status; // 转化到world坐标系下
    tf::TransformListener tf_listener;
    human_status = *detection_msg;
    if(human_status.update)
    {
      try{
        geometry_msgs::PoseStamped tf_before;
        tf_before.header.frame_id = "drone0_camera";
        // tf_before.header.stamp = this->uav_status.header.stamp; // 强制时间戳同步
        tf_before.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        tf_before.pose.position.x = human_status.raw_pcl_position.x;
        tf_before.pose.position.y = human_status.raw_pcl_position.y;
        tf_before.pose.position.z = human_status.raw_pcl_position.z;
        tf_listener.transformPose("world", tf_before, tf_human_status);
      }
      catch(tf::TransformException &ex)
      {
        ROS_ERROR("frame world is not exist",ex.what());
      }
    }
    nav_msgs::Odometry yolo_odom;
    yolo_odom.header.stamp = detection_msg->header.stamp;
    yolo_odom.header.frame_id = "world";
    yolo_odom.pose.pose.orientation.w = 1.0;

    Eigen::Vector3d p(tf_human_status.pose.position.x, tf_human_status.pose.position.y, tf_human_status.pose.position.z); //3行1列 的向量 代表x y z
    yolo_odom.pose.pose.position.x = p.x();
    yolo_odom.pose.pose.position.y = p.y();
    yolo_odom.pose.pose.position.z = p.z();
    yolo_odom_pub_.publish(yolo_odom);//发布目标在世界坐标系的坐标  yolo_odom话题  只有位置数据，没有速度
    // std::cout << "/yolo_odom pub" << std::endl;
    // update yolo odom
    double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
    if (update_dt > 3.0) {
      ekfPtr_->reset(p);
      ROS_WARN("ekf reset!");
    } else if (ekfPtr_->checkValid(p)) {
      ekfPtr_->update(p);
    } else {
      ROS_ERROR("update invalid!");
      return;
    }
    last_update_stamp_ = ros::Time::now();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now() - ros::Duration(10.0);

  std::vector<double> tmp;
  //nh.param<传入的参数类型>(launch文件中的参数名称, 程序中接受该参数的变量名称，该参数默认值）
  if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
  }
  if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
    cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
  }
  nh.getParam("cam_fx", fx_);
  nh.getParam("cam_fy", fy_);
  nh.getParam("cam_cx", cx_);
  nh.getParam("cam_cy", cy_);
  nh.getParam("pitch_thr", pitch_thr_);

  message_filters::Subscriber<object_detection_msgs::TargetStatus> yolo_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  ros::Timer ekf_predict_timer_;
//createTimer(Duration period, const TimerCallback& callback, bool oneshot = false,bool autostart = true)
//定时器即以一定的频率定时的调用一个回调函数

  ros::Subscriber single_odom_sub = nh.subscribe("odom", 100, &single_odom_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber single_yolo_sub = nh.subscribe("yolo", 1, &single_yolo_callback, ros::TransportHints().tcpNoDelay());
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  yolo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/yolo_odom", 1);

  int ekf_rate = 20;
  nh.getParam("ekf_rate", ekf_rate);
  ekfPtr_ = std::make_shared<Ekf>(1.0/ekf_rate);//圆括号里的是参数_dt

  // yolo_sub_.subscribe(nh, "yolo", 1, ros::TransportHints().tcpNoDelay());//订阅/yolov5trt/bboxes_pub话题
  // odom_sub_.subscribe(nh, "odom", 100, ros::TransportHints().tcpNoDelay());//订阅/drone0/odom  /vins_fusion/imu_propagate是无人机自身的odom
  // yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
  // yolo_odom_sync_Ptr_->registerCallback(boost::bind(&update_realsense_state_callback, _1, _2));

  ekf_predict_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_rate), &predict_state_callback);

  ros::spin();
  return 0;
}

