#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>


void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    // Controller controller(param);
    //LinearControl controller(param);
    PX4CtrlFSM fsm(param);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/drone6/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
                                         //对类方法来说，直接boost::bind(&类名::方法名，类实例指针，参数1，参数2）
                                         //这里就一个参数，用_1占位

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/drone6/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    // ros::Subscriber odom_sub =
    //     nh.subscribe<nav_msgs::Odometry>("/drone6/odom",
    //                                      100,
    //                                      boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
    //                                      ros::VoidConstPtr(),
    //                                      ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber odom_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    // ros::Subscriber local_sub =
    //         nh.subscribe<geometry_msgs::PoseStamped>("/drone6/mavros/local_position/pose",
    //                                      100,
    //                                      boost::bind(&PX4CtrlFSM::vision_pose_cb, &fsm, _1),
    //                                      ros::VoidConstPtr(),
    //                                      ros::TransportHints().tcpNoDelay());

    // ros::Subscriber cmd_sub =
    //     nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
    //                                                   100,
    //                                                   boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
    //                                                   ros::VoidConstPtr(),
    //                                                   ros::TransportHints().unreliable().maxDatagramSize(1000));
        ros::Subscriber cmd_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("cmd",
                                                    100,
                                                    boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().unreliable().maxDatagramSize(1000));

    // ros::Subscriber imu_sub =
    //     nh.subscribe<sensor_msgs::Imu>("/drone6/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
    //                                    100,
    //                                    boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
    //                                    ros::VoidConstPtr(),
    //                                    ros::TransportHints().unreliable().maxDatagramSize(1000));

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/drone6/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    // ros::Subscriber bat_sub =
    //     nh.subscribe<sensor_msgs::BatteryState>("/drone6/mavros/battery",
    //                                             100,
    //                                             boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
    //                                             ros::VoidConstPtr());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("/drone6/px4ctrl/takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    
    
    fsm.manual_flag_sub=nh.subscribe<std_msgs::UInt8>("/drone6_pub_trigger_flag",1,&PX4CtrlFSM::manual_flag_cb,&fsm);
    ///////？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
     fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::PositionTarget>("/drone6/mavros/setpoint_raw/local", 10);
    //fsm.ctrl_FCU_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone6/mavros/setpoint_position/local", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone6/traj_start_trigger", 10);

    //fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/drone6/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/drone6/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/drone6/mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/drone6/mavros/cmd/command");

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
