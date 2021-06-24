#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <uniarm_control/my_robot.h>

int main( int argc, char* argv[] )
{
  // ROSノード初期化
  ros::init(argc, argv, "uniarm_control_node");
  ros::NodeHandle nh;

  // Publisherの登録
  ros::Publisher cmd_dq1 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q1/command",10);

  // ROS control
  UniArm robot;
  controller_manager::ControllerManager cm(&robot);

  // サンプリング周期設定
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time t = ros::Time::now();
  ros::Rate rate(1000.0); //[Hz]

  // 変数宣言
  std_msgs::Float64 dq1;
  dq1.data = 0;

  while (ros::ok())
  {
    ros::Duration d = ros::Time::now() - t;
    ros::Time t = ros::Time::now();

    robot.read();
    cm.update(t, d);
    robot.write();

    if( dq1.data == 0 ){
        dq1.data = 5;
    }
    else{
      dq1.data = 0;
    }

    cmd_dq1.publish(dq1);

    rate.sleep();
  }

  return 0;
}
