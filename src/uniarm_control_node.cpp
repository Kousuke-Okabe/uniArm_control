#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <uniarm_control/my_robot.h>
#include <fstream>
#include <string>

#define nLink 3

double Kp=2;//中

static std::mutex JointStates_mutex;
static sensor_msgs::JointState JointState;
static std_msgs::Float64 q1,q2,q3, dq1,dq2,dq3, tau1,tau2,tau3;

void Subscribe_Joint_State(const sensor_msgs::JointState::ConstPtr &JointState){
//  std::lock_guard<std::mutex> lock(JointStates_mutex);

  q1.data = JointState->position[0];
  q2.data = JointState->position[1];
  q3.data = JointState->position[2];

  dq1.data = JointState->velocity[0];
  dq2.data = JointState->velocity[1];
  dq3.data = JointState->velocity[2];

  tau1.data = JointState->effort[0];
  tau2.data = JointState->effort[1];
  tau3.data = JointState->effort[2];
}

int main( int argc, char* argv[] ){
  // ROSノード初期化
  ros::init(argc, argv, "uniarm_control_node");
  ros::NodeHandle nh;

  // Publisherの登録
  ros::Publisher pub_dq1 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q1/command",10);
  ros::Publisher pub_dq2 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q2/command",10);
  ros::Publisher pub_dq3 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q3/command",10);

  // Subscriberの登録
  ros::Subscriber sub_joint_state = nh.subscribe("/uniarm/joint_states",10, Subscribe_Joint_State);

  // ROS control
  UniArm robot;
  controller_manager::ControllerManager cm(&robot);

  // サンプリング周期設定
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time t = ros::Time::now();
  ros::Rate rate(1000.0); //[Hz]

  // リファレンス読み込み
  static std::ifstream fin;
  static std::string s;
  static unsigned int N = 0;
  static double ref[nLink][60000];//中

  fin.open("./motion.ref");
  if(!fin){
    ROS_INFO("Reference file cannot read.");
  }

  while(getline(fin,s)){
    //ROS_INFO_STREAM("ref(" << k << ")=" << s);
    ref[0][N] = std::stod(s);//中
    ref[1][N] = std::stod(s);
    ref[2][N] = std::stod(s);
    N++;
  }
  fin.close();

  // 動作プログラム
  std_msgs::Float64 dq1_cmd, dq2_cmd, dq3_cmd;
  dq1_cmd.data = 0;
  dq2_cmd.data = 0;
  dq3_cmd.data = 0;

  static double q[nLink][60000]={}, dq[nLink][60000]={}, tau[nLink][60000]={};

//  double k=0;
//  while (ros::ok())
  for(int k=0; k<=N; k++)
  {
    ros::Duration d = ros::Time::now() - t;
    ros::Time t = ros::Time::now();

    robot.read();
    cm.update(t, d);
    robot.write();

    // 動作指令 Publish
    //dq1_cmd.data = ref[0][k];
    q[0][k] = q1.data;
    q[1][k] = q2.data;
    q[2][k] = q3.data;
    tau[0][k] = tau1.data;
    tau[1][k] = tau2.data;
    tau[2][k] = tau3.data;

    if(k==N){
      dq1_cmd.data = 0;  //中
      pub_dq1.publish(dq1_cmd);
      dq2_cmd.data = 0;
      pub_dq2.publish(dq2_cmd);
      dq3_cmd.data = 0; 
      pub_dq3.publish(dq3_cmd);
      robot.read();
      cm.update(t, d);
      robot.write();
    }
    else{
      dq1_cmd.data = Kp*(ref[0][k]-q[0][k]);  //中
      pub_dq1.publish(dq1_cmd);
      dq2_cmd.data = Kp*(ref[1][k]-q[1][k]);
      pub_dq2.publish(dq2_cmd);
      dq3_cmd.data = Kp*(ref[2][k]-q[2][k]); 
      pub_dq3.publish(dq3_cmd);
    }
    
   
   

//    ROS_INFO("angle %.2f  torque %.2f", q[0][k],tau[0][k]);
//  ROS_INFO("q1 angle %.2f  q2 angle %.2f  q3 angle %.2f", q1.data,q2.data,q3.data);

/*    dq1_cmd.data = k/10000;
    pub_dq1.publish(dq1_cmd);
    k = k+1;
*/
    rate.sleep();
  }

  //データをファイルへ保存
  static std::ofstream fout;
  fout.open("./data.txt");
  if(!fout){
    ROS_INFO_STREAM("Data file can not write.");
  }
  for(int k=0; k<N; k++){
    fout << " q1=, " << q[0][k] << ", tau2=, " << tau[0][k] << ", q2=, " << q[1][k] << ", tau2=, " << tau[1][k] << ", q3=, " << q[2][k] << ", tau3=, " << tau[2][k] << "\n"; //中 
  }
  fout.close();

  return 0;
}
