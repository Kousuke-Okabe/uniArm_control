#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <uniarm_control/my_robot.h>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <errno.h>
#define nLink 3

double Kp=5.5;//中

static std::mutex JointStates_mutex;
static sensor_msgs::JointState JointState;
static std_msgs::Float64 q1,q2,q3, dq1,dq2,dq3, tau1,tau2,tau3, accx,accy;

void Subscribe_Joint_State(const sensor_msgs::JointState::ConstPtr &JointState){
//  std::lock_guard<std::mutex> lock(JointStates_mutex);

  q1.data = JointState->position[0];
  q2.data = JointState->position[1];
  q3.data = JointState->position[2];

  dq1.data = JointState->velocity[0];
  dq2.data = JointState->velocity[1];
  dq3.data = JointState->velocity[2];
  accx.data = JointState->velocity[3];
  accy.data = JointState->velocity[4];

  tau1.data = JointState->effort[0];
  tau2.data = JointState->effort[1];
  tau3.data = JointState->effort[2];
}

int main( int argc, char* argv[] ){
  // ROSノード初期化
  ROS_INFO("main");
  ros::init(argc, argv, "uniarm_control_node");
  ros::NodeHandle nh;

  // Publisherの登録
  ROS_INFO("publisher");
  ros::Publisher pub_dq1 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q1/command",10);
  ros::Publisher pub_dq2 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q2/command",10);
  ros::Publisher pub_dq3 = nh.advertise<std_msgs::Float64>("/uniarm/velocity_controller_q3/command",10);

  // Subscriberの登録
  ROS_INFO("subscribe");
  ros::Subscriber sub_joint_state = nh.subscribe("/uniarm/joint_states",10, Subscribe_Joint_State);

  // ROS control
  ROS_INFO("ROS control");
  UniArm robot;
  controller_manager::ControllerManager cm(&robot);

  // サンプリング周期設定
  ROS_INFO("sampling");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time t = ros::Time::now();
  ros::Rate rate(1000.0); //[Hz]

  // リファレンス読み込み
  ROS_INFO("ref read");
  //static std::ifstream fin,fin2,fin3;
  //static std::string s,str;
  //static std::string s,s2,s3;
  static unsigned int N = 0;
  static double ref[nLink][60000];
  //int i=0,j=0;
  int itemp=0,k=0,error;//中
  FILE *fp;
	char fName[100] = {"./motion.ref"};


  /*fin.open("./motion.ref");
  if(!fin){
    ROS_INFO("Reference file cannot read.");
  }
  while(getline(fin,s)){
    //ROS_INFO_STREAM("ref(" << k << ")=" << s);
    ref[0][N] = std::stod(s);//中
    N++;
  }
  fin.close();

  fin2.open("./motion2.ref");
  if(!fin2){
    ROS_INFO("Reference file cannot read.");
  }
   while(getline(fin2,s2)){
    //ROS_INFO_STREAM("ref(" << k << ")=" << s);
    ref[1][N] = std::stod(s2);
  }
  fin2.close();

  fin3.open("./motion3.ref");
  if(!fin3){
    ROS_INFO("Reference file cannot read.");
  }
   while(getline(fin3,s3)){
    //ROS_INFO_STREAM("ref(" << k << ")=" << s);
    ref[2][N] = std::stod(s3);
  }
  fin3.close();*/
  /*fin.open("./motion.ref");
  if(!fin){
    ROS_INFO("Reference file cannot read.");
  }*/
  /*while(getline(fin,s)){
    //ROS_INFO_STREAM("ref(" << k << ")=" << s);
    ref[0][N] = std::stod(s);//中
    ref[1][N] = std::stod(s);
    ref[2][N] = std::stod(s);
    N++;
  }
  fin.close();*/
  /*while(getline(fin, str))
  {
        std::istringstream stream(str);
        // 区切り文字がなくなるまで文字を区切っていく
        while (getline(stream, s , ','))
        {
            // 区切られた文字がsに入る
            ref[i][j] = std::stod(s);
            j++;
        }
 
        j = 0;
        i++;  // 次行の配列に移る
  }
  fin.close();*/

  ROS_INFO("fopen");
  do{
		error = 0;
		//scanf("%s", fName);

		if( (fp = fopen(fName, "r")) == NULL ) {
			ROS_INFO("Reference file cannot read");
			error = 1;
		}

	}while( error == 1 );
  ROS_INFO("file read");
  do{
		for(k=0; k<nLink; k++){
			if( k==0 )
				itemp = fscanf(fp, "%lf", &ref[0][N]);
			else
				itemp = fscanf(fp, ",%lf", &ref[k][N]);
		}

		if( itemp != EOF )
			N++;

  }while( itemp!= EOF );
  ROS_INFO("fclose");
  if( fclose(fp) == EOF ) {
		ROS_INFO("file close error!!");
	}
  for(int a=0;a<nLink;a++){
    ROS_INFO_STREAM("ref[0][" << a << "]=" << ref[0][a]);
  }
  //ROS_INFO("%lf",ref[0][a]);
  // 動作プログラム
  std_msgs::Float64 dq1_cmd, dq2_cmd, dq3_cmd;
  dq1_cmd.data = 0;
  dq2_cmd.data = 0;
  dq3_cmd.data = 0;

  static double q[nLink][60000]={}, dq[nLink][60000]={}, tau[nLink][60000]={}, acc[2][600000];

//  double k=0;
//  while (ros::ok())
  for(int k=0; k<=N; k++)//中
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
    acc[0][k] = accx.data;
    acc[1][k] = accy.data;

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
      dq3_cmd.data = Kp*ref[2][k]; 
      pub_dq3.publish(dq3_cmd);
    }
     rate.sleep();
  }

   
   

//    ROS_INFO("angle %.2f  torque %.2f", q[0][k],tau[0][k]);
//  ROS_INFO("q1 angle %.2f  q2 angle %.2f  q3 angle %.2f", q1.data,q2.data,q3.data);

/*    dq1_cmd.data = k/10000;
    pub_dq1.publish(dq1_cmd);
    k = k+1;
*/
   
  //データをファイルへ保存
  static std::ofstream fout;
  fout.open("./data.txt");
  if(!fout){
    ROS_INFO_STREAM("Data file can not write.");
  }
  for(int k=0; k<N; k++){
    fout << " q1=, " << q[0][k] << ", tau1=, " << tau[0][k] << ", q2=, " << q[1][k] << ", tau2=, " << tau[1][k] << ", q3=, " << q[2][k] << ", tau3=, " << tau[2][k] << ", accx=, " << acc[0][k] << ", accy=, " << acc[1][k] << "\n"; //中 
  }
  fout.close();

  /*static std::ofstream fout;
  fout.open("./data2.txt");
  if(!fout){
    ROS_INFO_STREAM("Data file can not write.");
  }
  for(int k=0; k<N; k++){
    fout << " ref1=, " << ref[0][k] << "\n"; //中 
  }
  fout.close();*/

  return 0;
}
