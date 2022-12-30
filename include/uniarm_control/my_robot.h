#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <bits/stdc++.h>
//AIOヘッダファイル
#include "caio.h"

#define nLink	3
#define PCI_DA "contec_Ao"
#define PCI_AD "contec_Ai"
#define sensor	2//中

class UniArm : public hardware_interface::RobotHW
{
public:
  UniArm();
  ~UniArm();
  void read();
  void write();
private:
  long i;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[3]={0.0, 0.0, 0.0};
  double pos[3];
  double vel[3];//中
  double eff[3];
  double ord[3];

  // AIO
	long	Ret;                          //関数の戻り値
  char	DeviceName[20];               //デバイス名
	short	Id_Ao, Id_Ai;                 //ID
	char	ErrorString[256];             //エラーコード文字列
	short	AoRange, AiRange;             //レンジ
	short	AoChannels = nLink;           //使用チャネル数
	short	AiChannels = nLink;
	float	AoData[nLink], AiData[nLink*2+sensor];//中     //変換データ
	long	AoSamplingCount, AiSamplingCount;	 //現在のサンプリング回数
	long	AoStatus, AiStatus;                //現在のステータス
	long	AiSamplingTimes = 1;               //取得サンプリング回数
};

UniArm::UniArm()
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_q1("q1", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_q1);

  hardware_interface::JointStateHandle state_handle_q2("q2", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_q2);

  hardware_interface::JointStateHandle state_handle_q3("q3", &pos[2], &vel[2], &eff[2]);
  jnt_state_interface.registerHandle(state_handle_q3);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle vel_handle_q1(jnt_state_interface.getHandle("q1"), &cmd[0]);
  jnt_vel_interface.registerHandle(vel_handle_q1);

  hardware_interface::JointHandle vel_handle_q2(jnt_state_interface.getHandle("q2"), &cmd[1]);
  jnt_vel_interface.registerHandle(vel_handle_q2);

  hardware_interface::JointHandle vel_handle_q3(jnt_state_interface.getHandle("q3"), &cmd[2]);
  jnt_vel_interface.registerHandle(vel_handle_q3);
  
  registerInterface(&jnt_vel_interface);

  //----------------------------------------------------------------------------
  // Analog Output initialize
  //----------------------------------------------------------------------------
  //初期化処理
  strcpy(DeviceName, PCI_DA);
  Ret = AioInit(DeviceName, &Id_Ao);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoInit = %ld : %s", Ret, ErrorString);
		exit(0);
	}
	//デバイスのリセット
	Ret = AioResetDevice(Id_Ao);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoResetDevice = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//出力レンジの設定
	AoRange = 0; // ± 10V
	Ret = AioSetAoRangeAll(Id_Ao, AoRange);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoRangeAll = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//チャネル数の設定
	Ret = AioSetAoChannels(Id_Ao, AoChannels);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoChannels = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//メモリ形式の設定：FIFO
	Ret = AioSetAoMemoryType(Id_Ao, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoMemoryType = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//クロック種類の設定：内部
	Ret = AioSetAoClockType(Id_Ao, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoClockType = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//変換速度の設定：10usec
	Ret = AioSetAoSamplingClock(Id_Ao, 10);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoSamplingClock = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//開始条件の設定：ソフトウェア
	Ret = AioSetAoStartTrigger(Id_Ao, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoStartTrigger = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//停止条件の設定：設定回数変換終了
	Ret = AioSetAoStopTrigger(Id_Ao, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoSetAoStopTrigger = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
	//メモリのリセット
	Ret = AioResetAoMemory(Id_Ao);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AoResetAoMemory = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ao);
		exit(0);
	}
//  ROS_INFO("Analog Output initialized");


  //----------------------------------------------------------------------------
  // Analog Input initialize
  //----------------------------------------------------------------------------
  //初期化処理
  strcpy(DeviceName, PCI_AD);
	Ret = AioInit(DeviceName, &Id_Ai);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiInit = %ld : %s", Ret, ErrorString);
		exit(0);
	}
	//デバイスのリセット
	Ret = AioResetDevice(Id_Ai);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiResetDevice = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//入力レンジの設定
	AiRange = 0;	// pm10V
	Ret = AioSetAiRangeAll(Id_Ai, AiRange);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiRangeAll = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//チャネル数の設定
	Ret = AioSetAiChannels(Id_Ai, AiChannels*2+sensor);//中
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiChannels = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//メモリ形式の設定：FIFO
	Ret = AioSetAiMemoryType(Id_Ai, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiMemoryType = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//クロック種類の設定：内部
	Ret = AioSetAiClockType(Id_Ai, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiClockType = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//変換速度の設定：10usec
	Ret = AioSetAiSamplingClock(Id_Ai, 10*(AiChannels*2+sensor));//中
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiSamplingClock = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//開始条件の設定：ソフトウェア
	Ret = AioSetAiStartTrigger(Id_Ai, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiStartTrigger = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//停止条件の設定：設定回数変換終了
	Ret = AioSetAiStopTrigger(Id_Ai, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiStopTrigger = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
//  ROS_INFO("Analog Input initialized");
}

UniArm::~UniArm(){
  //----------------------------------------------------------------------------
  // Analog Output Finalize
  //----------------------------------------------------------------------------
  Ret = AioExit(Id_Ao);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    ROS_INFO("AoExit = %ld : %s", Ret, ErrorString);
    exit(0);
  }
  //リレーOFF
/*  for(i = 0 ; i < AoChannels ; i++){
    Ret = AioDisableAo(Id_Ao, i);
  }*/
//  ROS_INFO("Analog Output Finalize");

  //----------------------------------------------------------------------------
  // AD Finalize
  //----------------------------------------------------------------------------
  Ret = AioExit(Id_Ai);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiExit = %ld : %s", Ret, ErrorString);
		exit(0);
	}
//  ROS_INFO("Analog Input Finalize");

}

void UniArm::read()
{

  //----------------------------------------------------------------------------
	//	変換開始
	//----------------------------------------------------------------------------
  //サンプリング回数の設定：1回
	Ret = AioSetAiStopTimes(Id_Ai, 1);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiSetAiStopTimes = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//メモリのリセット
	Ret = AioResetAiMemory(Id_Ai);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiResetAiMemory = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}
	//変換開始
	Ret = AioStartAi(Id_Ai);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiStartAi = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}

	//	変換状態取得
	do{
		//ステータスの取得
		Ret = AioGetAiStatus(Id_Ai, &AiStatus);
		if(Ret != 0){
			AioGetErrorString(Ret, ErrorString);
			ROS_INFO("AiGetAiStatus = %ld : %s", Ret, ErrorString);
			Ret = AioExit(Id_Ai);
			exit(0);
		}
		//メモリ内格納サンプリング数の取得
		Ret = AioGetAiSamplingCount(Id_Ai, &AiSamplingCount);
		if(Ret != 0){
			AioGetErrorString(Ret, ErrorString);
			ROS_INFO("AiGetAiSamplingCount = %ld : %s", Ret, ErrorString);
			Ret = AioExit(Id_Ai);
			exit(0);
		}
		//ROS_INFO("Number of sampling: %ld  Status: %xH\r", AiSamplingCount, (unsigned int)AiStatus);
	}while((AiStatus & AIS_BUSY) == AIS_BUSY);

	//変換データ取得
	Ret = AioGetAiSamplingDataEx(Id_Ai, &AiSamplingTimes, &AiData[0]);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		ROS_INFO("AiGetAiSamplingDataEx = %ld : %s", Ret, ErrorString);
		Ret = AioExit(Id_Ai);
		exit(0);
	}

//  ROS_INFO("q1 angle %.2f  q2 angle %.2f", AiData[0],AiData[1]);

  pos[0] = AiData[1]/8*M_PI;
  pos[1] = AiData[3]/8*M_PI;
  pos[2] = AiData[5]/8*M_PI;

  vel[0] = (AiData[6]-0.991211)/0.1105*9.80665;
  vel[1] = (AiData[7]-0.97168)/0.1105*9.80665;
  ord[0] = cmd[0];
  ord[1] = cmd[1];
  ord[2] = cmd[2];

  eff[0] = AiData[0]*0.3;
  eff[1] = AiData[2]*0.3;
  eff[2] = AiData[4]*0.3;//中

}

void UniArm::write()
{
  //ROS_INFO_STREAM("q1 command " << cmd[0] << " q2 command " << cmd[1] << " q3 command " << cmd[2]);

  for(i=0; i<nLink; i++){
    AoData[i] = (float)( cmd[i] / (13*M_PI/30) );
  }

  //----------------------------------------------------------------------------
  // AD Contert
  //----------------------------------------------------------------------------
  //出力データ、サンプリング回数の設定：1回
  Ret = AioSetAoSamplingDataEx(Id_Ao, 1, &AoData[0]);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    ROS_INFO("AoSetAoSamplingDataEx = %ld : %s", Ret, ErrorString);
    Ret = AioExit(Id_Ao);
    exit(0);
  }
  //リレーON
  for(i = 0 ; i < AoChannels ; i++){
    Ret = AioEnableAo(Id_Ao, i);
  }
  //変換開始
  Ret = AioStartAo(Id_Ao);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    ROS_INFO("AoStartAo = %ld : %s", Ret, ErrorString);
    Ret = AioExit(Id_Ao);
    exit(0);
  }

  do{
    //ステータスの取得
    Ret = AioGetAoStatus(Id_Ao, &AoStatus);
    if(Ret != 0){
      AioGetErrorString(Ret, ErrorString);
      ROS_INFO("AoGetAoStatus = %ld : %s", Ret, ErrorString);
      Ret = AioExit(Id_Ao);
      exit(0);
    }
    //出力済サンプリング数の取得
    Ret = AioGetAoSamplingCount(Id_Ao, &AoSamplingCount);
    if(Ret != 0){
      AioGetErrorString(Ret, ErrorString);
      ROS_INFO("AoGetAoSamplingCount = %ld : %s", Ret, ErrorString);
      Ret = AioExit(Id_Ao);
      exit(0);
    }
  }while((AoStatus & AOS_BUSY) == AOS_BUSY);

}
