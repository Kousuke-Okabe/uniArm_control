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
//AIOヘッダファイル
#include "caio.h"

#define nLink	3
#define PCI_DA "contec_Ao"

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
  double vel[3];
  double eff[3];

  // DA
	long	Ret;                      //関数の戻り値
	char	DeviceName[256] = PCI_DA; //デバイス名
	short	Id;                       //ID
	char	ErrorString[256];         //エラーコード文字列
	short	AoRange;                  //レンジ
	short	AoChannels = nLink;       //使用チャネル数
	float	AoVolt;                   //変換データ(電圧)
	float	AoData[nLink];            //変換データ
	long	AoSamplingCount;	        //現在のサンプリング回数
	long	AoStatus;                 //現在のステータス
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
  // DA converter initialize
  //----------------------------------------------------------------------------
  Ret = AioInit(DeviceName, &Id);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioInit = %ld : %s\n\n", Ret, ErrorString);
		exit(0);
	}
	//デバイスのリセット
	Ret = AioResetDevice(Id);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioResetDevice = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//出力レンジの設定
	AoRange = 0; // ± 10V
	Ret = AioSetAoRangeAll(Id, AoRange);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoRangeAll = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//チャネル数の設定
	Ret = AioSetAoChannels(Id, AoChannels);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoChannels = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//メモリ形式の設定：FIFO
	Ret = AioSetAoMemoryType(Id, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoMemoryType = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//クロック種類の設定：内部
	Ret = AioSetAoClockType(Id, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoClockType = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//変換速度の設定：10usec
	Ret = AioSetAoSamplingClock(Id, 10);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoSamplingClock = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//開始条件の設定：ソフトウェア
	Ret = AioSetAoStartTrigger(Id, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoStartTrigger = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//停止条件の設定：設定回数変換終了
	Ret = AioSetAoStopTrigger(Id, 0);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioSetAoStopTrigger = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
	//メモリのリセット
	Ret = AioResetAoMemory(Id);
	if(Ret != 0){
		AioGetErrorString(Ret, ErrorString);
		printf("AioResetAoMemory = %ld : %s\n\n", Ret, ErrorString);
		Ret = AioExit(Id);
		exit(0);
	}
  //出力データ、サンプリング回数の設定：nLink=3?
  Ret = AioSetAoSamplingDataEx(Id, (long)AoChannels, &AoData[0]);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    printf("AioSetAoSamplingDataEx = %ld : %s\n\n", Ret, ErrorString);
    Ret = AioExit(Id);
    exit(0);
  }

  printf("AD converter initialized \n");
}

UniArm::~UniArm()
{
  //----------------------------------------------------------------------------
  // DA Ternimalize
  //----------------------------------------------------------------------------
  Ret = AioExit(Id);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    printf("AioExit = %ld : %s\n\n", Ret, ErrorString);
    exit(0);
  }
}

void UniArm::read()
{
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = 0.0;
  vel[0] = 0.0;
  vel[1] = 0.0;
  vel[2] = 0.0;
  eff[0] = 0.0;
  eff[1] = 0.0;
  eff[2] = 0.0;
}

void UniArm::write()
{
  //ROS_INFO_STREAM("q1 command " << cmd[0] << " q2 command " << cmd[1] << " q3 command " << cmd[2]);

  for(i=0; i<nLink; i++){
    AoData[i] = (float)cmd[i];
  }

  //----------------------------------------------------------------------------
  // AD Contert
  //----------------------------------------------------------------------------
  //出力データ、サンプリング回数の設定：nLink=3?
  Ret = AioSetAoSamplingDataEx(Id, 1, &AoData[0]);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    printf("AioSetAoSamplingDataEx = %ld : %s\n\n", Ret, ErrorString);
    Ret = AioExit(Id);
    exit(0);
  }

  //リレーON
  for(i = 0 ; i < AoChannels ; i++){
    Ret = AioEnableAo(Id, i);
  }

  //変換開始
  Ret = AioStartAo(Id);
  if(Ret != 0){
    AioGetErrorString(Ret, ErrorString);
    printf("AioStartAo = %ld : %s\n\n", Ret, ErrorString);
    Ret = AioExit(Id);
    exit(0);
  }

  do{
    //ステータスの取得
    Ret = AioGetAoStatus(Id, &AoStatus);
    if(Ret != 0){
      AioGetErrorString(Ret, ErrorString);
      printf("AioGetAoStatus = %ld : %s\n\n", Ret, ErrorString);
      Ret = AioExit(Id);
      exit(0);
    }
    //出力済サンプリング数の取得
    Ret = AioGetAoSamplingCount(Id, &AoSamplingCount);
    if(Ret != 0){
      AioGetErrorString(Ret, ErrorString);
      printf("AioGetAoSamplingCount = %ld : %s\n\n", Ret, ErrorString);
      Ret = AioExit(Id);
      exit(0);
    }
  }while((AoStatus & AOS_BUSY) == AOS_BUSY);

}
