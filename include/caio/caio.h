//==============================================================================================
//	AIO用ヘッダファイル
//==============================================================================================

//----------------------------------------------------------------------------------------------
//	入出力レンジ
//----------------------------------------------------------------------------------------------
#define	PM10						0	//±10V
#define	PM5							1	//±5V
#define	PM25						2	//±2.5V
#define	PM125						3	//±1.25V
#define	PM1							4	//±1V
#define	PM0625						5	//±0.625V
#define	PM05						6	//±0.5V
#define	PM03125						7	//±0.3125V
#define	PM025						8	//±0.25V
#define	PM0125						9	//±0.125V
#define	PM01						10	//±0.1V
#define	PM005						11	//±0.05V
#define	PM0025						12	//±0.025V
#define	PM00125						13	//±0.0125V
#define	PM001						14	//±0.01V
#define	P10							50	//0～10V
#define	P5							51	//0～5V
#define	P4095						52	//0～4.095V
#define	P25							53	//0～2.5V
#define	P125						54	//0～1.25V
#define	P1							55	//0～1V
#define	P05							56	//0～0.5V
#define	P025						57	//0～0.25V
#define	P01							58	//0～0.1V
#define	P005						59	//0～0.05V
#define	P0025						60	//0～0.025V
#define	P00125						61	//0～0.0125V
#define	P001						62	//0～0.01V
#define	P20MA						100	//0～20mA
#define	P4TO20MA					101	//4～20mA
#define	P1TO5						150	//1～5V

//----------------------------------------------------------------------------------------------
//	アナログ入力イベント																
//----------------------------------------------------------------------------------------------
#define	AIE_START			0x00000002	//AD変換開始条件成立イベント
#define	AIE_RPTEND			0x00000010	//リピート終了イベント
#define	AIE_END				0x00000020	//デバイス動作終了イベント
#define	AIE_DATA_NUM		0x00000080	//指定サンプリング回数格納イベント
#define	AIE_OFERR			0x00010000	//オーバーフローイベント
#define	AIE_SCERR			0x00020000	//サンプリングクロックエラーイベント
#define	AIE_ADERR			0x00040000	//AD変換エラーイベント

//----------------------------------------------------------------------------------------------
//	アナログ出力イベント																	
//----------------------------------------------------------------------------------------------
#define	AOE_START			0x00000002	//DA変換開始条件成立イベント
#define	AOE_RPTEND			0x00000010	//リピート終了イベント
#define	AOE_END				0x00000020	//デバイス動作終了イベント
#define	AOE_DATA_NUM		0x00000080	//指定サンプリング回数出力イベント
#define	AOE_SCERR			0x00020000	//サンプリングクロックエラーイベント
#define	AOE_DAERR			0x00040000	//DA変換エラーイベント

//----------------------------------------------------------------------------------------------
//	カウンタイベント																			
//----------------------------------------------------------------------------------------------
#define	CNTE_DATA_NUM		0x00000010	//比較カウント一致イベント
#define	CNTE_ORERR			0x00010000	//カウントオーバーランイベント
#define	CNTE_ERR			0x00020000	//カウンタ動作エラー

//----------------------------------------------------------------------------------------------
//	アナログ入力ステータス																
//----------------------------------------------------------------------------------------------
#define	AIS_BUSY			0x00000001	//デバイス動作中
#define	AIS_START_TRG		0x00000002	//開始トリガ待ち
#define	AIS_DATA_NUM		0x00000010	//指定サンプリング回数格納
#define	AIS_OFERR			0x00010000	//オーバーフロー
#define	AIS_SCERR			0x00020000	//サンプリングクロックエラー
#define	AIS_AIERR			0x00040000	//AD変換エラー
#define	AIS_DRVERR			0x00080000	//ドライバスペックエラー			

//----------------------------------------------------------------------------------------------
//	アナログ出力ステータス																	
//----------------------------------------------------------------------------------------------
#define	AOS_BUSY			0x00000001	//デバイス動作中
#define	AOS_START_TRG		0x00000002	//開始トリガ待ち
#define	AOS_DATA_NUM		0x00000010	//指定サンプリング回数出力
#define	AOS_SCERR			0x00020000	//サンプリングクロックエラー
#define	AOS_AOERR			0x00040000	//DA変換エラー
#define	AOS_DRVERR			0x00080000	//ドライバスペックエラー

//----------------------------------------------------------------------------------------------
//	カウンタステータス																
//----------------------------------------------------------------------------------------------
#define	CNTS_BUSY			0x00000001	//カウンタ動作中
#define	CNTS_DATA_NUM		0x00000010	//比較カウント一致
#define	CNTS_ORERR			0x00010000	//オーバーラン
#define	CNTS_ERR			0x00020000	//カウンタ動作エラー

//----------------------------------------------------------------------------------------------
//	アナログ入力メッセージ																	
//----------------------------------------------------------------------------------------------
#define AIOM_AIE_START			0x1000	//AD変換開始条件成立イベント
#define AIOM_AIE_RPTEND			0x1001	//リピート終了イベント
#define AIOM_AIE_END			0x1002	//デバイス動作終了イベント
#define AIOM_AIE_DATA_NUM		0x1003	//指定サンプリング回数格納イベント
#define AIOM_AIE_DATA_TSF		0x1007	//指定転送数毎イベント
#define AIOM_AIE_OFERR			0x1004	//オーバーフローイベント
#define AIOM_AIE_SCERR			0x1005	//サンプリングクロックエラーイベント
#define AIOM_AIE_ADERR			0x1006	//AD変換エラーイベント

//----------------------------------------------------------------------------------------------
//	アナログ出力メッセージ																
//----------------------------------------------------------------------------------------------
#define AIOM_AOE_START			0x1020	//DA変換開始条件成立イベント
#define AIOM_AOE_RPTEND			0x1021	//リピート終了イベント
#define AIOM_AOE_END			0x1022	//デバイス動作終了イベント
#define AIOM_AOE_DATA_NUM		0x1023	//指定サンプリング回数出力イベント
#define AIOM_AOE_DATA_TSF		0x1027	//指定転送数毎イベント
#define AIOM_AOE_SCERR			0x1025	//サンプリングクロックエラーイベント
#define AIOM_AOE_DAERR			0x1026	//DA変換エラーイベント

//----------------------------------------------------------------------------------------------
//	カウンタメッセージ																	
//----------------------------------------------------------------------------------------------
#define AIOM_CNTE_DATA_NUM		0x1042	//比較カウント一致イベント
#define AIOM_CNTE_ORERR			0x1043	//カウントオーバーランイベント
#define AIOM_CNTE_ERR			0x1044	//カウント動作エラーイベント

//----------------------------------------------------------------------------------------------
//	カウンタ動作モード																
//----------------------------------------------------------------------------------------------
#define	CNT_LOADPRESET		0x0000001	//プリセットカウント値のロード
#define	CNT_LOADCOMP		0x0000002	//比較カウント値のロード

//----------------------------------------------------------------------------------------------
//	イベントコントローラ接続先信号															
//----------------------------------------------------------------------------------------------
#define	AIOECU_DEST_AI_CLK			4	//アナログ入力サンプリングクロック
#define	AIOECU_DEST_AI_START		0	//アナログ入力変換開始信号
#define	AIOECU_DEST_AI_STOP			2	//アナログ入力変換停止信号
#define	AIOECU_DEST_AO_CLK			36	//アナログ出力サンプリングクロック
#define	AIOECU_DEST_AO_START		32	//アナログ出力変換開始信号
#define	AIOECU_DEST_AO_STOP			34	//アナログ出力変換停止信号
#define	AIOECU_DEST_CNT0_UPCLK		134	//カウンタ０アップクロック信号
#define	AIOECU_DEST_CNT1_UPCLK		135	//カウンタ１アップクロック信号
#define	AIOECU_DEST_CNT0_START		128	//カウンタ０動作開始信号
#define	AIOECU_DEST_CNT1_START		129	//カウンタ１動作開始信号
#define	AIOECU_DEST_CNT0_STOP		130	//カウンタ０動作停止信号
#define	AIOECU_DEST_CNT1_STOP		131	//カウンタ１動作停止信号
#define	AIOECU_DEST_MASTER1			104	//同期バスマスタ信号１
#define	AIOECU_DEST_MASTER2			105	//同期バスマスタ信号２
#define	AIOECU_DEST_MASTER3			106	//同期バスマスタ信号３

//----------------------------------------------------------------------------------------------
//	イベントコントローラ接続元信号															
//----------------------------------------------------------------------------------------------
#define	AIOECU_SRC_OPEN				-1	//未接続
#define	AIOECU_SRC_AI_CLK			4	//アナログ入力内部クロック信号
#define	AIOECU_SRC_AI_EXTCLK		146	//アナログ入力外部クロック信号
#define	AIOECU_SRC_AI_TRGSTART		144	//アナログ入力外部トリガ開始信号
#define	AIOECU_SRC_AI_LVSTART		28	//アナログ入力レベルトリガ開始信号
#define	AIOECU_SRC_AI_STOP			17	//アナログ入力変換回数終了信号（遅延なし）
#define	AIOECU_SRC_AI_STOP_DELAY	18	//アナログ入力変換回数終了信号（遅延あり）
#define	AIOECU_SRC_AI_LVSTOP		29	//アナログ入力レベルトリガ停止信号
#define	AIOECU_SRC_AI_TRGSTOP		145	//アナログ入力外部トリガ停止信号
#define	AIOECU_SRC_AO_CLK			66	//アナログ出力内部クロック信号
#define	AIOECU_SRC_AO_EXTCLK		149	//アナログ出力外部クロック信号
#define	AIOECU_SRC_AO_TRGSTART		147	//アナログ出力外部トリガ開始信号
#define	AIOECU_SRC_AO_STOP_FIFO		352	//アナログ出力指定回数出力終了信号（FIFO使用）
#define	AIOECU_SRC_AO_STOP_RING		80	//アナログ出力指定回数出力終了信号（RING使用）
#define	AIOECU_SRC_AO_TRGSTOP		148	//アナログ出力外部トリガ停止信号
#define	AIOECU_SRC_CNT0_UPCLK		150	//カウンタ０アップクロック信号
#define	AIOECU_SRC_CNT1_UPCLK		152	//カウンタ１アップクロック信号
#define	AIOECU_SRC_CNT0_CMP			288	//カウンタ０比較カウント一致
#define	AIOECU_SRC_CNT1_CMP			289	//カウンタ１比較カウント一致
#define	AIOECU_SRC_SLAVE1			136	//同期バススレーブ信号１
#define	AIOECU_SRC_SLAVE2			137	//同期バススレーブ信号２
#define	AIOECU_SRC_SLAVE3			138	//同期バススレーブ信号３
#define	AIOECU_SRC_START			384	//Ai, Ao, Cntソフトウェア開始信号
#define	AIOECU_SRC_STOP				385	//Ai, Ao, Cntソフトウェア停止信号

//----------------------------------------------------------------------------------------------
//	関数プロトタイプ
//----------------------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"{
#endif
//共通関数
long AioInit(char *DeviceName, short *Id);
long AioExit(short Id);
long AioResetDevice(short Id);
long AioGetErrorString(long ErrorCode, char *ErrorString);
long AioQueryDeviceName(short Index, char *DeviceName, char *Device);
long AioGetDeviceType(char *Device, short *DeviceType);
long AioSetControlFilter(short Id, short Signal, float Value);
long AioGetControlFilter(short Id, short Signal, float *Value);
long AioResetProcess(short Id);

//アナログ入力関数
long AioSingleAi(short Id, short AiChannel, long *AiData);
long AioSingleAiEx(short Id, short AiChannel, float *AiData);
long AioMultiAi(short Id, short AiChannels, long *AiData);
long AioMultiAiEx(short Id, short AiChannels, float *AiData);
long AioSingleAiSR(short Id, short AiChannel, long *AiData, unsigned short *Timestamp, unsigned char Mode);
long AioSingleAiExSR(short Id, short AiChannel, float *AiData, unsigned short *Timestamp, unsigned char Mode);
long AioMultiAiSR(short Id, short AiChannels, long *AiData, unsigned short *Timestamp, unsigned char Mode);
long AioMultiAiExSR(short Id, short AiChannels, float *AiData, unsigned short *Timestamp, unsigned char Mode);
long AioGetAiResolution(short Id, short *AiResolution);
long AioSetAiInputMethod(short Id, short AiInputMethod);
long AioGetAiInputMethod(short Id, short *AiInputMethod);
long AioGetAiMaxChannels(short Id, short *AiMaxChannels);
long AioSetAiChannels(short Id, short AiChannels);
long AioGetAiChannels(short Id, short *AiChannels);
long AioSetAiChannelSequence(short Id, short AiSequence, short AiChannel);
long AioGetAiChannelSequence(short Id, short AiSequence, short *AiChannel);
long AioSetAiRange(short Id, short AiChannel, short AiRange);
long AioSetAiRangeAll(short Id, short AiRange);
long AioGetAiRange(short Id, short AiChannel, short *AiRange);
long AioGetAiMemorySize(short Id, long *AiMemorySize);
long AioSetAiMemoryType(short Id, short AiMemoryType);
long AioGetAiMemoryType(short Id, short *AiMemoryType);
long AioSetAiRepeatTimes(short Id, long AiRepeatTimes);
long AioGetAiRepeatTimes(short Id, long *AiRepeatTimes);
long AioSetAiClockType(short Id, short AiClockType);
long AioGetAiClockType(short Id, short *AiClockType);
long AioSetAiSamplingClock(short Id, float AiSamplingClock);
long AioGetAiSamplingClock(short Id, float *AiSamplingClock);
long AioSetAiScanClock(short Id, float AiScanClock);
long AioGetAiScanClock(short Id, float *AiScanClock);
long AioSetAiClockEdge(short Id, short AoClockEdge);
long AioGetAiClockEdge(short Id, short *AoClockEdge);
long AioSetAiStartTrigger(short Id, short AiStartTrigger);
long AioGetAiStartTrigger(short Id, short *AiStartTrigger);
long AioSetAiStartLevel(short Id, short AiChannel, long AiStartLevel, short AiDirection);
long AioSetAiStartLevelEx(short Id, short AiChannel, float AiStartLevel, short AiDirection);
long AioGetAiStartLevel(short Id, short AiChannel, long *AiStartLevel, short *AiDirection);
long AioGetAiStartLevelEx(short Id, short AiChannel, float *AiStartLevel, short *AiDirection);
long AioSetAiStartInRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
long AioSetAiStartInRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
long AioGetAiStartInRange(short Id, short AiChannel, long *Level1, long *Level2, long *StateTimes);
long AioGetAiStartInRangeEx(short Id, short AiChannel, float *Level1, float *Level2, long *StateTimes);
long AioSetAiStartOutRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
long AioSetAiStartOutRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
long AioGetAiStartOutRange(short Id, short AiChannel, long *Level1, long *Level2, long *StateTimes);
long AioGetAiStartOutRangeEx(short Id, short AiChannel, float *Level1, float *Level2, long *StateTimes);
long AioSetAiStopTrigger(short Id, short AiStopTrigger);
long AioGetAiStopTrigger(short Id, short *AiStopTrigger);
long AioSetAiStopTimes(short Id, long AiStopTimes);
long AioGetAiStopTimes(short Id, long *AiStopTimes);
long AioSetAiStopLevel(short Id, short AiChannel, long AiStopLevel, short AiDirection);
long AioSetAiStopLevelEx(short Id, short AiChannel, float AiStopLevel, short AiDirection);
long AioGetAiStopLevel(short Id, short AiChannel, long *AiStopLevel, short *AiDirection);
long AioGetAiStopLevelEx(short Id, short AiChannel, float *AiStopLevel, short *AiDirection);
long AioSetAiStopInRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
long AioSetAiStopInRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
long AioGetAiStopInRange(short Id, short AiChannel, long *Level1, long *Level2, long *StateTimes);
long AioGetAiStopInRangeEx(short Id, short AiChannel, float *Level1, float *Level2, long *StateTimes);
long AioSetAiStopOutRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
long AioSetAiStopOutRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
long AioGetAiStopOutRange(short Id, short AiChannel, long *Level1, long *Level2, long *StateTimes);
long AioGetAiStopOutRangeEx(short Id, short AiChannel, float *Level1, float *Level2, long *StateTimes);
long AioSetAiStopDelayTimes(short Id, long AiStopDelayTimes);
long AioGetAiStopDelayTimes(short Id, long *AiStopDelayTimes);
long AioSetAiCallBackProc(short Id,
	long (*pProc)(short Id, short AiEvent, short wParam, long lParam, void *Param), long AiEvent, void *Param);
long AioSetAiEventSamplingTimes(short Id, long AiSamplingTimes);
long AioGetAiEventSamplingTimes(short Id, long *AiSamplingTimes);
long AioStartAi(short Id);
long AioStartAiSync(short Id, long TimeOut);
long AioStopAi(short Id);
long AioGetAiStatus(short Id, long *AiStatus);
long AioGetAiSamplingCount(short Id, long *AiSamplingCount);
long AioGetAiStopTriggerCount(short Id, long *AiStopTriggerCount);
long AioGetAiRepeatCount(short Id, long *AiRepeatCount);
long AioGetAiSamplingData(short Id, long *AiSamplingTimes, long *AiData);
long AioGetAiSamplingDataEx(short Id, long *AiSamplingTimes, float *AiData);
long AioResetAiStatus(short Id);
long AioResetAiMemory(short Id);

//アナログ出力関数
long AioSingleAo(short Id, short AoChannel, long AoData);
long AioSingleAoEx(short Id, short AoChannel, float AoData);
long AioMultiAo(short Id, short AoChannels, long *AoData);
long AioMultiAoEx(short Id, short AoChannels, float *AoData);
long AioGetAoResolution(short Id, short *AoResolution);
long AioSetAoChannels(short Id, short AoChannels);
long AioGetAoChannels(short Id, short *AoChannels);
long AioGetAoMaxChannels(short Id, short *AoMaxChannels);
long AioSetAoRange(short Id, short AoChannel, short AoRange);
long AioSetAoRangeAll(short Id, short AoRange);
long AioGetAoRange(short Id, short AoChannel, short *AoRange);
long AioSetAoMemoryType(short Id, short AoMemoryType);
long AioGetAoMemoryType(short Id, short *AoMemoryType);
long AioSetAoRepeatTimes(short Id, long AoRepeatTimes);
long AioGetAoRepeatTimes(short Id, long *AoRepeatTimes);
long AioSetAoClockType(short Id, short AoClockType);
long AioGetAoClockType(short Id, short *AoClockType);
long AioSetAoSamplingClock(short Id, float AoSamplingClock);
long AioGetAoSamplingClock(short Id, float *AoSamplingClock);
long AioSetAoClockEdge(short Id, short AoClockEdge);
long AioGetAoClockEdge(short Id, short *AoClockEdge);
long AioSetAoSamplingData(short Id, long AoSamplingTimes, long *AoData);
long AioSetAoSamplingDataEx(short Id, long AoSamplingTimes, float *AoData);
long AioGetAoSamplingTimes(short Id, long *AoSamplingTimes);
long AioSetAoStartTrigger(short Id, short AoStartTrigger);
long AioGetAoStartTrigger(short Id, short *AoStartTrigger);
long AioSetAoStopTrigger(short Id, short AoStopTrigger);
long AioGetAoStopTrigger(short Id, short *AoStopTrigger);
long AioSetAoCallBackProc(short Id,
	long (*pProc)(short Id, short AoEvent, short wParam, long lParam, void	*Param), long AoEvent, void	*Param);
long AioSetAoEventSamplingTimes(short Id, long AoSamplingTimes);
long AioGetAoEventSamplingTimes(short Id, long *AoSamplingTimes);
long AioStartAo(short Id);
long AioStopAo(short Id);
long AioEnableAo(short Id, short AoChannel);
long AioDisableAo(short Id, short AoChannel);
long AioGetAoStatus(short Id, long *AoStatus);
long AioGetAoSamplingCount(short Id, long *AoSamplingCount);
long AioGetAoRepeatCount(short Id, long *AoRepeatCount);
long AioResetAoStatus(short Id);
long AioResetAoMemory(short Id);

//デジタル入出力関数
long AioSetDiFilter(short Id, short Bit, float Value);
long AioGetDiFilter(short Id, short Bit, float *Value);
long AioInputDiBit(short Id, short DiBit, short * DiData);
long AioOutputDoBit(short Id, short DoBit, short DoData);
long AioInputDiByte(short Id, short DiPort, short * DiData);
long AioOutputDoByte(short Id, short DoPort, short DoData);

//カウンタ関数
long AioGetCntMaxChannels(short Id, short *CntMaxChannels);
long AioSetCntComparisonMode(short Id, short CntChannel, short CntMode);
long AioGetCntComparisonMode(short Id, short CntChannel, short *CntMode);
long AioSetCntPresetReg(short Id, short CntChannel, long PresetNumber, long *PresetData, short Flag);
long AioSetCntComparisonReg(short Id, short CntChannel, long ComparisonNumber, long *ComparisonData, short Flag);
long AioSetCntInputSignal(short Id, short CntChannel, short CntInputSignal);
long AioGetCntInputSignal(short Id, short CntChannel, short *CntInputSignal);
long AioSetCntCallBackProc(short Id, short CntChannel,
	long (*pProc)(short Id, short CntEvent, short wParam, long lParam, void	*Param), long CntEvent, void *Param);
long AioSetCntFilter(short Id, short CntChannel, short Signal, float Value);
long AioGetCntFilter(short Id, short CntChannel, short Signal, float *Value);
long AioStartCnt(short Id, short CntChannel);
long AioStopCnt(short Id, short CntChannel);
long AioPresetCnt(short Id, short CntChannel, long PresetData);
long AioGetCntStatus(short Id, short CntChannel, long *CntStatus);
long AioGetCntCount(short Id, short CntChannel, long *Count);
long AioResetCntStatus(short Id, short CntChannel, long CntStatus);

//イベントコントローラ
long AioSetEcuSignal(short Id, short Destination, short Source);
long AioGetEcuSignal(short Id, short Destination, short *Source);

#ifdef __cplusplus
};
#endif
