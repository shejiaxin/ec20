#ifndef __KSL_PLATFORM_H
#define __KSL_PLATFORM_H

#include "includes.h"

#ifdef	APP_KSL_PLATFORM_GLOBALS
#define	APP_KSL_EXT
#else
#define	APP_KSL_EXT	extern
#endif

#define  MAX_GENERAL_BUFFER_LEN       280 

#define  CUT_POWER_ALERT       0    //＃定义断电警示0
#define  FENCE_IN_ALERT        1    //＃定义在Alert 1围栏
#define  SOS_HELP_ALERT        2    //＃定义SOS救援中心2
#define  CAR_ALERT             3    //＃定义汽车警报3
#define  CAR_LOW_SPEED_ALERT   4    //＃定义汽车低速警示4
#define  CAR_HIGH_SPEED_ALERT  5    //＃定义车高速警示5
#define  FENCE_OUT_ALERT       6    //＃定义输出警示6围栏
#define  VIBRATION_ALERT       7    //＃定义震动器报警7

#define  PLATFORM_OFFSET_BIT   16
#define  COMMAND_OFFSET_BIT   20


APP_KSL_EXT uint32_t  GprsConnectTimeOut;// = THIRTY_MINUTE; 

#define  MAX_ALERT_SUM         8

typedef struct tagStructAlertData
{
    uint8_t         AlertFlag;
    uint16_t        AlertTimeoutCnt;
    uint8_t         AlertSum;
}StructAlertData;

typedef struct tagStructPlatformData
{
    StructAlertData  AlertType[MAX_ALERT_SUM];
    uint8_t          HaveAlertFlag;
    uint8_t          PlatformSendFailCnt;
    uint32_t         DeviceConnectCheckTimeCnt;
    uint8_t          HandshakeFailCnt;
    uint32_t         AnewConnectTimeCnt;
    uint8_t          QuerySoftVerFlag;   
    uint8_t          SendGpsDataType;
    uint16_t         ResendFailPacketRemainSeconds;
    uint16_t         SendRecordDataRemainSeconds;
    uint8_t          ResendPacketFlag;
}StructPlatformData_HY;


#define  GPS_DATA_TYPE_RECORD   1
#define  GPS_DATA_TYPE_NEW      2
#define  GPS_DATA_TYPE_RESEND   3

typedef struct
{
    uint8_t  device_ok;
    uint8_t  net_work;  
    uint8_t  connect_ok;
    uint8_t  rcv_error;
    uint8_t  send_fail_cnt;     //发送失败计数，大于5次重新初始化模块
    uint8_t  heartbeat_cnt;     //心跳包失败计数，大于5次重启模块
    uint16_t send_success_sum;  //发送成功总计数
    uint16_t send_fail_sum;     //发送失败总计数
} gprs_cfg_t;

extern gprs_cfg_t gprs_cfg;

//获取位置数据结构
typedef struct tagStructGetPosition
{
    uint16_t      RequestTimeoutCnt;
    uint8_t       RequestFlag;
    uint8_t       RequestTelephone[MAX_TELEPHONE_NUMBER_LEN];
}StructGetPosition;

//平台数据结构
typedef struct tagStructPlatform
{
    uint8_t       PlatformIpAddr[4];
#ifdef DOUBLE_IP_FUN 
    uint8_t       BackupIpAddr[4]; 
    uint8_t       MainBackupIPFlag;   //双ip切换标志，0=主Ip,1=备用ip
    uint32_t      AutoSwitchMainIpTime; //切到备用IP后超时需要切回主IP计时，默认1小时
#endif
    uint16_t      PlatformPort;
    uint8_t       PlatformDomainName[MAX_DOMAIN_NAME_LEN];//30 
    uint8_t       PlatformAddrFlag;//平台地址标志，0=IP,1=域名
    uint16_t      SendFailCnt;
    uint16_t      StatisticSendFailSum;
    uint16_t      StatisticSendSuccessSum;
    uint8_t       DeviceID[16];
    uint8_t       DeviceRegisterSuccess;
    uint16_t      DeviceRegisterTimeoutCnt;
}StructPlatformData;

APP_KSL_EXT uint8_t  ValidShakingFlag;
APP_KSL_EXT uint8_t  u8WheelRolling;
APP_KSL_EXT uint8_t  u8WheelAlarmFlag;
APP_KSL_EXT uint16_t u16WheelRollingDelay;

//APP_KSL_EXT uint16_t HeartbeatPacketsTimeCnt;  //心跳包发送间隔1分钟
//APP_KSL_EXT uint8_t  DormancyPackageSendCnt;

APP_KSL_EXT uint16_t AlarmDelayCheckGsensor;
APP_KSL_EXT uint8_t  AlartLockedFlag;    
APP_KSL_EXT uint8_t  NeddSleepFlag;
APP_KSL_EXT uint8_t  AccoffWakeupFlag;

APP_KSL_EXT uint8_t  GeneralBuffer[MAX_GENERAL_BUFFER_LEN]; 

APP_KSL_EXT StructPlatformData_HY PlatformData_HY;

APP_KSL_EXT StructGetPosition GetPositionData;

APP_KSL_EXT StructPlatformData PlatformData;

APP_KSL_EXT uint16_t PlatformRcvDataPtr;


//*************** 函数原型 ****************
void PlatformInit(void);
void PlatformRcvPacketProc(void);
void DeviceRegisterToPlatform(void);
void SendGeoToPlatform_Special(void);

uint8_t SendTcpPacket(uint8_t *pPacketBuffer, uint8_t PacketLen); 
void PlatformMessageProc(uint8_t *pMsgBuffer);
uint8_t SendGpsPacket(void);
uint8_t SendMessageBP05(void);
uint8_t SendMessageBP06(void); 
#ifdef ENABLE_AGPS_DOWN 
uint8_t SendMessageBP07(void); 
#endif
uint8_t SendMessageBP11(void);
uint8_t SendMessageBP15(void);

uint8_t SendMessageBR00(void); 
uint8_t SendMessageBP19(void);
uint8_t SendMessageBR01(void);
uint8_t SendMessageBO02(void);
uint8_t SendMessageBR04(void);
uint8_t SendMessageBR07(uint8_t ctrl_num);
uint8_t SendMessageBR08(uint8_t bms_num);
uint8_t SendMessageBR09(void);

uint32_t ChangeRemCtrID(uint8_t *pInitValue,uint8_t num);
    
void SendMessageBR02(void);
void SendMessageBO01(uint8_t Code);
void SendMessageBP00(void);   
void SendMessageBP25(void);
void SendMessageBR03(void);
void SendSoftWareVersion(void);

void     WriteDeviceIdTobuffer(void);
uint8_t  SendScreenMsgToCenter(uint8_t *pDataBuf, 
                                     uint8_t DataLen);
uint16_t  GenerateGprsPacket(uint8_t *pDataBuf, uint16_t DataLen);
uint8_t SendMessageStr(uint8_t* pStr);
void SendGeoToPlatform(void);

#endif  //__KSL_PLATFORM_H
