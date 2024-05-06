#ifndef __4G_EC20_H
#define __4G_EC20_H

#include "includes.h"

//通信模块状态检测脚定义
#define  GSM_STATUS_CHECK_PORT      GPIOB
#define  GSM_STATUS_CHECK_BIT       GPIO_Pin_15  //pb13-->pb15 
#define  GetGsmStatusState()        GPIO_ReadInputDataBit(GSM_STATUS_CHECK_PORT, GSM_STATUS_CHECK_BIT)
#define  GSM_STATUS_NORMAL          Bit_SET

//4G模块电源控制脚定义
#define  GSM_POWER_CTRL_PORT        GPIOB    
#define  GSM_POWER_CTRL_BIT         GPIO_Pin_9  
#define  OpenGsmPower()             GPIO_SetBits(GSM_POWER_CTRL_PORT, GSM_POWER_CTRL_BIT)
#define  CloseGsmPower()            GPIO_ResetBits(GSM_POWER_CTRL_PORT, GSM_POWER_CTRL_BIT)
#define  GetGsmPwrState()           GPIO_ReadInputDataBit(GSM_POWER_CTRL_PORT, GSM_POWER_CTRL_BIT)
#define  GSM_POWER_NORMAL           Bit_SET

//4G模块开机控制脚定义
#define  EC20_PWR_KEY_PORT          GPIOB    
#define  EC20_PWR_KEY_BIT           GPIO_Pin_14
#define  Ec20PwrKeyOutputLow()      GPIO_ResetBits(EC20_PWR_KEY_PORT, EC20_PWR_KEY_BIT)  
#define  Ec20PwrKeyOutputHigh()     GPIO_SetBits(EC20_PWR_KEY_PORT, EC20_PWR_KEY_BIT)


//任务优先级
#define GPRS_SEND_TASK_PRIO                    6
#define GPRS_POLL_TASK_PRIO                    7
#define GPRS_RECV_TASK_PRIO                    8 
#define GPRS_PLATFORM_TASK_PRIO                9

//任务堆栈大小
#define GPRS_POLL_TASK_STK_SIZE                500  
#define GPRS_SEND_TASK_STK_SIZE                500  
#define GPRS_RECV_TASK_STK_SIZE                500  
#define GPRS_PLATFORM_TASK_STK_SIZE            500  

//Socket网络状态
#define GPRS_SOCKET_INITIAL      0
#define GPRS_SOCKET_OPENING      1
#define GPRS_SOCKET_CONNECTED    2
#define GPRS_SOCKET_LISTENING    3
#define GPRS_SOCKET_CLOSE        4

//网络标识
#define NET_COPS_2G              2
#define NET_COPS_3G              3
#define NET_COPS_4G              4

typedef enum
{
    GPRS_NULL = 0,
    GPRS_POWER_OFF,
    GPRS_POWER_ON,
    GPRS_AT_SYNC,
    GPRS_INIT,
    GPRS_CHK_NET_STATE,
    GPRS_CHK_CONNECT_STATE,
    GPRS_CONNECT,
    GPRS_SHUT,
    GPRS_SEND_DATA,
}gprs_state_t;

typedef enum
{
    GPRS_ILDE_CHECK_STE = 0,
    GPRS_ILDE_CHECK_CSQ,
    GPRS_ILDE_CHECK_CBC,
    GPRS_ILDE_GPS_RMC,      //获取RMC数据
    GPRS_ILDE_GPS_GGA,      //获取GGA数据
    GPRS_ILDE_CHECK_SMS,    //空闲查询单条短信
    GPRS_ILDE_CHECK_COPS,   //空闲查询运营商和当前网络是4/3/2G?
}gprs_ilde_check_t;


extern at_t atCmd;
extern char *cmd_resp;
extern uint16_t resp_len;

void ec20_hw_init(void);
void gprs_pwr_off(void);
void Gprs_Poll_Task(void *p_arg);
void Gprs_Send_Task(void *p_arg);
void Gprs_rev_task(void *p_arg);
void ec20_task_creat(void);
void gprs_set_state(gprs_state_t state);
uint8_t gprs_send(uint8_t *data, uint16_t len);
gprs_state_t gprs_get_state(void);
unsigned char SendSms(unsigned char *pSendText, unsigned char *pSendNumber);
void ReadSms(uint8_t *TempBuf, unsigned char ucSmsPos);
void DeleteSms(unsigned char ucSmsPos);
void RcvSmsProc(unsigned char ucSmsPos);
void gprs_write(uint8_t *buf, uint16_t len);
void CallProc(unsigned char *pStrStart, unsigned char *pStrEnd);

uint8_t connect_agps_server(void);
uint8_t request_agps_from_server(void);
uint8_t agps_close(void);
#endif //__4G_BC20_H

