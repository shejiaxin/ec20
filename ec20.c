#include "includes.h"


#ifdef USE_EC20_MODULE


//任务控制块
static OS_TCB   GprsRecvTaskTCB;
static OS_TCB   GprsPlatformTaskTCB;
static OS_TCB   GprsPollTaskTCB;
static OS_TCB   GprsSendTaskTCB;

CPU_STK Gprs_Poll_Task_Stk[GPRS_POLL_TASK_STK_SIZE];
CPU_STK Gprs_Send_Task_Stk[GPRS_SEND_TASK_STK_SIZE];
CPU_STK Gprs_Recv_Task_Stk[GPRS_RECV_TASK_STK_SIZE];
CPU_STK Gprs_platform_Task_Stk[GPRS_PLATFORM_TASK_STK_SIZE];

static gprs_state_t gprs_state = GPRS_POWER_ON;
static uint8_t gprs_init_step = 0;
char *pServer = "119.123.124.61";
uint16_t pPort = 8832;
char *cmd_resp = NULL;        
uint16_t resp_len = 0;
static char *cmd_hdl = NULL;  
static uint8_t ec200t_flag = FALSE; 


/*
*********************************************************************************************************
*	函 数 名: ec20_hw_init
*	功能说明: ec20硬件io初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void  ec20_hw_init(void)
{
    GPIO_InitTypeDef  GpioInit;
    
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
    
    // 4G模块状态检测脚初始化 【模块开机输出高】
    GpioInit.GPIO_Pin   = GSM_STATUS_CHECK_BIT;
    GpioInit.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GSM_STATUS_CHECK_PORT, &GpioInit);
     
    // 4G模块电源控制脚初始化 【高有效】
    GpioInit.GPIO_Pin   = GSM_POWER_CTRL_BIT;
    GpioInit.GPIO_Mode  = GPIO_Mode_Out_PP;
    GpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GSM_POWER_CTRL_PORT, &GpioInit);
    CloseGsmPower(); 

    // 4G模块开机控制脚初始化 【高有效】
    GpioInit.GPIO_Pin   = EC20_PWR_KEY_BIT;
    GpioInit.GPIO_Mode  = GPIO_Mode_Out_PP;
    GpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EC20_PWR_KEY_PORT, &GpioInit);
    Ec20PwrKeyOutputLow();

    bsp_InitUart(GPRS_COM,115200);
    McuDMAChannel_Config(RCC_AHBPeriph_DMA1,DMA1_Channel7,(uint32_t)&USART2->DR,(uint32_t)AtCmdcom.AtTxBuf,\
                         DMA_DIR_PeripheralDST,DMA_Mode_Normal,QUEUE_LENGTH);
    //McuDMAChannel_Config(DMA1_Channel5,(uint32_t)&USART2->RDR,(uint32_t)AtCmdcom.AtRxBuf,\
    //                     DMA_DIR_PeripheralSRC,DMA_Mode_Normal,QUEUE_LENGTH);
                              
    //DMA_Cmd(DMA1_Channel5, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: gprs_set_state
*	功能说明: 设置gprs状态机
*	形    参：void
*	返 回 值: void
*********************************************************************************************************
*/
void gprs_set_state(gprs_state_t state)
{
    gprs_state = state;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_get_state
*	功能说明: 查询gprs状态机
*	形    参：void
*	返 回 值: void
*********************************************************************************************************
*/
gprs_state_t gprs_get_state(void)
{
    return gprs_state;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_write
*	功能说明: 向gprs模块发送数据，通过信号量约束
*	形    参：buf： 指向需要发送的缓存，len：内容长度
*	返 回 值: void
*********************************************************************************************************
*/
void gprs_write(uint8_t *buf, uint16_t len)
{
    OS_ERR err;
    
    OSSemPend((OS_SEM      *)&gprs_send_sem,     
              (OS_TICK      )0,                   
              (OS_OPT       )OS_OPT_PEND_BLOCKING, 
              (CPU_TS      *)0,                   
              (OS_ERR      *)&err);         
 
#ifdef DEBUG_OUTPUT 
    //printf("%s",buf);
    DEBUG_UART("%s",buf);
#endif
    //DEBUG_UART("%s",buf);
    
    gprs_dma_write(buf, len);
}

/*
*********************************************************************************************************
*	函 数 名: gprs_check
*	功能说明: 应答检索，通过信号量约束
*	形    参：res： 指令返回参考指针，time_out：超时时间
*	返 回 值: 0：成功，1：超时
*********************************************************************************************************
*/
int gprs_check(char *res, uint16_t time_out)
{
	static uint8_t rcv_err_cnt = 0,cnt=0;
    uint8_t i;
    uint32_t temp_time_out = time_out*2;
    OS_ERR err;
    cmd_hdl = res; 

    
    if(temp_time_out > 1000)
    {
        if(temp_time_out % 1000)
        {
           cnt = (temp_time_out / 1000)+1; 
        }
        else
        {
            cnt = temp_time_out / 1000;
        }
        for(i = 0; i < (cnt); i++)
        {
            OSSemPend((OS_SEM  *)&gprs_check_sem,
                      (OS_TICK  )1000,
                      (OS_OPT   )OS_OPT_PEND_BLOCKING,
                      (CPU_TS  *)0,
                      (OS_ERR  *)&err);
            if(err == OS_ERR_NONE)
            {
                return 0;
            }
            else if(err == OS_ERR_PEND_ABORT)
            {
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\n[ec20]===pend abort!!!===\r\n");
                #endif
                return 1;
            }
            
            #ifdef ENABLE_IDWG
            OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                        (OS_FLAGS      )GPRS_POLL_TASK_FEED_DOG,  /* 设置bit3 */
                        (OS_OPT        )OS_OPT_POST_FLAG_SET,
                        (OS_ERR       *)&err);
            OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                        (OS_FLAGS      )GPRS_SEND_TASK_FEED_DOG,  /* 设置bit4 */
                        (OS_OPT        )OS_OPT_POST_FLAG_SET,
                        (OS_ERR       *)&err);
            #endif
        }
        if(temp_time_out % 1000 != 0)
        {
            OSSemPend((OS_SEM  *)&gprs_check_sem,
                      (OS_TICK  )temp_time_out%1000,
                      (OS_OPT   )OS_OPT_PEND_BLOCKING,
                      (CPU_TS  *)0,
                      (OS_ERR  *)&err);
            if(err == OS_ERR_NONE)
            {
                return 0;
            }
        }
        
    }
    else  //要等的时间小于1000
    {
        OSSemPend((OS_SEM  *)&gprs_check_sem,
                  (OS_TICK  )temp_time_out,
                  (OS_OPT   )OS_OPT_PEND_BLOCKING,
                  (CPU_TS  *)0,
                  (OS_ERR  *)&err);
    }
    //cmd_hdl = NULL;
    
    if(err == OS_ERR_NONE)
    {
        rcv_err_cnt = 0;
        return 0;
    }
    else if(err == OS_ERR_TIMEOUT)
    {
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\n===at cmd timeout!!!===\r\n");
        #endif
		rcv_err_cnt++;
		if(rcv_err_cnt > 5)//连续5次AT无应答，重启模块
		{
            DEBUG_PRINT("\r\n===timeout 5 at no response!!!===\r\n");

            rcv_err_cnt = 0;
            gprs_cfg.connect_ok = 0;
			gprs_set_state(GPRS_POWER_OFF);
		}
        return 1;
    }
    else
    {
        DEBUG_PRINT("\r\n===exit wait!!!===\r\n");

        return 1;
    }
}

/*
*********************************************************************************************************
*	函 数 名: gprs_cmd_handle
*	功能说明: 响应内容处理，判断是否期望应答
*	形    参：cmd： 指向应答内容,cmd_resp:全局变量，指向应答内容，用于对接收内容处理
*	返 回 值: void
*********************************************************************************************************
*/
static void gprs_cmd_handle(char *cmd,uint16_t len)
{
    OS_ERR err;
    
    if (cmd_hdl)
    {
        if (strstr(cmd, cmd_hdl) != NULL)
        {
            cmd_hdl = NULL;
            cmd_resp = cmd; //指向AT响应返回的内容
            resp_len = len;

            OSSemPost(&gprs_check_sem,OS_OPT_POST_1,&err);
        }
    }
}

at_t atCmd = 
{
    gprs_check,
    gprs_write
};

#define AT_CMD(a, b, c) at_cmd(&atCmd, a, b, c)

/*
*********************************************************************************************************
*	函 数 名: ec20_pwr_on 
*	功能说明: ec20模块上电开机
*	形    参：void
*	返 回 值: void
*********************************************************************************************************
*/
static void ec20_pwr_on(void)
{
    uint8_t i, dly_cnt = 12;
    OS_ERR err;
    
    if (GetGsmPwrState() != GSM_POWER_NORMAL)
    {
        delay_ms(1000);
        OpenGsmPower();
        delay_ms(1000);
        Ec20PwrKeyOutputHigh();        
        delay_ms(1500);
        for(i = 0; i < 40; i++)
        {
            if (GetGsmStatusState() == GSM_STATUS_NORMAL)
            {
                DEBUG_PRINT("\r\n==ec20 power-on success!!!==\r\n");

                break;
            }
            delay_ms(100);
        }
        
//        while (GetGsmStatusState() != GSM_STATUS_NORMAL)
//        {
//            delay_ms(100);
//        }
        Ec20PwrKeyOutputLow();

        DEBUG_PRINT("\r\n[ INFO ]EC20 module Initialize.......Please wait!\r\n");

		while((DeviceInfo.CpinReadyFlag == FALSE) && (dly_cnt > 0))
		{
			dly_cnt--;

			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_DLY,&err);
			
			#ifdef ENABLE_IDWG
	        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
	                    (OS_FLAGS      )GPRS_POLL_TASK_FEED_DOG,  /* set bit3 */
	                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
	                    (OS_ERR       *)&err);
	        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
	                    (OS_FLAGS      )GPRS_SEND_TASK_FEED_DOG,  /* set bit4 */
	                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
	                    (OS_ERR       *)&err);
	        #endif
		}

		if(DeviceInfo.CpinReadyFlag)
			DEBUG_PRINT("\r\n[ INFO ]EC20 module startup ok!\r\n");

   } 
}

/*
*********************************************************************************************************
*	函 数 名: gprs_pwr_off 
*	功能说明: gprs模块关机
*	形    参：void
*	返 回 值: void
*********************************************************************************************************
*/
void gprs_pwr_off(void)
{   
    #ifdef ENABLE_IDWG
    OS_ERR err;
    #endif
    uint8_t i;

    if (GetGsmPwrState() == GSM_POWER_NORMAL)//&&(GetGsmStatusState() == GSM_STATUS_NORMAL))
    {
        #ifdef ENABLE_IDWG
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )GPRS_POLL_TASK_FEED_DOG,  /* 设置bit3 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )GPRS_SEND_TASK_FEED_DOG,  /* 设置bit4 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        #endif
       
//        #ifdef ENABLE_PHONE_CALL 
//        #ifdef ENABLE_IDWG
//        OSFlagPost(FLAG_TaskRunStatus,(OS_FLAGS)GPRS_SEND_TASK_FEED_DOG,OS_FLAG_SET,&err); /* 设置bit4 */
//        #endif
//        #endif
        
        delay_ms(1000);
        Ec20PwrKeyOutputHigh();
        delay_ms(1500);
        for(i = 0; i < 40; i++)
        {
            if (GetGsmStatusState() != GSM_STATUS_NORMAL)
            {
                DEBUG_PRINT("\r\n==ec20 power-off success!!!==\r\n");
                break;
            }
            delay_ms(100);
        }
        Ec20PwrKeyOutputLow();
        
        delay_ms(1000);
        CloseGsmPower();
        delay_ms(3000);
    }
}

/*
*********************************************************************************************************
*	函 数 名: WriteApnToModule
*	功能说明: 设置APN
*	形    参: void
*	返 回 值: 0--OK,1--Fail
*********************************************************************************************************
*/
uint8_t WriteApnToModule(void)
{
    uint8_t result = 1;
    uint8_t cmd_buf[100];
    //OS_ERR err;
    
    //AT+QICSGP=1,1,"UNINET","","",1
    //AT+QICSGP=1,1,"CMIOT","","",1
    /*OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);*/
    memset(cmd_buf,0,100);
    snprintf((char*)cmd_buf, sizeof(cmd_buf), "AT+QICSGP=1,1,\"");
    strncat((char*)cmd_buf,(const char*)DeviceData.GprsApn,sizeof(DeviceData.GprsApn));
    strncat((char*)cmd_buf,"\",\"\",\"\",1\r",11);
    
    if(AT_OK == AT_CMD((char*)cmd_buf, "OK", 1000))
    {
        result = 0;
    }
    else 
    {
        DEBUG_PRINT("\r\nEC20_APN_fail!!!");  
        result = 1;
        
    }   

    delay_ms(10);
    //OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return result;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_init 
*	功能说明: gprs模块初始化
*	形    参：void
*	返 回 值: 返回0初始化成功
*********************************************************************************************************
*/
static uint8_t gprs_init(void)
{
    OS_ERR err;
    uint8_t res = 1,i;
    uint8_t *p2,*p3,tmp;
    uint8_t SaveSerialNumber[15], ReadSerialNumber[16];
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);

    switch (gprs_init_step)
    {
    case 0:
        if (AT_CMD("ATE0\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;    
    case 1:
        if (AT_CMD("AT+IPR=115200\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
    case 2:
        if (AT_CMD("ATI\r", "OK", 200) == AT_OK)
        {
            if(strstr((const char*)cmd_resp,"EC200T") != NULL) 
            {
				DEBUG_PRINT("=== ec200t_flag ===\r\n");
                ec200t_flag = TRUE;
            }
                    
            gprs_init_step++;
        }
        break;
    case 3:
        if (AT_CMD("AT+CPIN?\r", "READY", 200) == AT_OK)
            gprs_init_step++;
        break;
//    case 4:
//        if(AT_CMD("AT+QGNSSC?\r", "+QGNSSC: 0", 100) == AT_OK)
//        {
//            if (AT_CMD("AT+QGNSSC=1\r", "OK", 1000) == AT_OK) 
//                gprs_init_step++;
//        }
//        else
//            gprs_init_step++; 
//        break;
//    case 5:
//        if (AT_CMD("AT+QGNSSDB=1\r", "OK", 100) == AT_OK) 
//            gprs_init_step++;
//        break;
//	case 6: 
//		if (AT_CMD("AT+QGNSSCMD=0,\"$PDTINFO\"\r", "OK", 100) == AT_OK) 
//            gprs_init_step++;
//		break;
    case 4:
        if (AT_CMD("AT+CREG=1\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
    
    case 5:
        if (AT_CMD("AT+CGREG=1\r", "OK", 200) == AT_OK) 
            gprs_init_step++;
        break;
    case 6:
        //if (AT_CMD("AT+CLIP=1\r", "OK", 200) == AT_OK) 
            gprs_init_step++;
        break;
    case 7:
        if (AT_CMD("AT+IPR=115200\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
    
    case 8:
        if (AT_CMD("AT+GSN\r", "86", 200) == AT_OK)
        {
            p2 = (uint8_t*)cmd_resp;
            if(p2) 
            {
                p3=(uint8_t*)strstr((const char*)p2,"86");
                
                strncpy((char*)ReadSerialNumber,(char*)p3,15);
                
                #ifdef DEBUG_OUTPUT0
                Debug_PrintData("Read IMEI:", ReadSerialNumber,ReadSerialNumber+15);
                #endif

                if (IsNumber(ReadSerialNumber, 15) != TRUE)
                {
                    break;
                }
                
                for (i = 0; i < PLATFORM_DEVICE_ID_LEN; i ++)
                {
                    PlatformData.DeviceID[i] = ReadSerialNumber[i];
                }
                PlatformData.DeviceID[15] = 0;
                
                if (ReadImei(SaveSerialNumber) == TRUE)
                {
                    /*if (CompareStr(SaveSerialNumber, ReadSerialNumber, 15) == FALSE)
                    {
                        #ifdef DEBUG_OUTPUT
                        DEBUG_PRINT("IMEI compare fail! %s", SaveSerialNumber);
                        #endif

                        return FALSE_Bit;
                    }*/
                }
                else
                {
                    #ifdef DEBUG_OUTPUT0
                    DEBUG_PRINT("first read IMEI.\r\n");
                    #endif

                    SaveImei(ReadSerialNumber);
                    PlatformInit();  
                }
                
                gprs_init_step++;
            }
        }
        break;
    case 9:
        if(AT_CMD("AT+QCCID\r","89",200) == AT_OK)
        {
            p2 = (uint8_t*)cmd_resp;
            if(p2) 
            {
                p3=(uint8_t*)strstr((const char*)p2,"89");
                
                strncpy((char*)DeviceInfo.SimCCIDNumber,(char*)p3,20);
                DeviceInfo.SimCCIDNumber[20] = '\0';
                
                #ifdef DEBUG_OUTPUT0
                Debug_PrintData("Read IMEI:", DeviceInfo.SimCCIDNumber,DeviceInfo.SimCCIDNumber+20);
                #endif                            
                
                gprs_init_step++;
            }
        }
        break;
    case 10: 
        if (AT_CMD("AT+CTZU=1\r", "OK", 200) == AT_OK) 
        {
            //AT+QICSGP=1,1,"CMIOT","","",1
            //if (AT_CMD("AT+QICSGP=1,1,\"mobile\","","",1\r\n", "OK", 100) == AT_OK)
            if(WriteApnToModule() == 0)
                gprs_init_step++;
        }
        break;
    /*case 11:
        if (AT_CMD("AT+COPS=?\r", "OK", 1500) == AT_OK) 
            gprs_init_step++;
        break;*/
    case 11:
        led_set_mode(CPIN_OK);
        delay_ms(1000);
        if (AT_CMD("AT+CGREG?\r", "+CGREG: ", 200) == AT_OK) //+CGREG: 1,1
        {
            //delay_ms(10);
            p2 = (uint8_t*)cmd_resp;
            if(p2)
            {
                p3=(uint8_t*)strstr((const char*)p2,",");
                tmp = atoi((char*)p3+1);
                
                if((1 == tmp) || (5 == tmp))
                {
                    #ifdef DEBUG_OUTPUT0
                    DEBUG_PRINT("\r\n[ INFO ]: 4g net ok!!!\r\n");
                    #endif
                    gprs_init_step++;
                }
            }
        }
        break;
    case 12:
        //delay_ms(1000);
        OSTimeDly(1000, OS_OPT_TIME_DLY, &err);
        if(AT_CMD("AT+CSQ\r","+CSQ",200) == AT_OK) //+CSQ: 24,0
        {
            //delay_ms(10);
            p2 = (uint8_t*)cmd_resp;
            if(p2)
            {
                p3=(uint8_t*)strstr((const char*)p2,":");
                tmp = atoi((char*)(p3+2));
                
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\n4g_csq=%d\r\n",tmp);
                #endif
                if(tmp <= 31)
                    DeviceInfo.ModuleSignalValue = tmp;
                gprs_init_step++;
            }
        }
        break;
    case 13:
        if(AT_CMD("AT+QNWINFO\r\n","+QNWINFO:",200) == AT_OK) //查询当前网络
        {
            p2 = (uint8_t*)cmd_resp;
            if(p2)
            {
                if(strstr((const char*)p2,"GSM") != NULL) 
                {
                    DeviceInfo.CopsFlag = NET_COPS_2G;
                }   
                else if(strstr((const char*)p2,"LTE") != NULL)
                {
                    DeviceInfo.CopsFlag = NET_COPS_4G;
                }
            }
            
            gprs_init_step++;
        }
        break;
    case 14:
        if (AT_CMD("AT+CMGF=1\r", "OK", 200) == AT_OK) //SMS format = text 0=PDU  1=text 
            gprs_init_step++;
        break;
    case 15:
        if (AT_CMD("AT+CSCS=\"GSM\"\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
//    case 14:
//        if (AT_CMD("AT+CSMP=17,167,0,0\r", "OK", 100) == AT_OK)
//            gprs_init_step++;
//        break;
    case 16:
        if (AT_CMD("AT+CNMI=2,1,0,1,0\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
    case 17:
        if (AT_CMD("AT+QURCCFG=\"urcport\",\"uart1\"\r", "OK", 200) == AT_OK)  
            gprs_init_step++;
        break;
    case 18:
        if (AT_CMD("AT+QURCCFG=?\r", "OK", 200) == AT_OK) 
            gprs_init_step++;
        break;
	case 19:  
        if (AT_CMD("AT+QISDE=0\r\n", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
    case 20: 
		#ifdef USE_BLE_MODULE
            BLE_ATcmd_init();   
         #endif
		 gprs_init_step++;
		break;
    case 21:
        if (AT_CMD("AT&W\r", "OK", 200) == AT_OK)
            gprs_init_step++;
        break;
    default:
        res = 0;
        gprs_init_step = 0;
        break;
    }

    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return res;
}

/*
*********************************************************************************************************
*	函 数 名: CheckGprsState 
*	功能说明: 查询模块gprs连接状态
*	形    参：无
*	返 回 值: 0&1--初始状态，2-连接，3-断开,刚开机未连接时查询，只返回OK
*********************************************************************************************************
*/
static uint8_t CheckGprsState(void)
{
    uint8_t res = GPRS_SOCKET_OPENING;
    uint8_t *p2,*p3,tmp;
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
          (OS_TICK  )0,
          (OS_OPT   )OS_OPT_PEND_BLOCKING,
          (CPU_TS  *)0,
          (OS_ERR  *)&err);
    //+QISTATE: 0,"TCP","119.123.126.159",8832,1234,2,1,0  
    if(AT_OK == AT_CMD("AT+QISTATE=1,0\r", "+QISTATE:", 200))	 
    {
        p2 = (uint8_t*)cmd_resp;
        if(p2) 
        {
            p3=(uint8_t*)strstr((const char*)p2,"1234");
            p2=(uint8_t*)strstr((const char*)p3,",");
            tmp = atoi((char*)p2+1);
                
            if(2 == tmp) 
            {
                //DEBUG_PRINT("\r\n[ INFO ]: 4g net ok!!!\r\n");
                gprs_cfg.connect_ok = 1;
                res = GPRS_SOCKET_CONNECTED;
            }
            else if((3 == tmp)||(4 == tmp)) 
            {
                DEBUG_PRINT("\r\n[ INFO ]: 4G net close!!!\r\n");

                if(AT_OK == AT_CMD("AT+QICLOSE=0\r", "CLOSE OK", 1000))
                {
                    gprs_set_state(GPRS_CONNECT);
                }
                gprs_cfg.connect_ok = 0;
                res = GPRS_SOCKET_CLOSE;
            }
            else if(0 == tmp) 
            {
                DEBUG_PRINT("\r\n[ INFO ]: 4G net is Initial!!!\r\n");

                res = GPRS_SOCKET_INITIAL;
            }
        }
    }
        
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return res;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_close 
*	功能说明: 关闭连接
*	形    参: void
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t gprs_close(void)
{
    uint8_t result = 1;
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
   
    //if (AT_CMD("AT+QICLOSE=0\r", "CLOSE OK", 1000) == AT_OK) 
    if (AT_CMD("AT+QIDEACT=1\r", "OK", 4000) == AT_OK)
        result = 0;

    delay_ms(150);

    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return result;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_read_rssi 
*	功能说明: 查询信号强度
*	形    参: rssi 信号强度
*	返 回 值: 0--成功
*********************************************************************************************************
*/
static uint8_t gprs_read_rssi(uint8_t *rssi)
{
    char *data_ptr = NULL;
    char num_buf[4] = {0};
    uint8_t i = 0,res = 1;
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    //+CSQ: 31,0\r\n\r\nOK
    if (AT_CMD("AT+CSQ\r", "+CSQ", 200) == AT_OK) //+CSQ: 25,0
    {
        data_ptr = cmd_resp;
        if(data_ptr)
        {
            while (*data_ptr < '0' || *data_ptr > '9')
            {
                data_ptr++;
                if(++i>10)break;
            }
            
            i = 0;
            while (*data_ptr >= '0' && *data_ptr <= '9')
            {
                num_buf[i++] = *data_ptr++;
                if(i>10)break;
            }

            *rssi = (uint8_t)atoi(num_buf);
            
            res = 0;
        }
    }

    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return res;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_read_cops 
*	功能说明: 查询当前网络
*	形    参: cops
              “GSM”
              “GPRS”
              “EDGE”
              “WCDMA”
              “HSDPA”
              “HSUPA”
              “HSPA+”
              “TDSCDMA”
              “TDD LTE”
              “FDD LTE”
*	返 回 值: 0--成功
*********************************************************************************************************
*/
static uint8_t gprs_read_cops(uint8_t *cops)
{
    char *data_ptr1 = NULL;//,*data_ptr2 = NULL;
    //char num_buf[4] = {0};
    uint8_t res = 1;
    OS_ERR err;
                  
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    
    //+QNWINFO: “FDD LTE”,46011,“LTE BAND 3”,1825
    if(AT_CMD("AT+QNWINFO\r\n","+QNWINFO:",300) == AT_OK)
    {
//        delay_ms(10); 
        data_ptr1 = cmd_resp;
        if(data_ptr1) 
        {
            /*data_ptr2 = strstr((const char*)data_ptr1,",");
            data_ptr1 = strstr((const char*)data_ptr2+1,",");
            data_ptr2 = strstr((const char*)data_ptr1+1,",");
            data_ptr2++;//跳过逗号
            while (*data_ptr2 >= '0' && *data_ptr2 <= '9')
                num_buf[i++] = *data_ptr2++;
            
            *cops = (uint16_t)atoi(num_buf);*/
            

            if(strstr((const char*)data_ptr1,"GSM") != NULL)
            {
                *cops = 2;
            }
            
            if(strstr((const char*)data_ptr1,"TDSCDMA") != NULL)
            {
                *cops = 3;
            }
            
            if(strstr((const char*)data_ptr1,"LTE") != NULL)
            {
                *cops = 4;
            }
            
            res = 0;
        }
    }
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return res;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_read_cbc 
*	功能说明: 查询模块供电
*	形    参: cbc 模块电池电压
*	返 回 值: 0--成功
*********************************************************************************************************
*/
static uint8_t gprs_read_cbc(uint16_t *cbc)
{
    char *data_ptr1 = NULL,*data_ptr2 = NULL;
    char num_buf[4] = {0};
    uint8_t i = 0,res = 1;
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    //+CBC: 0,0,3356
    if (AT_CMD("AT+CBC\r", "+CBC", 200) == AT_OK)
    {
        data_ptr1 = cmd_resp;
        if(data_ptr1) 
        {
            data_ptr2 = strstr((const char*)data_ptr1,",");
            data_ptr1 = strstr((const char*)data_ptr2+1,",");
            data_ptr1++;//跳过逗号
            while (*data_ptr1 >= '0' && *data_ptr1 <= '9')
                num_buf[i++] = *data_ptr1++;
            
            *cbc = (uint16_t)atoi(num_buf);
            
            res = 0;
        }
    }
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);

    return res;
}

#ifdef ENABLE_AT_PORT_READ_GPS_INFO
/*
*********************************************************************************************************
*	函 数 名: gprs_read_gprmc 
*	功能说明: 读取gprmc信息
*	形    参: 
*	返 回 值: 0--成功
*********************************************************************************************************
*/
extern OS_MEM GpsBufPoolMem; 
uint8_t  *p_mem_gps = NULL;                     
//extern OS_EVENT  *pGPS_Q;
extern OS_TCB    GPSProcTaskTCB;
static uint8_t gprs_read_gprmc(void)
{
    char *data_ptr1 = NULL,*data_ptr2 = NULL;
    uint8_t res = 1;
    OS_ERR err;

    OSSemPend((OS_SEM      *)&gprs_busy_sem,        
              (OS_TICK      )0,                    
              (OS_OPT       )OS_OPT_PEND_BLOCKING, 
              (CPU_TS      *)0,                    
              (OS_ERR      *)&err);                
    //+QGPSGNMEA: $GPRMC,090546.00,A,2238.650113,N,11402.491720,E,0.0,0.0,150720,2.3,W,A*2D
    if (AT_CMD("AT+QGPSGNMEA=\"RMC\"\r\n", "+QGPSGNMEA:", 100) == AT_OK)
    {
//        delay_ms(10);
        data_ptr1 = cmd_resp;
        if(data_ptr1) 
        {
            data_ptr2 = strstr((const char*)data_ptr1,"$");
            data_ptr2++; //跳过$
			p_mem_gps = OSMemGet((OS_MEM      *)&GpsBufPoolMem,
                                   (OS_ERR    *)&err);

            if((p_mem_gps == NULL) && (err == OS_ERR_MEM_NO_FREE_BLKS))
            {
                DEBUG_PRINT("\r\nmalloc memory faill_rmc!!!");

                return res;
            }

            memset(p_mem_gps,0,GPS_MEM_Zone);
            strcpy((char*)p_mem_gps,data_ptr2);
            
            //OSQPost (pGPS_Q,p_mem_gps);
			OSTaskQPost ((OS_TCB       *)&GPSProcTaskTCB,
                          (void        *)p_mem_gps,
                          (OS_MSG_SIZE  )GPS_MEM_Zone,
                          (OS_OPT       )OS_OPT_POST_FIFO,
                          (OS_ERR      *)&err);
                               
            res = 0;
        }
    }

    //delay_ms(50); 
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);

    return res;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_read_gpgga 
*	功能说明: 读取gpgga信息
*	形    参: 
*	返 回 值: 0--成功
*********************************************************************************************************
*/

static uint8_t gprs_read_gpgga(void)
{
    char *data_ptr1 = NULL,*data_ptr2 = NULL;
    uint8_t res = 1;
    OS_ERR err;
    
    OSSemPend((OS_SEM      *)&gprs_busy_sem,       
              (OS_TICK      )0,                    
              (OS_OPT       )OS_OPT_PEND_BLOCKING, 
              (CPU_TS      *)0,                    
              (OS_ERR      *)&err);                
    //+QGPSGNMEA: $GPGGA,090549.00,2238.650122,N,11402.491717,E,1,09,0.8,94.7,M,-1.0,M,,*48
    if (AT_CMD("AT+QGPSGNMEA=\"GGA\"\r\n", "+QGPSGNMEA:", 100) == AT_OK)
    {
        data_ptr1 = cmd_resp;
        if(data_ptr1) 
        {
            data_ptr2 = strstr((const char*)data_ptr1,"$");
            data_ptr2++; //跳过$
			p_mem_gps = OSMemGet((OS_MEM      *)&GpsBufPoolMem,
                                   (OS_ERR    *)&err);
            if((p_mem_gps == NULL) && (err == OS_ERR_MEM_NO_FREE_BLKS))
            {
                DEBUG_PRINT("\r\nmalloc memory faill_gga!!!");

                return res;
            }

            memset(p_mem_gps,0,GPS_MEM_Zone);
            strcpy((char*)p_mem_gps,data_ptr2);
            
            //OSQPost (pGPS_Q,p_mem_gps);
			OSTaskQPost ((OS_TCB       *)&GPSProcTaskTCB,
                          (void        *)p_mem_gps,
                          (OS_MSG_SIZE  )GPS_MEM_Zone,
                          (OS_OPT       )OS_OPT_POST_FIFO,
                          (OS_ERR      *)&err);
            res = 0;
        }
    }

    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);

    return res;
}
#endif //ENABLE_AT_PORT_READ_GPS_INFO

/*
*********************************************************************************************************
*	函 数 名: gprs_connect 
*	功能说明: 通过TCP方式连接指定服务器
*	形    参: ip:指向ip地址，port:指向端口
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t gprs_connect(char *ip, uint16_t port)
{
    static uint8_t fristFlag = 0;
    uint8_t *p2,*p3;
    uint16_t tmp=1;
    uint8_t result = 1;
    OS_ERR err;
    
    //char cmd_buf[50];
//    #if OS_CRITICAL_METHOD == 3    
//    OS_CPU_SR  cpu_sr;
//    #endif
    char TempBuff[70],i,ucPos;  //AT+QIOPEN=1,0,"TCP","119.123.126.159",8832,1234,1
    static const uint8_t AT_QIOPEN[] = {"AT+QIOPEN=1,0,\"TCP\",\"218.205.065.162\",5566,1234,1"};
    
    if(gprs_cfg.connect_ok)
        return !result;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    
    memset(TempBuff,0,70);
    for(i = 0; AT_QIOPEN[i] != '\0'; i++)
        TempBuff[i] = AT_QIOPEN[i];

    ucPos = 21;
    if(PlatformData.PlatformAddrFlag == 1) 
    {
        i = 0;
#ifdef DOUBLE_DNS_FUN
		if(PlatformData.MainBackupDNSFlag == 0)
#endif
		{
	        while(PlatformData.PlatformDomainName[i] != 0)
	        {
	            TempBuff[ucPos] = PlatformData.PlatformDomainName[i];
	            i++;
	            ucPos++;
	            
	            if(i > MAX_DOMAIN_NAME_LEN)
	            {
	                TempBuff[ucPos] = 0;
	                break;
	            }
	        }
		}
#ifdef DOUBLE_DNS_FUN
		else
		{
			while(PlatformData.BackupDomainName[i] != 0)
	        {
	            TempBuff[ucPos] = PlatformData.BackupDomainName[i];
	            i++;
	            ucPos++;
	            
	            if(i > MAX_DOMAIN_NAME_LEN)
	            {
	                TempBuff[ucPos] = 0;
	                break;
	            }
	        }
		}
#endif
        TempBuff[ucPos++] = '"';
    }
    else
    {
        //双IP切换功能 
#ifdef DOUBLE_IP_FUN 
        if(PlatformData.MainBackupIPFlag == 0)
#endif
        {
            for (i = 0; i < 4; i ++)
            {
                if(PlatformData.PlatformIpAddr[i] / 100 != 0)
                {
                    TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformIpAddr[i] / 100);
                }
                TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformIpAddr[i] % 100 / 10);
                TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformIpAddr[i] % 10);
                TempBuff[ucPos ++] = '.';
            }
        }
#ifdef DOUBLE_IP_FUN 
        else 
        {
            for (i = 0; i < 4; i ++)
            {
                if(PlatformData.BackupIpAddr[i] / 100 != 0)
                {
                    TempBuff[ucPos ++] = ValueToChar(PlatformData.BackupIpAddr[i] / 100);
                }
                TempBuff[ucPos ++] = ValueToChar(PlatformData.BackupIpAddr[i] % 100 / 10);
                TempBuff[ucPos ++] = ValueToChar(PlatformData.BackupIpAddr[i] % 10);
                TempBuff[ucPos ++] = '.';
            }
        }
#endif
        TempBuff[ucPos-1] = '"';
    }
    
    TempBuff[ucPos++] = ',';
    if(PlatformData.PlatformPort < 10000)
    {
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort / 1000);
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 1000 / 100);
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 100 / 10);
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 10);
    }
    else
    {
        if(PlatformData.PlatformPort / 10000 != 0)
        {
            TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort / 10000);
        }
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 10000 / 1000);
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 1000 / 100);
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 100 / 10);
        TempBuff[ucPos ++] = ValueToChar(PlatformData.PlatformPort % 10);
    }
    TempBuff[ucPos ++] = ',';
    TempBuff[ucPos ++] = '1';
    TempBuff[ucPos ++] = '2';
    TempBuff[ucPos ++] = '3';
    TempBuff[ucPos ++] = '4';
    TempBuff[ucPos ++] = ',';
    TempBuff[ucPos ++] = '1';
    TempBuff[ucPos ++] = 0x0D;
    //TempBuff[i ++] = 0x0A;
    
    //AT+QIOPEN=1,0,"TCP","119.123.126.159",8832,1234,1
    //snprintf(cmd_buf, sizeof(cmd_buf), "AT+QIOPEN=1,0,\"TCP\",\"%s\",%d,1234,1\r\n", ip, port);
//    if (AT_CMD(TempBuff, "+QIOPEN: 0,0", 2000) == AT_OK)
//        result = 0; 
    
    if(fristFlag == 0)
    { 
        fristFlag = 1;

        DeviceInfo.SendTcpDataFlag = TRUE;
        //OS_ENTER_CRITICAL();
        DeviceInfo.SendTcpDataDlyCnt  = TEN_SECOND;
        //OS_EXIT_CRITICAL();       
    }
    
    if (AT_CMD(TempBuff, "+QIOPEN: ", 6000) == AT_OK)
    {
        p2 = (uint8_t*)cmd_resp;
        if(p2) 
        {
            p3=(uint8_t*)strstr((const char*)p2,",");
            if(p3)
            {
                tmp = atoi((char*)p3+1);
                if(0 == tmp)
                {
                    fristFlag = 0;
                    DeviceInfo.SendTcpDataFlag = FALSE;
                    DeviceInfo.SendTcpDataDlyCnt  = 0;
                    DeviceInfo.NeedSaveBlindAreaFlag = FALSE;
                    
                    result = 0;
                }
                else
                {
                    DEBUG_PRINT("\r\nerr=%d,Socket bind failed\r\n",tmp);
                }
            }
        }
    }
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return result;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_send 
*	功能说明: 发送gprs数据包
*	形    参: data:指向发送数据的缓存，len:数据长度
*	返 回 值: 0-成功，1-失败
*********************************************************************************************************
*/
uint8_t gprs_send(uint8_t *data, uint16_t len)
{
    uint8_t result = 1;
    uint8_t cmd_buf[200];
    OS_ERR err; 

    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    memset(cmd_buf,0,200);
    snprintf((char*)cmd_buf, sizeof(cmd_buf), "AT+QISEND=0,%d\r",len);
    if (AT_CMD((char*)cmd_buf, ">", 100) == AT_OK)
    {

        memset(cmd_buf,0,200);
        snprintf((char*)cmd_buf,len+1,"%s",(const char*)data);
        strncat((char*)cmd_buf,"\r\n",2);
        
        if(AT_OK == AT_CMD((char*)cmd_buf, "SEND OK", 3000))
        {
            result = 0;
            led_set_mode(SEND_DATA);
        }
        else 
        {
            DEBUG_PRINT("\r\nEC20_Send_fail!!!"); 
            result = 1;
        }   
    }
    else 
    {
        
        memset(cmd_buf,0,200);
        snprintf((char*)cmd_buf,len+1,"%s",(const char*)data);
        strncat((char*)cmd_buf,"\r\n",2);
        
        if(AT_OK == AT_CMD((char*)cmd_buf, "SEND OK", 3000))
        {
            result = 0;
            led_set_mode(SEND_DATA);
        }
        else 
        {
            DEBUG_PRINT("\r\nEC20_Send_fail_2!!!");  

            result = 1;
        }
    }
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return result;
}

/*
*********************************************************************************************************
*	函 数 名: gprs_at_sync 
*	功能说明: 同步波特率，同时检查模块是否工作
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t gprs_at_sync(void)
{
    uint8_t res = 1;
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    if(AT_OK == AT_CMD("AT\r", "OK", 100))
    {
        res = 0;
    }
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err); 
    
    return res;
}

static uint8_t check_net_state(void)
{
    uint8_t *p2,*p3,tmp;
    uint8_t res = 1;
    OS_ERR err;
     
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    //+CGATT: 1
    if (AT_CMD("AT+CGATT?\r", "+CGATT", 100) == AT_OK)
    {
        p2 = (uint8_t*)cmd_resp;
        if(p2) 
        {
            p3=(uint8_t*)strstr((const char*)p2,":");
            tmp = atoi((char*)p3+2); //跳过冒号和空格
            
            if(1 == tmp)
            {
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\n[ INFO ]: cgatt=%d!!!\r\n",tmp);
                #endif
                led_set_mode(REG_NET_OK);
                res = 0; 
            }
        }
    }
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return res;
}

unsigned char SendSms(unsigned char *pSendText, unsigned char *pSendNumber)
{
    unsigned char  i,j,TempBuf[100];
    OS_ERR err;
    uint8_t res = 1;

    DEBUG_PRINT("\r\nSend SMS:%s",pSendNumber);
    DEBUG_PRINT("\r\nSend Data:%s",pSendText);
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    
    TempBuf[0] = 'A';
    TempBuf[1] = 'T';
    TempBuf[2] = '+';
    TempBuf[3] = 'C';
    TempBuf[4] = 'M';
    TempBuf[5] = 'G';
    TempBuf[6] = 'S';
    TempBuf[7] = '=';
    TempBuf[8] = '"';
    for(i=0;i<100 && pSendNumber[i] != '\0';i++) //1064899031506
        TempBuf[9+i] = pSendNumber[i];
    
    i=i+9;
    
    TempBuf[i++] = '"';
    TempBuf[i++] = 0xff;
    
    //AT+CMGS="1064899031506"
    
    if (j == 0) //取消发送
    {
        //GSM_Com_Send_Byte(0x1B); //ESC
        return FALSE;
    }
    
    for(i=0;i<100 && pSendText[i] != '\0';i++)
        TempBuf[i] = pSendText[i];
    
    //TempBuf[i++] = 0x1A;  

    if(AT_CMD((char*)TempBuf, "+CMGS:", 100) == AT_OK)
        res = 0;
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err); 
    
    return res;
}

void ReadSms(uint8_t *TempBuf, uint8_t ucSmsPos)
{
    uint8_t i;
    #define AT_CMGR_SIZE    8
    static const uint8_t AT_CMGR[] = {"AT+CMGR="};

    for(i=0;i<AT_CMGR_SIZE;i++)
        TempBuf[i] = AT_CMGR[i];
    
    if (ucSmsPos < 10)
    {
        TempBuf[8] = ValueToChar(ucSmsPos); 
        TempBuf[9] = 0x0D;
        TempBuf[10] = '\0';
    }
    else
    {
        TempBuf[8] = ValueToChar(ucSmsPos / 10);    
        TempBuf[9] = ValueToChar(ucSmsPos % 10);
        TempBuf[10] = 0x0D;
        TempBuf[11] = '\0';  
    }
    //AT_CMD((char*)TempBuf, "OK", 100);
}

void DeleteSms(uint8_t ucSmsPos)
{
    uint8_t i;//, ucPos;
    uint8_t TempBuf[12];
    #define AT_CMGR_SIZE    8
    static const uint8_t AT_CMGD[] = {"AT+CMGD="};
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    for(i=0;i<AT_CMGR_SIZE;i++)
        TempBuf[i] = AT_CMGD[i];
    i = 3;
    while (i--)
    {
        if (ucSmsPos < 10)
        {
            TempBuf[8] = ValueToChar(ucSmsPos); 
            TempBuf[9] = 0x0D;
            TempBuf[10] = 0;
        }
        else
        {
            TempBuf[8] = ValueToChar(ucSmsPos / 10);    
            TempBuf[9] = ValueToChar(ucSmsPos % 10);
            TempBuf[10] = 0x0D;
            TempBuf[11] = 0;        
        }
        
        if (AT_CMD((char*)TempBuf, "OK", 100) == AT_OK)
        {
            break; //删除成功
        }
    }
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err); 
}

/*
+CMGR: "REC UNREAD","1064899110237",,"19/06/06,10:58:55+32"
reboot 123456
*/
void RcvSmsProc(unsigned char ucSmsPos)
{   
    uint16_t ucPos,j,k;
    uint8_t ucReadCnt, n;
    uint8_t TimeBuf[12];
    uint8_t *data_ptr = NULL;
//    char num_buf[4] = {0};
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    ucReadCnt = 3;
    while (ucReadCnt--)
    {
        ReadSms(TimeBuf, ucSmsPos);         //读短消息
        if (AT_CMD((char*)TimeBuf, "+CMGR:", 100) == AT_OK)
        {
    //        delay_ms(10);
            data_ptr = (unsigned char *)cmd_resp;
            if(data_ptr)
            {
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nsms-->%s",data_ptr);
                #endif
                
                //提取电话号码
                ucPos = SearchStr(data_ptr, data_ptr + 100, "\",\"");
                ucPos += 3;
                //第一个可能是"+"
                u8GprsSMSTelNum[0] = *(data_ptr + ucPos);
                if ((u8GprsSMSTelNum[0] != '+') && (IsNumber(&u8GprsSMSTelNum[0], 1) == FALSE_Bit))
                    continue;
                
                ucPos++;
                j = 1;
                while (*(data_ptr + ucPos) != '"')
                {               
                    u8GprsSMSTelNum[j++] = *(data_ptr + ucPos);
                    if (j >= MAX_PHONE_LEN)
                        break;          

                    ucPos++;
                }
                if(j >= MAX_PHONE_LEN)
                    continue;

                u8GprsSMSTelNum[j] = 0; //号码结束
                if (IsNumber(u8GprsSMSTelNum + 1, StrLen(u8GprsSMSTelNum + 1)) == FALSE_Bit)
                    continue;

                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nucSmsTelephone:%s",u8GprsSMSTelNum);
                #endif
                
                //提取短信内容
                n = ucPos;
                ucPos = SearchStr(data_ptr + n, data_ptr + 100, "\x0D\x0A");
                if (ucPos == STR_SEARCH_FAIL)
                {
                    continue;
                }
                ucPos += 2; 
                k = 0;
                while((data_ptr[n + ucPos] != 0x0D) || (data_ptr[n + ucPos + 1] != 0x0A))
                {
                    SmsProcBuffer[k ++] = data_ptr[n + ucPos];
                    ucPos ++;
                    if (k >= MAX_SMS_BUFFER_LEN)
                    {
                        k -= 9; 
                        break;
                    }
                }
    //            SmsContentLen = k;
                SmsProcBuffer[k ++] = 0;
                SmsProcBuffer[k ++] = 0;

                if(k > 100) 
                {
                    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
                    DeleteSms(ucSmsPos);
                    return;
                }
                
                DEBUG_PRINT("\r\nTextLen:%d", k);
                DEBUG_PRINT("\r\nSmsText:%s", SmsProcBuffer);
                
                if (k >= MAX_SMS_BUFFER_LEN) 
                {
                    continue;
                }

                break;
            }
        }
        else 
        {
            OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
            return;
        }
    }
    
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    DeleteSms(ucSmsPos);
    
    if (ucReadCnt == 255)
    {
        //读短信失败
        DEBUG_PRINT("Read SMS Fail.");

        return;
    } 

    DEBUG_PRINT("\r\nSMSTel:%s",u8GprsSMSTelNum);
    DEBUG_PRINT("\r\n%s",SmsProcBuffer);

    SmsProc(SmsProcBuffer, u8GprsSMSTelNum);
}

#ifdef ENABLE_PHONE_CALL 
//**********************************************************
void GetTelephone(unsigned char *pStrStart, unsigned char *pStrEnd, unsigned char *ucCallTelephone)
{
    unsigned char ucPos, i;


    ucPos = SearchStr(pStrStart, pStrEnd, "+CLIP:");
    if (ucPos == STR_SEARCH_FAIL)
    {
        ucCallTelephone[0] = 0;
        return;
    }

    ucPos += 8;  //6 指到电话号码第一个字符

    ucCallTelephone[0] = *(pStrStart + ucPos);
    if ((ucCallTelephone[0] != '+') && (IsNumber(&ucCallTelephone[0], 1) == FALSE))
    {
        ucCallTelephone[0] = 0;
        return;
    }
    ucPos ++;
    i = 1;
    while (*(pStrStart + ucPos) != '"')
    {
        ucCallTelephone[i ++] = *(pStrStart + ucPos);
        if (i >= MAX_TELEPHONE_NUMBER_LEN)
        {
            ucCallTelephone[0] = 0;
            return;
        }
        ucPos ++;
    }
    ucCallTelephone[i] = 0; //结束电话号码
    //不包括第一个，第一个可能不是数字
    if (IsNumber(ucCallTelephone + 1, StrLen(ucCallTelephone + 1)) == FALSE)
    {
        ucCallTelephone[0] = 0;
        return;
    }

    DEBUG_PRINT("\r\nCall Telephone:%s",ucCallTelephone);
}


//*****************************************
void CallRcvMonitorProc(unsigned char *ucCallTelephone)
{
    //临时保存号码
    if ((ucCallTelephone[0] != 0) && (DeviceInfo.ucAdminState == DEVICE_NO_ADMIN))
    {
    //  CopyStr(DeviceInfo.Telephone[4].Number, ucCallTelephone);
    }

    //-------------------------------------------------
    if (DeviceInfo.MonitorMode == ALL_MONITOR_MODE)
    {
        DEBUG_PRINT("\r\nAll monitor.");

        //监听
        (AT_CMD("ATA\r", "OK", 100) == AT_OK);
        
        HadConnected_bit = TRUE_Bit;        
        DeviceInfo.DeviceCallFlag = TRUE;
        DeviceInfo.CallStatusCheckRemainSeconds = 20;
        DeviceInfo.DeviceMonitorFlag = TRUE;
    }
    else if (DeviceInfo.MonitorMode == RESTRICT_MONITOR_MODE)
    {
        if (DeviceInfo.ucAdminState == DEVICE_NO_ADMIN)
        {
            DEBUG_PRINT("\r\nCallNoAdmin");

            //来电处理
            //开始监听
            (AT_CMD("ATA\r", "OK", 100) == AT_OK);
            HadConnected_bit = TRUE_Bit;    
            DeviceInfo.DeviceCallFlag = TRUE;
            DeviceInfo.CallStatusCheckRemainSeconds = 20;
            DeviceInfo.DeviceMonitorFlag = TRUE;
        }
        else
        {
            DEBUG_PRINT("\r\nCallAdmin");

            if (FindTelephone(ucCallTelephone) == FIND_TELEPHONE_FAIL)
            {
                //挂掉
                (AT_CMD("ATH\r", "OK", 100) == AT_OK);
            }
            else
            {
                //监听
                (AT_CMD("ATA\r", "OK", 100) == AT_OK);
                HadConnected_bit = TRUE_Bit;    
                DeviceInfo.DeviceCallFlag = TRUE;
                DeviceInfo.CallStatusCheckRemainSeconds = 20;
                DeviceInfo.DeviceMonitorFlag = TRUE;
            }
        }
    }
    else
    {
        DEBUG_PRINT("\r\nClose Monitor.");

        return;
    }
}


//*****************************************
void CallRcvProc(unsigned char *ucCallTelephone)
{
    DEBUG_PRINT("\r\nMonitor Mode.");

    CallRcvMonitorProc(ucCallTelephone);
}


//*************************************
void CallProc(unsigned char *pStrStart, unsigned char *pStrEnd)
{
    unsigned char ucCallTelephone[MAX_TELEPHONE_NUMBER_LEN];


    #ifdef DEBUG_OUTPUT0
    DEBUG_PRINT("\r\nCallProc:%s", pStrStart);
    #endif

    //获取电话号码
    GetTelephone(pStrStart, pStrEnd, ucCallTelephone);
    CallRcvProc(ucCallTelephone);
}
#endif //#ifdef ENABLE_PHONE_CALL

/*
*********************************************************************************************************
*	函 数 名: gprs_init_handle
*	功能说明: gprs模块AT初始化处理
*	形    参: void
*	返 回 值: 无
*********************************************************************************************************
*/
void gprs_init_handle(void)
{
    #ifdef ENABLE_IDWG
    OS_ERR err;
    #endif
    
    uint8_t rsp,i;
    static uint8_t err_cnt = 0,shut_cnt=0,rboot_tim=1,conncet_err_cnt=0;

    switch (gprs_state)
    {
        case GPRS_POWER_OFF:
            led_set_mode(CPIN_ERR);
            gprs_pwr_off();
            gprs_state = GPRS_POWER_ON;
            DeviceInfo.CpinReadyFlag = FALSE; 
            break;
        case GPRS_POWER_ON:
            gprs_cfg.net_work = 0;
            gprs_cfg.connect_ok = 0;
            gprs_cfg.device_ok = 0;
            gprs_cfg.rcv_error = 0;
            gprs_cfg.send_success_sum = 0;
            gprs_cfg.send_fail_sum = 0;
            HaveSmsAT_bit = FALSE_Bit;
            Sms_pos = 0;
            //memset(AtCmdcom.AtRxBuf,0,strlen(AtCmdcom.AtRxBuf));
            ec20_pwr_on();
            gprs_state = GPRS_AT_SYNC;
            break;
        case GPRS_AT_SYNC:
            if(gprs_at_sync() == 0)
            {
                //DEBUG_PRINT("\r\nAT cmd ok!!!");
                err_cnt = 0;
                gprs_cfg.device_ok = 1;
                gprs_state = GPRS_INIT;
            }
            else
            {
                delay_ms(1000);
                if (++err_cnt >= 8)
                {
                    err_cnt = 0;
                    DEBUG_PRINT("\r\n[ ERROR ]: Sync Baud rate Fail!!!");

                    gprs_state = GPRS_POWER_OFF;
                }
            }
            break;
        case GPRS_INIT:
            if (gprs_init() == 0)
            {
                gprs_state = GPRS_CHK_NET_STATE;
                DeviceInfo.DeviceWorkState = DEVICE_WORK_STATE_RUNNING;
            }
            break;
        case GPRS_CHK_NET_STATE:      
            if(check_net_state() == 0)
            {
                gprs_cfg.net_work = 1;
                gprs_state = GPRS_CHK_CONNECT_STATE;
            }
            break;
        
        case GPRS_CHK_CONNECT_STATE:
            rsp = CheckGprsState();
            switch(rsp)
            {
                case GPRS_SOCKET_INITIAL:     
                case GPRS_SOCKET_OPENING:       //默认上电初始状态
                    gprs_cfg.connect_ok = 0;
                    gprs_cfg.send_fail_cnt = 0;
                    gprs_state = GPRS_CONNECT;
                    break;
                case GPRS_SOCKET_CONNECTED:     //连接正常
                case GPRS_SOCKET_CLOSE:         //连接断开
                    if(0 != gprs_close())
                        delay_ms(2000);
                    if(++conncet_err_cnt > 5)
                    {
                        conncet_err_cnt = 0;
                        delay_ms(3000); 
                        gprs_set_state(GPRS_POWER_OFF);
                    }
                    break;
            }
            break;
        
        case GPRS_CONNECT:
            conncet_err_cnt = 0;
            if (AT_CMD("AT+QIACT=1\r", "OK", 1000) != AT_OK) 
            {
                delay_ms(1000);  
                gprs_state = GPRS_SHUT; 
                break;
            }
            
            AT_CMD("AT+QIACT?\r", "OK", 300); 

			if(ec200t_flag) 
			{
				AT_CMD("AT+QISDE=0\r\n", "OK", 200);
			}
			
            if (0 == gprs_connect(pServer, pPort))
            {
                DEBUG_PRINT("\r\n>>>>>>>>>>> CONNECT SERVER SUCCEED!!! >>>>>>>>>>>\r\n");

                gprs_cfg.connect_ok = 1;
                AutoTrackData.RemainTime = 2;
                shut_cnt = 0;
                rboot_tim = 1;
				
				if(PlatformData_HY.HandshakeFailCnt != 0) 
					PlatformData_HY.DeviceConnectCheckTimeCnt = THREE_SECOND;
				
                gprs_state = GPRS_SEND_DATA;
                led_set_mode(CONNECT_OK);
            }
            else
            {
                DEBUG_PRINT("\r\n[ ERROR ]: Connect server fail %d time!!!\r\n",shut_cnt+1);
          
                gprs_set_state(GPRS_SHUT);
                
                for(i = 0; i < shut_cnt+1; i++)
                {
                    #ifdef ENABLE_IDWG
                    OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                                (OS_FLAGS      )GPRS_POLL_TASK_FEED_DOG,  /* 设置bit3 */
                                (OS_OPT        )OS_OPT_POST_FLAG_SET,
                                (OS_ERR       *)&err);
                    #endif
                    
                    delay_ms(5000);
                }
                if (++shut_cnt >= (10*rboot_tim))
                {
                    if(rboot_tim++ > 5) rboot_tim = 1;
                    
                    #ifdef DOUBLE_IP_FUN 
                    if(PlatformData.MainBackupIPFlag == 0)
                    {
                        PlatformData.MainBackupIPFlag = 1;
                        PlatformData.AutoSwitchMainIpTime = 60;
                    }
                    else
                        PlatformData.MainBackupIPFlag = 0;
                    #endif
        
                    shut_cnt = 0;
                    gprs_cfg.connect_ok = 0;
                    DEBUG_PRINT("\r\n[ ERROR ]: Timeout %d minute connect server fail ,reboot module!!!",((rboot_tim-1)*10));

                    delay_ms(5000); 
                    gprs_set_state(GPRS_POWER_OFF);
                    break;
                }
            }
            break;

        case GPRS_SHUT:
            gprs_init_step = 0;
            gprs_cfg.net_work = 0;
            gprs_cfg.connect_ok = 0;
            //gprs_set_led_mode(LED_SYS_RUN);
            if(gprs_close() == 0)
                gprs_state = GPRS_CHK_NET_STATE;//GPRS_INIT;
            else
                delay_ms(2000);
            break;
        default:
            break;
    }
    delay_ms(200);
}

/*
*********************************************************************************************************
*	函 数 名: gprs_get_ipd
*	功能说明: 获取ip数据头
*	形    参: void
*	返 回 值: 成功返回指向内容的地址，否则返回空指针
*********************************************************************************************************
*/
static int ipd_len = 0;
static uint8_t *gprs_get_ipd(char *data_ptr)
{
    char *data_ptr_t = NULL;
    data_ptr_t = strstr(data_ptr, "+QIURC:");
    if (data_ptr_t != NULL)
    {
        if (sscanf(data_ptr_t, "+QIURC: \"recv\",0,%d", &ipd_len) == 1)
        {
            data_ptr_t = strchr(data_ptr_t, '(');  //:

            if (data_ptr_t != NULL)
            {
                //data_ptr_t++;
                return (uint8_t *)(data_ptr_t);
            }
        }
    }
    return NULL;
}


///*
//*********************************************************************************************************
//*	函 数 名: gprs_get_ftpget
//*	功能说明: 获取ftp数据头
//*	形    参: void
//*	返 回 值: 成功返回指向内容的地址，否则返回空指针
//*********************************************************************************************************
//*/
//static int ftpget_len = 0;
//static uint8_t *gprs_get_ftpget(char *data_ptr)
//{
//    char *data_ptr_t = NULL;
//    data_ptr_t = strstr(data_ptr, "+FTPGET:");
//    if (data_ptr_t != NULL)
//    {
//        if (sscanf(data_ptr_t, "+FTPGET: 2,%d", &ftpget_len) == 1)
//        {
//            data_ptr_t = strchr(data_ptr_t, '\n');  //:

//            if (data_ptr_t != NULL)
//            {
//                //data_ptr_t++;
//                return (uint8_t *)(data_ptr_t);
//            }
//        }
//    }
//    return NULL;
//}
//#endif  //#ifdef  FTP_UPGRAGE

/*
*********************************************************************************************************
*	函 数 名: gprs_urc_rev_handle
*	功能说明: gprs模块接收处理
*	形    参: void
*	返 回 值: 无
*********************************************************************************************************
*/
void gprs_urc_rev_handle(char* data_ptr,uint16_t len)
{
    uint8_t /*p1[100] = {0},*/*p2 = NULL;
    OS_ERR err;
    #ifdef FTP_UPGRAGE
    uint16_t ucPos = 0,SumSize = 0, m;
    char *data_ptr1 = NULL,*data_ptr2 = NULL;
    #endif
    //static uint8_t err_cnt = 0;
    uint8_t tmp = 0;
    
    //收到数据处理
    if (data_ptr != NULL)
    {
        //fish_print("%s",data_ptr);
#ifdef FTP_UPGRAGE
        if(AtFTPGET.DownFlag != TRUE)
#endif
            DEBUG_UART("%s",data_ptr);
            //DEBUG_PRINT("%s",data_ptr);

        gprs_cmd_handle(data_ptr,len);
        
        if ((strstr(data_ptr, "+PDP: DEACT") != NULL) && (gprs_cfg.net_work))
        {
            DEBUG_PRINT("WARN:	+PDP: DEACT\r\n");

            #ifdef FTP_UPGRAGE
            if(AtFTPGET.DownFlag == TRUE) 
            {
                AtFTPGET.DownFlag = FALSE;
                gprs_cfg.connect_ok = 0;
                //gprs_set_state(GPRS_SHUT);
            }
            #endif
        }
		else if (strstr(data_ptr, "+CPIN: READY") != NULL) 
		{
			DeviceInfo.CpinReadyFlag = TRUE;
		}
		else if (strstr(data_ptr, "+CREG:") != NULL) 
		{ 
			//+CREG: 1
			while (*data_ptr < '0' || *data_ptr > '9')
            data_ptr++;

            //tmp = (uint8_t)atoi(data_ptr);
            tmp = CharToValue(*data_ptr);
            
            DEBUG_PRINT("---urc_creg[%c]: %d---\r\n", *data_ptr, tmp);

			if((1 != tmp) && (5 != tmp) && (gprs_cfg.connect_ok))
			{
				DEBUG_PRINT("\r\n[ WARN ]: cs domain abnormal,anew connect!!!\r\n");

                if(AtFTPGET.DownFlag != TRUE)
                {
                    gprs_cfg.connect_ok = 0;
                    gprs_set_state(GPRS_SHUT);
                }
			}
		}
		else if (strstr(data_ptr, "+CGREG:") != NULL) 
		{
			//+CGREG: 1
			while (*data_ptr < '0' || *data_ptr > '9')
            data_ptr++;

            //tmp = (uint8_t)atoi(data_ptr);
            tmp = CharToValue(*data_ptr);
            
            DEBUG_PRINT("---urc_cgreg[%c]: %d---\r\n", *data_ptr, tmp);

			if((1 != tmp) && (5 != tmp) && (gprs_cfg.connect_ok))
			{
				DEBUG_PRINT("\r\n[ WARN ]: ps domain abnormal,anew connect!!!\r\n");

                if(AtFTPGET.DownFlag != TRUE)
                {
                    gprs_cfg.connect_ok = 0;
                    gprs_set_state(GPRS_SHUT);
                }
			}
		}
        else if (strstr(data_ptr, "NO CARRIER") != NULL)
        {
            DeviceInfo.DeviceCallFlag = FALSE;
        }
#ifdef ENABLE_PHONE_CALL 
        else if (strstr(data_ptr, "+CLIP: ") != NULL) //+CLIP: "18680368846",128,,,,0
        {
            CallProc((unsigned char*)data_ptr, (unsigned char*)(data_ptr + 50));
        }
#endif
        else if (strstr(data_ptr, "RING") != NULL)
        {
            HaveRingAT_bit = TRUE_Bit;
        }
#ifdef  FTP_UPGRAGE 
        else if((strstr(data_ptr,"CONNECT") != NULL))
        {
            #ifdef DEBUG_OUTPUT0
            DEBUG_PRINT("\r\n===len=%d===",len);
            Debug_PrintData("2Packet->",(unsigned char*)data_ptr,(unsigned char*)(data_ptr + len));
            #endif
            //data_ptr += 2;//跳过回车换行
            data_ptr1 = strstr((const char*)data_ptr,"CONNECT\r\n");
            if(data_ptr1)
            {
                data_ptr1 += 9; //跳过CONNECT\r\n
                #ifdef DEBUG_OUTPUT0
                //Debug_PrintData("3Packet->",(unsigned char*)data_ptr1,(unsigned char*)(data_ptr1 + len));
                #endif
                
                ucPos = 0;
                for(m = 0; m < 1050; m++)
                {
                    //判断结束符\r\nOK\r\n
                    if((data_ptr1[ucPos+m] == 0x0D)&&(data_ptr1[ucPos+m+1] == 0x0A)
                        &&(data_ptr1[ucPos+m+2] == 0x4F)&&(data_ptr1[ucPos+m+3] == 0x4B)
                        &&(data_ptr1[ucPos+m+4] == 0x0D)&&(data_ptr1[ucPos+m+5] == 0x0A))
                    {
                        break;
                    }
                    FtpTempBuffer[m] = data_ptr1[ucPos + m];
                }
                if(m >= 1050)
                {
                    DEBUG_PRINT("\r\nftp receive lenght error[%d]!!!", m);

                    return;
                }
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\n===m=%d===",m);
                #endif
                
                data_ptr1 += m;//跳过内容
                data_ptr2 = strstr((const char*)data_ptr1,"+QFTPGET: "); //+QFTPGET: 0,300
                if(data_ptr2)
                {
                    data_ptr1 = strstr((const char*)data_ptr2,",");
                    data_ptr1 += 1;//跳过逗号
                    
                    SumSize = 0;
                    SumSize = atoi((char*)(data_ptr1));

                    DEBUG_PRINT("\r\nSumSize:%d\r\n",SumSize);

                    AtFTPGET.len = SumSize;
                    
                    if(m == AtFTPGET.len)
                    {
                        OSSemPost(&ftp_recv_sem,OS_OPT_POST_1,&err);
                    }
                    else
                    {
                        return;
                    }
                }
            }
        }
#endif
        else if ((strstr(data_ptr,"+CMTI:") != NULL)) //+CMTI: "ME",2
        {     
            while (*data_ptr < '0' || *data_ptr > '9')
            data_ptr++;

            Sms_pos = (uint8_t)atoi(data_ptr);

            DEBUG_PRINT("\r\nSms_pos=%d",Sms_pos);

            if(Sms_pos > 10) Sms_pos = 0;
            HaveSmsAT_bit = TRUE_Bit;
        }
        else if ((strstr(data_ptr, "+QIURC: \"closed\",0") != NULL) && gprs_cfg.connect_ok)
        {
            DEBUG_PRINT("\r\n[ WARN ]: Connect is closed,anew connect!!!\r\n");

            gprs_cfg.connect_ok = 0;
            gprs_set_state(GPRS_SHUT);
        }
        else if ((strstr(data_ptr, "+QIURC: \"pdpdeact\",1") != NULL) && gprs_cfg.connect_ok)
        {
            DEBUG_PRINT("WARN:	+PDP: DEACT\r\n");
            
            gprs_cfg.connect_ok = 0;
            gprs_set_state(GPRS_POWER_OFF);
        }
        else if (strstr(data_ptr, "ERROR") != NULL)
        {
            //if(gprs_state == GPRS_CONNECT)
            //{
                DEBUG_PRINT("\r\n====response error,Abort!!!====\r\n");

                //delay_ms(100);
                //查询错误代码   AT+QIGETERROR
                //if (AT_CMD("AT+QIGETERRO\r", "OK", 2000) == AT_OK)
                
                gprs_cfg.rcv_error++;
#ifdef ENABLE_AGPS
                if(AgpsData.need_down_agps_flag == 0)
#else
                if(DeviceInfo.DeviceCommunicationFlag == TRUE) //如果是在发tcp数据时返回ERROR 
#endif
                {
                    DEBUG_PRINT("\r\n====shut net,reconnect!!!====\r\n");
                    
                    DeviceInfo.DeviceCommunicationFlag = FALSE;
                    gprs_cfg.connect_ok = 0;
                    gprs_set_state(GPRS_SHUT);
                }
                
                #ifdef FTP_UPGRAGE
				if(AtFTPGET.DownFlag == TRUE)
	            {
                    AtFTPGET.DownFlag = FALSE;
	                gprs_cfg.connect_ok = 0;
	                //gprs_set_state(GPRS_SHUT);
	            }
				#endif
                
                OSSemPendAbort ((OS_SEM  *)&gprs_check_sem,
                                (OS_OPT   )OS_OPT_PEND_ABORT_1,
                                (OS_ERR  *)&err);
            //}
            
            if(gprs_cfg.rcv_error >= 10)
            {
                DEBUG_PRINT("\r\n====timeout 10 time ERROR,reboot!!!====\r\n");

                gprs_cfg.rcv_error = 0;
                gprs_cfg.connect_ok = 0;
                gprs_set_state(GPRS_POWER_OFF);
            }
        }
        else
        {
            //接收到数据显示
            p2 = gprs_get_ipd(data_ptr);
            if(p2 != NULL)
            {
                if(gprs_cfg.connect_ok)
                {
                    #ifdef DEBUG_OUTPUT0
                    //sprintf((char*)p1,"\r\n>>>>> Receive [%d] Byte from server >>>>>\r\n ",ipd_len);
                    //DEBUG_PRINT("p1=%s",p1);
                    //DEBUG_PRINT("RcvData=%s",p2);
                    #endif
                    
                    PlatformRcvDataPtr = (uint16_t)ipd_len-1;
                    strncpy((char*)PlatformRcvDataBuffer,(char*)p2,ipd_len);
                    OSSemPost(&gprs_platform_sem,OS_OPT_POST_1,&err);
                }
            }
            else
            {
                gprs_cfg.rcv_error = 0;
                //gprs_cmd_handle(data_ptr,len);
            }
        }
        
//        #ifdef DEBUG_OUTPUT
//        //fish_print("%s",data_ptr);
//        DEBUG_PRINT("%s",data_ptr);
//        #endif
    }
}

/*
*********************************************************************************************************
*	函 数 名: gprs_cycle_handle
*	功能说明: gprs循环检测GPRS模块状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
#define LINK_TIME_OUT 1020
void gprs_cycle_handle(void)
{
    uint16_t time_out = 0;
    
    if (gprs_cfg.connect_ok == 0)
    {
        if (gprs_cfg.device_ok)
        {
            if (++time_out >= LINK_TIME_OUT)
            {
                time_out = 0;
                if (gprs_state == GPRS_CONNECT)
                {
                    gprs_cfg.connect_ok = 0;
                    DEBUG_PRINT("\r\nreconnect 4G Module!!!");
                    gprs_set_state(GPRS_SHUT);
                }
                else
                {
                    gprs_cfg.connect_ok = 0;
                    DEBUG_PRINT("\r\nreboot 4G Module!!!");
                    gprs_set_state(GPRS_POWER_OFF);
                }
            }
        }
        else
            time_out = 0;
    }
    else
    {
        time_out = 0;
        if(gprs_cfg.send_fail_cnt >= 5)
        {
            gprs_cfg.send_fail_cnt = 0;
            gprs_cfg.connect_ok = 0;
            DEBUG_PRINT("\r\n[ WARN ]: Send gprs datapacked fail 5 time,anew init module!!!");
            gprs_set_state(GPRS_INIT);
        }
        
        if(gprs_cfg.heartbeat_cnt >= 8)
        {
            DEBUG_PRINT("\r\n[ WARN ]: heartbeat packet timeout,reboot module!!!");
            //gprs_set_state(GPRS_POWER_OFF);
            //gprs_cfg.connect_ok = 0;	
            gprs_cfg.heartbeat_cnt = 0;
        }
    }
}

/*
*********************************************************************************************************
*	函 数 名: sim800_gprs_poll
*	功能说明: sim800GPRS轮询处理
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void sim800_gprs_poll(void)
{
    gprs_init_handle();
    gprs_cycle_handle();
}

/*
*********************************************************************************************************
*	函 数 名: Gprs_Poll_Task 
*	功能说明: GPRS轮询任务
*	形    参：p_arg
*	返 回 值: 无
*********************************************************************************************************
*/
void Gprs_Poll_Task(void *p_arg)
{
    OS_ERR err;

    (void)p_arg;
 
#ifdef  ENABLE_AGPS    
    agps_init();
#endif
    
	//printf("\r\ngprs poll task run!!!");
    while(1)
    {
        sim800_gprs_poll();
    
        OSTimeDlyHMSM (0, 0, 0, 50,OS_OPT_TIME_PERIODIC,&err);
        
        #ifdef ENABLE_IDWG
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )GPRS_POLL_TASK_FEED_DOG,  /* 设置bit3 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        #endif
    }
}

void Gsm_Idle_loop(void)
{
    static gprs_ilde_check_t ilde_check_state = GPRS_ILDE_CHECK_STE;
    uint8_t rssi=0,cops=0;
    uint16_t cbc;
    static uint8_t Time = 0;

#ifdef FTP_UPGRAGE 
    if(FtpUpFlag == TRUE) return;  //升级停止查询 
#endif
    
#ifdef  ENABLE_AGPS
    if(AgpsData.need_down_agps_flag == 1)return;  //下载agps，暂停查询 
#endif
    
    Time++;

    if(Gsm_IsHaveSmsInitiativeHint()) //有短信，先读短信 
        ilde_check_state = GPRS_ILDE_CHECK_SMS;
    
#ifdef ENABLE_AT_PORT_READ_GPS_INFO     
    if(Time >= 20)
#else
    if(Time >= 20)
#endif
    {
        Time = 0;
 
        switch(ilde_check_state)
        {
            case GPRS_ILDE_CHECK_STE:
                if(gprs_cfg.connect_ok == 1) 
                {
                    if(2 == CheckGprsState())
                        ilde_check_state = GPRS_ILDE_CHECK_CSQ;
                }
                else
                {
                    ilde_check_state = GPRS_ILDE_CHECK_CSQ;
                }
                    
                break;
            
            case GPRS_ILDE_CHECK_CSQ:
                if(0 == gprs_read_rssi(&rssi))
                {
                    #ifdef DEBUG_OUTPUT0
                    DEBUG_PRINT("gsm_rssi=%d\r\n",rssi);
                    #endif
                    if(rssi<=31)
                        DeviceInfo.ModuleSignalValue = rssi;
                    ilde_check_state = GPRS_ILDE_CHECK_SMS;//GPRS_ILDE_CHECK_CBC;
                }
                break;
            
            case GPRS_ILDE_CHECK_CBC:
                if(0 == gprs_read_cbc(&cbc))
                {
                    #ifdef DEBUG_OUTPUT0
                    DEBUG_PRINT("vbat=%dmV\r\n",cbc);	
                    #endif
                    
                    DeviceInfo.VbatValue = cbc; 
                    
                    #ifdef ENABLE_AT_PORT_READ_GPS_INFO
                    ilde_check_state = GPRS_ILDE_GPS_RMC;
                    #else
                    ilde_check_state = GPRS_ILDE_CHECK_SMS;
                    #endif
                }
                break;
                
            case GPRS_ILDE_CHECK_SMS:
                ProcessingAllSms();   //处理所有短信
                ilde_check_state = GPRS_ILDE_CHECK_COPS;
                break;
            
            case GPRS_ILDE_CHECK_COPS: //查询当前网络 
                if(0 == gprs_read_cops(&cops))
                {
                    #ifdef DEBUG_OUTPUT0
                    DEBUG_PRINT("\r\ncops=%d",cops);
                    #endif
                    if(cops == 2)
                        DeviceInfo.CopsFlag = NET_COPS_2G;
                    else if(cops == 3)
                        DeviceInfo.CopsFlag = NET_COPS_3G;
                    else if(cops == 4)
                        DeviceInfo.CopsFlag = NET_COPS_4G;
                    
                    ilde_check_state = GPRS_ILDE_CHECK_STE;
                }
                break;
#ifdef ENABLE_AT_PORT_READ_GPS_INFO               
            case GPRS_ILDE_GPS_RMC:
                if(0 == gprs_read_gprmc())
                {	
                    ilde_check_state = GPRS_ILDE_GPS_GGA;
                }
                break;
            
            case GPRS_ILDE_GPS_GGA:
                if(0 == gprs_read_gpgga())
                {	
                    if(gprs_cfg.connect_ok != 0) 
                        ilde_check_state = GPRS_ILDE_CHECK_STE;
                    else
                        ilde_check_state = GPRS_ILDE_CHECK_CSQ;
                }
                break;
#endif
                
            default:
                break;
        } 
    }
}

/*
*********************************************************************************************************
*	函 数 名: Gprs_Send_Task 
*	功能说明: GPRS发送任务
*	形    参：p_arg
*	返 回 值: 无
*********************************************************************************************************
*/
void Gprs_Send_Task(void *p_arg)
{
    #ifdef ENABLE_IDWG
    OS_ERR err;
    #endif
    (void)p_arg;
    //uint8_t err;
    
	//printf("\r\ngprs send task run!!!");
    
    while(1)
    {
        delay_ms(100);
        if((gprs_state > GPRS_INIT)&&(DeviceInfo.DeviceCommunicationFlag == FALSE)) 
        {
            Gsm_Idle_loop();
        }

#ifdef FTP_UPGRAGE        
        if(FtpUpFlag != TRUE)
#endif
        {
            DeviceRegisterToPlatform(); 
            GsmGpsFunctionProc();    
            SystemPowerCheck();       
            #ifndef  BEEP_SOUND_PLAY
            VibrationAlertProc();    
            #endif
        }
        
        #ifdef  ENABLE_AGPS
        request_agps_proc();
        #endif
        
        #ifdef ENABLE_IDWG
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )GPRS_SEND_TASK_FEED_DOG,  /* 设置bit4 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        #endif
    }
}  

//int uart_read_all(AtCmd_st *at_com, uint8_t *buf)
//{
//    CPU_SR_ALLOC();  
//        
//    if (at_com->rx_size == 0)
//        return 0;

//    OS_CRITICAL_ENTER();
////    debug_print("\r\nlen:%drecv:%s",st_com->rx_size,st_com->RxBuf);
//    memcpy(buf, at_com->AtRxBuf, at_com->rx_size);
//    int recv_len = at_com->rx_size;
//    memset(at_com->AtRxBuf, 0, QUEUE_LENGTH);
//    at_com->rx_size = 0;
//    OS_CRITICAL_EXIT();
//    
//    return recv_len;
//}

/*
*********************************************************************************************************
*	函 数 名: Gprs_rev_task 
*	功能说明: GPRS接收任务
*	形    参：p_arg
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t gprs_tmp_buf[1050];
void Gprs_rev_task(void *p_arg)
{
    OS_ERR err;
    uint16_t len;
//    uint16_t ucPos = 0,SumSize = 0,m;
//    char *data_ptr1 = NULL,*data_ptr2 = NULL;
    
//    printf("\r\ngprs_rev_task_run!!!");
    while (1)
    {
        OSSemPend((OS_SEM  *)&gprs_recv_sem,
                  (OS_TICK  )OSCfg_TickRate_Hz/50, 
                  (OS_OPT   )OS_OPT_PEND_BLOCKING,
                  (CPU_TS  *)0,
                  (OS_ERR  *)&err);
        if(OS_ERR_NONE == err)
        {
            memset(gprs_tmp_buf, 0, 1050);
            if(ENPROCESS == ComQueueOut(&gprs_serial, (char*)gprs_tmp_buf, (uint16_t*)&len))
            {
                //if(len > 200) printf("\r\nlen=%d,%s\r\n",len,gprs_tmp_buf);
                #ifdef DEBUG_OUTPUT0
                if(len > 200)
                {
                    Debug_PrintData("1Packet->",(unsigned char*)gprs_tmp_buf,(unsigned char*)(gprs_tmp_buf + len));
                }
                #endif
                gprs_urc_rev_handle((char*)gprs_tmp_buf, len);
            }
        } 
        
        //SystemLedCtrl();
        
        #ifdef ENABLE_IDWG
        OSFlagPost ((OS_FLAG_GRP  *)&FLAG_TaskRunStatus,
                    (OS_FLAGS      )GPRS_REV_PROC_FEED_DOG,  /* 设置bit2 */
                    (OS_OPT        )OS_OPT_POST_FLAG_SET,
                    (OS_ERR       *)&err);
        #endif
    }
}

/*
*********************************************************************************************************
*	函 数 名: Gprs_platform_proc_task 
*	功能说明: 平台数据处理任务
*	形    参：p_arg
*	返 回 值: 无
*********************************************************************************************************
*/
void Gprs_platform_proc_task(void *p_arg)
{
    OS_ERR err; 
    p_arg = p_arg;
    
//    printf("\r\nGprs_platform_proc_task Run!!!");
    while (1)
    {
        OSSemPend((OS_SEM      *)&gprs_platform_sem,    
                  (OS_TICK      )OSCfg_TickRate_Hz/10, 
                  (OS_OPT       )OS_OPT_PEND_BLOCKING,
                  (CPU_TS      *)0,                    
                  (OS_ERR      *)&err);
        if(OS_ERR_NONE == err)
        {
            PlatformRcvPacketProc(); 
        }
    }
}

#ifdef  ENABLE_AGPS
//连接AGPS服务器
//成功返回0，否则返回1
uint8_t connect_agps_server(void)
{
    uint8_t *p2,*p3;
    uint16_t tmp=1;
    uint8_t result = 1;
    OS_ERR err;
    char TempBuff[80],i;  //AT+QIOPEN=1,0,"TCP","119.123.126.159",8832,1234,1
    
    char AT_QIOPEN[80] = "AT+QIOPEN=1,1,\"TCP\",\"119.123.125.11\",8832,1234,1";
        
    if(gprs_cfg.connect_ok == 1)
    {
        OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
        
        memset(TempBuff,0,60);
        for(i = 0; AT_QIOPEN[i] != '\0'; i++)
            TempBuff[i] = AT_QIOPEN[i];
        TempBuff[i] = 0x0D;
        
        //AT+QIOPEN=1,1,"TCP","www.gnss-aide.com",2621,1234,1  121.41.40.95
        if (AT_CMD(TempBuff, "+QIOPEN: ", 6000) == AT_OK)  //+QIOPEN: 0,0 最长等待60S
        {
            p2 = (uint8_t*)cmd_resp;
            if(p2) 
            {
                p3=(uint8_t*)strstr((const char*)p2,",");
                tmp = atoi((char*)p3+1);
                
                if(0 == tmp)
                {
                    result = 0;
                }
                else
                {
                    DEBUG_PRINT("\r\nerr=%d,Socket bind failed\r\n",tmp);
                }
            }
        }
        
        //delay_ms(50);
        
        OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    }
    return result;
}

//const char CMD_BUF[100]="user=freetrial;pwd=123456;cmd=full;gnss=gps+bd;lat=30.5;lon=120.5;";
char CMD_BUF[100] = "user=freetrial/pwd=123456/cmd=full/gnss=gps+bd/lat=30.5/lon=120.5/";
//uint8_t CMD_BUF[68]={
//    0x75,0x73,0x65,0x72,0x3D,0x66,0x72,0x65,0x65,0x74,
//    0x72,0x69,0x61,0x6C,0x3B,0x70,0x77,0x64,0x3D,0x31,
//    0x32,0x33,0x34,0x35,0x36,0x3B,0x63,0x6D,0x64,0x3D,
//    0x66,0x75,0x6C,0x6C,0x3B,0x67,0x6E,0x73,0x73,0x3D,
//    0x67,0x70,0x73,0x2B,0x62,0x64,0x3B,0x6C,0x61,0x74,
//    0x3D,0x33,0x30,0x2E,0x35,0x3B,0x6C,0x6F,0x6E,0x3D,
//    0x31,0x32,0x30,0x2E,0x35,0x3B,0x0D,0x0A};
//发送请求辅助数据
uint8_t request_agps_from_server(void)
{
    char cmd_buf[100];
    uint8_t result = 1;
    OS_ERR err;
    
    //AT+QICFG="dataformat",1,0
    //if (AT_CMD("AT+QICFG=\"dataformat\",1,0\r", "OK", 100) == AT_OK)
                
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
    memset(cmd_buf,0,100);
    
    snprintf((char*)cmd_buf, sizeof(cmd_buf), "AT+QISEND=1,%d,",strlen(CMD_BUF));  
    strncat((char*)cmd_buf,(const char*)CMD_BUF,strlen(CMD_BUF));
    strncat((char*)cmd_buf,"\r",1);
    
//    snprintf((char*)cmd_buf, sizeof(cmd_buf), "AT+QISENDEX=1,%d,",sizeof(CMD_BUF));  
//    for(i = 0; i < 68; i++)
//            cmd_buf[17] = CMD_BUF[i];
    
    AgpsData.agps_rcv_flag = 1;
    
    if(AT_OK == AT_CMD((char*)cmd_buf, "SEND OK", 3000))
    {
        if(gprs_cfg.rcv_error == 0) 
        {
            led_set_mode(SEND_DATA);
            DEBUG_PRINT("\r\n发送请求AGPS数据成功!!!\r\n");
            result = 0;
        }
        else gprs_cfg.rcv_error = 0;
    }
    else 
    {
        DEBUG_PRINT("\r\n发送请求AGPS数据失败!!!\r\n");
    }
    
    //delay_ms(500);
    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    //AT_CMD("AT+QICFG=\"dataformat\",0,0\r", "OK", 100);
        
    return result;
}

/*
*********************************************************************************************************
*	函 数 名: agps_close 
*	功能说明: 关闭agps连接
*	形    参: void
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t agps_close(void)
{
    uint8_t result = 1;
    OS_ERR err;
    
    OSSemPend((OS_SEM  *)&gprs_busy_sem,
              (OS_TICK  )0,
              (OS_OPT   )OS_OPT_PEND_BLOCKING,
              (CPU_TS  *)0,
              (OS_ERR  *)&err);
   
    if (AT_CMD("AT+QICLOSE=1\r", "CLOSE OK", 1000) == AT_OK)
        result = 0;

    delay_ms(150);

    OSSemPost(&gprs_busy_sem,OS_OPT_POST_1,&err);
    
    return result;
}
#endif //#ifdef  ENABLE_AGPS

/*
*********************************************************************************************************
*	函 数 名: ec20_task_creat 
*	功能说明: 任务创建
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ec20_task_creat(void)
{
    OS_ERR err;
    
    ec20_hw_init();

    OSTaskCreate((OS_TCB     *)&GprsRecvTaskTCB,
                 (CPU_CHAR   *)"gprs_recv_task",
                 (OS_TASK_PTR ) Gprs_rev_task,
                 (void       *) 0,
                 (OS_PRIO     ) GPRS_RECV_TASK_PRIO,
                 (CPU_STK    *)&Gprs_Recv_Task_Stk[0],
                 (CPU_STK_SIZE) GPRS_RECV_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) GPRS_RECV_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);   
    
    OSTaskCreate((OS_TCB     *)&GprsPlatformTaskTCB,
                 (CPU_CHAR   *)"gprs_platform_task",
                 (OS_TASK_PTR ) Gprs_platform_proc_task,
                 (void       *) 0,
                 (OS_PRIO     ) GPRS_PLATFORM_TASK_PRIO,
                 (CPU_STK    *)&Gprs_platform_Task_Stk[0],
                 (CPU_STK_SIZE) GPRS_PLATFORM_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) GPRS_PLATFORM_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err); 
                           
    OSTaskCreate((OS_TCB     *)&GprsPollTaskTCB,
                 (CPU_CHAR   *)"gprs_poll_task",
                 (OS_TASK_PTR ) Gprs_Poll_Task,
                 (void       *) 0,
                 (OS_PRIO     ) GPRS_POLL_TASK_PRIO,
                 (CPU_STK    *)&Gprs_Poll_Task_Stk[0],
                 (CPU_STK_SIZE) GPRS_POLL_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) GPRS_POLL_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err); 

                                 
    OSTaskCreate((OS_TCB     *)&GprsSendTaskTCB,
                 (CPU_CHAR   *)"gprs_send_task",
                 (OS_TASK_PTR ) Gprs_Send_Task,
                 (void       *) 0,
                 (OS_PRIO     ) GPRS_SEND_TASK_PRIO,
                 (CPU_STK    *)&Gprs_Send_Task_Stk[0],
                 (CPU_STK_SIZE) GPRS_SEND_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) GPRS_SEND_TASK_STK_SIZE, 
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);                
    
}

#endif //#ifdef USE_EC20_MODULE
