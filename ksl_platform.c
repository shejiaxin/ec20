#define APP_KSL_PLATFORM_GLOBALS

#include "includes.h"


gprs_cfg_t gprs_cfg = {0};

void PlatformInit_Special(void)
{
    unsigned char i;

    #ifdef DEBUG_OUTPUT0
    DEBUG_PRINT("\r\nHY Platform");
    #endif

    DeviceInfo.SimCcidUpFlag = FALSE;               
    DeviceInfo.SendSimCcidTimeCnt = THIRTY_SECOND;   
    
    DeviceInfo.SimBmsIdUpFlag = FALSE;
    DeviceInfo.SendBmsIdTimeCnt = ONE_MINUTE; 
    DeviceInfo.SendBmsMsgTimeCnt = 50;
    DeviceInfo.SendCtrlMsgTimeCnt = 40; 
	
    PlatformData.DeviceRegisterSuccess     = FALSE;
    PlatformData.DeviceRegisterTimeoutCnt  = THIRTY_SECOND;

    PlatformData_HY.DeviceConnectCheckTimeCnt = TWO_MINUTE; 
    PlatformData_HY.PlatformSendFailCnt       = 0;
    PlatformData_HY.HandshakeFailCnt          = 0;
    PlatformData_HY.HaveAlertFlag = FALSE;
    PlatformData_HY.QuerySoftVerFlag = FALSE;
    for (i = 0; i < MAX_ALERT_SUM; i ++)
    {
        PlatformData_HY.AlertType[i].AlertFlag = FALSE;
        PlatformData_HY.AlertType[i].AlertSum  = 0;
    }

    //platform device id
    ReadDeviceId(); 

    #ifdef DEBUG_OUTPUT0
    Debug_PrintData("HY Platform ID:",PlatformData.DeviceID,PlatformData.DeviceID + 15);
    #endif

    ValidShakingFlag   = FALSE;
    u8WheelRolling     = 0;
    u8WheelAlarmFlag     = FALSE;
    u16WheelRollingDelay = THREE_SECOND;
    AlartLockedFlag    = FALSE;           
    NeddSleepFlag      = FALSE;
    AccoffWakeupFlag   = FALSE;
//    HeartbeatPacketsTimeCnt = ONE_MINUTE;  
    AlarmDelayCheckGsensor = TEN_SECOND;
    GprsConnectTimeOut = FIFTEEN_MINUTE; 
    
    GetPositionData.RequestTimeoutCnt = 0;
    GetPositionData.RequestFlag = FALSE;
    GetPositionData.RequestTelephone[0] = 0;

#ifdef DOUBLE_IP_FUN 
    PlatformData.MainBackupIPFlag = 0; 
    PlatformData.AutoSwitchMainIpTime = 60;
#endif

//    PlatformData_HY.AnewConnectTimeCnt = ONE_HOUR;  
    PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_NEW;
    
    PlatformData_HY.ResendFailPacketRemainSeconds = 0;
    PlatformData_HY.SendRecordDataRemainSeconds   = 0;
    PlatformData_HY.ResendPacketFlag              = FALSE;
    AutoTrackData.RemainTime = 30;
}

//平台初始化
void PlatformInit(void)
{
    ReadIpAndPort();
    ReadDomainName();
    
    #ifdef DEBUG_OUTPUT0
    Debug_PrintValue("IP Addr1:", PlatformData.PlatformIpAddr[0]);
    Debug_PrintValue("IP Addr2:", PlatformData.PlatformIpAddr[1]);
    Debug_PrintValue("IP Addr3:", PlatformData.PlatformIpAddr[2]);
    Debug_PrintValue("IP Addr4:", PlatformData.PlatformIpAddr[3]);
    Debug_PrintValueInt("Port:", PlatformData.PlatformPort);
    #endif

    PlatformData.SendFailCnt = 0;
    
    //具体平台初始化
    PlatformInit_Special();
}

//*************************************************
//设备注册到平台
void DeviceRegisterToPlatform(void)
{
    #ifdef  USE_MJ_MULTI_MODE
    static uint8_t send_br07_flag = 0, send_br08_flag = 0;
    #endif
    
    if(DeviceInfo.DeviceCallFlag == TRUE)
    {
        return;
    } 
    
    
    if ((DeviceInfo.SimCcidUpFlag == FALSE) && (DeviceInfo.SendSimCcidTimeCnt == 0) 
        && (PlatformData.DeviceRegisterSuccess == TRUE)) 
    {
        DeviceInfo.SendSimCcidTimeCnt = ONE_MINUTE;
        //发送SIM卡CCID
        //DEBUG_PRINT("\r\nSend SIM CCID msg.");

        SendMessageBP06();
        delay_ms(1000);
        SendSoftWareVersion();
    }
    
    if ((DeviceInfo.SimBmsIdUpFlag == FALSE) && (DeviceInfo.SendBmsIdTimeCnt == 0) 
        && (PlatformData.DeviceRegisterSuccess == TRUE)) 
    {
        DeviceInfo.SendBmsIdTimeCnt = ONE_MINUTE;
        //发送电池ID消息
        //DEBUG_PRINT("\r\nSend BMS BP11 msg.\r\n");
        
        #ifdef  USE_BH_LEASE_MODE
        SendMessageBP11();
        /*delay_ms(1000);
        //DEBUG_PRINT("\r\nSend NFC BP15 msg.\r\n");
        SendMessageBP15();*/
        #endif
    }

#ifdef  USE_BH_LEASE_MODE 
    
    if ((DeviceInfo.SendBmsMsgTimeCnt == 0) && 
        (PlatformData.DeviceRegisterSuccess == TRUE) &&
        (GpsFunctionState & FUNC_AUTO_TRACK_BIT) != 0)
    {
        DeviceInfo.SendBmsMsgTimeCnt = 60;
        
        //DEBUG_PRINT("\r\nSend BMS BR04 msg.\r\n");

        SendMessageBR04();
    }
    
    
    if ((DeviceInfo.SendCtrlMsgTimeCnt == 0) && 
        (PlatformData.DeviceRegisterSuccess == TRUE) && 
        (GpsFunctionState & FUNC_AUTO_TRACK_BIT) != 0)
    {
        DeviceInfo.SendCtrlMsgTimeCnt = 60;
        
        //DEBUG_PRINT("\r\nSend BMS BR09 msg.\r\n");
        
        SendMessageBR09();
    }
#endif
#ifdef USE_MJ_MULTI_MODE    
    
    if ((send_br08_flag == 0) && 
        (DeviceInfo.SendBmsMsgTimeCnt == 0) && 
        (PlatformData.DeviceRegisterSuccess == TRUE))
    {
        send_br08_flag = 1;
        DeviceInfo.SendBmsMsgTimeCnt = 5;
        
        SendMessageBR08(BMS_NUM_1);
    }
    else if ((send_br08_flag == 1) && 
            (DeviceInfo.SendBmsMsgTimeCnt == 0) && 
            (PlatformData.DeviceRegisterSuccess == TRUE))
    {
        send_br08_flag = 0;
        DeviceInfo.SendBmsMsgTimeCnt = 75;
        SendMessageBR08(BMS_NUM_2);
    }

	
    if ((send_br07_flag == 0) && 
        (DeviceInfo.SendCtrlMsgTimeCnt == 0) && 
        (PlatformData.DeviceRegisterSuccess == TRUE))
    {
        send_br07_flag = 1;
        DeviceInfo.SendCtrlMsgTimeCnt = 5;
        SendMessageBR07(CTL_NUM_1);
    }
    else if ((send_br07_flag == 1) && 
            (DeviceInfo.SendCtrlMsgTimeCnt == 0) && 
            (PlatformData.DeviceRegisterSuccess == TRUE))
	{
        send_br07_flag = 0;
		DeviceInfo.SendCtrlMsgTimeCnt = 55;
		SendMessageBR07(CTL_NUM_2);
	}
#endif  //#ifdef  USE_MJ_MULTI_MODE

    
    if((PlatformData_HY.DeviceConnectCheckTimeCnt == 0) && (PlatformData.DeviceRegisterSuccess == TRUE))
    {     
        PlatformData_HY.DeviceConnectCheckTimeCnt = TWO_MINUTE;
        PlatformData_HY.ResendFailPacketRemainSeconds = 5;
        PlatformData_HY.HandshakeFailCnt ++;
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nSend Handshake Msg.");
        #endif
        //连接正常，发送握手信号消息
        SendMessageBP00();
    }

    if ((PlatformData_HY.PlatformSendFailCnt >= 5) && (DeviceInfo.GsmRegisteredFlag == TRUE))    
    {
        DEBUG_PRINT("\r\n---Send fail 5,anew register.---\r\n");
         
        gprs_set_state(GPRS_SHUT);
        
        //PlatformData.DeviceRegisterSuccess = FALSE;
        PlatformData.DeviceRegisterTimeoutCnt = FIVE_MINUTE;
        PlatformData_HY.PlatformSendFailCnt      = 0;
    }

    if ((PlatformData_HY.HandshakeFailCnt >= 2)&&(PlatformData_HY.ResendFailPacketRemainSeconds == 0))
    {   
        PlatformData_HY.HandshakeFailCnt = 0;

        DEBUG_PRINT("\r\n----Handshake fail 2,anew register!!!----\r\n");
        
#ifdef DOUBLE_IP_FUN 
        if(PlatformData.MainBackupIPFlag == 0)
        {
            PlatformData.MainBackupIPFlag = 1;
            PlatformData.AutoSwitchMainIpTime = 60;
        }
        else
            PlatformData.MainBackupIPFlag = 0;
        
#endif 
        
        PlatformData_HY.DeviceConnectCheckTimeCnt = TEN_SECOND; 
        
        //PlatformData.DeviceRegisterSuccess = FALSE; 
        //INTX_DISABLE();
        PlatformData.DeviceRegisterTimeoutCnt = TEN_SECOND;
        //INTX_ENABLE();
       
        gprs_set_state(GPRS_SHUT);
    } 
    
    
    #ifdef STM32RECORD  
    //发送记录的数据包
    if((PlatformData_HY.SendRecordDataRemainSeconds == 0) && (PlatformData.StatisticSendSuccessSum != 0) 
       && (RecordState.ReadOffsetPtr != RecordState.WriteOffsetPtr)  && (PlatformData.SendFailCnt == 0)
       && (gprs_cfg.connect_ok == 1))  
    {
        PlatformData_HY.SendRecordDataRemainSeconds = 10;
        
        if((ReadRecordFromMemory(&ReadRecordData) == TRUE) && (gprs_cfg.connect_ok == 1))
        {
            unsigned char SendResult;
            
            DEBUG_PRINT("\r\nSend Record Data.");
            
            PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_RECORD;  
            if(ReadRecordData.Alert != 0xFF)
            {
                SendResult = SendMessageBO02();
            }
            else 
            {
                SendResult = SendMessageBR01();
            }
            PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_NEW;
        
            if (SendResult == TRUE)
            {
                //发送记录数据成功，记录数减少一个
                ChangeReadRecordPtr();
            }
        }
    }
    #endif  //#ifdef STM32RECORD  
    
    #ifdef SPI_FLASH_RECORD  
    //发送记录的数据包
    if((PlatformData_HY.SendRecordDataRemainSeconds == 0) && (PlatformData.DeviceRegisterSuccess == TRUE) 
       && (RecordState.ReadOffsetPtr != RecordState.WriteOffsetPtr) && (PlatformData.SendFailCnt == 0)
       &&(gprs_cfg.connect_ok == 1)) 
    {
        PlatformData_HY.SendRecordDataRemainSeconds = 10;
              
        if(SpiFlashReadRecordFromMemory(&SpiReadRecordData) == TRUE)
        {
            unsigned char SendResult;
                     
            DEBUG_PRINT("\r\n---Send Record Data.---\r\n");
            
            PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_RECORD;  
            if(SpiReadRecordData.Alert != 0xFF)
            {
                SendResult = SendMessageBO02();
            }
            else 
            {
                SendResult = SendMessageBR01();
            }
            
            PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_NEW;
        
            if (SendResult == TRUE)
            {
                //发送记录数据成功，记录数减少一个
                SpiFlashChangeReadRecordPtr();
            }
        }
        
    }
    #endif  //#ifdef SPI_FLASH_RECORD  
}


//*********************************************
unsigned char CharAdd(unsigned char Char1, unsigned char Char2)
{
    unsigned char Char;

    Char = CharToValue(Char1) + CharToValue(Char2);
    Char = ValueToChar(Char % 10);

    return Char;
}

uint8_t SendTcpPacketExt(uint8_t *pPacketBuffer, uint16_t PacketLen)
{
    //uint16_t  n,res,i;
    //uint8_t buf[8]; 
    //uint8_t tempbuf[30]="AT+CIPSEND=";
    
    if (DeviceInfo.DeviceCallFlag == TRUE)
    {
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nCall,Stop Send Data.");
        #endif
        return FALSE;        
    } 

    #ifdef DEBUG_OUTPUT0
    Debug_PrintData("Send Data Packet->",pPacketBuffer,pPacketBuffer + PacketLen);
    #endif

    #ifdef TEST_OUTPUT  
    pPacketBuffer[PacketLen++] = '\0'; 
    Test_Output_Tcp_Data(pPacketBuffer);
    #endif
    
    if( 0 == gprs_cfg.connect_ok) 
        return FALSE; 

    if(0 == gprs_send((uint8_t*)pPacketBuffer,PacketLen))
    {
        gprs_cfg.send_fail_cnt = 0; 
        gprs_cfg.send_success_sum++;
        return TRUE;
    }
    else
    {
        gprs_cfg.send_fail_cnt++;
        gprs_cfg.send_fail_sum++;
        
        DEBUG_UART("\r\n[ ERROR ]: Send gps datapacked fail!!!\r\n");

        gprs_cfg.connect_ok = 0;
        gprs_set_state(GPRS_SHUT);
        return FALSE;
    }
}

//*********************************************
unsigned char SendTcpPacket(unsigned char *pPacketBuffer, unsigned char PacketLen)
{
    unsigned char SendState;

    
    if (DeviceInfo.DeviceWorkState != DEVICE_WORK_STATE_RUNNING) 
    {
        DEBUG_PRINT("\r\nModule No Work.");

        return FALSE;
    }
    
    if (DeviceInfo.DeviceCallFlag == TRUE)  
    {
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nCall Status,Stop Send Data.");
        #endif
        return FALSE;
    }

    DeviceInfo.DeviceCommunicationFlag = TRUE;
    
    SendState = SendTcpPacketExt(pPacketBuffer, PacketLen);
    
    DeviceInfo.DeviceCommunicationFlag = FALSE;
    
    //发送结果处理
    if (SendState == FALSE)
    {
        PlatformData.SendFailCnt ++;
        PlatformData.StatisticSendFailSum ++;
        PlatformData_HY.PlatformSendFailCnt ++;
    }
    else
    {
        DeviceInfo.SendTcpDataFlag = FALSE;
        DeviceInfo.SendTcpDataDlyCnt  = 0;
        DeviceInfo.NeedSaveBlindAreaFlag = FALSE;
        
        PlatformData.SendFailCnt = 0;
        PlatformData_HY.PlatformSendFailCnt = 0;
        PlatformData.StatisticSendSuccessSum ++;
    }

    #ifdef DEBUG_OUTPUT0
    if(PlatformData.StatisticSendSuccessSum % 10 == 0)
    {
        DEBUG_UART("\r\nHandshakeFailCnt:%d",PlatformData_HY.HandshakeFailCnt);
        DEBUG_UART("\r\nStatisticSendFailSum:%d", PlatformData.StatisticSendFailSum);
        DEBUG_UART("\r\nStatisticSendSuccessSum:%d\r\n", PlatformData.StatisticSendSuccessSum);
    }
    #endif

    GpsData.SendGpsPacketValid = FALSE;
    CloseSystemLed();

    return SendState;
}


//************************************************
//长度83个字节
//(863412040166772BP05863412040166772190609A2238.6541N11402.4895E000.00927580.000000000062L00000032S2513T000004)
//(863412040166780BP05863412040166780170924V0000.0000N00000.0000E000.0000000000.0010000
void WriteGpsDataToBufferNew(uint8_t StartAddr)
{
    uint8_t i;
	#ifdef MAIN_BOARD_TEMP_CHK
    uint8_t gpsTemperatureBuff[6];
	#endif
    //#ifdef ENABLE_CTRL_COMM
    uint8_t ctrlTemperatureBuff[6];
    //#endif
    uint8_t BatteryPower[6];
    
   // static uint8_t  First_Flag = FALSE;  
    
    //日期
    //GPS未定位，判断年为80，改为17
    if((GpsData.GpsSignalFlag == FALSE) && (GpsData.UTC_InitDate[4] == '8') 
        && (GpsData.UTC_InitDate[5] == '0'))
    {
        GeneralBuffer[StartAddr]     = '1';  //year
        GeneralBuffer[StartAddr + 1] = '9';
    }
    else
    {
        GeneralBuffer[StartAddr]     = GpsData.UTC_InitDate[4];  //year
        GeneralBuffer[StartAddr + 1] = GpsData.UTC_InitDate[5];
    }
    GeneralBuffer[StartAddr + 2] = GpsData.UTC_InitDate[2];  //month
    GeneralBuffer[StartAddr + 3] = GpsData.UTC_InitDate[3];
    GeneralBuffer[StartAddr + 4] = GpsData.UTC_InitDate[0];  //day
    GeneralBuffer[StartAddr + 5] = GpsData.UTC_InitDate[1];
      
    //GPS数据是否有效的标字
    if ((GpsData.GpsSignalFlag == TRUE) && (GpsData.bDataState == GPS_NEW_DATA))   
    {      
        GeneralBuffer[StartAddr + 6] = 'A';
    }
    else
    {     
        GeneralBuffer[StartAddr + 6] = 'V';
    }
//=========================================================================  
    //通过置GpsData.SendGpsPacketValid == FALSE过滤 
    if((GpsData.GpsSignalFlag == FALSE) || (GpsData.bDataState == GPS_OLD_DATA)
        || (DeviceInfo.Gps_hdop >= 3))
    {
        for (i = 0; i < 9; i++)
        {
            GpsData.InitLatitude[i]  = DeviceData.BackupLatitude[i];  //MyBackupLatitude[i];
        }

        for (i = 0; i < 10; i++)
        {
            GpsData.InitLongitude[i] = DeviceData.BackupLongitude[i];  //MyBackupLongitude[i];
        }
    }
//=========================================================================
    
    //纬度
    for (i = 0; i < 9; i++)
    {
        GeneralBuffer[StartAddr + 7 + i]  = GpsData.InitLatitude[i];
    }
    GeneralBuffer[StartAddr + 16]  = GpsData.LatitudeIndicator;

    //经度
    for (i = 0; i < 10; i++)   
    {
        GeneralBuffer[StartAddr + 17 + i]  = GpsData.InitLongitude[i];
    }
    GeneralBuffer[StartAddr + 27]  = GpsData.LongitudeIndicator;
    
    //速度
    if (GpsData.SendGpsPacketValid == TRUE)
    {
        for (i = 0; i < 5; i ++)   //speed
        {
            GeneralBuffer[StartAddr + 28 + i]  = GpsData.Speed[i];
        }
    }
    else
    {   
        for (i = 0; i < 5; i ++)   //speed
        {
            GeneralBuffer[StartAddr + 28 + i]  = '0';
        }
        GeneralBuffer[StartAddr + 28 + 3]  = '.';
    }

    //时间
    GeneralBuffer[StartAddr + 33] = GpsData.UTC_InitTime[0];  //hour
    GeneralBuffer[StartAddr + 34] = GpsData.UTC_InitTime[1];
    GeneralBuffer[StartAddr + 35] = GpsData.UTC_InitTime[2];  //minute
    GeneralBuffer[StartAddr + 36] = GpsData.UTC_InitTime[3];
    GeneralBuffer[StartAddr + 37] = GpsData.UTC_InitTime[4];  //second
    GeneralBuffer[StartAddr + 38] = GpsData.UTC_InitTime[5];
     
    //方向
    for (i = 0; i < 6; i++)
    {
        GeneralBuffer[StartAddr + 39 + i] = GpsData.Direction[i];
    }

    //IO状态
    for (i = 0; i < 8; i++)
    {
        GeneralBuffer[StartAddr + 45 + i] = '0';
    }
    
    if(F_CarPwrOn == FALSE)
    {
        GeneralBuffer[StartAddr + 45] = '1';
    }
    
//    if (DeviceInfo.CarAccWork == TRUE)
//    {
//        GeneralBuffer[StartAddr + 46] = '1';
//    }
    
//    if (F_CarPwrLow == TRUE) 
//    {
//        GeneralBuffer[StartAddr + 47] = '1';
//    } 
    
    if(PlatformData.MainBackupIPFlag == 1) 
    {
        GeneralBuffer[StartAddr + 48] = '1';
    }
    
    //------------电池电量------------- 
    NumberValueToHexStr((DeviceInfo.InputValue/100), BatteryPower);
    if((BatteryPower[0] < '0') || (BatteryPower[0] > 'F')) BatteryPower[0] = '0';
    if((BatteryPower[1] < '0') || (BatteryPower[1] > 'F')) BatteryPower[1] = '0';
    if((BatteryPower[2] < '0') || (BatteryPower[2] > 'F')) BatteryPower[2] = '0';

    GeneralBuffer[StartAddr + 50] = BatteryPower[0];
    GeneralBuffer[StartAddr + 51] = BatteryPower[1];
    GeneralBuffer[StartAddr + 52] = BatteryPower[2];
    
    /*NumberValueToHexStr((CtrlCommData.BatteryLevel/10), BatteryPower);
    GeneralBuffer[StartAddr + 50] = BatteryPower[0];
    GeneralBuffer[StartAddr + 51] = BatteryPower[1];
    GeneralBuffer[StartAddr + 52] = BatteryPower[2];*/
    //------------------------------------------
    //里程
    GeneralBuffer[StartAddr + 53] = 'L';
    for (i = 0; i < 8; i ++)
    {   
        GeneralBuffer[StartAddr + 54 + i] = GpsData.SumMeter[i];
    }

    //GSM信号值及GPS有效卫星数量
    GeneralBuffer[StartAddr + 62] = 'S';
    GeneralBuffer[StartAddr + 63] = ValueToChar(DeviceInfo.ModuleSignalValue % 100 / 10);
    GeneralBuffer[StartAddr + 64] = ValueToChar(DeviceInfo.ModuleSignalValue % 10);
    
    if ((GpsData.GpsSignalFlag == TRUE) && (GpsData.bDataState == GPS_NEW_DATA))   
    {
        GeneralBuffer[StartAddr + 65] = ValueToChar(DeviceInfo.GpsSatelliteTotal % 100 / 10);
        GeneralBuffer[StartAddr + 66] = ValueToChar(DeviceInfo.GpsSatelliteTotal % 10);
    }
    else
    {
        GeneralBuffer[StartAddr + 65] = '0';
        GeneralBuffer[StartAddr + 66] = '0';
    }
    
    //当前网络
    GeneralBuffer[StartAddr + 67] = ValueToChar(DeviceInfo.CopsFlag % 10);
    
    //电动车行车状态
    GeneralBuffer[StartAddr + 68] = 'T';  
#ifdef ENABLE_CTRL_COMM
    GeneralBuffer[StartAddr + 69] = ValueToChar((CtrlCommData.FailureState[0] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 70] = ValueToChar((CtrlCommData.FailureState[0] & 0x0F));
    GeneralBuffer[StartAddr + 71] = ValueToChar((CtrlCommData.FailureState[1] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 72] = ValueToChar((CtrlCommData.FailureState[1] & 0x0F));
#endif
#ifdef USE_BH_LEASE_MODE
    GeneralBuffer[StartAddr + 69] = ValueToChar((EbikeData.State[0] & 0xF0) >> 4);  
    GeneralBuffer[StartAddr + 70] = ValueToChar((EbikeData.State[0] & 0x0F));
    GeneralBuffer[StartAddr + 71] = ValueToChar((EbikeData.Fault[0] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 72] = ValueToChar((EbikeData.Fault[0] & 0x0F));
#endif
    GeneralBuffer[StartAddr + 73] = ValueToChar((gCarFunctionState & 0xF0) >> 4); 
    GeneralBuffer[StartAddr + 74] = ValueToChar((gCarFunctionState & 0x0F));

    GeneralBuffer[StartAddr + 75] = 'P'; 
#ifdef MAIN_BOARD_TEMP_CHK
    NumberValueToHexStr(DeviceInfo.TemperatureValue, gpsTemperatureBuff);
    GeneralBuffer[StartAddr + 76] = gpsTemperatureBuff[1];
    GeneralBuffer[StartAddr + 77] = gpsTemperatureBuff[2];
#else
	GeneralBuffer[StartAddr + 76] = '0';
    GeneralBuffer[StartAddr + 77] = '0';
#endif
#ifdef ENABLE_CTRL_COMM
    NumberValueToHexStr(CtrlCommData.ControlTemp, ctrlTemperatureBuff);
    GeneralBuffer[StartAddr + 78] = ctrlTemperatureBuff[1];
    GeneralBuffer[StartAddr + 79] = ctrlTemperatureBuff[2];
#else
    NumberValueToHexStr(EbikeData.Temperature, ctrlTemperatureBuff);
    GeneralBuffer[StartAddr + 78] = ctrlTemperatureBuff[1];
    GeneralBuffer[StartAddr + 79] = ctrlTemperatureBuff[2];
#endif

    GeneralBuffer[StartAddr + 80] = 'Q';
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[StartAddr + 81] = ValueToChar((Rs485Data.soc1 & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 82] = ValueToChar((Rs485Data.soc1 & 0x0F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[StartAddr + 81] = ValueToChar((LionBatData.soc & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 82] = ValueToChar((LionBatData.soc & 0x0F));
#else
    GeneralBuffer[StartAddr + 81] = '0';
    GeneralBuffer[StartAddr + 82] = '0';
#endif
}

#ifdef  USE_BH_LEASE_MODE
//************************************************
//长度32个字节 
//(863412040166772BR0435T0000P64R0041L3E80S084614)
void WriteBmsDataToBuffer(uint8_t StartAddr)
{
    uint8_t tmp=0;
#ifdef ENABLE_EXT_RS485
    uint16_t ave_cur = Rs485Data.ave_current/100;
#elif defined (ENABLE_CAN_COMM)
    uint16_t ave_cur = LionBatData.ave_current;
#else
    uint16_t ave_cur = 0;
#endif
    
    //电池内部温度,16进制格式
#ifdef ENABLE_EXT_RS485
    tmp = Rs485Data.temperature / 10;  //去掉小数，只上传整数
#elif defined (ENABLE_CAN_COMM)
    tmp = LionBatData.temperature / 10;
#else
    tmp = 0;
#endif
    GeneralBuffer[StartAddr]     = ValueToChar((tmp & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 1] = ValueToChar((tmp & 0x0F));
    //充电循环次数,16进制格式
    GeneralBuffer[StartAddr + 2] = 'T'; 
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[StartAddr + 3] = ValueToChar(Rs485Data.cycle_charge_cnt >> 12);
    GeneralBuffer[StartAddr + 4] = ValueToChar((Rs485Data.cycle_charge_cnt & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 5] = ValueToChar((Rs485Data.cycle_charge_cnt & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 6] = ValueToChar((Rs485Data.cycle_charge_cnt & 0x000F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[StartAddr + 3] = ValueToChar(BmsAppendData.cycle_charge_cnt >> 12);
    GeneralBuffer[StartAddr + 4] = ValueToChar((BmsAppendData.cycle_charge_cnt & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 5] = ValueToChar((BmsAppendData.cycle_charge_cnt & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 6] = ValueToChar((BmsAppendData.cycle_charge_cnt & 0x000F));
#else
    GeneralBuffer[StartAddr + 3] = '0';
    GeneralBuffer[StartAddr + 4] = '0';
    GeneralBuffer[StartAddr + 5] = '0';
    GeneralBuffer[StartAddr + 6] = '0';
#endif
    
    //电池健康度百分比，SOH=%0--100%
    GeneralBuffer[StartAddr + 7] = 'P';
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[StartAddr + 8] = ValueToChar((Rs485Data.soh & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 9] = ValueToChar((Rs485Data.soh & 0x0F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[StartAddr + 8] = ValueToChar((LionBatData.soh & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 9] = ValueToChar((LionBatData.soh & 0x0F));
#else
    GeneralBuffer[StartAddr + 8] = '0';
    GeneralBuffer[StartAddr + 9] = '0';
#endif

    //剩余容量，单位mAh，固定传5位，不足前补0 
    GeneralBuffer[StartAddr + 10] = 'R';
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[StartAddr + 11] = ValueToChar(Rs485Data.residue_capacity >> 12);
    GeneralBuffer[StartAddr + 12] = ValueToChar((Rs485Data.residue_capacity & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 13] = ValueToChar((Rs485Data.residue_capacity & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 14] = ValueToChar((Rs485Data.residue_capacity & 0x000F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[StartAddr + 11] = ValueToChar(BmsAppendData.residue_capacity >> 12);
    GeneralBuffer[StartAddr + 12] = ValueToChar((BmsAppendData.residue_capacity & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 13] = ValueToChar((BmsAppendData.residue_capacity & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 14] = ValueToChar((BmsAppendData.residue_capacity & 0x000F));
#else
    GeneralBuffer[StartAddr + 11] = '0';
    GeneralBuffer[StartAddr + 12] = '0';
    GeneralBuffer[StartAddr + 13] = '0';
    GeneralBuffer[StartAddr + 14] = '0';
#endif

    //满电容量，单位mAh，固定传5位，不足前补0   16000mAh
    GeneralBuffer[StartAddr + 15] = 'L';
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[StartAddr + 16] = ValueToChar(Rs485Data.full_capacity >> 12);
    GeneralBuffer[StartAddr + 17] = ValueToChar((Rs485Data.full_capacity & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 18] = ValueToChar((Rs485Data.full_capacity & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 19] = ValueToChar((Rs485Data.full_capacity & 0x000F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[StartAddr + 16] = ValueToChar(BmsAppendData.full_capacity >> 12);
    GeneralBuffer[StartAddr + 17] = ValueToChar((BmsAppendData.full_capacity & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 18] = ValueToChar((BmsAppendData.full_capacity & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 19] = ValueToChar((BmsAppendData.full_capacity & 0x000F));
#else
    GeneralBuffer[StartAddr + 16] = '0';
    GeneralBuffer[StartAddr + 17] = '0';
    GeneralBuffer[StartAddr + 18] = '0';
    GeneralBuffer[StartAddr + 19] = '0';
#endif

    //电池状态，3字节=6位字符
    GeneralBuffer[StartAddr + 20] = 'S';
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[StartAddr + 21] = ValueToChar((Rs485Data.status[0] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 22] = ValueToChar((Rs485Data.status[0] & 0x0F));
    GeneralBuffer[StartAddr + 23] = ValueToChar((Rs485Data.status[1] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 24] = ValueToChar((Rs485Data.status[1] & 0x0F));
    GeneralBuffer[StartAddr + 25] = ValueToChar((Rs485Data.status[2] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 26] = ValueToChar((Rs485Data.status[2] & 0x0F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[StartAddr + 21] = ValueToChar((LionBatData.status[0] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 22] = ValueToChar((LionBatData.status[0] & 0x0F));
    GeneralBuffer[StartAddr + 23] = ValueToChar((LionBatData.status[1] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 24] = ValueToChar((LionBatData.status[1] & 0x0F));
    GeneralBuffer[StartAddr + 25] = ValueToChar((LionBatData.status[2] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 26] = ValueToChar((LionBatData.status[2] & 0x0F));
#else
    GeneralBuffer[StartAddr + 21] = '0';
    GeneralBuffer[StartAddr + 22] = '0';
    GeneralBuffer[StartAddr + 23] = '0';
    GeneralBuffer[StartAddr + 24] = '0';
    GeneralBuffer[StartAddr + 25] = '0';
    GeneralBuffer[StartAddr + 26] = '0';
#endif
 
    if(ave_cur == 0) 
    {
        GeneralBuffer[StartAddr + 27] = 'V'; 
        GeneralBuffer[StartAddr + 28] = '0';
        GeneralBuffer[StartAddr + 29] = '0';
        GeneralBuffer[StartAddr + 30] = '0';
        GeneralBuffer[StartAddr + 31] = '0';
    }
    else
    {
        ave_cur = ave_cur ^ 0xFFFF;
        GeneralBuffer[StartAddr + 27] = 'V'; 
        GeneralBuffer[StartAddr + 28] = ValueToChar(ave_cur >> 12);
        GeneralBuffer[StartAddr + 29] = ValueToChar((ave_cur & 0x0F00) >> 8);
        GeneralBuffer[StartAddr + 30] = ValueToChar((ave_cur & 0x00F0) >> 4);
        GeneralBuffer[StartAddr + 31] = ValueToChar((ave_cur & 0x000F));
    }
}

//************************************************
//单模控制器数据，长度31个字节 
//(863412040166772BR09)
void WriteSingleModeDataToBuffer(uint8_t StartAddr)
{
    GeneralBuffer[StartAddr]      = ',';
    /*
    控制器状态2
    Bit0：助力状态，0:无效，1:有效
    Bit1：推车状态，0:无效，1:有效
    Bit2：倒车状态，0:无效，1:有效
    Bit3：驻车状态，0:无效，1:有效
    Bit4：防盗状态，0:无效，1:有效
    Bit5：巡航状态，0:无效，1:有效
    Bit6：限速状态，0:无效，1:有效
    */
    GeneralBuffer[StartAddr + 1]  = ValueToChar((EbikeData.State[1] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 2]  = ValueToChar((EbikeData.State[1] & 0x0F));
    GeneralBuffer[StartAddr + 3]  = ',';
    /*
    控制器故障2
    Bit0：放电低温保护，0:无效，1:有效
    Bit1：放电高温保护，0:无效，1:有效
    Bit2~7：保留
    */
    GeneralBuffer[StartAddr + 4]  = ValueToChar((EbikeData.Fault[1] & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 5]  = ValueToChar((EbikeData.Fault[1] & 0x0F));
    GeneralBuffer[StartAddr + 6]  = ',';
    //控制器电压等级
    GeneralBuffer[StartAddr + 7]  = ValueToChar(EbikeData.VoltGrade%10);
    GeneralBuffer[StartAddr + 8]  = ',';
    //控制器软件版本号
    GeneralBuffer[StartAddr + 9]  = ValueToChar((EbikeData.SoftVer & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 10] = ValueToChar((EbikeData.SoftVer & 0x0F));
    GeneralBuffer[StartAddr + 11] = ',';
    //控制器车速，单位：km/h
    GeneralBuffer[StartAddr + 12] = ValueToChar((EbikeData.Speed & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 13] = ValueToChar((EbikeData.Speed & 0x0F));
    GeneralBuffer[StartAddr + 14] = ',';
    //单次行程，单位：km
    GeneralBuffer[StartAddr + 15] = ValueToChar(EbikeData.Trip >> 12);
    GeneralBuffer[StartAddr + 16] = ValueToChar((EbikeData.Trip & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 17] = ValueToChar((EbikeData.Trip & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 18] = ValueToChar((EbikeData.Trip & 0x000F));
    GeneralBuffer[StartAddr + 19] = ',';
    //控制器累计总行程，单位：km
    GeneralBuffer[StartAddr + 20] = ValueToChar(EbikeData.TotalMileage >> 28);
    GeneralBuffer[StartAddr + 21] = ValueToChar((EbikeData.TotalMileage & 0x0F000000) >> 24);
    GeneralBuffer[StartAddr + 22] = ValueToChar((EbikeData.TotalMileage & 0x00F00000) >> 20);
    GeneralBuffer[StartAddr + 23] = ValueToChar((EbikeData.TotalMileage & 0x000F0000) >> 16);
    GeneralBuffer[StartAddr + 24] = ValueToChar((EbikeData.TotalMileage & 0x0000F000) >> 12);
    GeneralBuffer[StartAddr + 25] = ValueToChar((EbikeData.TotalMileage & 0x00000F00) >> 8);
    GeneralBuffer[StartAddr + 26] = ValueToChar((EbikeData.TotalMileage & 0x000000F0) >> 4);
    GeneralBuffer[StartAddr + 27] = ValueToChar((EbikeData.TotalMileage & 0x0000000F));
    GeneralBuffer[StartAddr + 28] = ',';
    /*
    电动车状态位，1字节，8bit
    Bit0：电池仓锁状态，0:未上锁，1:已上锁
    //Bit1：头盔在位状态，0:不在位，1:在位
    //Bit2：头盔锁状态，0:未上锁，1:已上锁
    Bit1~7：保留
    */
    GeneralBuffer[StartAddr + 29] = ValueToChar((DeviceInfo.EbikeState.bitFlag & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 30] = ValueToChar((DeviceInfo.EbikeState.bitFlag & 0x0F));
}
#endif //#ifdef USE_BH_LEASE_MODE

#ifdef USE_MJ_MULTI_MODE
//************************************************
//多模共轨控制器数据，长度55个字节
//(867881040215878BR07,1,031002,2D,03D,001C8,55,3E,030C,1C20,05DC,0370,814202)
//(867881040215878BR07,2,031000,2D,03D,001C8,55,3E,0316,1C20,05DC,0370,814202)
void WriteCtrlDataToBuffer(uint8_t StartAddr, uint8_t num)
{
	if(num > CTL_NUM_2)
	{
		DEBUG_PRINT("---- ctrl num error!!! ----\r\n");
		return;
	}

	GeneralBuffer[StartAddr]      = ',';
	//控制器编号，[1&2]
	GeneralBuffer[StartAddr + 1]  = ValueToChar((num+1)%10); //加1为了对应控制器编号1、2
    GeneralBuffer[StartAddr + 2]  = ',';
	//状态信息,对应id 0x230,16进制格式
    GeneralBuffer[StartAddr + 3]  = ValueToChar((CtrlData[num].state.byte1.gear_state & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 4]  = ValueToChar((CtrlData[num].state.byte1.gear_state & 0x0F));
    GeneralBuffer[StartAddr + 5]  = ValueToChar((CtrlData[num].state.byte2.body_state & 0xF0) >> 4);
	GeneralBuffer[StartAddr + 6]  = ValueToChar((CtrlData[num].state.byte2.body_state & 0x0F));
	GeneralBuffer[StartAddr + 7]  = ValueToChar((CtrlData[num].state.byte3.device_state & 0xF0) >> 4);
	GeneralBuffer[StartAddr + 8]  = ValueToChar((CtrlData[num].state.byte3.device_state & 0x0F));
	GeneralBuffer[StartAddr + 9]  = ',';
    
    //显示信息，对应id 0x234,16进制格式
    GeneralBuffer[StartAddr + 10] = ValueToChar((CtrlData[num].disp_msg.vehicle_speed & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 11] = ValueToChar((CtrlData[num].disp_msg.vehicle_speed & 0x0F));
	GeneralBuffer[StartAddr + 12] = ',';
	GeneralBuffer[StartAddr + 13] = ValueToChar((CtrlData[num].disp_msg.trip & 0x0F00) >> 8);
	GeneralBuffer[StartAddr + 14] = ValueToChar((CtrlData[num].disp_msg.trip & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 15] = ValueToChar((CtrlData[num].disp_msg.trip & 0x000F));
	GeneralBuffer[StartAddr + 16] = ',';
	GeneralBuffer[StartAddr + 17] = ValueToChar((CtrlData[num].disp_msg.odo & 0x000F0000) >> 16);
	GeneralBuffer[StartAddr + 18] = ValueToChar((CtrlData[num].disp_msg.odo & 0x0000F000) >> 12);
	GeneralBuffer[StartAddr + 19] = ValueToChar((CtrlData[num].disp_msg.odo & 0x00000F00) >> 8);
	GeneralBuffer[StartAddr + 20] = ValueToChar((CtrlData[num].disp_msg.odo & 0x000000F0) >> 4);
    GeneralBuffer[StartAddr + 21] = ValueToChar((CtrlData[num].disp_msg.odo & 0x0000000F));
	GeneralBuffer[StartAddr + 22] = ',';

	//显示信息1，对应id 0x238,16进制格式
    GeneralBuffer[StartAddr + 23] = ValueToChar((CtrlData[num].disp_msg1.percent_voltage & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 24] = ValueToChar((CtrlData[num].disp_msg1.percent_voltage & 0x0F));
	GeneralBuffer[StartAddr + 25] = ',';
	GeneralBuffer[StartAddr + 26] = ValueToChar((CtrlData[num].disp_msg1.percent_current & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 27] = ValueToChar((CtrlData[num].disp_msg1.percent_current & 0x0F));
	GeneralBuffer[StartAddr + 28] = ',';
    GeneralBuffer[StartAddr + 29] = ValueToChar(CtrlData[num].disp_msg1.ctrl_temp >> 12);
    GeneralBuffer[StartAddr + 30] = ValueToChar((CtrlData[num].disp_msg1.ctrl_temp & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 31] = ValueToChar((CtrlData[num].disp_msg1.ctrl_temp & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 32] = ValueToChar((CtrlData[num].disp_msg1.ctrl_temp & 0x000F));
	GeneralBuffer[StartAddr + 33] = ',';

    //控制器规格参数信息，对应id 0x218,16进制格式
    GeneralBuffer[StartAddr + 34] = ValueToChar(CtrlData[num].volt_cur_temp.battery_voltage >> 12);
    GeneralBuffer[StartAddr + 35] = ValueToChar((CtrlData[num].volt_cur_temp.battery_voltage & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 36] = ValueToChar((CtrlData[num].volt_cur_temp.battery_voltage & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 37] = ValueToChar((CtrlData[num].volt_cur_temp.battery_voltage & 0x000F));
	GeneralBuffer[StartAddr + 38] = ',';
	GeneralBuffer[StartAddr + 39] = ValueToChar(CtrlData[num].volt_cur_temp.bat_discharge_cur >> 12);
    GeneralBuffer[StartAddr + 40] = ValueToChar((CtrlData[num].volt_cur_temp.bat_discharge_cur & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 41] = ValueToChar((CtrlData[num].volt_cur_temp.bat_discharge_cur & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 42] = ValueToChar((CtrlData[num].volt_cur_temp.bat_discharge_cur & 0x000F));
	GeneralBuffer[StartAddr + 43] = ',';
	GeneralBuffer[StartAddr + 44] = ValueToChar(CtrlData[num].volt_cur_temp.mos_temp >> 12);
    GeneralBuffer[StartAddr + 45] = ValueToChar((CtrlData[num].volt_cur_temp.mos_temp & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 46] = ValueToChar((CtrlData[num].volt_cur_temp.mos_temp & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 47] = ValueToChar((CtrlData[num].volt_cur_temp.mos_temp & 0x000F));
	GeneralBuffer[StartAddr + 48] = ',';

    //故障信息，对应id 0x265,16进制格式
    GeneralBuffer[StartAddr + 49]   = ValueToChar((CtrlData[num].Fault.byte1.malf_state1 & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 50] = ValueToChar((CtrlData[num].Fault.byte1.malf_state1 & 0x0F));
    GeneralBuffer[StartAddr + 51] = ValueToChar((CtrlData[num].Fault.byte2.malf_state2 & 0xF0) >> 4);
	GeneralBuffer[StartAddr + 52] = ValueToChar((CtrlData[num].Fault.byte2.malf_state2 & 0x0F));
	GeneralBuffer[StartAddr + 53] = ValueToChar((CtrlData[num].Fault.byte3.malf_state3 & 0xF0) >> 4);
	GeneralBuffer[StartAddr + 54] = ValueToChar((CtrlData[num].Fault.byte3.malf_state3 & 0x0F));
	
}

//************************************************
//多模共轨BMS数据，长度74个字节
//(867881040215878BR08,1,6,14,14,0A,2,0000,20E5,0004,0000,63,035A,0358,1C20,1E,64,0898,0BB8,0120)
//(867881040215878BR08,2,6,14,14,0A,2,0000,20E5,0004,0000,63,035A,0358,1C20,1E,64,0898,0BB8,0120)
void WriteBmsDataToBuffer(uint8_t StartAddr, uint8_t num)
{
	if(num > BMS_NUM_2)
	{
		DEBUG_PRINT("---- bms num error!!! ----\r\n");
		return;
	}

	GeneralBuffer[StartAddr]      = ',';
	//BMS编号，[1&2]
	GeneralBuffer[StartAddr + 1]  = ValueToChar((num+1)%10); //加1为了对应BMS编号1、2
    GeneralBuffer[StartAddr + 2]  = ',';
	//电芯相关信息,对应id 0x210,16进制格式
	GeneralBuffer[StartAddr + 3]  = ValueToChar(BmsData[num].bat_type.materials   % 10);
	GeneralBuffer[StartAddr + 4]  = ',';
	GeneralBuffer[StartAddr + 5]  = ValueToChar((BmsData[num].bat_type.cell_total & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 6]  = ValueToChar((BmsData[num].bat_type.cell_total & 0x0F));
	GeneralBuffer[StartAddr + 7]  = ',';
	GeneralBuffer[StartAddr + 8]  = ValueToChar((BmsData[num].bat_type.dsg_ratio & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 9]  = ValueToChar((BmsData[num].bat_type.dsg_ratio & 0x0F));
	GeneralBuffer[StartAddr + 10] = ',';
	GeneralBuffer[StartAddr + 11] = ValueToChar((BmsData[num].bat_type.chg_ratio & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 12] = ValueToChar((BmsData[num].bat_type.chg_ratio & 0x0F));
	GeneralBuffer[StartAddr + 13] = ',';
	
	//放电状态和电流信息,对应id 0x105,16进制格式
    GeneralBuffer[StartAddr + 14] = ValueToChar(BmsData[num].c_d_msg.dsg % 10);
	GeneralBuffer[StartAddr + 15] = ',';
	GeneralBuffer[StartAddr + 16] = ValueToChar(BmsData[num].c_d_msg.dsg_curr >> 12);
    GeneralBuffer[StartAddr + 17] = ValueToChar((BmsData[num].c_d_msg.dsg_curr & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 18] = ValueToChar((BmsData[num].c_d_msg.dsg_curr & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 19] = ValueToChar((BmsData[num].c_d_msg.dsg_curr & 0x000F));
	GeneralBuffer[StartAddr + 20] = ',';
    GeneralBuffer[StartAddr + 21] = ValueToChar(BmsData[num].c_d_msg.bat_volt >> 12);
    GeneralBuffer[StartAddr + 22] = ValueToChar((BmsData[num].c_d_msg.bat_volt & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 23] = ValueToChar((BmsData[num].c_d_msg.bat_volt & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 24] = ValueToChar((BmsData[num].c_d_msg.bat_volt & 0x000F));
	GeneralBuffer[StartAddr + 25] = ',';
	
    //显示信息，对应id 0x20C,16进制格式
    GeneralBuffer[StartAddr + 26] = ValueToChar(BmsData[num].disp_msg.cycle_num >> 12);
    GeneralBuffer[StartAddr + 27] = ValueToChar((BmsData[num].disp_msg.cycle_num & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 28] = ValueToChar((BmsData[num].disp_msg.cycle_num & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 29] = ValueToChar((BmsData[num].disp_msg.cycle_num & 0x000F));
	GeneralBuffer[StartAddr + 30] = ',';
	GeneralBuffer[StartAddr + 31] = ValueToChar(BmsData[num].disp_msg.charge_times >> 12);
    GeneralBuffer[StartAddr + 32] = ValueToChar((BmsData[num].disp_msg.charge_times & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 33] = ValueToChar((BmsData[num].disp_msg.charge_times & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 34] = ValueToChar((BmsData[num].disp_msg.charge_times & 0x000F));
	GeneralBuffer[StartAddr + 35] = ',';

	//SOC和温度信息,对应id 0x101
	GeneralBuffer[StartAddr + 36] = ValueToChar((BmsData[num].soc_temp.soc & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 37] = ValueToChar((BmsData[num].soc_temp.soc & 0x0F));
	GeneralBuffer[StartAddr + 38] = ',';
	GeneralBuffer[StartAddr + 39] = ValueToChar(BmsData[num].soc_temp.fet_temp >> 12);
    GeneralBuffer[StartAddr + 40] = ValueToChar((BmsData[num].soc_temp.fet_temp & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 41] = ValueToChar((BmsData[num].soc_temp.fet_temp & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 42] = ValueToChar((BmsData[num].soc_temp.fet_temp & 0x000F));
	GeneralBuffer[StartAddr + 43] = ',';
	GeneralBuffer[StartAddr + 44] = ValueToChar(BmsData[num].soc_temp.amb_temp >> 12);
    GeneralBuffer[StartAddr + 45] = ValueToChar((BmsData[num].soc_temp.amb_temp & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 46] = ValueToChar((BmsData[num].soc_temp.amb_temp & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 47] = ValueToChar((BmsData[num].soc_temp.amb_temp & 0x000F));
	GeneralBuffer[StartAddr + 48] = ',';
	
	//BMS额定参数信息，对应id 0x270,16进制格式
	GeneralBuffer[StartAddr + 49] = ValueToChar(BmsData[num].rated_param.rated_voltage >> 12);
    GeneralBuffer[StartAddr + 50] = ValueToChar((BmsData[num].rated_param.rated_voltage & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 51] = ValueToChar((BmsData[num].rated_param.rated_voltage & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 52] = ValueToChar((BmsData[num].rated_param.rated_voltage & 0x000F));
	GeneralBuffer[StartAddr + 53] = ',';
    GeneralBuffer[StartAddr + 54] = ValueToChar((BmsData[num].rated_param.nominal_capacity & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 55] = ValueToChar((BmsData[num].rated_param.nominal_capacity & 0x0F));
	GeneralBuffer[StartAddr + 56] = ',';
    GeneralBuffer[StartAddr + 57] = ValueToChar((BmsData[num].rated_param.soh & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 58] = ValueToChar((BmsData[num].rated_param.soh & 0x0F));
	GeneralBuffer[StartAddr + 59] = ',';
	GeneralBuffer[StartAddr + 60] = ValueToChar(BmsData[num].rated_param.residual_capacity >> 12);
    GeneralBuffer[StartAddr + 61] = ValueToChar((BmsData[num].rated_param.residual_capacity & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 62] = ValueToChar((BmsData[num].rated_param.residual_capacity & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 63] = ValueToChar((BmsData[num].rated_param.residual_capacity & 0x000F));
	GeneralBuffer[StartAddr + 64] = ',';
	GeneralBuffer[StartAddr + 65] = ValueToChar(BmsData[num].rated_param.full_chg_capacity >> 12);
    GeneralBuffer[StartAddr + 66] = ValueToChar((BmsData[num].rated_param.full_chg_capacity & 0x0F00) >> 8);
    GeneralBuffer[StartAddr + 67] = ValueToChar((BmsData[num].rated_param.full_chg_capacity & 0x00F0) >> 4);
    GeneralBuffer[StartAddr + 68] = ValueToChar((BmsData[num].rated_param.full_chg_capacity & 0x000F));
	GeneralBuffer[StartAddr + 69] = ',';

    //故障信息，对应id 0x261,16进制格式
    GeneralBuffer[StartAddr + 70] = ValueToChar((BmsData[num].Fault.byte1.data & 0xF0) >> 4);
    GeneralBuffer[StartAddr + 71] = ValueToChar((BmsData[num].Fault.byte1.data & 0x0F));
    GeneralBuffer[StartAddr + 72] = ValueToChar((BmsData[num].Fault.byte2.data & 0xF0) >> 4);
	GeneralBuffer[StartAddr + 73] = ValueToChar((BmsData[num].Fault.byte2.data & 0x0F));
}
#endif //#ifndef  USE_MJ_MULTI_MODE 

#ifdef STM32RECORD
//************************************************
//长度62个字节
void WriteGpsDataToBufferRecord(uint8_t StartAddr,
                                RecordDataStruct *pDataBuffer)  
{
    uint8_t i, ucTmp;
    uint32_t        dwTmp;
    uint8_t BatteryPower[6]={0};
    
    
    //报警
    if((PlatformData_HY.SendGpsDataType == GPS_DATA_TYPE_RECORD)
        && (pDataBuffer->Alert != 0xFF))  
      GeneralBuffer[StartAddr-1]  = pDataBuffer->Alert;
    
    //日期
    //year
    GeneralBuffer[StartAddr]     = ValueToChar(pDataBuffer->bcdYear >> 4);  
    GeneralBuffer[StartAddr + 1] = ValueToChar(pDataBuffer->bcdYear & 0x0F);
    //month
    GeneralBuffer[StartAddr + 2] = ValueToChar(pDataBuffer->bcdMonth >> 4); 
    GeneralBuffer[StartAddr + 3] = ValueToChar(pDataBuffer->bcdMonth & 0x0F);
    //day
    GeneralBuffer[StartAddr + 4] = ValueToChar(pDataBuffer->bcdDay >> 4);    
    GeneralBuffer[StartAddr + 5] = ValueToChar(pDataBuffer->bcdDay & 0x0F);

    //GPS数据是否有效的标字
    if (pDataBuffer->PositionFlag == TRUE)      
    {
        GeneralBuffer[StartAddr + 6] = 'A';
    }
    else
    {
        GeneralBuffer[StartAddr + 6] = 'V';
    }

    //纬度
    dwTmp = pDataBuffer->ulLatitude;   
    GeneralBuffer[StartAddr + 7]  = ValueToChar(dwTmp / 10000000);
    dwTmp = dwTmp % 10000000;
    GeneralBuffer[StartAddr + 8]  = ValueToChar(dwTmp / 1000000);    
    dwTmp = dwTmp % 1000000;
    dwTmp = dwTmp * 3 / 5;
    GeneralBuffer[StartAddr + 9]  = ValueToChar(dwTmp / 100000);
    dwTmp = dwTmp % 100000;    
    GeneralBuffer[StartAddr + 10] = ValueToChar(dwTmp / 10000);  
    dwTmp = dwTmp % 10000;
    if (IsNumber(&GeneralBuffer[StartAddr + 7], 4) == FALSE)
    {
        for(i = 0; i < 5; i++)
           GeneralBuffer[StartAddr + 7 + i] = '0';
    }
    GeneralBuffer[StartAddr + 11] = '.';    
    GeneralBuffer[StartAddr + 12] = ValueToChar(dwTmp / 1000);
    dwTmp = dwTmp % 1000;
    GeneralBuffer[StartAddr + 13] = ValueToChar(dwTmp / 100);
    dwTmp = dwTmp % 100;
    GeneralBuffer[StartAddr + 14] = ValueToChar(dwTmp / 10);
    dwTmp = dwTmp % 10;
    GeneralBuffer[StartAddr + 15] = ValueToChar(dwTmp % 10);
    if (IsNumber(&GeneralBuffer[StartAddr + 12], 4) == FALSE)
    {
        for(i = 0; i < 5; i++)
           GeneralBuffer[StartAddr + 12 + i] = '0';
    }
    
    GeneralBuffer[StartAddr + 16] = GpsData.LatitudeIndicator;

    //经度
    dwTmp = pDataBuffer->ulLongitude;   
    GeneralBuffer[StartAddr + 17] = ValueToChar(dwTmp / 100000000);
    dwTmp = dwTmp % 100000000;
    GeneralBuffer[StartAddr + 18] = ValueToChar(dwTmp / 10000000);    
    dwTmp = dwTmp % 10000000;
    GeneralBuffer[StartAddr + 19] = ValueToChar(dwTmp / 1000000);
    dwTmp = dwTmp % 1000000;
    dwTmp = dwTmp * 3 / 5;
    GeneralBuffer[StartAddr + 20] = ValueToChar(dwTmp / 100000);
    dwTmp = dwTmp % 100000;
    GeneralBuffer[StartAddr + 21] = ValueToChar(dwTmp / 10000);
    dwTmp = dwTmp % 10000;

    if (IsNumber(&GeneralBuffer[StartAddr + 17], 5) == FALSE)
    {
        for(i = 0; i < 6; i++)
            GeneralBuffer[StartAddr + 17 + i] = '0';
    }
    GeneralBuffer[StartAddr + 22] = '.';
    GeneralBuffer[StartAddr + 23] = ValueToChar(dwTmp / 1000);
    dwTmp = dwTmp % 1000;
    GeneralBuffer[StartAddr + 24] = ValueToChar(dwTmp / 100);
    dwTmp = dwTmp % 100;
    GeneralBuffer[StartAddr + 25] = ValueToChar(dwTmp / 10);
    dwTmp = dwTmp % 10;
    GeneralBuffer[StartAddr + 26] = ValueToChar(dwTmp % 10);

    if (IsNumber(&GeneralBuffer[StartAddr + 23], 4) == FALSE)
    {
        for(i = 0; i < 5; i++)
            GeneralBuffer[StartAddr + 23 + i] = '0';
    }
    GeneralBuffer[StartAddr + 27] = GpsData.LongitudeIndicator;

    //速度
    ucTmp = pDataBuffer->ucSpeed;
    if (ucTmp < 200)
    {
        GeneralBuffer[StartAddr + 28] = ValueToChar(ucTmp / 100);
        ucTmp %= 100;
        GeneralBuffer[StartAddr + 29] = ValueToChar(ucTmp / 10);
        ucTmp %= 10;
        GeneralBuffer[StartAddr + 30] = ValueToChar(ucTmp);
        GeneralBuffer[StartAddr + 31] = '.';
        GeneralBuffer[StartAddr + 32] = '0';
    }
    else
    {
        GeneralBuffer[StartAddr + 28] = '0';
        GeneralBuffer[StartAddr + 29] = '0';
        GeneralBuffer[StartAddr + 30] = '0';
        GeneralBuffer[StartAddr + 31] = '.';
        GeneralBuffer[StartAddr + 32] = '0';
    }
    
    //时间
    //hour
    GeneralBuffer[StartAddr + 33] = ValueToChar(pDataBuffer->bcdHour >> 4); 
    GeneralBuffer[StartAddr + 34] = ValueToChar(pDataBuffer->bcdHour & 0x0F);
    //minute
    GeneralBuffer[StartAddr + 35] = ValueToChar(pDataBuffer->bcdMinute >> 4);  
    GeneralBuffer[StartAddr + 36] = ValueToChar(pDataBuffer->bcdMinute & 0x0F);
    //second
    GeneralBuffer[StartAddr + 37] = ValueToChar(pDataBuffer->bcdSecond >> 4);  
    GeneralBuffer[StartAddr + 38] = ValueToChar(pDataBuffer->bcdSecond & 0x0F);
     
    //方向
    if (pDataBuffer->ucDirection >= 50)
    {
        dwTmp = pDataBuffer->ucDirection * 2;
        GeneralBuffer[StartAddr + 39] = ValueToChar(dwTmp / 100);
        dwTmp %= 100;
        GeneralBuffer[StartAddr + 40] = ValueToChar(dwTmp / 10);
        dwTmp %= 10;
        GeneralBuffer[StartAddr + 41] = ValueToChar(dwTmp);
        GeneralBuffer[StartAddr + 42] = '.';
        GeneralBuffer[StartAddr + 43] = '0';
        GeneralBuffer[StartAddr + 44] = '0';
    }
    else if (pDataBuffer->ucDirection >= 5)
    {
        dwTmp = pDataBuffer->ucDirection * 2;
        GeneralBuffer[StartAddr + 39] = ValueToChar(dwTmp / 10);
        dwTmp %= 10;
        GeneralBuffer[StartAddr + 40] = ValueToChar(dwTmp);
        GeneralBuffer[StartAddr + 41] = '.';
        GeneralBuffer[StartAddr + 42] = '0';
        GeneralBuffer[StartAddr + 43] = '0';
        GeneralBuffer[StartAddr + 44] = '0';
    }
    else
    {
        dwTmp = pDataBuffer->ucDirection * 2;
        GeneralBuffer[StartAddr + 39] = ValueToChar(dwTmp);
        GeneralBuffer[StartAddr + 40] = '.';
        GeneralBuffer[StartAddr + 41] = '0';
        GeneralBuffer[StartAddr + 42] = '0';
        GeneralBuffer[StartAddr + 43] = '0';
        GeneralBuffer[StartAddr + 44] = '0';
    }

    //IO状态
    for (i = 0; i < 8; i++)
    {
        GeneralBuffer[StartAddr + 45 + i] = '0';
    }
    if ((pDataBuffer->State & 0x80) != 0)
    {
        GeneralBuffer[StartAddr + 45] = '1';
    }
    if ((pDataBuffer->State & 0x40) != 0)
    {
        GeneralBuffer[StartAddr + 46] = '1';
    }
    if ((pDataBuffer->State & 0x20) != 0)  //电瓶低电
    {
        GeneralBuffer[StartAddr + 47] = '1';
    }
    
    //电池电量
    NumberValueToHexStr((DeviceInfo.InputValue/100), BatteryPower);
    if((BatteryPower[0] < '0') || (BatteryPower[0] > 'F')) BatteryPower[0] = '0';
    if((BatteryPower[1] < '0') || (BatteryPower[1] > 'F')) BatteryPower[1] = '0';
    if((BatteryPower[2] < '0') || (BatteryPower[2] > 'F')) BatteryPower[2] = '0';
    GeneralBuffer[StartAddr + 50] = BatteryPower[0];
    GeneralBuffer[StartAddr + 51] = BatteryPower[1];
    GeneralBuffer[StartAddr + 52] = BatteryPower[2];
    
    //里程
    GeneralBuffer[StartAddr + 53] = 'L';
    
    dwTmp = pDataBuffer->dwSumMeter;

    if (dwTmp < 99999999)
    {
        for (i = 0; i < 8; i ++)
        {
            GeneralBuffer[StartAddr + 54 + 7 - i] = ValueToChar(dwTmp % 16);
            dwTmp /= 16;
        }
    }
    else
    {
        for (i = 0; i < 8; i ++)
        {
            GeneralBuffer[StartAddr + 54 + 7 - i] = '0';
        }
    }
    
    //GSM信号值及GPS有效卫星数量
    GeneralBuffer[StartAddr + 62] = 'S';  
    GeneralBuffer[StartAddr + 63] = ValueToChar(pDataBuffer->GsmCsqValue % 100 / 10);
    GeneralBuffer[StartAddr + 64] = ValueToChar(pDataBuffer->GsmCsqValue % 10);
    GeneralBuffer[StartAddr + 65] = ValueToChar(pDataBuffer->GpsSatelliteNum % 100 / 10);
    GeneralBuffer[StartAddr + 66] = ValueToChar(pDataBuffer->GpsSatelliteNum % 10);
}
#endif //#ifdef STM32RECORD

#ifdef SPI_FLASH_RECORD
//************************************************
//长度67个字节
void WriteGpsDataToBufferRecord(uint8_t StartAddr,
                                SPI_GpsRecordDataStruct *pDataBuffer)  
{
    uint8_t i, ucTmp;
    uint32_t        dwTmp;
    uint8_t BatteryPower[6]={0};    

    ///报警 
    if((PlatformData_HY.SendGpsDataType == GPS_DATA_TYPE_RECORD)
        && (pDataBuffer->Alert != 0xFF))  
      GeneralBuffer[StartAddr-1]  = pDataBuffer->Alert;
    
    //日期
    //year
    GeneralBuffer[StartAddr]     = ValueToChar(pDataBuffer->bcdYear >> 4);  
    GeneralBuffer[StartAddr + 1] = ValueToChar(pDataBuffer->bcdYear & 0x0F);
    //month
    GeneralBuffer[StartAddr + 2] = ValueToChar(pDataBuffer->bcdMonth >> 4); 
    GeneralBuffer[StartAddr + 3] = ValueToChar(pDataBuffer->bcdMonth & 0x0F);
    //day
    GeneralBuffer[StartAddr + 4] = ValueToChar(pDataBuffer->bcdDay >> 4);    
    GeneralBuffer[StartAddr + 5] = ValueToChar(pDataBuffer->bcdDay & 0x0F);

    //GPS数据是否有效的标字
    if (pDataBuffer->PositionFlag == TRUE)    
    {
        GeneralBuffer[StartAddr + 6] = 'A';
    }
    else
    {
        GeneralBuffer[StartAddr + 6] = 'V';
    }

    //纬度
    dwTmp = pDataBuffer->ulLatitude;   
    GeneralBuffer[StartAddr + 7]  = ValueToChar(dwTmp / 10000000);
    dwTmp = dwTmp % 10000000;
    GeneralBuffer[StartAddr + 8]  = ValueToChar(dwTmp / 1000000);    
    dwTmp = dwTmp % 1000000;
    dwTmp = dwTmp * 3 / 5;
    GeneralBuffer[StartAddr + 9]  = ValueToChar(dwTmp / 100000);
    dwTmp = dwTmp % 100000;    
    GeneralBuffer[StartAddr + 10] = ValueToChar(dwTmp / 10000);  
    dwTmp = dwTmp % 10000;

    if (IsNumber(&GeneralBuffer[StartAddr + 7], 4) == FALSE)
    {
        for(i = 0; i < 5; i++)
           GeneralBuffer[StartAddr + 7 + i] = '0';
    }

    GeneralBuffer[StartAddr + 11] = '.';    
    GeneralBuffer[StartAddr + 12] = ValueToChar(dwTmp / 1000);
    dwTmp = dwTmp % 1000;
    GeneralBuffer[StartAddr + 13] = ValueToChar(dwTmp / 100);
    dwTmp = dwTmp % 100;
    GeneralBuffer[StartAddr + 14] = ValueToChar(dwTmp / 10);
    dwTmp = dwTmp % 10;
    GeneralBuffer[StartAddr + 15] = ValueToChar(dwTmp % 10);

    if (IsNumber(&GeneralBuffer[StartAddr + 12], 4) == FALSE)
    {
        for(i = 0; i < 5; i++)
           GeneralBuffer[StartAddr + 12 + i] = '0';
    }
    
    GeneralBuffer[StartAddr + 16] = GpsData.LatitudeIndicator;

    //经度
    dwTmp = pDataBuffer->ulLongitude;   
    GeneralBuffer[StartAddr + 17] = ValueToChar(dwTmp / 100000000);
    dwTmp = dwTmp % 100000000;
    GeneralBuffer[StartAddr + 18] = ValueToChar(dwTmp / 10000000);    
    dwTmp = dwTmp % 10000000;
    GeneralBuffer[StartAddr + 19] = ValueToChar(dwTmp / 1000000);
    dwTmp = dwTmp % 1000000;
    dwTmp = dwTmp * 3 / 5;
    GeneralBuffer[StartAddr + 20] = ValueToChar(dwTmp / 100000);
    dwTmp = dwTmp % 100000;
    GeneralBuffer[StartAddr + 21] = ValueToChar(dwTmp / 10000);
    dwTmp = dwTmp % 10000;

    if (IsNumber(&GeneralBuffer[StartAddr + 17], 5) == FALSE)
    {
        for(i = 0; i < 6; i++)
            GeneralBuffer[StartAddr + 17 + i] = '0';
    }

    GeneralBuffer[StartAddr + 22] = '.';
    GeneralBuffer[StartAddr + 23] = ValueToChar(dwTmp / 1000);
    dwTmp = dwTmp % 1000;
    GeneralBuffer[StartAddr + 24] = ValueToChar(dwTmp / 100);
    dwTmp = dwTmp % 100;
    GeneralBuffer[StartAddr + 25] = ValueToChar(dwTmp / 10);
    dwTmp = dwTmp % 10;
    GeneralBuffer[StartAddr + 26] = ValueToChar(dwTmp % 10);

    if (IsNumber(&GeneralBuffer[StartAddr + 23], 4) == FALSE)
    {
        for(i = 0; i < 5; i++)
            GeneralBuffer[StartAddr + 23 + i] = '0';
    }
    GeneralBuffer[StartAddr + 27] = GpsData.LongitudeIndicator;

    //速度
    ucTmp = pDataBuffer->ucSpeed;
    if (ucTmp < 200)
    {
        GeneralBuffer[StartAddr + 28] = ValueToChar(ucTmp / 100);
        ucTmp %= 100;
        GeneralBuffer[StartAddr + 29] = ValueToChar(ucTmp / 10);
        ucTmp %= 10;
        GeneralBuffer[StartAddr + 30] = ValueToChar(ucTmp);
        GeneralBuffer[StartAddr + 31] = '.';
        GeneralBuffer[StartAddr + 32] = '0';
    }
    else
    {
        GeneralBuffer[StartAddr + 28] = '0';
        GeneralBuffer[StartAddr + 29] = '0';
        GeneralBuffer[StartAddr + 30] = '0';
        GeneralBuffer[StartAddr + 31] = '.';
        GeneralBuffer[StartAddr + 32] = '0';
    }
    
    //时间
    //hour
    GeneralBuffer[StartAddr + 33] = ValueToChar(pDataBuffer->bcdHour >> 4); 
    GeneralBuffer[StartAddr + 34] = ValueToChar(pDataBuffer->bcdHour & 0x0F);
    //minute
    GeneralBuffer[StartAddr + 35] = ValueToChar(pDataBuffer->bcdMinute >> 4);  
    GeneralBuffer[StartAddr + 36] = ValueToChar(pDataBuffer->bcdMinute & 0x0F);
    //second
    GeneralBuffer[StartAddr + 37] = ValueToChar(pDataBuffer->bcdSecond >> 4);  
    GeneralBuffer[StartAddr + 38] = ValueToChar(pDataBuffer->bcdSecond & 0x0F);
     
    //方向
    if (pDataBuffer->ucDirection >= 50)
    {
        dwTmp = pDataBuffer->ucDirection * 2;
        GeneralBuffer[StartAddr + 39] = ValueToChar(dwTmp / 100);
        dwTmp %= 100;
        GeneralBuffer[StartAddr + 40] = ValueToChar(dwTmp / 10);
        dwTmp %= 10;
        GeneralBuffer[StartAddr + 41] = ValueToChar(dwTmp);
        GeneralBuffer[StartAddr + 42] = '.';
        GeneralBuffer[StartAddr + 43] = '0';
        GeneralBuffer[StartAddr + 44] = '0';
    }
    else if (pDataBuffer->ucDirection >= 5)
    {
        dwTmp = pDataBuffer->ucDirection * 2;
        GeneralBuffer[StartAddr + 39] = ValueToChar(dwTmp / 10);
        dwTmp %= 10;
        GeneralBuffer[StartAddr + 40] = ValueToChar(dwTmp);
        GeneralBuffer[StartAddr + 41] = '.';
        GeneralBuffer[StartAddr + 42] = '0';
        GeneralBuffer[StartAddr + 43] = '0';
        GeneralBuffer[StartAddr + 44] = '0';
    }
    else
    {
        dwTmp = pDataBuffer->ucDirection * 2;
        GeneralBuffer[StartAddr + 39] = ValueToChar(dwTmp);
        GeneralBuffer[StartAddr + 40] = '.';
        GeneralBuffer[StartAddr + 41] = '0';
        GeneralBuffer[StartAddr + 42] = '0';
        GeneralBuffer[StartAddr + 43] = '0';
        GeneralBuffer[StartAddr + 44] = '0';
    }

    //IO状态
    for (i = 0; i < 8; i++)
    {
        GeneralBuffer[StartAddr + 45 + i] = '0';
    }
    if ((pDataBuffer->State & 0x80) != 0)
    {
        GeneralBuffer[StartAddr + 45] = '1';
    }
    if ((pDataBuffer->State & 0x40) != 0)
    {
        GeneralBuffer[StartAddr + 46] = '1';
    }
    if ((pDataBuffer->State & 0x20) != 0)  //电瓶低电
    {
        GeneralBuffer[StartAddr + 47] = '1';
    }
    
    //------------电池电量-------------
    NumberValueToHexStr((DeviceInfo.InputValue/100), BatteryPower);
    if((BatteryPower[0] < '0') || (BatteryPower[0] > 'F')) BatteryPower[0] = '0';
    if((BatteryPower[1] < '0') || (BatteryPower[1] > 'F')) BatteryPower[1] = '0';
    if((BatteryPower[2] < '0') || (BatteryPower[2] > 'F')) BatteryPower[2] = '0';
    GeneralBuffer[StartAddr + 50] = BatteryPower[0];
    GeneralBuffer[StartAddr + 51] = BatteryPower[1];
    GeneralBuffer[StartAddr + 52] = BatteryPower[2];
    //------------------------------------------
    
    //里程
    GeneralBuffer[StartAddr + 53] = 'L';
    
    dwTmp = pDataBuffer->dwSumMeter;
    if (dwTmp < 99999999)
    {
        for (i = 0; i < 8; i ++)
        {
            GeneralBuffer[StartAddr + 54 + 7 - i] = ValueToChar(dwTmp % 16);
            dwTmp /= 16;
        }
    }
    else
    {
        for (i = 0; i < 8; i ++)
        {
            GeneralBuffer[StartAddr + 54 + 7 - i] = '0';
        }
    }
    
    //GSM信号值及GPS有效卫星数量
    GeneralBuffer[StartAddr + 62] = 'S';  
    GeneralBuffer[StartAddr + 63] = ValueToChar(pDataBuffer->GsmCsqValue % 100 / 10);
    GeneralBuffer[StartAddr + 64] = ValueToChar(pDataBuffer->GsmCsqValue % 10);
    GeneralBuffer[StartAddr + 65] = ValueToChar(pDataBuffer->GpsSatelliteNum % 100 / 10);
    GeneralBuffer[StartAddr + 66] = ValueToChar(pDataBuffer->GpsSatelliteNum % 10);
    
}
#endif //#ifdef SPI_FLASH_RECORD

//***********************************************************
//长度83个字节
void WriteGpsDataToBuffer(unsigned char StartAddr)
{
    CPU_SR_ALLOC(); 
    
    OS_CRITICAL_ENTER();   //进入临界段(无法被中断打断)
    
    if (PlatformData_HY.SendGpsDataType == GPS_DATA_TYPE_RECORD)
    {
      #ifdef STM32RECORD  
        WriteGpsDataToBufferRecord(StartAddr, &ReadRecordData);
      #endif
        
      #ifdef SPI_FLASH_RECORD      
        WriteGpsDataToBufferRecord(StartAddr, &SpiReadRecordData);
      #endif
        
    }
    else if (PlatformData_HY.SendGpsDataType == GPS_DATA_TYPE_RESEND)
    {
        #ifdef STM32RECORD  
        WriteGpsDataToBufferRecord(StartAddr, &WriteRecordData);
        #endif
        
        #ifdef SPI_FLASH_RECORD  
        WriteGpsDataToBufferRecord(StartAddr, &SpiWriteRecordData);
        #endif
    }
    else
    {
        WriteGpsDataToBufferNew(StartAddr);
    }
    OS_CRITICAL_EXIT();    //退出临界段   
}

//************************************************
void WriteIMEIToBuffer(void) 
{
    uint8_t i;
     
    for (i = 0; i < 15; i ++)
    {
        GeneralBuffer[1 + i] = PlatformData.DeviceID[i];
    }
}

//************************************************
void WriteDeviceIdTobuffer(void)
{
    uint8_t i;
         
    for (i = 0; i < 15; i ++)
    {
        GeneralBuffer[1 + i] = PlatformData.DeviceID[i];
    }
}

//***********************************************
//发送终端注册消息
uint8_t SendMessageBP05(void)
{
    uint8_t i, Result;
    
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
 
    //命令
    GeneralBuffer[16] = 'B'; 
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '5';
    //终端ID
    for (i = 0; i < 15; i++)
    {
        GeneralBuffer[20 + i] = PlatformData.DeviceID[i];
    }
    //GPS数据,83个字节
    WriteGpsDataToBuffer(35);
    GeneralBuffer[118] = ')';  //packet end 

    DeviceInfo.SendTcpDataFlag = TRUE;
    //OS_ENTER_CRITICAL();
    DeviceInfo.SendTcpDataDlyCnt  = TEN_SECOND;//每次发送数据计时10秒，超时10秒仍未发成功，开始存盲区
    //OS_EXIT_CRITICAL();

    
    Result = SendTcpPacket(GeneralBuffer, 119); 
      
    return Result;
}

//***********************************************
//发送SIM卡CCID号，用于平台绑定设备IMEI
uint8_t SendMessageBP06(void)
{
    unsigned char i, Result;
       
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '6';
    //SIM卡20位CCID号码
    for (i = 0; i < 20; i++)
    {
        GeneralBuffer[20 + i] = DeviceInfo.SimCCIDNumber[i];
    }
    
    GeneralBuffer[40] = 'R';   //蓝牙地址标识
    
#if defined(USE_BLE_MODULE)  
    //12位蓝牙地址
    for (i = 0; i < 12; i++)
    {
        GeneralBuffer[41 + i] = BleData.BleMAC[i];
    }
#else
    //未使用蓝牙功能，蓝牙地址清0
    for (i = 0; i < 12; i++)
    {
        GeneralBuffer[41 + i] = '0';
    }
#endif  //#ifdef USE_BLE_MODULE

    GeneralBuffer[53] = ')'; 

    Result = SendTcpPacket(GeneralBuffer, 54);
    
    return Result;
}

#ifdef ENABLE_AGPS_DOWN
//***********************************************
//向平台请求AGPS数据 
uint8_t SendMessageBP07(void)
{
    uint8_t Result;

    GeneralBuffer[0] = '(';  //数据包开始
    //终端IMEI号
    WriteIMEIToBuffer();
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '7';
    GeneralBuffer[20] = ')';  

    Result = SendTcpPacket(GeneralBuffer, 21);   
  
    return Result;
}
#endif

//***********************************************
//发送BMS厂商名称,ID,版本，用于平台绑定设备 
//(863412040166772BP11AEJCBH10AMB11002,0000000000000,101100)
uint8_t SendMessageBP11(void)
{
    uint8_t i, Result;
       
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '1';
    GeneralBuffer[19] = '1';

    //电池ID，16位
    GeneralBuffer[20] = '0';  //强制为0，方便服务端查找电池编号绑定
    for (i = 0; i < 15; i++)
    {
        #ifdef ENABLE_EXT_RS485
        GeneralBuffer[21 + i] = Rs485Data.id_code[i+1];
        #elif defined (ENABLE_CAN_COMM)
        GeneralBuffer[21 + i] = LionBatData.id_code[i+1];
        #else
        GeneralBuffer[21 + i] = '0';
        #endif
    }
    
    GeneralBuffer[36] = ',';   
    
    //制造商名称,PHYLION BATTERY
//    if(Rs485Data.manufacturer_name[0] == 0x00) //非ascii码
//    {
//        for (i = 0; i < 16; i++)
//        {
//            GeneralBuffer[37 + i] = '0';
//        }
//    }
//    else
//    {
    
#ifdef ENABLE_EXT_RS485
        for (i = 0; i < 16; i++)
        {
            GeneralBuffer[37 + i] = Rs485Data.manufacturer_name[i];
        }
#else
        for (i = 0; i < 10; i++)
        {
            GeneralBuffer[37 + i] = '0';
        }
        
        for (i = 0; i < 6; i++)
        {
            #ifdef ENABLE_CAN_COMM
            GeneralBuffer[37 + 10 + i] = BmsAppendData.manufacturer_name[i];
            #else
            GeneralBuffer[37 + 10 + i] = '0';
            #endif
        }
        
#endif

//    }
    
    GeneralBuffer[53] = ',';   //版本标识 
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[54] = ValueToChar(Rs485Data.sf_ver / 100 );
    GeneralBuffer[55] = ValueToChar(Rs485Data.sf_ver % 100 / 10);
    GeneralBuffer[56] = ValueToChar(Rs485Data.sf_ver % 10);
    GeneralBuffer[57] = ValueToChar(Rs485Data.hw_ver / 100 );
    GeneralBuffer[58] = ValueToChar(Rs485Data.hw_ver % 100 / 10);
    GeneralBuffer[59] = ValueToChar(Rs485Data.hw_ver % 10);
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[54] = ValueToChar(LionBatData.sf_ver / 100 );
    GeneralBuffer[55] = ValueToChar(LionBatData.sf_ver % 100 / 10);
    GeneralBuffer[56] = ValueToChar(LionBatData.sf_ver % 10);
    GeneralBuffer[57] = ValueToChar(LionBatData.hw_ver / 100 );
    GeneralBuffer[58] = ValueToChar(LionBatData.hw_ver % 100 / 10);
    GeneralBuffer[59] = ValueToChar(LionBatData.hw_ver % 10);
#else
    GeneralBuffer[54] = '0';
    GeneralBuffer[55] = '0';
    GeneralBuffer[56] = '0';
    GeneralBuffer[57] = '0';
    GeneralBuffer[58] = '0';
    GeneralBuffer[59] = '0';
#endif        
    GeneralBuffer[60] = ')'; 

    Result = SendTcpPacket(GeneralBuffer, 61);
    
    return Result;
    
}

//***********************************************
//发送NFC id和租赁时间信息 
uint8_t SendMessageBP15(void)
{
    uint8_t Result;
       
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令 (865860042367897BP150400A76506B6,210701,210731)
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '1';
    GeneralBuffer[19] = '5';
    
    GeneralBuffer[20] = ValueToChar((DeviceInfo.NFC_id[0] & 0xF0) >> 4);
    GeneralBuffer[21] = ValueToChar(DeviceInfo.NFC_id[0] & 0x0F);
    GeneralBuffer[22] = ValueToChar((DeviceInfo.NFC_id[1] & 0xF0) >> 4);
    GeneralBuffer[23] = ValueToChar(DeviceInfo.NFC_id[1] & 0x0F);
    GeneralBuffer[24] = ValueToChar((DeviceInfo.NFC_id[2] & 0xF0) >> 4);
    GeneralBuffer[25] = ValueToChar(DeviceInfo.NFC_id[2] & 0x0F);
    GeneralBuffer[26] = ValueToChar((DeviceInfo.NFC_id[3] & 0xF0) >> 4);
    GeneralBuffer[27] = ValueToChar(DeviceInfo.NFC_id[3] & 0x0F);
    GeneralBuffer[28] = ValueToChar((DeviceInfo.NFC_id[4] & 0xF0) >> 4);
    GeneralBuffer[29] = ValueToChar(DeviceInfo.NFC_id[4] & 0x0F);
    GeneralBuffer[30] = ValueToChar((DeviceInfo.NFC_id[5] & 0xF0) >> 4);
    GeneralBuffer[31] = ValueToChar(DeviceInfo.NFC_id[5] & 0x0F);
    GeneralBuffer[32] = ',';
    GeneralBuffer[33] = ValueToChar((DeviceInfo.LeaseStartTime.Year - 2000) / 10);
    GeneralBuffer[34] = ValueToChar((DeviceInfo.LeaseStartTime.Year - 2000) % 10);
    GeneralBuffer[35] = ValueToChar(DeviceInfo.LeaseStartTime.Mon / 10);
    GeneralBuffer[36] = ValueToChar(DeviceInfo.LeaseStartTime.Mon % 10);
    GeneralBuffer[37] = ValueToChar(DeviceInfo.LeaseStartTime.Day / 10);
    GeneralBuffer[38] = ValueToChar(DeviceInfo.LeaseStartTime.Day % 10);
    GeneralBuffer[39] = ',';
    GeneralBuffer[40] = ValueToChar((DeviceInfo.LeaseEndTime.Year - 2000) / 10);
    GeneralBuffer[41] = ValueToChar((DeviceInfo.LeaseEndTime.Year - 2000) % 10);
    GeneralBuffer[42] = ValueToChar(DeviceInfo.LeaseEndTime.Mon / 10);
    GeneralBuffer[43] = ValueToChar(DeviceInfo.LeaseEndTime.Mon % 10);
    GeneralBuffer[44] = ValueToChar(DeviceInfo.LeaseEndTime.Day / 10);
    GeneralBuffer[45] = ValueToChar(DeviceInfo.LeaseEndTime.Day % 10);
    GeneralBuffer[46] = ')'; 

    Result = SendTcpPacket(GeneralBuffer, 47);
    
    return Result;
}

//**************************************************
//握手信号消息
void SendMessageBP00(void)  
{
    //unsigned char i;
    uint8_t BatteryPower[6];
	#ifdef MAIN_BOARD_TEMP_CHK
    uint8_t gpsTemperatureBuff[6];
	#endif
    //#ifdef ENABLE_CTRL_COMM
    uint8_t ctrlTemperatureBuff[6];
    //#endif
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '0';
    //终端ID
    /*for (i = 0; i < 15; i++)
    {
        GeneralBuffer[17 + i] = PlatformData.DeviceID[i];
    }*/
    //握手内容
    GeneralBuffer[20] = 'H';
    GeneralBuffer[21] = 'S';
    GeneralBuffer[22] = 'O';
    GeneralBuffer[23] = ValueToChar(DeviceInfo.CopsFlag % 10); //当前网络
    //------------电池电量-------------   
    NumberValueToHexStr((DeviceInfo.InputValue/100), BatteryPower);
    if((BatteryPower[0] < '0') || (BatteryPower[0] > 'F')) BatteryPower[0] = '0';
    if((BatteryPower[1] < '0') || (BatteryPower[1] > 'F')) BatteryPower[1] = '0';
    if((BatteryPower[2] < '0') || (BatteryPower[2] > 'F')) BatteryPower[2] = '0';
    GeneralBuffer[24] = BatteryPower[0];
    GeneralBuffer[25] = BatteryPower[1];
    GeneralBuffer[26] = BatteryPower[2];
    //------------------------------------------
    //温度数据
    GeneralBuffer[27] = 'P'; 
#ifdef MAIN_BOARD_TEMP_CHK
		//----GPS设备温度 范围从-30--125度----
		NumberValueToHexStr(DeviceInfo.TemperatureValue, gpsTemperatureBuff);
		GeneralBuffer[28] = gpsTemperatureBuff[1];
		GeneralBuffer[29] = gpsTemperatureBuff[2];
#else
		GeneralBuffer[28] = '0';
		GeneralBuffer[29] = '0';
#endif
    //----控制器温度  范围从-30--125度-----
#ifdef ENABLE_CTRL_COMM
    NumberValueToHexStr(CtrlCommData.ControlTemp, ctrlTemperatureBuff);
    GeneralBuffer[30] = ctrlTemperatureBuff[1];
    GeneralBuffer[31] = ctrlTemperatureBuff[2];
#else
    NumberValueToHexStr(EbikeData.Temperature, ctrlTemperatureBuff);
    GeneralBuffer[30] = ctrlTemperatureBuff[1];
    GeneralBuffer[31] = ctrlTemperatureBuff[2];
#endif
    //电动车行车状态
    GeneralBuffer[32] = 'T';  
#ifdef ENABLE_CTRL_COMM
    GeneralBuffer[33] = ValueToChar((CtrlCommData.FailureState[0] & 0xF0) >> 4);
    GeneralBuffer[34] = ValueToChar((CtrlCommData.FailureState[0] & 0x0F));
    GeneralBuffer[35] = ValueToChar((CtrlCommData.FailureState[1] & 0xF0) >> 4);
    GeneralBuffer[36] = ValueToChar((CtrlCommData.FailureState[1] & 0x0F));
#endif
#ifdef USE_BH_LEASE_MODE
    GeneralBuffer[33] = ValueToChar((EbikeData.State[0] & 0xF0) >> 4);  
    GeneralBuffer[34] = ValueToChar((EbikeData.State[0] & 0x0F));
    GeneralBuffer[35] = ValueToChar((EbikeData.Fault[0] & 0xF0) >> 4);
    GeneralBuffer[36] = ValueToChar((EbikeData.Fault[0] & 0x0F));
#endif
    GeneralBuffer[37] = ValueToChar((gCarFunctionState & 0xF0) >> 4);  //遥控器状态
    GeneralBuffer[38] = ValueToChar((gCarFunctionState & 0x0F));
    //电量百分比数据 
    GeneralBuffer[39] = 'Q';
#ifdef ENABLE_EXT_RS485
    GeneralBuffer[40] = ValueToChar((Rs485Data.soc1 & 0xF0) >> 4);
    GeneralBuffer[41] = ValueToChar((Rs485Data.soc1 & 0x0F));
#elif defined (ENABLE_CAN_COMM)
    GeneralBuffer[40] = ValueToChar((LionBatData.soc & 0xF0) >> 4);
    GeneralBuffer[41] = ValueToChar((LionBatData.soc & 0x0F));
#else
    GeneralBuffer[40] = '0';
    GeneralBuffer[41] = '0';
#endif
    GeneralBuffer[42] = ')';  //packet end

    SendTcpPacket(GeneralBuffer, 43);   
}

//**************************************************
//握手信号消息
void SendMessageBP25(void)
{
    //uint8_t i;
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '2';
    GeneralBuffer[19] = '5';
    GeneralBuffer[20] = ')';  //packet end 

    SendTcpPacket(GeneralBuffer, 21);   
}

//*********************************************************
//(867010032284672BP01XR701_F303_V1.1.7,Oct 29 2018)
void SendSoftWareVersion(void)
{
    uint8_t i;
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '1';

    for (i = 0; i < 17; i ++)
    {
        GeneralBuffer[20 + i] = app_Version[i];
    }
    GeneralBuffer[37] = ',';  //37
    for (i = 0; i < 11; i ++)
    {
        GeneralBuffer[38 + i] = Date[i];
    }
    GeneralBuffer[38 + i] = ')'; 

    SendTcpPacket(GeneralBuffer, 50);
}
//*********************************************************
//发送等时连续回传消息
uint8_t SendMessageBR00(void)
{
    uint8_t Result;

    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '0';
    //GPS数据，83字节
    WriteGpsDataToBuffer(20);
    GeneralBuffer[103] = ')';  //packet end
    
    DeviceInfo.SendTcpDataFlag = TRUE;
    //OS_ENTER_CRITICAL();
    DeviceInfo.SendTcpDataDlyCnt  = TEN_SECOND;//每次发送数据计时10秒，超时10秒仍未发成功，开始存盲区
    //OS_EXIT_CRITICAL();
    
    Result = SendTcpPacket(GeneralBuffer, 104);   //100 
    
    return Result;
}

#ifdef  USE_BH_LEASE_MODE 
//*********************************************************
//定时发送BMS数据 
//(863412040166772BR0435T0000P64R0041L3E80S084614V00C8)
uint8_t SendMessageBR04(void)
{
    uint8_t Result;

    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();

    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '4';
    //BMS数据 32字节
    WriteBmsDataToBuffer(20);
    GeneralBuffer[52] = ')';  //packet end
    
    Result = SendTcpPacket(GeneralBuffer, 53);   
    
    return Result;
}

//************************************************
//定时发送控制器信息
uint8_t SendMessageBR09(void)
{
    uint8_t Result;

    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();

    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '9';
    //控制器数据 31字节
    WriteSingleModeDataToBuffer(20);
    GeneralBuffer[51] = ')';  //packet end
    
    Result = SendTcpPacket(GeneralBuffer, 52);   
    
    return Result;
}
#endif //#ifdef USE_BH_LEASE_MODE

#ifdef USE_MJ_MULTI_MODE
//*********************************************************
//定时发送多模共轨控制器数据 
//(867881040215878BR07,1,031002,2D,03D,001C8,55,3E,030C,1C20,05DC,0370,814202)
//(867881040215878BR07,2,031000,2D,03D,001C8,55,3E,0316,1C20,05DC,0370,814202)
uint8_t SendMessageBR07(uint8_t ctrl_num)
{
	uint8_t Result;

	GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();

    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '7';
    //控制器数据 55字节
    WriteCtrlDataToBuffer(20, ctrl_num);
    GeneralBuffer[75] = ')'; 
    
    Result = SendTcpPacket(GeneralBuffer, 76);   
    
    return Result;
}

//*********************************************************
//定时发送多模共轨控制器数据 
//(867881040215878BR08,1,6,14,14,0A,2,0000,20E5,0004,0000,63,035A,0358,1C20,1E,64,0898,0BB8,0120)
//(867881040215878BR08,2,6,14,14,0A,2,0000,20E5,0004,0000,63,035A,0358,1C20,1E,64,0898,0BB8,0120)
uint8_t SendMessageBR08(uint8_t bms_num)
{
	uint8_t Result;

	GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();

    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '8';
    //BMS数据 74字节
    WriteBmsDataToBuffer(20, bms_num);
    GeneralBuffer[94] = ')'; 
    
    Result = SendTcpPacket(GeneralBuffer, 95);   
    
    return Result;
}
#endif //#ifndef  USE_MJ_MULTI_MODE 

//*********************************************************
//发送记录数据
uint8_t SendMessageBR01(void)
{
    uint8_t Result;
    
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '0'; 
    //GPS数据
    WriteGpsDataToBuffer(20);
    GeneralBuffer[87] = ')';  //packet end

    //记录数据不发电动车状态信息，节省流量
    Result = SendTcpPacket(GeneralBuffer, 88);
  
    return Result;
}

//*********************************************************
//报警数据补偿回传消息
uint8_t SendMessageBO02(void)
{
    uint8_t Result;
    
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    //(861510030185611BO027170325A2238.6502N11402.4939E000.0092828116.00010000BFL00000003S3014)
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'O';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '2';
    #ifdef STM32RECORD 
    GeneralBuffer[20] = ReadRecordData.Alert;  
    #endif
    #ifdef SPI_FLASH_RECORD
    GeneralBuffer[20] = SpiReadRecordData.Alert;
    #endif
    
    //GPS数据
    PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_RECORD;  
    WriteGpsDataToBuffer(21);
    GeneralBuffer[93] = ')';  //packet end

    Result = SendTcpPacket(GeneralBuffer, 94); //89  
  
    return Result;
}


//*********************************************************
//发送连续回传结束消息
void SendMessageBR02(void)
{
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '2';
    //GPS数据
    WriteGpsDataToBuffer(20);
    GeneralBuffer[87] = ')';  //packet end

    SendTcpPacket(GeneralBuffer, 88);  //83 
}


//*********************************************************
//应答等时连续回传设置消息
void SendMessageBS08(unsigned char *pMsgBuffer)
{
    uint8_t i;


    for (i = 0; i < 25; i++)
    {
        GeneralBuffer[i] = pMsgBuffer[i];
    }
    //命令
    GeneralBuffer[13] = 'B';
    GeneralBuffer[14] = 'S';
    GeneralBuffer[15] = '0';
    GeneralBuffer[16] = '8';
    
    GeneralBuffer[25] = ')';  //packet end

    SendTcpPacket(GeneralBuffer, 26);
}


//**************************************************************
//警报消息
void SendMessageBO01(unsigned char Code)
{
    uint8_t SendResult;  
    
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'O';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '1';
    //报警码
    GeneralBuffer[20] = Code;         
        
    //
    BO01_State = Code;
    
    #ifdef STM32RECORD  
    //保存数据到缓冲
    WriteGpsToBuffer(&WriteRecordData); 
    #endif
    
    //先保存要发的数据     
    #ifdef SPI_FLASH_RECORD
    SpiFlashWriteGpsToBuffer(&SpiWriteRecordData);   
    #endif
    BO01_State = 0xFF; 
       
    //GPS数据,83个字节
    WriteGpsDataToBuffer(21);
    GeneralBuffer[104] = ')';  //packet end  

    SendResult = SendTcpPacket(GeneralBuffer, 105);  
  
    if (SendResult == TRUE)  //发送结果处理，发送失败，写入flash
    {
        PlatformData_HY.ResendPacketFlag = FALSE;
    }
    else
    {
        //必须定位成功，非静止状态才保存盲区 
        //if ((GpsData.GpsSignalFlag == TRUE) && (GpsData.StopStateFlag == FALSE))
        //{  //报警数据不是必须要定位
          #ifdef STM32RECORD  
            WriteRecordToMemory(&WriteRecordData);
            DEBUG_PRINT("\r\nSave Alarm data to flash!");
          #endif
            
          #ifdef SPI_FLASH_RECORD          
            SpiFlashWriteRecordToMemory(&SpiWriteRecordData); 
            #ifdef DEBUG_OUTPUT0
            DEBUG_PRINT("\r\nSave Alarm data to Ext_flash!");
            #endif
          #endif            
        //}
    } 
}


//***********************************
//应答点名信息
//void SendMessageBP04(void)
//{
//    GeneralBuffer[0] = '(';  //数据包开始

//    //15位终端IMEI号 
//    WriteIMEIToBuffer();
//    
//    //命令
//    GeneralBuffer[16] = 'B';
//    GeneralBuffer[17] = 'P';
//    GeneralBuffer[18] = '0';
//    GeneralBuffer[19] = '4';
//    //GPS数据
//    WriteGpsDataToBuffer(20);
//    GeneralBuffer[99] = ')';  //packet end

//    SendTcpPacket(GeneralBuffer, 100);
//}

//*********************************************************
//应答上报蓝牙和遥控器开关状态  (777777777777777BP101)
void SendMessageBP10(void)
{
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'P';
    GeneralBuffer[18] = '1';
    GeneralBuffer[19] = '0';
    //开关状态位
    if((gCarFunctionState & FUNC_OPEN_CLOSE_REM_BIT) == 0) 
    {
        GeneralBuffer[20] = '1'; //蓝牙开
    }
    else
    {
        GeneralBuffer[20] = '0'; //蓝牙关
    }
    GeneralBuffer[21] = ')';  //packet end
    
    SendTcpPacket(GeneralBuffer, 22);
}

//*********************************************************
//应答设置车速上下限消息
void SendMessageBP12(unsigned char *pMsgBuffer)
{
    uint8_t i;


    for (i = 0; i < 25; i++)
    {
        GeneralBuffer[i] = pMsgBuffer[i];
    }
    //命令
    GeneralBuffer[13] = 'B';
    GeneralBuffer[14] = 'P';
    GeneralBuffer[15] = '1';
    GeneralBuffer[16] = '2';
    
    GeneralBuffer[25] = ')';  //packet end

    SendTcpPacket(GeneralBuffer, 26);
}

//*********************************************************
//应答查询遥控器ID
void SendMessageBU04(uint32_t RemAddr)
{
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'U';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '4';
    #ifdef RCV_DATA_1527_MSB
    //遥控器ID码  2字节
    GeneralBuffer[20] = ValueToChar(RemAddr >> 12);
    GeneralBuffer[21] = ValueToChar((RemAddr & 0x0F00) >> 8);
    GeneralBuffer[22] = ValueToChar((RemAddr & 0x00F0) >> 4);
    GeneralBuffer[23] = ValueToChar((RemAddr & 0x000F));
    GeneralBuffer[24] = ')';  //packet end
    SendTcpPacket(GeneralBuffer, 25);
    #else
    //遥控器ID码  5位 2个半字节
    GeneralBuffer[20] = ValueToChar(RemAddr >> 16);
    GeneralBuffer[21] = ValueToChar((RemAddr & 0x0F000) >> 12);
    GeneralBuffer[22] = ValueToChar((RemAddr & 0x00F00) >> 8);
    GeneralBuffer[23] = ValueToChar((RemAddr & 0x000F0) >> 4);
    GeneralBuffer[24] = ValueToChar((RemAddr & 0x0000F));
    GeneralBuffer[25] = ')';  //packet end
    SendTcpPacket(GeneralBuffer, 26);
    #endif
}

//**************************************************************
//通用应答
unsigned char SendMessageStr(unsigned char* pStr)
{
    OS_ERR err;
    uint8_t i, cnt;

    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号 
    WriteIMEIToBuffer();
    
    //数据
    i = 0;
    while (pStr[i] != 0)
    {
        GeneralBuffer[16 + i] = pStr[i];
        i ++;
    }

    GeneralBuffer[16 + i] = ')';  //packet end

    cnt = 3;   
    while (cnt --)
    {
        if (SendTcpPacket(GeneralBuffer, 17 + i) == TRUE)
        {
            return TRUE;
        }
        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_DLY, &err); 
    }
    
    return FALSE;
}


//*********************************************************
//发送请求位置消息
void SendMessageBR03(void)
{
    GeneralBuffer[0] = '(';  //数据包开始

    //15位终端IMEI号
    WriteIMEIToBuffer();
    
    //命令
    GeneralBuffer[16] = 'B';
    GeneralBuffer[17] = 'R';
    GeneralBuffer[18] = '0';
    GeneralBuffer[19] = '3';
    //GPS数据
    WriteGpsDataToBuffer(20);
    GeneralBuffer[89] = ')';  //packet end
    SendTcpPacket(GeneralBuffer, 95);
}


//*********************************************
unsigned int GetGeoValue(unsigned char *pBuff)
{
    uint16_t   value;
    uint8_t i;


    value = 0;
    for (i = 0; i < 2; i ++)
    {
        value = value * 10 + CharToValue(pBuff[i]);
    }
    for (i = 3; i < 6; i ++)
    {
        value = value * 10 + CharToValue(pBuff[i]);
    }
    return value;
}

//***************************************************
//修改遥控器ID码
uint32_t ChangeRemCtrID(uint8_t *pInitValue,uint8_t num)
{
    uint32_t value;
    uint8_t i;


    i = 0;
    value = 0;
    while (i != num)
    {
        value = value * 16 + CharToValue(*(pInitValue + i));
        i++;
    }
    return value;
}


//****************************************************************************
void PlatformMessageProc(uint8_t *pMsgBuffer)
{
    uint8_t i;//, tmp;
    uint16_t     uiTmp,j;
    //uint8_t beep_cnt;
    #ifdef FTP_UPGRAGE
    CPU_SR_ALLOC();
    #endif
    //OS_ERR err;
    uint8_t ucPos = 0;
    #ifdef ENABLE_AGPS_DOWN
    uint16_t temp = 0;
    #endif

    #ifdef FTP_UPGRAGE
    uint8_t nCount;
    uint16_t nID;
    uint16_t dwTemp;
    #endif
    
    uint8_t id;
    id = 0;

    #ifdef ENABLE_AGPS_DOWN
     //(c) 1997-2009 u-blox AGContent-Length: 3192
    if (CompareStr(pMsgBuffer + 14 + id, "u-blox", 6) == TRUE)  
    {
        for (i = 0; i < 4; i++)
        {
            temp = temp * 10 + CharToValue(pMsgBuffer[40 + i]);
        }
        
        USART3_SendStr(pMsgBuffer + 79,temp);
        
        FirstPowerFlag = FALSE; //AGPS下载成功
        
        #ifdef DEBUG_OUTPUT0
        //DEBUG_PRINT("\r\nAGPS-Revc-OK! Rev_Data: %s",pMsgBuffer);
        //DEBUG_PRINT("\r\nData_Len: %d",temp);
        //DEBUG_PRINT("\r\nAGPS Download OK!");  //需要关闭打印信息，共用模拟串口
        #endif
        
        return;
    }
    #endif //#ifdef ENABLE_AGPS_DOWN
    
    //--------------------------------------------------
    //终端注册响应消息
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP05", 4) == TRUE)
    {
        //DEBUG_PRINT("\r\nregister success.");

        #ifdef FTP_UPGRAGE
        //升级成功，清0升级参数
        //Flash_UpIl();        
        
        DEBUG_PRINT("\r\n---State  = 0x%02X---", EEP_UpdateConveyMessage.UpdateState);
        DEBUG_PRINT("\r\n---Result = 0x%02X---\r\n", EEP_UpdateConveyMessage.UpdateResult);
        //上报平台升级状态 (BS07)
        switch(CheckIapUpgradeState())
        {
            case IAP_UP_GRADE_OK:
                SendMessageStr("BS071");  
                break;
                
            case IAP_UP_GRADE_MD5_ERROR:
                SendMessageStr("BS072"); 
                break;
                
            case IAP_UP_GRADE_WRITE_FAIL:
                SendMessageStr("BS073"); 
                break;
                
            default:
                break;
        }
        #endif
    
        PlatformData.DeviceRegisterSuccess = TRUE;   
        
        return;
    }
    //--------------------------------------------------
    //CCID消息响应
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP06", 4) == TRUE)
    {
        //DEBUG_PRINT("\r\nCCID register success.");
        
        SendMessageBP10(); 
        
        DeviceInfo.SimCcidUpFlag = TRUE;
        return;
    }
    
    //--------------------------------------------------
    //BMS id消息响应 
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP11", 4) == TRUE)
    {
        DeviceInfo.SimBmsIdUpFlag = TRUE;
        
        DEBUG_PRINT("---bms id register success!!!---\r\n");      
        
        return;
    }
    
    //-------------------------------------------------
    //等时连续回传设置消息
//    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AR00", 4) == TRUE)
//    {
//        //判断数据是否正确
//        if (IsHexNumber(pMsgBuffer + 20, 8) == FALSE)
//        { 
//            return;
//        }

//        //获取间隔时间
//        AutoTrackData.IntervalTime = 0;
//        
//        for (i = 0; i < 4; i++)
//        {
//            AutoTrackData.IntervalTime = AutoTrackData.IntervalTime * 0x10 + CharToValue(pMsgBuffer[20 + i]);
//        }
//        
//        if (AutoTrackData.IntervalTime < AUTO_TRACK_DATA_INTERVAL_MIN_TIME)
//        {
//            if (AutoTrackData.IntervalTime == 0)
//            {
//                AutoTrackData.IntervalTime = 60;
//            }
//            else
//            {
//                AutoTrackData.IntervalTime = AUTO_TRACK_DATA_INTERVAL_MIN_TIME;
//            }
//        }
//        
//        //INTX_DISABLE();
//        AutoTrackData.RemainTime   = AutoTrackData.IntervalTime;
//        //INTX_ENABLE();
//        
//        #ifdef DEBUG_OUTPUT0
//        Debug_PrintValue("auto track interval high:",
//                         AutoTrackData.IntervalTime / 256);
//        Debug_PrintValue("auto track interval low:",
//                         AutoTrackData.IntervalTime % 256);
//        #endif
//        SaveAccInterval(AutoTrackData.IntervalTime, AutoTrackData.IntervalTime);

//        //获取回传总时间
//        tmp = 0;
//        for (i = 0; i < 2; i++)  //获取小时
//        {
//            tmp = tmp * 0x10 + CharToValue(pMsgBuffer[24 + i]);
//        }
//        AutoTrackData.SumTime = tmp * 3600;  //转换成秒
//        tmp = 0;
//        for (i = 0; i < 2; i++)  //获取分钟
//        {
//            tmp = tmp * 0x10 + CharToValue(pMsgBuffer[27 + i]);
//        }
//        AutoTrackData.SumTime += tmp * 60;  //转换成秒
//        
//        #ifdef DEBUG_OUTPUT0
//        Debug_PrintValue("auto track sum time high:",
//                         AutoTrackData.SumTime / 256);
//        Debug_PrintValue("auto track sum time low:",
//                         AutoTrackData.SumTime % 256);
//        #endif

//        if (AutoTrackData.IntervalTime == 0)
//        {
//            //停止回传
//            GpsFunctionState &= ~FUNC_AUTO_TRACK_BIT;
//        }
//        else
//        {
//            if (AutoTrackData.SumTime == 0)
//            {
//                AutoTrackData.SumTime = 0xFFFFFFFF; //无限次数
//            }

//            AutoTrackData.bSendFlag = FALSE;
//            GpsFunctionState |= FUNC_AUTO_TRACK_BIT;
//            GpsData.bDataState = GPS_OLD_DATA;
//            OpenGpsPower();
//        }
//        //应答等时连续回传设置消息
//        SendMessageBS08(pMsgBuffer);
//        return;
//    }

    //------------------------------------------------------------
    //上行警报消息应答处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AS01", 4) == TRUE)
    {
        if (PlatformData.DeviceRegisterSuccess == TRUE)
        {
            /*if ((pMsgBuffer[17] - '0') < MAX_ALERT_SUM)
            {
                PlatformData_HY.AlertType[pMsgBuffer[17] - '0'].AlertFlag = FALSE;
                #ifdef DEBUG_OUTPUT0
                Debug_PrintValue("Ack Code:", pMsgBuffer[17] - '0');
                #endif
            }*/
            
            PlatformData_HY.HaveAlertFlag = FALSE; //收到平台应答，不需要再报警
        }
    }

    //------------------------------------------------------------
    //握手消息处理  (861694037570601AP01)
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP01", 4) == TRUE)
    {
        //DEBUG_PRINT("---Handshake ACK success!!!---\r\n");
        //DispTaskInfo();  //打印任务信息 
        
        GprsConnectTimeOut = FIFTEEN_MINUTE;
        PlatformData_HY.HandshakeFailCnt = 0;
        return ;
    }

    //------------------------------------------------------------
    //一次点名消息处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP00", 4) == TRUE)
    {
        DEBUG_PRINT("\r\nSend Enquiry Message.");

        //应答点名信息
        //SendMessageBP04();
        return ;
    }

    //------------------------------------------------------------
    //设置车速上下限消息处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP12", 4) == TRUE)
    {
        uint16_t uiTmp;

        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nSet Car Speed Alert.");
        #endif
        
        if (pMsgBuffer[20] == 'H')
        {
            uiTmp = 0;
            for (i = 0; i < 3; i ++)
            {
                uiTmp = uiTmp * 10 + CharToValue(pMsgBuffer[21 + i]);
            }
            if (uiTmp == 0)
            {
                SpeedAlertData.EnableOverSpeedAlert = FALSE;
            }
            else
            {
                SpeedAlertData.EnableOverSpeedAlert = TRUE;
                SpeedAlertData.OverSpeed = uiTmp;
                GpsFunctionState |= FUNC_OVERSPEED_ALERT_BIT;
                SpeedAlertData.SendDirection = SEND_TO_PLATFORM;
                SpeedAlertData.OverSpeedCnt = 0;
                SpeedAlertData.OverSpeedAlertFlag = FALSE;
                SpeedAlertData.NextAlertTimeCnt = 0;
            }
            #ifdef DEBUG_OUTPUT0
            Debug_PrintValueInt("Over:", uiTmp);
            #endif
        }
        if (pMsgBuffer[24] == 'L')
        {
            uiTmp = 0;
            for (i = 0; i < 3; i ++)
            {
            uiTmp = uiTmp * 10 + CharToValue(pMsgBuffer[25 + i]);
            }
            if (uiTmp == 0)
            {
                SpeedAlertData.EnableLowSpeedAlert = FALSE;
            }
            else
            {
                SpeedAlertData.EnableLowSpeedAlert = TRUE;
                SpeedAlertData.LowSpeed = uiTmp;
                GpsFunctionState |= FUNC_OVERSPEED_ALERT_BIT;
                SpeedAlertData.SendDirection = SEND_TO_PLATFORM;
                SpeedAlertData.LowSpeedCnt = 0;
                SpeedAlertData.LowSpeedAlertFlag = FALSE;
            }
            #ifdef DEBUG_OUTPUT0
            Debug_PrintValueInt("Low:", uiTmp);
            #endif
        }
        
        SpeedAlertData.NewGpsData = FALSE;
        if ((SpeedAlertData.EnableLowSpeedAlert == FALSE)
            && (SpeedAlertData.EnableOverSpeedAlert == FALSE))
        {
            GpsFunctionState &= ~FUNC_OVERSPEED_ALERT_BIT;
        }
        SendMessageBP12(pMsgBuffer);
    }

    
 
#ifdef USE_BH_LEASE_MODE 
    //一键启动熄火控制
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV04", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            OneKeyStop(GPRS_CMD);
        }
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            
            //if((gCarFunctionState & FUNC_START_STOP_BIT) == 0) 
            //{
                OneKeyStart(GPRS_CMD);
            //}
        }
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nCarFunctionState:0x%02X",gCarFunctionState);
        #endif
        //SaveCarFunctionState();   //保存状态 
        return;
    }
#endif

    //一键恢复霍尔控制
    /*if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV05", 4) == TRUE)
    {       
        #ifdef USE_VIRTUAL_COM
        Disable_App_Receive_INT(); 
        #endif
        
        switch(pMsgBuffer[COMMAND_OFFSET_BIT])
        {
            case '1':
                DEBUG_PRINT("\r\nRepair failure hall");

                SendMessageStr("BV051");
#ifdef ENABLE_CTRL_COMM
                SendRepairHallCmd();
#endif
            break;
                
            default:
            break;
        }
        #ifdef USE_VIRTUAL_COM
        Enable_App_Receive_INT();
        #endif
		return ;
    }*/
    
#ifdef USE_BH_LEASE_MODE
    //寻车功能，控制电动车喇叭响
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV06", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            HornCar(GPRS_CMD);
        }
    }
    
    //设防解防功能  0:解防  1:设防
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV07", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            //设防状态下
            //if((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) != 0)
            //{
                unLockCar(GPRS_CMD);
            //}
        }
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            //非设防状态下
            //if((gCarFunctionState & FUNC_LOCKED_UNLOCKED_BIT) == 0) 
            //{
                LockCar(GPRS_CMD);
            //}
        }    
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nCarFunctionState:0x%02X",gCarFunctionState);
        #endif
        //SaveCarFunctionState();
    }
    

    //开电池仓锁  1:打开
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV09", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            //开电池仓锁
            if(BatBoxLockProc(1))
                SendMessageStr("BV091");
            else
                SendMessageStr("BV090");
        }
    }
#endif

    //开座垫锁  1:打开
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV08", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            /*OpenSeatLock();
            OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_DLY, &err);
            CloseSeatLock(); //打开后恢复，以便能锁上
            SendMessageStr("BV081");*/
            
            if(BatBoxLockProc(1))
                SendMessageStr("BV081");
            else
                SendMessageStr("BV080");
        }
    }


    //打开关闭蓝牙及遥控  0:关闭，:打开
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV12", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            gCarFunctionState |= FUNC_OPEN_CLOSE_REM_BIT; //关闭遥控功能
            SendMessageStr("BV120");
        }
		else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
		{
			gCarFunctionState &= ~FUNC_OPEN_CLOSE_REM_BIT; 
            SendMessageStr("BV121");
		}
		SaveCarFunctionState(); 
    }
    

    //设置控制器定速巡航，0：关，1：开
    /*if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV13", 4) == TRUE)
    {
        #ifdef ENABLE_CTRL_COMM
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            SendMessageStr("BV130");
            SendSetCruiseModeCmd(0x00);
        }
		else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
		{
            SendMessageStr("BV131");
            SendSetCruiseModeCmd(0x01);
		}
        #endif
    }*/

#ifdef USE_BH_LEASE_MODE    

    //设置速度模式 低/中/高速模式
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV14", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            SendMessageStr("BV141");
            #ifdef ENABLE_CTRL_COMM
            SendSetSpeedModeCmd(0x01);
            #endif
            #ifdef USE_BH_LEASE_MODE
            SetCtrlSpeedModeCmd(0x01);
            #endif
        }
		else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '2')
		{
            SendMessageStr("BV142");
            #ifdef ENABLE_CTRL_COMM
            SendSetSpeedModeCmd(0x02);
            #endif
            #ifdef USE_BH_LEASE_MODE
            SetCtrlSpeedModeCmd(0x02);
            #endif
		}
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '3')
		{ 
            SendMessageStr("BV143");
            #ifdef ENABLE_CTRL_COMM
            SendSetSpeedModeCmd(0x03);
            #endif
            #ifdef USE_BH_LEASE_MODE
            SetCtrlSpeedModeCmd(0x03);
            #endif
		}
    }


    //设置控制器电压等级 48V/60V/72V 
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV19", 4) == TRUE)
    { 
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            SendMessageStr("BV191");
            SetCtrlVoltGradeCmd(0x01);
        }
		else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '2')
		{
            SendMessageStr("BV192");
            SetCtrlVoltGradeCmd(0x02);
		}
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '3')
		{ 
            SendMessageStr("BV193");
            SetCtrlVoltGradeCmd(0x03);
		}
    }
    

    /*
      此功能由服务端检测区域，如果接近禁行区域，下发命令，
      由设备播放“已超出经营范围，请及时返回”
    */
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV20", 4) == TRUE)
    {
        if(DeviceInfo.SoundMode == VOICE_MODE)
            OneWriteSendCmd(EXCEEDED_BUSINESS);
        
        //SendMessageStr("BV20");
    }
#endif //USE_BH_LEASE_MODE
    

    //设置控制器软硬启动模式，0：软启动，1：硬启动
    /*if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV15", 4) == TRUE)
    {
        #ifdef ENABLE_CTRL_COMM
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            SendMessageStr("BV150");
            SendSetStartModeCmd(0x00);
        }
		else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
		{
            SendMessageStr("BV151");
            SendSetStartModeCmd(0x01);
		}
        #endif
    }*/
    

    //指定RGB颜色 (861694037570601AV101)
    /*if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV10", 4) == TRUE)
    {
        if(pMsgBuffer[COMMAND_OFFSET_BIT] =='1')
        {
            RgbData.RgbDispState = SPECIFY_COLOR_LED;
            RgbData.RgbSate++;
            if(RgbData.RgbSate>=6)
                RgbData.RgbSate= 0;
            
            SendMessageStr("BV101");
        }
        return;
    }
    
    //七彩灯和循环彩灯(861694037570601AV110)
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV11", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')  //七彩灯
        {
            RgbData.RgbDispState = SEVEN_COLOR_LED;
            SendMessageStr("BV110");
            return;
        }
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1') //循环彩灯
        {
            RgbData.RgbDispState = CYCLE_COLOR_LED;
            SendMessageStr("BV111");
            return;
        }
    }*/

    
    //开关振动检测
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV03", 4) == TRUE)
    {         
        switch(pMsgBuffer[COMMAND_OFFSET_BIT])  
        {
            case '0':
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nClose Shaking detection!");
                #endif
                GpsFunctionState &= ~FUNC_VIBRATION_ON_OFF_BIT; 
                SendMessageStr("BV030");
            break;
                
            case '1':
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nShaking detection 1 Level");
                #endif
                GpsFunctionState |= FUNC_VIBRATION_ON_OFF_BIT;  

                DeviceData.Gsensor_threshold_Value = VIBRATION_THRESHOLD_1;
                #ifdef ENABLE_G_SENSOR_CHECK
                  #ifdef G_SENSOR_LIS3DH
                    Lis3dh_Init_LowScale();
                  #elif defined G_SENSOR_BMA250
                    Bma250RegConfig();
                  #elif defined G_SENSOR_MSA300
                    MSA300_Init();
                  #endif
                #endif
                SendMessageStr("BV031");
            break;            
                
            case '2':
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nShaking detection 2 Level");
                #endif
                GpsFunctionState |= FUNC_VIBRATION_ON_OFF_BIT;  

                DeviceData.Gsensor_threshold_Value = VIBRATION_THRESHOLD_2;  
                #ifdef ENABLE_G_SENSOR_CHECK
                  #ifdef G_SENSOR_LIS3DH
                    Lis3dh_Init_LowScale();
                  #elif defined G_SENSOR_BMA250
                    Bma250RegConfig();
                  #elif defined G_SENSOR_MSA300
                    MSA300_Init();
                  #endif
                #endif
                SendMessageStr("BV032");
            break;
                
            case '3':
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nShaking detection 3 Level");
                #endif
                GpsFunctionState |= FUNC_VIBRATION_ON_OFF_BIT;  

                DeviceData.Gsensor_threshold_Value = VIBRATION_THRESHOLD_3; 
                #ifdef ENABLE_G_SENSOR_CHECK
                  #ifdef G_SENSOR_LIS3DH
                    Lis3dh_Init_LowScale();
                  #elif defined G_SENSOR_BMA250
                    Bma250RegConfig();
                  #elif defined G_SENSOR_MSA300
                    MSA300_Init();
                  #endif
                #endif              
                SendMessageStr("BV033");
            break;
                
            case '4':
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nShaking detection 4 Level");
                #endif
                GpsFunctionState |= FUNC_VIBRATION_ON_OFF_BIT;  
                    
                DeviceData.Gsensor_threshold_Value = VIBRATION_THRESHOLD_4; 
                #ifdef ENABLE_G_SENSOR_CHECK
                  #ifdef G_SENSOR_LIS3DH
                    Lis3dh_Init_LowScale();
                  #elif defined G_SENSOR_BMA250
                    Bma250RegConfig();
                  #elif defined G_SENSOR_MSA300
                    MSA300_Init();
                  #endif
                #endif                 
                SendMessageStr("BV034");
            break;
                
            case '5':
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nShaking detection 5 Level");
                #endif
                GpsFunctionState |= FUNC_VIBRATION_ON_OFF_BIT;  
                    
                DeviceData.Gsensor_threshold_Value = VIBRATION_THRESHOLD_5; 
                #ifdef ENABLE_G_SENSOR_CHECK
                  #ifdef G_SENSOR_LIS3DH
                    Lis3dh_Init_LowScale();
                  #elif defined G_SENSOR_BMA250
                    Bma250RegConfig();
                  #elif defined G_SENSOR_MSA300
                    MSA300_Init();
                  #endif
                #endif                 
                SendMessageStr("BV035");
            break;
                
            default:
                #ifdef DEBUG_OUTPUT0
                DEBUG_PRINT("\r\nShaking detection 3 Level");
                #endif
                GpsFunctionState |= FUNC_VIBRATION_ON_OFF_BIT;  
                    
                DeviceData.Gsensor_threshold_Value = VIBRATION_THRESHOLD_3;   
                #ifdef ENABLE_G_SENSOR_CHECK
                  #ifdef G_SENSOR_LIS3DH
                    Lis3dh_Init_LowScale();
                  #elif defined G_SENSOR_BMA250
                    Bma250RegConfig();
                  #elif defined G_SENSOR_MSA300
                    MSA300_Init();
                  #endif
                #endif                
                SendMessageStr("BV033");
            break;      
        }

        SaveGpsFunctionState();
    }

    //设置BMS供电输出  0:关闭  1:打开 
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV16", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            DEBUG_PRINT("\r\nClose BMS power...!");
            
            #ifdef ENABLE_EXT_RS485 
            bms_output_ctrl(0x00);
            #endif
            DeviceInfo.PlatformSetBmsPWRon = FALSE;
            SendMessageStr("BV160");
        }
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            DEBUG_PRINT("\r\nOpen BMS power...!");
               
            #ifdef ENABLE_EXT_RS485
            bms_output_ctrl(0x01);
            #endif
            DeviceInfo.PlatformSetBmsPWRon = TRUE;
            SendMessageStr("BV161");
        }    
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("gCarFunctionState:0x%02X",CarFunctionState);
        #endif
        //SaveCarFunctionState();   //保存状态
        return;
    }
    
    //=================================================================
    //设置BMS充电类型  0:判断电压  1:握手通讯
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AV17", 4) == TRUE)
    {
        if (pMsgBuffer[COMMAND_OFFSET_BIT] == '0')
        {
            DEBUG_PRINT("\r\n判断电压充电方式...!");
            
            #ifdef ENABLE_EXT_RS485 
            bms_set_charge_type(0x00);
            #endif
            //gCarFunctionState &= ~FUNC_CHARGE_TYPE_BIT;
            SendMessageStr("BV170");
        }
        else if (pMsgBuffer[COMMAND_OFFSET_BIT] == '1')
        {
            DEBUG_PRINT("\r\n握手通讯充电方式...!");
               
            #ifdef ENABLE_EXT_RS485 
            bms_set_charge_type(0x01);
            #endif
            //gCarFunctionState |= FUNC_CHARGE_TYPE_BIT;
            SendMessageStr("BV171");
        }

        return;
    }
    
    //------------------------------------------------------------
    //控制重启
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AT00", 4) == TRUE)
    {
        DEBUG_PRINT("\r\nReboot system...");

        SendMessageStr("BT00");

        SaveSumMeter();
        while (1); //看门狗复位
    }

    //-------------------------------------------------
    //设置ACC开发送数据间隔消息
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AR05", 4) == TRUE)
    {
        //判断数据是否正确
        if (IsHexNumber(pMsgBuffer + COMMAND_OFFSET_BIT, 4) == FALSE)
        {
            return;
        }

        //获取间隔
        uiTmp = 0;
        for (i = 0; i < 4; i ++)
        {
            uiTmp = uiTmp * 16 + CharToValue(pMsgBuffer[COMMAND_OFFSET_BIT + i]);
        }
        if (uiTmp < AUTO_TRACK_DATA_INTERVAL_MIN_TIME)
        {
                uiTmp = AUTO_TRACK_DATA_INTERVAL_MIN_TIME;
        }
        
        SaveAccInterval(uiTmp, DeviceInfo.AccCloseInterval);

        if (DeviceInfo.CarAccWork == TRUE)
        {
            AutoTrackData.IntervalTime = DeviceInfo.AccOpenInterval;            
            //INTX_DISABLE();
            AutoTrackData.RemainTime   = DeviceInfo.AccOpenInterval;
            //INTX_ENABLE();
        }
        SendMessageStr("BR05");
        return ;
    }

    //-------------------------------------------------
    //设置ACC关发送数据间隔消息
//    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AR06", 4) == TRUE)
//    {
//        //判断数据是否正确
//        if (IsHexNumber(pMsgBuffer + COMMAND_OFFSET_BIT, 4) == FALSE)
//        { 
//            return;
//        }

//        //获取间隔
//        uiTmp = 0;
//        for (i = 0; i < 4; i ++)
//        {
//            uiTmp = uiTmp * 16 + CharToValue(pMsgBuffer[COMMAND_OFFSET_BIT + i]);
//        }
//        if (uiTmp < AUTO_TRACK_DATA_INTERVAL_MIN_TIME)
//        {
//            uiTmp = AUTO_TRACK_DATA_INTERVAL_MIN_TIME;
//        }
//            
//        SaveAccInterval(DeviceInfo.AccOpenInterval, uiTmp);

//        if (DeviceInfo.CarAccWork == FALSE)
//        {
//            AutoTrackData.IntervalTime = DeviceInfo.AccCloseInterval;
//            //INTX_DISABLE();
//            AutoTrackData.RemainTime   = DeviceInfo.AccCloseInterval;
//            //INTX_ENABLE();
//        }
//        SendMessageStr("BR06");
//        return ;
//    }

    //-------------------------------------------------
    //应答获取位置消息
//    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AR03", 4) == TRUE)
//    {
//        #ifdef DEBUG_OUTPUT
//        DEBUG_PRINT("\r\nCH: Ack Get Position..");
//        #endif
//        
//        //SendSmsChineseLen(GetPositionData.RequestTelephone, pMsgBuffer + 21, pMsgBuffer[20]);
//        
//        GetPositionData.RequestFlag = FALSE;
//        return ;
//    }

    //-------------------------------------------------
    //应答获取位置消息
//    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AR04", 4) == TRUE)
//    {
//        #ifdef  SMS_ENGLISH_VERSION
//        #ifdef DEBUG_OUTPUT
//        DEBUG_PRINT("\r\nEN: Ack Get Position..");
//        #endif

//        SendSms(pMsgBuffer + COMMAND_OFFSET_BIT, GetPositionData.RequestTelephone);
//        GetPositionData.RequestFlag = FALSE;
//        #endif
//    }

    //-------------------------------------------------------
    //设置IP地址和端口
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP03", 4) == TRUE)
    {
        uint8_t IP[4];
        uint16_t      Port;

        for (i = 0; i < 4; i ++)
        {
            IP[i] = CharToValue(pMsgBuffer[20 + i * 3]);
            IP[i] = IP[i] * 10 + CharToValue(pMsgBuffer[21 + i * 3]);
            IP[i] = IP[i] * 10 + CharToValue(pMsgBuffer[22 + i * 3]);
        }
        Port = 0;
        for (i = 0; i < 5; i ++)
        {
            Port = Port * 10 + CharToValue(pMsgBuffer[32 + i]);
        }
        
        SendMessageStr("BP02");
        //保存IP地址和端口
        SaveIpAndPort(IP, Port);
        DEBUG_UART("\r\nsystem will reboot!!!");
        while (1);    //看门狗复位
    }

    //-------------------------------------------------------
    //设置域名和端口  (863412040166772AP09gps.znsjdz.com 08833)
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP09", 4) == TRUE)
    {
        i = 0;
        while(pMsgBuffer[20 + i] != ' ')
        {
            PlatformData.PlatformDomainName[i] = pMsgBuffer[20 + i];
            DeviceData.PlatformDomainName[i] = pMsgBuffer[20 + i];
            i ++;
            if (i >= MAX_DOMAIN_NAME_LEN)
            {
                i --;
                break;
            }
        }
        PlatformData.PlatformDomainName[i] = 0;
        DeviceData.PlatformDomainName[i] = 0;
        
        //得端口值
        uiTmp = 0;
        i ++; //跳过域名和端口中间的空格
        while (CompareStr(pMsgBuffer + 20 + i, ")", 1) != TRUE)
        {
            if (IsNumber(pMsgBuffer + 20 + i, 1) == FALSE)
            {
                //"端口错误1" 
                DEBUG_PRINT("\r\nPort error1!");
                DEBUG_PRINT("\r\nucPos:%d",i);
                return ;
            }
            uiTmp = uiTmp * 10 + CharToValue(pMsgBuffer[20 + i]);
            i ++;
            if (i > (20 + i + 5))
            {
                //"端口错误2"
                DEBUG_PRINT("\r\nPort error2!");
                return;
            }
        }
        PlatformData.PlatformPort = uiTmp;
        DeviceData.PlatformPort = uiTmp;

        SendMessageStr("BP02");
        
        SaveDomainName(PlatformData.PlatformDomainName,PlatformData.PlatformPort);
        DEBUG_PRINT("\r\n设置域名成功! 域名:%s,端口:%d",PlatformData.PlatformDomainName,PlatformData.PlatformPort);
        DEBUG_PRINT("\r\nsystem will reboot!!!\r\n");

        while (1);    //看门狗复位
    }
    
#ifdef DOUBLE_IP_FUN    
    //----------------------------------------------------
    //设置双IP，主用IP+备用IP 
    //(866104020094747AP1311502919203804111122233308831)
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP13", 4) == TRUE)
    {
        uint8_t IP[4],BKIP[4];
        uint16_t      Port;

        for (i = 0; i < 4; i ++)
        {
            IP[i] = CharToValue(pMsgBuffer[20 + i * 3]);
            IP[i] = IP[i] * 10 + CharToValue(pMsgBuffer[21 + i * 3]);
            IP[i] = IP[i] * 10 + CharToValue(pMsgBuffer[22 + i * 3]);
        }
        for (i = 0; i < 4; i ++)
        {
            BKIP[i] = CharToValue(pMsgBuffer[32 + i * 3]);
            BKIP[i] = BKIP[i] * 10 + CharToValue(pMsgBuffer[33 + i * 3]);
            BKIP[i] = BKIP[i] * 10 + CharToValue(pMsgBuffer[34 + i * 3]);
        }
        
        Port = 0;
        for (i = 0; i < 5; i ++)
        {
            Port = Port * 10 + CharToValue(pMsgBuffer[44 + i]);
        }
        
        SendMessageStr("BP13");
        //保存IP地址和端口
        SaveDoubleIpAndPort(IP, BKIP,Port);

        DEBUG_UART("\r\nsystem will reboot!!!");
        while (1);
    }
#endif
    
    //----------------------------------------------------
    //设置APN
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP04", 4) == TRUE)
    {
        i = 0;
        while (pMsgBuffer[COMMAND_OFFSET_BIT + i] != ')')
        {
           i ++;
        }
        SaveApn(i, pMsgBuffer + COMMAND_OFFSET_BIT);
        SendMessageStr("BP03");
        while (1); //重启
    }

    //----------------------------------------------------
    //读取终端版本
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP07", 4) == TRUE)
    {
        PlatformRcvDataBuffer[0] = 'B';
        PlatformRcvDataBuffer[1] = 'P';
        PlatformRcvDataBuffer[2] = '0';
        PlatformRcvDataBuffer[3] = '1';
        for (i = 0; i < 17; i ++)
        {
            PlatformRcvDataBuffer[4 + i] = app_Version[i];
        }
        PlatformRcvDataBuffer[21] = ',';
        for (i = 0; i < 11; i ++)
        {
            PlatformRcvDataBuffer[22 + i] = Date[i];
        }
        PlatformRcvDataBuffer[33] = 0;

        SendMessageStr(PlatformRcvDataBuffer);
        return ;
    }
    
    //----------------------------------------------------
    //读取控制器版本(863412040166780BP08XR705_EC20_V0.0.3,JBDM60VB_V0_8)
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP08", 4) == TRUE)
    {
        #ifdef ENABLE_CTRL_COMM 
        CtrlCommData.ReadCtrlSoftVer = FALSE;
        if ((gCarFunctionState & FUNC_START_STOP_BIT) == 0) 
        {
            PlatformData_HY.QuerySoftVerFlag = TRUE;
            Disable_Phase_INT(); //失能轮动检测，否则会误报轮动报警
            OpenCenterCtrlPin();
            delay_ms(1000);
        }
 
        SendDataToCtrl(QueryCtrVer,8); 
        delay_ms(1000);
        #endif
        
        ucPos = 0;
        PlatformRcvDataBuffer[ucPos++] = 'B';
        PlatformRcvDataBuffer[ucPos++] = 'P';
        PlatformRcvDataBuffer[ucPos++] = '0';
        PlatformRcvDataBuffer[ucPos++] = '8';
        for (i = 0; i < 17; i ++)
        {
            PlatformRcvDataBuffer[ucPos++] = app_Version[i];
        }
        PlatformRcvDataBuffer[ucPos++] = ',';
        
        /*for (i = 0; i < 17; i ++)
        {
            #ifdef ENABLE_CTRL_COMM 
            if(CtrlCommData.SoftVer[i] == '\0')
                break;
            PlatformRcvDataBuffer[ucPos++] = CtrlCommData.SoftVer[i];
            #else
            PlatformRcvDataBuffer[ucPos++] = '0';
            #endif
        }*/
        
        PlatformRcvDataBuffer[ucPos++] = ValueToChar(EbikeData.SoftVer/100);
        PlatformRcvDataBuffer[ucPos++] = ValueToChar(EbikeData.SoftVer%100/10);
        PlatformRcvDataBuffer[ucPos++] = ValueToChar(EbikeData.SoftVer%10);
        
        PlatformRcvDataBuffer[ucPos++] = 0;

        SendMessageStr(PlatformRcvDataBuffer);
        return ;
    }
    
    //----------------------------------------------------
    //读取租赁时间
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AP15", 4) == TRUE)
    {
        //发送NFC租赁消息
        //DEBUG_PRINT("\r\nSend NFC BP15 msg.\r\n");
        
        SendMessageBP15();

        return ;
    }
    
    //------------------------------------------------------------
    //取消所有报警消息处理
    if (CompareStr(pMsgBuffer + 13 + PLATFORM_OFFSET_BIT, "AV02", 4) == TRUE)
    {
        for (i = 0; i < MAX_ALERT_SUM; i ++)
        {
             PlatformData_HY.AlertType[i].AlertFlag = FALSE;
        }
        
        //取消超速报警
        GpsFunctionState &= ~FUNC_OVERSPEED_ALERT_BIT;

        SendMessageStr("BS21");
        return ;
    }
    
    //------------------------------------------------------------
    //恢复出厂设置消息处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX00", 4) == TRUE)
    {
        SendMessageStr("BS00");
        SystemParamFactorySettingsProc();
        while (1); //看门狗复位
    }
    
    //------------------------------------------------------------
    //里程清零消息处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX01", 4) == TRUE)
    {
        ClearMeter();
        SendMessageStr("BS04");
        return ;
    }

    /*#ifdef FTP_UPGRAGE  
    //FTP远程升级  平台下发命令格式 (866104020098038AX02183.13.182.164:21)
    //------------------------------------------------------------
    //启动升级消息处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX02", 4) == TRUE)
    {
        uint8_t i,n, ucPos;
        uint16_t uiTmp;

        ucPos = 20;
        //获取服务器IP地址
        for (n = 0; n < 3; n ++)
        {
            uiTmp = 0;
            i = 0;
            //找'.'
            while (CompareStr(pMsgBuffer + ucPos, ".", 1) != TRUE)
            {
                if (IsNumber(pMsgBuffer + ucPos, 1) == FALSE)
                {
                    #ifdef DEBUG_OUTPUT0
                    printf("IP address error1.");
                    #endif
                    return;
                }
                uiTmp = uiTmp * 10 + CharToValue(pMsgBuffer[ucPos]);
                ucPos ++;
                i ++;
                if (i > 3)
                {
                    #ifdef DEBUG_OUTPUT0
                    printf("IP address error2.");
                    #endif
                    return;
                }
            }
            if (uiTmp > 255)
            {
                #ifdef DEBUG_OUTPUT0
                printf("IP address error3.");
                #endif
                return;
            }
            AtFTPGET.FtpUpgradeAddr[n] = uiTmp;
            ucPos ++;   //'.'
        }

        uiTmp = 0;
        i = 0;
        //找不是数字
        while (IsNumber(pMsgBuffer + ucPos, 1) == TRUE)
        {
            uiTmp = uiTmp * 10 + CharToValue(pMsgBuffer[ucPos]);
            ucPos ++;
            i ++;
            if (i > 3)
            {
                #ifdef DEBUG_OUTPUT0
                printf("IP address error4.");
                #endif
                return;
            }
        }
        if (uiTmp > 255)
        {
            #ifdef DEBUG_OUTPUT0
            printf("IP address error5.");
            #endif
            return;
        }
        AtFTPGET.FtpUpgradeAddr[3] = uiTmp;

        if (CompareStr(pMsgBuffer + ucPos, ":", 1) != TRUE)
        {
            #ifdef DEBUG_OUTPUT0
            printf("IP address error6.");
            #endif
            return;
        }
        ucPos += 1;  //跳过 ":"

        //获取服务器端口
        uiTmp = 0;
        i = 0;
        while ((CompareStr(pMsgBuffer + ucPos, ")", 1) != TRUE)
               && (CompareStr(pMsgBuffer + ucPos, "\x00", 1) != TRUE))
        {
            if (IsNumber(pMsgBuffer + ucPos, 1) == FALSE)
            {
                #ifdef DEBUG_OUTPUT0
                printf("Port not digital!");
                #endif
                return;
            }
            uiTmp = uiTmp * 10 + CharToValue(pMsgBuffer[ucPos]);
            ucPos ++;
            i ++;
            if (i > 5)
            {
                #ifdef DEBUG_OUTPUT0
                printf("Port error!");
                #endif
                return;
            }
        }

        AtFTPGET.FtpUpgradePort = uiTmp;

        #ifdef DEBUG_OUTPUT
        DEBUG_PRINT("\r\n==FTP IP:%d.%d.%d.%d Port:%d",AtFTPGET.FtpUpgradeAddr[0],
        AtFTPGET.FtpUpgradeAddr[1],AtFTPGET.FtpUpgradeAddr[2],
        AtFTPGET.FtpUpgradeAddr[3],AtFTPGET.FtpUpgradePort);
        #endif

        SendMessageStr("BS05");  //应答平台进入升级

        SaveSumMeter();  //升级前保存参数
            
        FtpUpFlag = TRUE;
        FtpDownLoadTimeOutCnt = FIVE_MINUTE; //超时5分钟，退出升级
        CloseSystemLed();
        CloseGpsSignalLed();
        
        FTP_Upgrade_Proc();  //进入FTP升级
    }
    //#endif  //#ifdef FTP_UPGRAGE*/

    //------------------------------------------------------------
    //里程初始化消息处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX03", 4) == TRUE)
    {
        ClearMeter();
        InitSumMeter(pMsgBuffer + COMMAND_OFFSET_BIT);
        SendMessageStr("BS06");
		return ;
    }
    
    //------------------------------------------------------------
    //平台查询遥控器ID处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX04", 4) == TRUE)
    {
        SendMessageBU04(DeviceData.KeyAddress);
		return ;
    }

#ifdef USE_RF_REMOTE    
    //------------------------------------------------------------
    //平台设置遥控器ID处理
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX06", 4) == TRUE)
    {
        #ifdef RCV_DATA_1527_MSB
        uint16_t remAdr = 0;
        #else   
        uint32_t remAdr = 0;
        #endif
        
        //判断数据是否正确
        if (IsHexNumber(pMsgBuffer + COMMAND_OFFSET_BIT, 4) == FALSE)
        {
            SendMessageStr("BU060"); //错误，非16进制数据
            return;
        }
        
        #ifdef RCV_DATA_1527_MSB
        remAdr = (uint16_t)ChangeRemCtrID(pMsgBuffer + COMMAND_OFFSET_BIT,4);
        #else
        remAdr = ChangeRemCtrID(pMsgBuffer + COMMAND_OFFSET_BIT,5);
        #endif
        
        #ifdef RCV_DATA_1527_MSB
        DEBUG_PRINT("\r\nREM_ADDRESS:%04X",remAdr);    
        #else   
        DEBUG_PRINT("\r\nREM_ADDRESS:%05X",remAdr);
        #endif
        
        RemKeyData.KeyAddress = remAdr;
        
        SendMessageStr("BU061");
        
        SaveRemCtrState();  //保存新遥控器ID
		return ;
    }
#endif

    //----------------------------------------------------
    //设置NFC id  (868626042198930AX0904000789A8BA) 
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX09", 4) == TRUE)
    {
        uint8_t tmpBuf[10];
        uint8_t m;
        
        i = 0;
        while (pMsgBuffer[COMMAND_OFFSET_BIT + i] != ')')
        {
           i ++;
        }
        
        m = 0;
        for(j = 0; j < (i/2); j++)
        {
            tmpBuf[j] = (CharToValue(pMsgBuffer[COMMAND_OFFSET_BIT + m]) << 4) 
                       + CharToValue(pMsgBuffer[COMMAND_OFFSET_BIT + m + 1]);
            
            m += 2;
        }
        
        DEBUG_PRINT("NFC id:");
        for(i = 0; i < j; i++)
           DEBUG_UART("%02X ",tmpBuf[i]);
        DEBUG_PRINT("\r\n");
        
        SaveNFC_id(tmpBuf);
        SendMessageStr("BP04");
    }
    
    //----------------------------------------------------
    //设置租车日期  (868626042198930AX10210701,210731) 
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX10", 4) == TRUE)
    {
        uint8_t  _year_s=0, _year_e=0, _mon_s=0, _day_s=0, _mon_e=0, _day_e=0;
        
        _year_s = (CharToValue(pMsgBuffer[20]) * 10) + CharToValue(pMsgBuffer[21]);
        _mon_s  = (CharToValue(pMsgBuffer[22]) * 10) + CharToValue(pMsgBuffer[23]);
        _day_s  = (CharToValue(pMsgBuffer[24]) * 10) + CharToValue(pMsgBuffer[25]);
        
        ucPos = 25;
        while (pMsgBuffer[ucPos++] != ',') //查找时间后面的逗号
        {
            if (ucPos > 31)
            {
                return;
            }
        }
        
        _year_e = (CharToValue(pMsgBuffer[27]) * 10) + CharToValue(pMsgBuffer[28]);
        _mon_e  = (CharToValue(pMsgBuffer[29]) * 10) + CharToValue(pMsgBuffer[30]);
        _day_e  = (CharToValue(pMsgBuffer[31]) * 10) + CharToValue(pMsgBuffer[32]);
        
        if((_year_s > 20) && (_year_s < 99) &&
           (_year_e > 20) && (_year_e < 99) &&
           (_mon_s >= 01) && (_mon_s <= 12) &&
           (_mon_e >= 01) && (_mon_e <= 12) &&
           (_day_s >= 01) && (_day_s <= 31) &&
           (_day_e >= 01) && (_day_e <= 31))
        {
            DeviceInfo.LeaseStartTime.Year = _year_s + 2000;
            DeviceInfo.LeaseStartTime.Mon  = _mon_s;
            DeviceInfo.LeaseStartTime.Day  = _day_s;
            DeviceInfo.LeaseEndTime.Year = _year_e + 2000;
            DeviceInfo.LeaseEndTime.Mon  = _mon_e;
            DeviceInfo.LeaseEndTime.Day  = _day_e;
            
            gCarFunctionState &= ~FUNC_OPEN_CLOSE_REM_BIT; //收到租车，默认开蓝牙 
            DeviceData.CarFunctionState = gCarFunctionState;
            
            SaveLeaseTime();
            SendMessageStr("BP09");
        }
    }
    
    #ifdef FTP_UPGRAGE
    //(861510030185611AX07;1;CMNET;121.196.226.32;21;ftpdown;1234;M100_SIM868_Offset0x3000_V1_0_7.bin;FTP/XR701/;1;1;6B3461D24A9DED66D816AFE061AF534C;05;)  //146byte
    if (CompareStr(pMsgBuffer + PLATFORM_OFFSET_BIT + id, "AX07", 4) == TRUE)
    {
        OS_CRITICAL_ENTER();
        
        j = 0;
        nID = 0;
        nCount = 0;
        while(nID<=11)
        {
            if(pMsgBuffer[21+nCount++] == ';')
            {           
                if(nID == 0)         //FTP升级模式，1:给自已MCU升级，2:给控制器升级
                {
                    if(j<(nCount-1))
                    {
                         EEP_UpdateConveyMessage.UpdateMode = CharToValue(pMsgBuffer[21+j]);
                         if(EEP_UpdateConveyMessage.UpdateMode > 0x02)
                            EEP_UpdateConveyMessage.UpdateMode = 0x01;

                         DEBUG_PRINT("\r\n========================================================");
                         DEBUG_PRINT("\r\nUpdateMode=%d,nCount=%d,j=%d",EEP_UpdateConveyMessage.UpdateMode,nCount,j);
                    }
                }
                else if(nID == 1)    //FTP升级APN
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;((i<MAX_GPRS_APN_LEN) && ((j+i)<(nCount-1))); i++) 
                            EEP_UpdateConveyMessage.UpdateApn[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdateApn[i] = '\0';
                        
                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdateApn=%s,nCount=%d,j=%d",EEP_UpdateConveyMessage.UpdateApn,nCount,j);
                        #endif
                    }
                }
                else if(nID == 2)    //FTP升级IP
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;((i<SERVER_IP_LEN) && ((j+i)<(nCount-1))); i++)     
                            EEP_UpdateConveyMessage.UpdateIP[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdateIP[i] = '\0';
                        
                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdateIP=%s,nCount=%d,j=%d",EEP_UpdateConveyMessage.UpdateIP,nCount,j);
                        #endif
                    }
                }
                else if(nID == 3)    //FTP升级端口
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;(i<6 && ((j+i)<(nCount-1)));i++)
                            EEP_UpdateConveyMessage.UpdatePort[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdatePort[i] = '\0';
                        
                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdatePort=%s,nCount=%d,j=%d",EEP_UpdateConveyMessage.UpdatePort,nCount,j);
                        #endif
                    }
                }
                else if(nID == 4)   //FTP升级登入用户名
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;((i<15) && ((j+i)<(nCount-1))); i++)        
                            EEP_UpdateConveyMessage.UpdateUserNema[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdateUserNema[i] = '\0';
                        
                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdateUserNema=%s",EEP_UpdateConveyMessage.UpdateUserNema);
                        #endif
                    }
                }
                else if(nID == 5)  //FTP升级登入密码
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;(i<15 && ((j+i)<(nCount-1)));i++)
                            EEP_UpdateConveyMessage.UpdatePassword[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdatePassword[i] = '\0';
                        
                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdatePassword=%s",EEP_UpdateConveyMessage.UpdatePassword);
                        #endif
                    }
                }
                else if(nID == 6)  //FTP升级文件名(版本)
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;((i<40) && ((j+i)<(nCount-1)));i++)
                            EEP_UpdateConveyMessage.UpdateFileNema[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdateFileNema[i] =  '\0';

                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdateFileNema=%s",EEP_UpdateConveyMessage.UpdateFileNema);
                        #endif
                    }
                }
                else if(nID == 7)  //FTP升级路径
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;(i<sizeof(EEP_UpdateConveyMessage.UpdateFilePath) && ((j+i)<(nCount-1)));i++)
                            EEP_UpdateConveyMessage.UpdateFilePath[i] = pMsgBuffer[21+j+i];
                        EEP_UpdateConveyMessage.UpdateFilePath[i] = '\0';

                        #ifdef DEBUG_OUTPUT0
                        DEBUG_PRINT("\r\nUpdateFilePath=%s",EEP_UpdateConveyMessage.UpdateFilePath);
                        #endif
                    }
                }
                else if(nID == 8)  //FTP升级后是否恢复出厂设置
                {
                    if(j<(nCount-1))
                    {
                        EEP_UpdateConveyMessage.UpdateFactoryResetFlag = CharToValue(pMsgBuffer[21+j]);
                        EEP_UpdateConveyMessage.UpdateFactoryResetFlag = 0; //强制为0，即不需要恢复出厂设置 
                        
                        #ifdef DEBUG_OUTPUT
                        DEBUG_PRINT("\r\nUpdateFactoryResetFlag=%d",EEP_UpdateConveyMessage.UpdateFactoryResetFlag);
                        #endif
                    }
                }
                else if(nID == 9)  //FTP升级是否启用MD5校验串
                {
                    if(j<(nCount-1))
                    {
                        if(pMsgBuffer[21+j] == '0')
                            EEP_UpdateConveyMessage.UpdateFileMD5CheckEnableFlag = 0x00;
                        else if(pMsgBuffer[21+j] == '1')
                            EEP_UpdateConveyMessage.UpdateFileMD5CheckEnableFlag = 0x01;    
                    }                               
                }
                else if(nID == 10) //FTP升级MD5校验串
                {
                    if(j<(nCount-1))
                    {
                        for(i=0;(i<(32/2) && ((j+i*2)<(nCount-1)));i++)
                                EEP_UpdateConveyMessage.UpdateFileMD5Check[i] = (CharToValue(pMsgBuffer[21+j+i*2]) << 4) + CharToValue(pMsgBuffer[21+j+i*2+1]);
                        EEP_UpdateConveyMessage.UpdateFileMD5Check[i] = '\0';

                        DEBUG_PRINT("\r\nMD5：");
                        for(i=0;i<16;i++)
                            DEBUG_PRINT("%02X",EEP_UpdateConveyMessage.UpdateFileMD5Check[i]);
                    }
                }
                else if(nID ==11)  //FTP升级超时值
                {
                    if(j<(nCount-1))
                    {
                        dwTemp = 0;
                        for(i=0;(i<5 && ((j+i)<(nCount-1)));i++)
                            dwTemp = dwTemp*10 + CharToValue(pMsgBuffer[21+j+i]);
                        //EEP_UpdateConveyMessage.UpdateTimeOut = dwTemp;
                        if(dwTemp == 0)  //如果平台设置为无限超时，终端改死10分钟//
                            EEP_UpdateConveyMessage.UpdateTimeOut = 3;
                        else
                            EEP_UpdateConveyMessage.UpdateTimeOut = dwTemp;

                        DEBUG_PRINT("\r\nUpdateTimeOut=%d",EEP_UpdateConveyMessage.UpdateTimeOut);
                        DEBUG_PRINT("\r\n========================================================");
                    }
                }                       
                nID++;
                j = nCount;
            }
        }  
        
        OS_CRITICAL_EXIT();
        
        SendMessageStr("BS05");  //应答平台进入升级
        
        //SaveSumMeter();  
        
        WriteUpdateConveyMessage_EEPROM(); //升级前保存参数
        
        FtpUpFlag = TRUE;
        OS_CRITICAL_ENTER();
        FtpDownLoadTimeOutCnt = EEP_UpdateConveyMessage.UpdateTimeOut*6000; //超时5分钟，退出升级   
        OS_CRITICAL_EXIT();

        CloseSystemLed();
        CloseGpsSignalLed();
        
        FTP_Upgrade_Proc();     //进入FTP升级     
    }
    #endif  //#ifdef FTP_UPGRAGE
}

//平台接收数据包处理
void PlatformRcvPacketProc(void)
{
    if(PlatformRcvDataPtr<14)   //平台最小字节数15
    {
        DEBUG_PRINT("\r\nPlatformRcvDataPtr < 14");

        return;
    }
    
    if( (PlatformRcvDataBuffer[0] != '(') || (PlatformRcvDataBuffer[PlatformRcvDataPtr] != ')'))
    {
        DEBUG_PRINT("\r\nPlatform data format  error!");

        return;
    }
    
    #ifdef DEBUG_OUTPUT0           
    if(PlatformRcvDataPtr < 40)
    {
        DEBUG_PRINT("\r\nRcv Data:%s",PlatformRcvDataBuffer);
    }
    #endif

    #ifdef TEST_OUTPUT 
    if(PlatformRcvDataPtr < 40)
    {
        Test_Receive_Tcp_Data();
    }
    #endif
    
    PlatformMessageProc(PlatformRcvDataBuffer);
    memset(PlatformRcvDataBuffer,0,sizeof(PlatformRcvDataBuffer)); 
    PlatformRcvDataPtr = 0;
}

unsigned char SendGpsPacket(void)
{
    unsigned char SendResult;  
        
    /*if (CheckConnectionStatus() == FALSE)  
    {
        //PlatformData.DeviceRegisterSuccess = FALSE; 
    }*/
    
    if (PlatformData.DeviceRegisterSuccess == TRUE)
    {
        //发送等时连续回传消息
        SendResult = SendMessageBR00();
        
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\n==StructDeviceData=%d",sizeof(StructDeviceData));
        #endif 
    }
    else
    {
        //发送终端注册消息
        #ifdef DEBUG_OUTPUT0
        DEBUG_PRINT("\r\nSend register msg.\r\n");
        #endif
        SendResult = SendMessageBP05();
    }
    
    return SendResult;
    
}

//定时发位置数据到平台
void SendGeoToPlatform_Special(void)
{
    unsigned char SendResult;   
    
    if(DeviceInfo.DeviceCallFlag == TRUE)//检查是否在通话或监听状态，在通话或监听状态不要发数据
    {
        return;
    }
    
    //先保存要发的数据
    #ifdef SPI_FLASH_RECORD  
    SpiFlashWriteGpsToBuffer(&SpiWriteRecordData); 
    #endif
    
    #ifdef STM32RECORD  
    WriteGpsToBuffer(&WriteRecordData);  //保存未发成功的数据到FLASH
    #endif
    
    PlatformData_HY.SendGpsDataType = GPS_DATA_TYPE_NEW;
    
    
    SendResult = SendGpsPacket();
    
    if (SendResult == TRUE)
    {
        PlatformData_HY.ResendPacketFlag = FALSE;
    }
    else
    {
        if ((GpsData.GpsSignalFlag == TRUE) 
            && (GpsData.StopStateFlag == FALSE) 
            && (DeviceInfo.GpsSatelliteTotal > 6)) 
        {
          #ifdef STM32RECORD  
            WriteRecordToMemory(&WriteRecordData);  
            DEBUG_PRINT("\r\nSave Blind data to flash1!");
          #endif
          
          #ifdef SPI_FLASH_RECORD          
            SpiFlashWriteRecordToMemory(&SpiWriteRecordData);  
            DEBUG_PRINT("\r\nSave Blind data to Ext_flash1!");
          #endif            
        }
    } 
}

//**************************************
//发数据到平台
void SendGeoToPlatform(void)
{
    //DeviceInfo.DeviceCommunicationFlag = TRUE;
    
    SendGeoToPlatform_Special();
    
    //DeviceInfo.DeviceCommunicationFlag = FALSE;
}
