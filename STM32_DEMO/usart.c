#define __USART_GLOBALS
#include "usart.h"


#define USART3_Waiting   0
#define USART3_Receiving 1
#define USART3_Success   2 
#define USART3_Failed    3
u8 USART3_state = USART3_Waiting;//接收状态标记	
int8_t USART3_Rx_index=0;       
int8_t USART3_Tx_index=0;
u16 RxLength = 8;
u8  UsartRx_Info[USART_Rx_LEN];
u8  UsartTx_Info[USART_Tx_LEN];



void PID_ParamsSet(MsgsFrame_struct *pidSet);
void PID_ParamsUpload(char motor_ID);
void PID_Regulator_Decode(void);

void USART3_InitConfig(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	/*enabe clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	/*open the alternative function*/
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
	
	/*Configure PD8,PD9 as GPIO_InitStruct1 input*/
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate            = 115200;
	USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits            = USART_StopBits_1;
	USART_InitStruct.USART_Parity              = USART_Parity_No;
	USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStruct);
	
	USART_Cmd(USART3,ENABLE);		
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
//  USART_ITConfig(USART3,USART_IT_TC,ENABLE );
	
	NVIC_InitStruct.NVIC_IRQChannel                    = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
 }


void PID_ParamsSet(MsgsFrame_struct *pidSet)
{
    unsigned char index = pidSet->PID.motor_ID;
    Motor_PID[index].pid_mode = pidSet->PID.PID_Mode;
    Motor_PID[index].p = pidSet->PID.Kp_value.tempFloat;
    Motor_PID[index].i = pidSet->PID.Ki_value.tempFloat;
    Motor_PID[index].d = pidSet->PID.Kd_value.tempFloat;
    
    Motor_PID[index].max_out = pidSet->PID.PID_out_max.tempFloat;
}

#define QUERY_CMD_ID 0X10
#define SET_CMD_ID 0X01
#define UPLOAD_PARAMS_CMD_ID 0x10
#define DATA_LENGTH_UPLOAD_PID_PATAMS 30
void PID_ParamsUpload(char motor_ID)
{
//    if ( (PID_Regulator_Msg.cmd_ID == QUERY_CMD_ID) || (PID_Regulator_Msg.cmd_ID == SET_CMD_ID))
//    {
//        
//    }  
    unsigned char i=0,j=0;
    static unsigned char seq = 0;
    u8 usart_tx[200];
    u8 crc8;
    uint16_t crc16;
    MsgsFrame_struct tempMsgs;
    
    tempMsgs.SOF = 0xA5;
    tempMsgs.Data_Length = DATA_LENGTH_UPLOAD_PID_PATAMS;
    tempMsgs.seq = seq;
    seq++;
    //tempMsgs.crc8 = 0;//
    tempMsgs.cmd_ID = UPLOAD_PARAMS_CMD_ID;
    tempMsgs.PID.motor_ID = motor_ID;
    tempMsgs.PID.Kp_value.tempFloat = Motor_PID[motor_ID].p;
    tempMsgs.PID.Ki_value.tempFloat = Motor_PID[motor_ID].i;
    tempMsgs.PID.Kd_value.tempFloat = Motor_PID[motor_ID].d;
    tempMsgs.PID.P_out_max.tempFloat = 111;//TODO
    tempMsgs.PID.I_out_max.tempFloat = 111;//TODO
    tempMsgs.PID.D_out_max.tempFloat = 111;//TODO
    tempMsgs.PID.PID_out_max.tempFloat = Motor_PID[motor_ID].max_out;
    
    usart_tx[i] = tempMsgs.SOF;
    usart_tx[i++] = tempMsgs.Data_Length;
    usart_tx[i++] = tempMsgs.seq;
    
    crc8 = Get_CRC8_Check_Sum(usart_tx,i,0xff);
    usart_tx[i++] = crc8;
    usart_tx[i++] = tempMsgs.cmd_ID;
    usart_tx[i++] = tempMsgs.PID.motor_ID;
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.Kp_value.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.Ki_value.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.Kd_value.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.P_out_max.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.I_out_max.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.D_out_max.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.PID_out_max.tempChar[j];
    
    crc16 = Get_CRC16_Check_Sum(usart_tx,i,0xff);
    usart_tx[i++] = crc16&0x00ff;
    usart_tx[i++] = (crc16>>8)&0x00ff;    
    
    for (j=0;j<i;j++)
    {
        while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
        USART_SendData(USART3, usart_tx[j]);
    }    
}

void PID_Regulator_Decode(void)
{
    int i = 0,j=0;    
    PID_Regulator_Msg.SOF = UsartRx_Info[i];
    PID_Regulator_Msg.Data_Length = UsartRx_Info[i++];
    PID_Regulator_Msg.seq = UsartRx_Info[i++];
    PID_Regulator_Msg.crc8 = UsartRx_Info[i++];
    PID_Regulator_Msg.cmd_ID = UsartRx_Info[i++];
    if ( PID_Regulator_Msg.cmd_ID == 0x00)//上位机->下位机，查询板载PID参数
    {
        PID_Regulator_Msg.PID.motor_ID = UsartRx_Info[i++];
        
        for(j=0;j<2;j++) PID_Regulator_Msg.crc16.tempChar[j] = UsartRx_Info[i++];        
    }
    else if ( PID_Regulator_Msg.cmd_ID == 0x01)//上位机->下位机，PID参数设置
    {
        PID_Regulator_Msg.PID.motor_ID = UsartRx_Info[i++];
        PID_Regulator_Msg.PID.PID_Mode = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.Kp_value.tempChar[j] = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.Ki_value.tempChar[j] = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.Kd_value.tempChar[j] = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.P_out_max.tempChar[j] = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.I_out_max.tempChar[j] = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.D_out_max.tempChar[j] = UsartRx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.PID_out_max.tempChar[j] = UsartRx_Info[i++];
        
        for(j=0;j<2;j++) PID_Regulator_Msg.crc16.tempChar[j] = UsartRx_Info[i++];
        
        //参数赋值、存储操作
        PID_ParamsSet(&PID_Regulator_Msg);        
    }    
}
//串口接收上位机信息
void USART3_IRQHandler(void)    
{
    u8 Res = 0;
    if(USART_GetITStatus(USART3, USART_IT_RXNE|USART_IT_ORE_RX) != RESET)  //接收中断
	{
      Res =USART_ReceiveData(USART3);//	//读取接收到的数据
      if( (USART3_state == USART3_Waiting)&&(Res == 0xA5) ) 
      {
          USART3_state = USART3_Receiving;
          USART3_Rx_index = 0;                                      //接收数组的计数器
          UsartRx_Info[USART3_Rx_index] = Res;
          USART3_Rx_index++;
      }
      else if( USART3_state == USART3_Receiving )
      {
          if( USART3_Rx_index == 1 )//Data Length
          {
              UsartRx_Info[USART3_Rx_index] = Res;
              USART3_Rx_index++;
              RxLength = Res;
          }
          else if( USART3_Rx_index == 2 )//Seq
          {
              UsartRx_Info[USART3_Rx_index] = Res;
              USART3_Rx_index++;
          }
          else if( USART3_Rx_index == 3 )
          {
              if (  Verify_CRC8_Check_Sum(UsartRx_Info,4)) ;	
              else         //CRC校验错误，重新等待读取
              {	
                   RxLength=8 ;
                   USART3_Rx_index = 0;
                   USART3_state = USART3_Waiting;	
              }
          }
          else if (USART3_Rx_index < RxLength)
          { 				
              UsartRx_Info[USART3_Rx_index]=Res ;
              USART3_Rx_index++;
              if (USART3_Rx_index == RxLength) //接收完成 ，结束了
              {               
                  //首先进行crc校验应该                   
                  if (Verify_CRC16_Check_Sum(UsartRx_Info, RxLength ))   PID_Regulator_Decode();
                  RxLength=8 ;
                  USART3_Rx_index = 0;
                  USART3_state = USART3_Waiting;	                		   
              }
          }
          
      }
      USART_ClearFlag(USART3,USART_IT_RXNE|USART_IT_ORE_RX);
    }     
    PID_ParamsUpload(PID_Regulator_Msg.PID.motor_ID);
}
