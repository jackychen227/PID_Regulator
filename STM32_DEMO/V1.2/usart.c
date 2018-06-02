#define  __USART6_GLOBALS

#include "usart.h"

#define USART6_Waiting   0
#define USART6_Receiving 1
#define USART6_Success   2 
#define USART6_Failed    3

u8 USART6_state = USART6_Waiting;//接收状态标记	
u8 USART6_Rx_index=0;       
u8 USART6_Tx_index=0;
unsigned char Usart6Rx_Info[USART_Rx_LEN];
unsigned char Usart6Tx_Info[USART_Tx_LEN];

/**
  * @brief  重定义fputc函数（禁用半主机模式）
  * @param  
  * @retval 
  */

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
// 发送数据
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART6,USART_FLAG_TC) == RESET);
	USART_SendData(USART6, (uint8_t)ch);
	return (ch);
}
#endif

/**
  * @brief  初始化USART6
  * @param  void
  * @retval void
  * @notes  USART6_TX-->PG14      USART6_RX-->PG9---->YOLO信息接收        USART6_TX---->串口打印
  */
void USART6_InitConfig(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	/*enabe clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	/*open the alternative function*/
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	
	/*Configure PB10,PB11 as GPIO_InitStruct1 input*/
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate            = 115200;
	USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits            = USART_StopBits_1;
	USART_InitStruct.USART_Parity              = USART_Parity_No;
	USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6,&USART_InitStruct);
	
	USART_Cmd(USART6,ENABLE);		
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel                    = USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
}

u16 rxLength = 8;
#define QUERY_CMD_ID 0X10
#define SET_CMD_ID 0X01
#define UPLOAD_PARAMS_CMD_ID 0x10
#define DATA_LENGTH_UPLOAD_PID_PATAMS 30
void PID_ParamsSet(MsgsFrame_struct *pidSet);
void PID_ParamsUpload(char motor_ID);
void PID_Regulator_Decode(void);

void PID_ParamsSet(MsgsFrame_struct *pidSet)
{
    unsigned char index = pidSet->PID.motor_ID;
    Motor_PID[index].pid_mode = pidSet->PID.PID_Mode;
    Motor_PID[index].p = pidSet->PID.Kp_value.tempFloat;
    Motor_PID[index].i = pidSet->PID.Ki_value.tempFloat;
    Motor_PID[index].d = pidSet->PID.Kd_value.tempFloat;
    
    Motor_PID[index].P_out_max = pidSet->PID.P_out_max.tempFloat;
    Motor_PID[index].I_out_max = pidSet->PID.I_out_max.tempFloat;
    Motor_PID[index].D_out_max = pidSet->PID.D_out_max.tempFloat;
    Motor_PID[index].PID_out_max = pidSet->PID.PID_out_max.tempFloat;
    
    Motor_PID[index].max_out = pidSet->PID.PID_out_max.tempFloat;
}

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
    tempMsgs.PID.PID_Mode = Motor_PID[motor_ID].pid_mode;
    tempMsgs.PID.Kp_value.tempFloat = Motor_PID[motor_ID].p;
    tempMsgs.PID.Ki_value.tempFloat = Motor_PID[motor_ID].i;
    tempMsgs.PID.Kd_value.tempFloat = Motor_PID[motor_ID].d;
    tempMsgs.PID.P_out_max.tempFloat = Motor_PID[motor_ID].P_out_max;//TODO
    tempMsgs.PID.I_out_max.tempFloat = Motor_PID[motor_ID].I_out_max;//TODO
    tempMsgs.PID.D_out_max.tempFloat = Motor_PID[motor_ID].D_out_max;//TODO
    tempMsgs.PID.PID_out_max.tempFloat = Motor_PID[motor_ID].PID_out_max;
    tempMsgs.PID.PID_out_max.tempFloat = Motor_PID[motor_ID].max_out;
    
    usart_tx[i++] = tempMsgs.SOF;
    usart_tx[i++] = tempMsgs.Data_Length;
    usart_tx[i++] = tempMsgs.seq;
    
    crc8 = Get_CRC8_Check_Sum(usart_tx,i,0xff);
    usart_tx[i++] = crc8;
    usart_tx[i++] = tempMsgs.cmd_ID;
    usart_tx[i++] = tempMsgs.PID.motor_ID;
    usart_tx[i++] = tempMsgs.PID.PID_Mode;
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.Kp_value.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.Ki_value.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.Kd_value.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.P_out_max.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.I_out_max.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.D_out_max.tempChar[j];
    for(j=0;j<4;j++) usart_tx[i++] = tempMsgs.PID.PID_out_max.tempChar[j];
    
    crc16 = Get_CRC16_Check_Sum(usart_tx,i,0xffff);
    usart_tx[i++] = crc16&0xff;
    usart_tx[i++] = (crc16>>8)&0xff;    
    
    for (j=0;j<i;j++)
    {
        while (USART_GetFlagStatus(USART6,USART_FLAG_TC) == RESET);
        USART_SendData(USART6, usart_tx[j]);
    }    
}

void PID_Regulator_Decode(void)
{
    int i = 0,j=0;    
    PID_Regulator_Msg.SOF = Usart6Rx_Info[i++];
    PID_Regulator_Msg.Data_Length = Usart6Rx_Info[i++];
    PID_Regulator_Msg.seq = Usart6Rx_Info[i++];
    PID_Regulator_Msg.crc8 = Usart6Rx_Info[i++];
    PID_Regulator_Msg.cmd_ID = Usart6Rx_Info[i++];
    if ( PID_Regulator_Msg.cmd_ID == 0x00)//上位机->下位机，查询板载PID参数
    {
        PID_Regulator_Msg.PID.motor_ID = Usart6Rx_Info[i++];
        
        for(j=0;j<2;j++) PID_Regulator_Msg.crc16.tempChar[j] = Usart6Rx_Info[i++];        
    }
    else if ( PID_Regulator_Msg.cmd_ID == 0x01)//上位机->下位机，PID参数设置
    {
        PID_Regulator_Msg.PID.motor_ID = Usart6Rx_Info[i++];
        PID_Regulator_Msg.PID.PID_Mode = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.Kp_value.tempChar[j] = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.Ki_value.tempChar[j] = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.Kd_value.tempChar[j] = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.P_out_max.tempChar[j] = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.I_out_max.tempChar[j] = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.D_out_max.tempChar[j] = Usart6Rx_Info[i++];
        for(j=0;j<4;j++) PID_Regulator_Msg.PID.PID_out_max.tempChar[j] = Usart6Rx_Info[i++];
        
        for(j=0;j<2;j++) PID_Regulator_Msg.crc16.tempChar[j] = Usart6Rx_Info[i++];
        
        //参数赋值、存储操作
        PID_ParamsSet(&PID_Regulator_Msg);        
    }    
}

static uint8_t upload_flag=0;
//串口接收上位机信息
void USART6_IRQHandler(void)    
{
    u8 Res = 0;
    if(USART_GetITStatus(USART6, USART_IT_RXNE|USART_IT_ORE_RX) != RESET)  //接收中断
	{
      Res =USART_ReceiveData(USART6);//	//读取接收到的数据
      if( (USART6_state == USART6_Waiting)&&(Res == 0xA5) ) 
      {
          USART6_state = USART6_Receiving;
          USART6_Rx_index = 0;                                      //接收数组的计数器
          Usart6Rx_Info[USART6_Rx_index] = Res;
          USART6_Rx_index++;
      }
      else if( USART6_state == USART6_Receiving )
      {
          if( USART6_Rx_index == 1 )//Data Length
          {
              Usart6Rx_Info[USART6_Rx_index] = Res;
              USART6_Rx_index++;
              rxLength = Res+7;
          }
          else if( USART6_Rx_index == 2 )//Seq
          {
              Usart6Rx_Info[USART6_Rx_index] = Res;
              USART6_Rx_index++;
          }
          else if( USART6_Rx_index == 3 )
          {
              Usart6Rx_Info[USART6_Rx_index] = Res;
              USART6_Rx_index++;
              if (  Verify_CRC8_Check_Sum(Usart6Rx_Info,4)) 
              {
                NULL ;
              }                  
              else         //CRC校验错误，重新等待读取
              {	
                   rxLength=8 ;
                   USART6_Rx_index = 0;
                   USART6_state = USART6_Waiting;	
              }
          }
          else if (USART6_Rx_index < rxLength)
          { 				
              Usart6Rx_Info[USART6_Rx_index]=Res ;
              USART6_Rx_index++;
              if (USART6_Rx_index == rxLength) //接收完成 ，结束了
              {               
                  //首先进行crc校验应该                   
                  if (Verify_CRC16_Check_Sum(Usart6Rx_Info, rxLength ))  
                  {
                      PID_Regulator_Decode();
                      upload_flag = 1;
                  }                
                  rxLength=8 ;
                  USART6_Rx_index = 0;
                  USART6_state = USART6_Waiting;	                		   
              }
          }          
      }
      USART_ClearFlag(USART6,USART_IT_RXNE|USART_IT_ORE_RX);
    }  
    
    if(upload_flag)
    {
        PID_ParamsUpload(PID_Regulator_Msg.PID.motor_ID);
        upload_flag=0;
    }
}
