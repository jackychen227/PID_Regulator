#ifndef __USART6_H
#define __USART6_H
#include "stm32f4xx.h"

#define USART_Rx_LEN  			200  	//定义最大接收字节数 200
#define USART_Tx_LEN        200

#ifdef  __USART6_GLOBALS
#define __USART6_EXT
#else
#define __USART6_EXT  extern
#endif

typedef struct{
    unsigned char motor_ID;
    unsigned char PID_Mode;//增量式：1；位置式：0
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Kp_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Ki_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Kd_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }P_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }I_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }D_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }PID_out_max;
}PID_struct;

typedef struct{
    unsigned char SOF;
    unsigned char Data_Length;
    unsigned char seq;
    unsigned char crc8;
    unsigned char cmd_ID;
    PID_struct PID;
    union{
        unsigned char tempChar[2];
        int tempInt;
    }crc16;
}MsgsFrame_struct;

typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  float P_out_max, I_out_max, D_out_max,PID_out_max;
  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;

#define CHASSIS_MOTOR_MOTOR_LF 1
#define CHASSIS_MOTOR_MOTOR_RF 2
#define CHASSIS_MOTOR_MOTOR_RB 3
#define CHASSIS_MOTOR_MOTOR_LB 4
#define YAW_MOTOR_ID 5
#define PITCH_MOTOR_ID 6

__USART6_EXT pid_t Motor_PID[10];
__USART6_EXT MsgsFrame_struct PID_Regulator_Msg;
void USART6_InitConfig(void);
void PID_ParamsUpload(char motor_ID);

#endif