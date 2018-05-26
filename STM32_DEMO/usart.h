#ifndef __USART_H
#define __USART_H
#include "main.h"

#define USART_Rx_LEN  			200  	//定义最大接收字节数 200
#define USART_Tx_LEN        200

#ifdef __USART_GLOBALS
#define __USART_EXT
#else 
#define __USART_EXT extern
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
        uint32_t tempFloat;
    }P_out_max;
    union{
        unsigned char tempChar[4];
        uint32_t tempFloat;
    }I_out_max;
    union{
        unsigned char tempChar[4];
        uint32_t tempFloat;
    }D_out_max;
    union{
        unsigned char tempChar[4];
        uint32_t tempFloat;
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

  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;


__USART_EXT pid_t Motor_PID[10];
__USART_EXT MsgsFrame_struct PID_Regulator_Msg;
void USART3_InitConfig(void);

#endif
