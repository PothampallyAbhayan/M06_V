
#ifndef _SUPPORT_H_
#define _SUPPORT_H_

#include "math.h"
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include <signal.h>
#include <time.h>
#include <sys/signalfd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <pthread.h>
#include <semaphore.h>
#include <termios.h>
#include "./libmodbus/include/modbus/modbus.h"
#include "./libmodbus/include/modbus/modbus-rtu.h"
#include "buffer.h"
/*****************Stack support variable************************************************/
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "icos_mbusdb_api.h"
/***Macro defenition****/
#define PRI_SEC_GAIN     0x05
#define SYS_PARA         0x06
#define PRI_PARA         0x08
#define SEC_WINDING_PARA 0x09
#define SEC_PARA_PANEL1  0x50
#define SEC_PARA_PANEL2  0x51
#define SEC_PARA_PANEL3  0x60
#define SEC_PARA_PANEL4  0x61
#define CT_CURRENT       0x52
#define FW_VERSION       0x56
#define CHN_CT_GAIN      0x5B
#define CHN_KW_GAIN      0x5B
#define PANEL_1_2_GAIN   0x5C
#define PANEL_3_4_GAIN   0x6C
#define DC_OFFSET        0x5C
#define CHN_WATTAGE      0x5F

#define TOT_SENSING_BRD  0x04
#define GAIN_LIMITER1    0x54
#define GAIN_LIMITER2    0x0E

#define FRAME_LIMIT      21

/***global variables****/
int    sock_can = -1;
int    sock_mod = -1;
int    sock_sig = -1;
int    uart_fd  = -1;
uint16_t can_data_para;
uint16_t can_data;

uint16_t buf_PRI_SEC_GAIN[20*4];
uint16_t buf_SYS_PARA[2*4];
uint16_t buf_PRI_PARA[20*4];
uint16_t buf_SEC_WINDING_PARA[38*4];
uint16_t buf_SEC_PARA_PANEL1[25*4];
uint16_t buf_SEC_PARA_PANEL2[25*4];
uint16_t buf_SEC_PARA_PANEL3[25*4];
uint16_t buf_SEC_PARA_PANEL4[25*4];
uint16_t buf_CT_CURRENT[84*4];
uint16_t buf_FW_VERSION[8*4];
uint16_t buf_CHN_CT_GAIN[84*4];
uint16_t buf_CHN_KW_GAIN[84*4];
uint16_t buf_PANEL_1_2_GAIN[14*4];
uint16_t buf_PANEL_3_4_GAIN[14*4];
uint16_t buf_DC_OFFSET[7*4];
uint16_t buf_CHN_WATTAGE[84*4];
typedef struct 
{
  fd_set read_fds;
  fd_set write_fds;
  int    max_fd;


  int    sock_mod;
  int    sock_can;
  int    sock_tcp_modbus;
  int    sock_config;
  int    sock_adc;
  int    sock_initKWH;
  int    sock_initKWHerr;
  int    sock_initIPsysKWHerr;
  int    sock_initIPsysKWH;
  int    sock_initOPsysKWHerr;
  int    sock_initOPsysKWH;
  int    uart_fd;
} THREAD_CTX;

/***function prototype****/
#if 0
void *handler_func ( void * );
int cp_can_data_to_buf(void);
int send_data_to_display(void); 
int initiate_timer(unsigned int , unsigned int );
#endif
void *handler_func ( void * );
void *modtcp_handler ( void * );
void *uart_handler(void *);
int cp_can_data_to_buf(void);
int copy_CAN_Rx_to_buffer(void);
int send_data_to_display(void); 
int initiate_timer(unsigned int , unsigned int );
int set_timer_0(long long sec, long nsec);
void *  write_message(void*  );
void *  db_filling(void * );
void *  shm_filling(void *);
void AmbientTempCalc(void);
void Cal_Fan_Current(unsigned char, unsigned char);
void RMS_Fan_Current(void);
double Square_root(unsigned int);
int initiate_timer_type(struct itimerspec *,unsigned int , unsigned int );
int clear_timer_type(struct itimerspec *);
void log_init(void);
//void add_to_log(void);
void add_to_log(/*unsigned short,*/ /*unsigned char,*/ unsigned long );
void stamp_time(void);
void timerHandler( int , siginfo_t *, void * );
int maketimer( timer_t *);
int cleartimer(timer_t *);
int settimer( timer_t *, int , int  );
int initiate_timer_type1(timer_t *timerID,unsigned int sec, unsigned int msec);
int init_timer_routine(timer_t *timerID);
int st_can_filter(canid_t id,canid_t mask_t,int filter_no);
void transmit_pkt(void );
void analytics(unsigned int *, unsigned int *);
void Calculation(void);
void Pri_Winding_Calculation(void);
void Sec_Winding_Calculation(void);
void Branch_CT_Calculation(void);
void LoadPer_Calc(unsigned int );
void Max_Min_Parameter(unsigned int);
void Status_Update(unsigned int);
void Demand_Calc(unsigned int reg);
void KWH_Calc(unsigned int);
void Initialize_Variables();
unsigned int set_address(void); 
void SetDefault();
void CalculateThreshold();
int modbus_parameter_filling(unsigned int ,modbus_mapping_t *);
int mod_reg_filling(unsigned int* ,modbus_mapping_t *);
int dummy_fun(unsigned int ,unsigned short *);//added for testing
//extern struct can_frame frame;

#endif /* _SUPPORT_H_ */
