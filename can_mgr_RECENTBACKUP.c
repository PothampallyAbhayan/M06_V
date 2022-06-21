
#include"support.h"
#include"PDC.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <ifaddrs.h>
#include<time.h>
#include<net/if.h>
#include<sys/mman.h>
#include<sys/stat.h>
#include<unistd.h>
#include<fcntl.h>
#include<math.h>
#include"unistd.h"
#include <sys/time.h> //753

#include "nos_i2c.h"

/*****************Extern variables***********************************************/
extern sensor_PKT  *sensor_Rx_pkt_pool_hdr ;
extern sensor_PKT  *sensor_Tx_pkt_pool_hdr ;
extern sensor_PKT  *sensor_Rx_pkt_hdr ;
extern sensor_PKT  *sensor_Tx_pkt_hdr ;
extern Can_data	Data;
extern ADC_FAN_CURRENT        AdcFanCurrent;
extern _FAN_PARAMETER         strFanParameter;
extern EE_DATA_UNION	Data_In;
extern EE_SYS_INFO_UNION	wSysInfo;
extern Can_data	Data;
extern SYSTEM_STATUS_UPDATE	System_Status;
/*****************Extern functions**********************************************/
extern void *bacnet_handler(void *ctx);
extern void Analog_Input_Present_Value_Set(uint32_t ,float );
/******************Function declaration*****************************************/

void create_db_field(NOS_DB_HANDLE  *,int );
void DB_entry_creation_for(unsigned int ,unsigned int,unsigned short,NOS_DB_HANDLE* );
void System_Status_Update(void);
void* BAC_update(void *);//Defined in bacmain.c
void ADC_process(void);
void config_process(void);
void event_process(void);
int BACnet_process(void);
static void icos_test_adc(int addr, int channel);
int bit_position(unsigned int);      
void Dynammic_sorting(void);
int get_adc_avg(int adc_flag, int chn_addr, int chn_no,int *avg_value, int *off_count, int *str_count);


#define NO_OF_SBOARDS	    4

/************************ MACRO Definition for first 32 Events ************************/

#define IP_UNDER_VOLTAGE            0

#define IP_UNDER_CURRENT	    IP_UNDER_VOLTAGE + 1

#define IP_OVER_VOLTAGE		    IP_UNDER_CURRENT + 1

#define IP_OVER_CURRENT		    IP_OVER_VOLTAGE + 1

#define OP_UNDER_VOLTAGE 	    IP_OVER_CURRENT + 1

#define OP_UNDER_CURRENT	    OP_UNDER_VOLTAGE + 1

#define OP_OVER_VOLTAGE		    OP_UNDER_CURRENT + 1

#define OP_OVER_CURRENT		    OP_OVER_VOLTAGE + 1

#define BRANCH_OVER_CURRENT	    OP_OVER_CURRENT + 1

#define FAN_FAILURE		    BRANCH_OVER_CURRENT + 1

#define IP_BREAKER		    FAN_FAILURE + 1

#define IP_BREAKER_TRIP		    IP_BREAKER + 1

#define OP_BREAKER		    IP_BREAKER_TRIP + 1

#define OP_BREAKER_TRIP	            OP_BREAKER + 1

#define PANEL_OVERLOAD		    OP_BREAKER_TRIP + 1        

#define PANEL_OVERLOAD_140	    PANEL_OVERLOAD + 1

#define XMR_OT		            PANEL_OVERLOAD_140 + 1

#define POWER_FACTOR		    XMR_OT + 1

#define NEUTRAL_CURRENT		    POWER_FACTOR + 1

#define OUTPUT_VTHD		    NEUTRAL_CURRENT + 1

#define OUTPUT_CTHD		    OUTPUT_VTHD + 1

#define PANEL_OL_TRIP		    OUTPUT_CTHD + 1

#define AMBIENT_TEMP     PANEL_OL_TRIP + 1

#define GROUND_CURRENT  AMBIENT_TEMP + 1

/*************************** MACRO defenition ***************************/

#define MBUS_IF_INDEX_OFFSET               0x7000
#define FILEPATH "/tmp/data"
#define NUMINTS (5000)
#define FILESIZE (NUMINTS* sizeof(int))

/*********************************** MACRO FOR NV COPY ***********************************/

#define DB_FIELD  1
#define SHM_FIELD 2
#define FIELD_TOTALENTRY 2500      

#define PRIMARY_OFFSET	   (sizeof(PRI_STRUCTURE_PARAMETER)/4)

#define SECONDARY_OFFSET   (sizeof(SEC_STRUCTURE_PARAMETER)/4) + PRIMARY_OFFSET

#define STATUS_UPDATE	   2*(sizeof(SYSTEM_STATUS_UPDATE)/4) + SECONDARY_OFFSET

#define SYS_NV             (sizeof(EE_SYS_INFO)/4) + STATUS_UPDATE

#define FW_VERSION0        (sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + SYS_NV
//#define FW_VERSION1        (sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + FW_VERSION0
//#define FW_VERSION2        (sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + FW_VERSION1
//#define FW_VERSION3        (sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + FW_VERSION2

#define SYSTEMPARAMETER	      (sizeof(SYS_STRUCTURE_PARAMETER)/4) + FW_VERSION0

#define RMS0_OFFSET     (sizeof(DSP_PANEL_PARAMETER)/4) + SYSTEMPARAMETER
#define RMS1_OFFSET	(sizeof(DSP_PANEL_PARAMETER)/4) + RMS0_OFFSET

#define KW0_OFFSET	(sizeof(DSP_PANEL_PARAMETER)/4) + RMS1_OFFSET
#define KW1_OFFSET	(sizeof(DSP_PANEL_PARAMETER)/4) + KW0_OFFSET

#define LOAD0_OFFSET	(sizeof(DSP_PANEL_PARAMETER)/4) + KW1_OFFSET
#define LOAD1_OFFSET	(sizeof(DSP_PANEL_PARAMETER)/4) + LOAD0_OFFSET 

#define PANEL11_UNDERCURR_FLAG 	(sizeof(CT_CURRENT_STATUS_FLAG)/4) + LOAD1_OFFSET

#define PANEL12_UNDERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL11_UNDERCURR_FLAG
#define PANEL13_UNDERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL12_UNDERCURR_FLAG
#define PANEL14_UNDERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL13_UNDERCURR_FLAG

#define PANEL11_OVERCURR_FLAG 	(sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL14_UNDERCURR_FLAG
#define PANEL12_OVERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL11_OVERCURR_FLAG
#define PANEL13_OVERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL12_OVERCURR_FLAG
#define PANEL14_OVERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL13_OVERCURR_FLAG

#define PANEL21_UNDERCURR_FLAG 	(sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL14_OVERCURR_FLAG
#define PANEL22_UNDERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL21_UNDERCURR_FLAG
#define PANEL23_UNDERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL22_UNDERCURR_FLAG
#define PANEL24_UNDERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL23_UNDERCURR_FLAG

#define PANEL21_OVERCURR_FLAG 	(sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL24_UNDERCURR_FLAG
#define PANEL22_OVERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL21_OVERCURR_FLAG
#define PANEL23_OVERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL22_OVERCURR_FLAG
#define PANEL24_OVERCURR_FLAG  (sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL23_OVERCURR_FLAG

#define KWH0_OFFSET	       (sizeof(DSP_KWH_PANEL1_PARAMETER)/4) + PANEL24_OVERCURR_FLAG
#define KWH1_OFFSET	       (sizeof(DSP_KWH_PANEL1_PARAMETER)/4) + KWH0_OFFSET

#define CT_GAIN		       (sizeof(DSP_PANEL_PARAMETER)/4) + KWH1_OFFSET
#define KW_GAIN		       (sizeof(DSP_PANEL_PARAMETER)/4) + CT_GAIN
#define SYSTEM_GAINS	       (sizeof(DSP_PANEL_PARAMETER)/4) + KW_GAIN


#define KWH0_ERROR	       (sizeof(CT_LONG_BUFF)/4) + SYSTEM_GAINS
#define KWH1_ERROR	       (sizeof(CT_LONG_BUFF)/4) + KWH0_ERROR	

#define SYS_KWH_ERROR	       (sizeof(_KWH_ERROR)/4) + KWH1_ERROR

#define CURR0_DEMAND	       (sizeof(DSP_PANEL_PARAMETER)/4) + SYS_KWH_ERROR
#define CURR1_DEMAND           (sizeof(DSP_PANEL_PARAMETER)/4) + CURR0_DEMAND

#define KW0_DEMAND       (sizeof(DSP_PANEL_PARAMETER)/4) + CURR1_DEMAND
#define KW1_DEMAND	 (sizeof(DSP_PANEL_PARAMETER)/4) + KW0_DEMAND

#define MAX_CURR_DEMAND0       (sizeof(DSP_PANEL_PARAMETER)/4) + KW1_DEMAND
#define MAX_CURR_DEMAND1       (sizeof(DSP_PANEL_PARAMETER)/4) + MAX_CURR_DEMAND0

#define MAX_KW_DEMAND0	       (sizeof(DSP_PANEL_PARAMETER)/4) + MAX_CURR_DEMAND1
#define MAX_KW_DEMAND1         (sizeof(DSP_PANEL_PARAMETER)/4 + MAX_KW_DEMAND0)	 	       
	

/*********************************** MACRO FOR CAN***********************************/

#define PRI_OFFSET	(sizeof(PRI_STRUCTURE_PARAMETER)/4)
#define SEC_OFFSET      ((sizeof(SEC_STRUCTURE_PARAMETER)/4) + PRI_OFFSET)
#define RMS_0_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + SEC_OFFSET)
#define RMS_1_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + RMS_0_OFFSET)
#define RMS_2_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + RMS_1_OFFSET)
#define RMS_3_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + RMS_2_OFFSET)
#define KW_0_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + RMS_3_OFFSET)
#define KW_1_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + KW_0_OFFSET)
#define KW_2_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + KW_1_OFFSET)
#define KW_3_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + KW_2_OFFSET)	
#define LOAD_0_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + KW_3_OFFSET)
#define LOAD_1_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + LOAD_0_OFFSET)
#define LOAD_2_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + LOAD_1_OFFSET)
#define LOAD_3_OFFSET	((sizeof(DSP_PANEL_PARAMETER)/4) + LOAD_2_OFFSET)
#define MAX_MIN_PRIMARY	  ((sizeof(MAX_MIN_PARAMTER)/4) + LOAD_3_OFFSET)
#define CT_MAX_CURR_DEMAND_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + MAX_MIN_PRIMARY)
#define CT_MAX_CURR_DEMAND_1  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_0)	
#define CT_MAX_CURR_DEMAND_2  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_1)
#define CT_MAX_CURR_DEMAND_3  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_2)

#define CT_MAX_KW_DEMAND_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_3)
#define CT_MAX_KW_DEMAND_1  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_0)	
#define CT_MAX_KW_DEMAND_2  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_1)
#define CT_MAX_KW_DEMAND_3  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_2)

#define MAXIMUM_0_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_3)
#define MAXIMUM_1_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MAXIMUM_0_OFFSET)	
#define MAXIMUM_2_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MAXIMUM_1_OFFSET)
#define MAXIMUM_3_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MAXIMUM_2_OFFSET)

#define MINIMUM_0_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MAXIMUM_3_OFFSET)
#define MINIMUM_1_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MINIMUM_0_OFFSET)	
#define MINIMUM_2_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MINIMUM_1_OFFSET)
#define MINIMUM_3_OFFSET  ((sizeof(DSP_PANEL_PARAMETER)/4) + MINIMUM_2_OFFSET)

#define CT_CURR_DEMAND_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + MINIMUM_3_OFFSET)
#define CT_CURR_DEMAND_1  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_CURR_DEMAND_0)	
#define CT_CURR_DEMAND_2  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_CURR_DEMAND_1)
#define CT_CURR_DEMAND_3  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_CURR_DEMAND_2)

#define CT_KW_DEMAND_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_CURR_DEMAND_3)
#define CT_KW_DEMAND_1  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_KW_DEMAND_0)	
#define CT_KW_DEMAND_2  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_KW_DEMAND_1)
#define CT_KW_DEMAND_3  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_KW_DEMAND_2)

#define CT_MAX_CURR_DEMAND_24HR_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_KW_DEMAND_3)
#define CT_MAX_CURR_DEMAND_24HR_1  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_24HR_0)	
#define CT_MAX_CURR_DEMAND_24HR_2  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_24HR_1)
#define CT_MAX_CURR_DEMAND_24HR_3  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_24HR_2)

#define CT_MAX_KW_DEMAND_24HR_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_CURR_DEMAND_24HR_3)
#define CT_MAX_KW_DEMAND_24HR_1  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_24HR_0)	
#define CT_MAX_KW_DEMAND_24HR_2  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_24HR_1)
#define CT_MAX_KW_DEMAND_24HR_3  ((sizeof(DSP_PANEL_PARAMETER)/4) + CT_MAX_KW_DEMAND_24HR_2)

#define PRI_STATUS_FLAG		 ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + CT_MAX_KW_DEMAND_24HR_3)

#define SEC_STATUS_FLAG		 ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PRI_STATUS_FLAG)

#define PANEL11_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + SEC_STATUS_FLAG)
#define PANEL12_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL11_UNDERCURR_STATUS_FLAG)
#define PANEL13_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL12_UNDERCURR_STATUS_FLAG)
#define PANEL14_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL13_UNDERCURR_STATUS_FLAG)  

#define PANEL11_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL14_UNDERCURR_STATUS_FLAG)
#define PANEL12_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL11_OVERCURR_STATUS_FLAG)
#define PANEL13_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL12_OVERCURR_STATUS_FLAG)
#define PANEL14_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL13_OVERCURR_STATUS_FLAG)

#define PANEL21_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL14_OVERCURR_STATUS_FLAG)
#define PANEL22_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL21_UNDERCURR_STATUS_FLAG)
#define PANEL23_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL22_UNDERCURR_STATUS_FLAG)
#define PANEL24_UNDERCURR_STATUS_FLAG ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL23_UNDERCURR_STATUS_FLAG) 

#define PANEL21_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL24_UNDERCURR_STATUS_FLAG)
#define PANEL22_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL21_OVERCURR_STATUS_FLAG)
#define PANEL23_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL22_OVERCURR_STATUS_FLAG)
#define PANEL24_OVERCURR_STATUS_FLAG  ((sizeof(CT_CURRENT_STATUS_FLAG)/4) + PANEL23_OVERCURR_STATUS_FLAG)

#define KWH_0_OFFSET		      ((sizeof(DSP_KWH_PANEL1_PARAMETER)/4) + PANEL24_OVERCURR_STATUS_FLAG)
#define KWH_1_OFFSET		      ((sizeof(DSP_KWH_PANEL1_PARAMETER)/4) + KWH_0_OFFSET)
#define KWH_2_OFFSET		      ((sizeof(DSP_KWH_PANEL1_PARAMETER)/4) + KWH_1_OFFSET)
#define KWH_3_OFFSET		      ((sizeof(DSP_KWH_PANEL1_PARAMETER)/4) + KWH_2_OFFSET)

#define SYS			      ((sizeof(EE_SYS_INFO)/4) + KWH_3_OFFSET)

#define FIRMWARE_0		      ((sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + SYS)
#define FIRMWARE_1		      ((sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + FIRMWARE_0)
#define FIRMWARE_2		      ((sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + FIRMWARE_1)
#define FIRMWARE_3		      ((sizeof(FIRMWARE_VERSION_STRUCTURE)/4) + FIRMWARE_2)	

#define SYSTEM_STATUS		      ((sizeof(SYSTEM_STATUS_UPDATE)/4) + FIRMWARE_3)	
#define SYSTEM_PARAMETER	      ((sizeof(SYS_STRUCTURE_PARAMETER)/4) + FIRMWARE_3)

#define NO_OF_PANLES 2

/********************************** Macro for mmap **********************************/

#define KWH_PRIMARY 0
#define KWH_SECONDARY KWH_PRIMARY + 2
#define KWH_PANEL1 KWH_SECONDARY + 6
#define KWH_PANEL2 KWH_PANEL1 + 168

#define KWHERROR_PRIMARY KWH_PANEL2 + 168
#define KWHERROR_SECONDARY KWHERROR_PRIMARY + 1
#define KWHERROR_PANEL1 KWHERROR_SECONDARY + 3
#define KWHERROR_PANEL2 KWHERROR_PANEL1 + 84

#define KWHTOTAL 518
#define FILESIZE_KWHDATA  (KWHTOTAL* sizeof(int))

/********************************** Typedefs **********************************/

typedef enum _modbus_reg_type_ {

    INPUT_REGISTER,
    HOLDING_REGISTER,
    COIL,
    DISCRETE_INPUT,
    RAW_REQUEST,

} MODBUS_REG_TYPE;

typedef union _mod_data
    {
      icos_int16 data;
      icos_uchar cdata[2];
    }mod_data;
typedef union _data_union
{
    unsigned char data[2];
    unsigned short reg_value;		
}data_union;

typedef struct _Temp_Reg
{
    unsigned int    if_index;
    unsigned int  reg_addr;
    data_union	    reg_d;
    MODBUS_REG_TYPE reg_type;
    unsigned char   reg_flag;	
}Temp_Reg; 


typedef union _fan_status{
  unsigned char fan_err_status_array[12];
  unsigned short fan_err_status_dummy;
}fan_status;


/********************************** Private variables **********************************/

static unsigned int Tx_frame_count = 0;
volatile static icos_uint32 if_index_cur = MBUS_IF_INDEX_OFFSET;
volatile static icos_uint32 if_index_start = MBUS_IF_INDEX_OFFSET;
 
/********************************** Global variables **********************************/

//extern int    uart_fd;                                       // Used for Uart communication
struct termios SerialPortSettings;                    // POSIX Terminal Definitions 
unsigned short SBoard1[PANEL_GAIN];
unsigned short SBoard2[PANEL_GAIN];
unsigned short System_gain[SYSTEM_GAIN]; 
struct can_frame temp1_frame;                         // Requesting Gain from S-1
struct can_frame temp2_frame;                         // Sending Gain to S-1
struct can_frame temp3_frame;			      // Sending Gain to S-2
struct can_frame *temp1_ptr = &temp1_frame;
struct can_frame *temp2_ptr = &temp2_frame;
struct can_frame *temp3_ptr = &temp3_frame;
unsigned int *dptr1;
unsigned int *dptr2;
unsigned int *dptr3;
unsigned short calib_branch_data[84];
unsigned short calib_kw_data[84];
struct can_frame frame;
struct canfd_frame fd_frame;
struct sockaddr_can addr;
struct ifreq ifr;
struct can_filter board_filter[3];
fan_status err_status;
ULONG_2WORD dwCalcBuff[6];
unsigned short buf[20];
unsigned char fan_fail[12];
int fd1[2];  // Used to store two ends of first pipe 
int fd2[2];  // Used to store two ends of second pipe 
int fd3[2];  // Used to store two ends of third pipe 
int fd4[2];  // Used to store two ends of fourth pipe
int fd5[2];  // Used to store two ends of fifth pipe
/***************For system KWH - start*******************/
int fd7[2];  // Used to store two ends of second pipe 
int fd8[2];  // Used to store two ends of second pipe
int fd9[2];  // Used to store two ends of second pipe
int fd10[2]; // Used to store two ends of second pipe 
/***************End**************************************/ 
unsigned short fan_err_status = 0;
unsigned short temperature_status;
unsigned short fan_alarm;
unsigned int status_result;
unsigned char copy_flag = 0;
unsigned char  buzz_cmd;
int  buzz_status;
unsigned char  no_xfmr;
unsigned int   no_xfmr_fd;
unsigned int   panel_fd;   // For panel OL
unsigned int   fan_cntr_fd;      // for fan_cntr parameters (04/02/22)
unsigned int   thd_fd; 
unsigned int   dynamic_mappingfd;  // For Dynamic Mapping
int buzz_silence_fd;       // Buzz_Silence FD
int dry_contact_fd;        // Dry Contact FD
int dry_contact_on_off_fd; // Dry Contact ON OFF FD
unsigned int dry_contact_flag[6]; // Dry contact event donfiguration
unsigned int dry_on_off_flag[6]; // Dry contact on off configuration 
unsigned int  alarm_status = 0; // Alarm status byte
volatile unsigned char alarm_flag = 0;
unsigned int   fan_cntr = 0;

unsigned int fan_num;
unsigned int fan_num_fd; 
unsigned int fan_bits;
unsigned char  fannum_buff[10] = {0};

//useful to check time taken
unsigned int jack;
struct timeval stop_t, start_t;

unsigned char  thd_buffer[100] = {0}; 

//For fan offsets
unsigned int fancalib_flagfd;
unsigned char fanflag_buffer[6];
unsigned int fan_offsetfd;
unsigned char fanoffset_buffer[100];
unsigned char *cptr_offset_dummy;

//For ntp sync flags
unsigned int ntp_fd;
unsigned char ntpflag_buffer[6];
unsigned int ntpsync_flag = 0;
struct tm time_1 = { 0 };


unsigned char kwh_buffer[5000]={0};
unsigned char *kwh_dummy;

unsigned int fan_err_cntr[12] = {0,0,0,0,0,0,0,0};
unsigned int fan_no_err_cntr[12];

unsigned char  buffer1[10] = {0};
unsigned char  panel_buffer[100] = {0};  // For panel OL
unsigned char  fan_cntr_buffer[100] = {0};     // for fan_cntr parameters (04/02/22)
unsigned char  *cptr_fan_cntr_dummy; 
unsigned char  *cptr_panel_dummy;        // For panel OL
unsigned char  dynamic_buffer[1500] = {0}; // For Dynamic Mapping
unsigned char  *cptr_dynamic_dummy;
unsigned char  *cptr_thd_dummy; 
volatile unsigned char flag;
volatile unsigned char time_stamp_flag;
volatile char pool_pos;
unsigned char chn_no=0;
unsigned char chn_addr=0x48;
int *map;
int *map_etc;
unsigned int event_flag[70];
unsigned int wGroundCurrentCntr;
unsigned int wPFCntr;			    // Counter for PF Alarm
unsigned int AmbientTempcntr;
unsigned int dwSecDevPhaseAB;
unsigned int dwSecDevPhaseBC;
unsigned int dwSecDevPhaseCA;
unsigned int dwPriDevPhaseAB;
unsigned int dwPriDevPhaseBC;
unsigned int dwPriDevPhaseCA;
unsigned int dwSecVoltAvg;
unsigned int dwPriVoltAvg;
unsigned int wPhaseA_Board_I_PanelCurrent[12];                 // Added for Panel OL Alarm // Date : 17/04/2020
unsigned int wPhaseB_Board_I_PanelCurrent[12];
unsigned int wPhaseC_Board_I_PanelCurrent[12];
unsigned int wPhaseA_PanelCurrent[3];
unsigned int wPhaseB_PanelCurrent[3];
unsigned int wPhaseC_PanelCurrent[3];
unsigned int wPDU_Parameters[25];                    // Added for Panel OL Alarm // Date : 13/07/2021
unsigned int wTHD_Parameters[3]; 	// Added to read THD_parameters using Utility (15/11/21)
unsigned int Dynamic_Buffer[84*NO_OF_SBOARDS];       // Added for Dynamic Mapping
unsigned int fan_cntr_Parameters[8];  //added on 04/02/22
unsigned int wMax_Panel_Limit;
unsigned int Fan_cntr_48 = 0;
unsigned int Fan_cntr_49 = 0;
unsigned int Fan_cntr_4B = 0;
unsigned int Fan_cntr = 0;
unsigned int Fan_alarm_cntr = 0;
unsigned int fan_rec_cntr = 0; //added on 27/10/21
unsigned int Fan_flag =0;
unsigned int Fan_flag1=0;
unsigned char req[100];
unsigned int wCalc_Cntr,k,wDemandSumCntr[4],regs,wKwh_Calc_Cntr;
unsigned int wDemandChkCntr1s_10ms;
unsigned int wDemandChkCntr1hr_10ms;
unsigned int wDemandChkCntr24hr_10ms;
unsigned long dwMathBuff;
int     log_fd;
int     *shmptr;
int ip_fd;
unsigned int ipv4_addr;
unsigned int nbytes1;
unsigned long BAC_id = 0xffff;
unsigned long* pBAC_id = NULL;
unsigned short  system_stat;
static unsigned short op_breaker_status = 0;   // O/P breaker delay counter
static unsigned short input_undervolt_count = 0; // Input UnderVolt delay counter
static unsigned int branch_count = 0;
sem_t semaphore_1;
sensor_PKT *pkt_ptr = NULL;
sensor_PKT  sensor_Rx_buffers[MAX_POOL_BUFFERS];
sensor_PKT  sensor_Tx_buffers[1]; 
timer_t timerid;
timer_t firstTimerID;
timer_t secondTimerID;
timer_t thirdTimerID;
timer_t fourthTimerID;
timer_t sixthTimerID;
timer_t seventhTimerID;
timer_t eighthTimerID;

timer_t copyTimerID;
timer_t calibration_timerID;                           // Calibration timer
timer_t S_Board_Update_timer;			       // Timer for Updating S-Board Gain values	
unsigned int     calibration_flag = 1;
unsigned int     calibration_count = 0;
unsigned int fancalib_flag = 0;

sigset_t mask;
pthread_t p_thread,thread_w,tcp_thread;
//pthread_t db_thread;						// Thread for DB filling
pthread_t shm_thread;						// Thread for shard memory filling
pthread_t uart_thread;                                          // Thread for uart handler
THREAD_CTX p_ctx;
THREAD_CTX p_ctx1;
pthread_mutex_t lock1,db_lock,lock2;
pthread_mutex_t kw_lock1, kw_lock2; 

FILE *logfile;
clock_t start;
modbus_t *mod_ctx,*mod_ctx1;
modbus_t *tcp_ctx;
modbus_mapping_t *mapping ;
sensor_PKT *copy_CAN_Rx_to_buffer_2(sensor_PKT *p_ptr);
unsigned char wMboard_Fw_Ver[16] = {'M','-','0','S','T','T',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

/****************************************** BACnet ******************************************/

pthread_t bacnet_thread;
pthread_t update_thread;

/**************************** Stack support variables and functions ****************************/

//NOS_DB_HANDLE         *p_db = NULL;//DK removed
NOS_DB_HANDLE         *p_db1 = NULL;
icos_int16            modbus_bus_no = 0; //rs485a = 0, rs485b = 1
icos_int16            slave_id = 1;
icos_uint32           reg_add = 0x800;
icos_uchar            *data = NULL;
icos_uint32           size, if_index;
icos_uchar            data_wr[4] = {1,2,3,4};
mod_data reg_data;

/************************************* REG Filling for DB *************************************/

Temp_Reg	Reg[sizeof(Data)/4];
unsigned short  Current_Buffer[84*NO_OF_SBOARDS];
unsigned short  KW_Buffer[84*NO_OF_SBOARDS];
unsigned short  ITHD_Buffer[84*NO_OF_SBOARDS];

/*************************************** Child process ***************************************/

pid_t adc_pid;
pid_t config_pid;
pid_t event_pid;
pid_t BACnet_pid;

/***************************EpochTime**************************************************************************/

unsigned int epochtime;
unsigned short epochtime_flag;
int epochtime_cntr; 

/************************ NTP server ************************/

unsigned int ntpserverfd;
unsigned char ntp_buffer[40];
unsigned char *cptr_ntp_dummy;

/********************************** initialising fan counters **********************************/

int chn_address[8] = {0x49, 0x49, 0x49, 0x48, 0x48, 0x48, 0x48, 0x4B};
int chn_number[8] = {1, 2, 3, 0, 1, 2, 3, 0};
int z_Offset[8] = {0,0,0,0,0,0,0,0};
int adc_flag;
//status count for GPIO status
unsigned int status_count[5] = {0, 0, 0, 0, 0};

/*********************************** Main function starts ***********************************/

int main()
{
    temp1_ptr->can_dlc = 8;
    dptr1 = (unsigned int *)temp1_ptr->data;
    *dptr1 = 0x00050101;
    temp1_ptr->can_id = 0x7E7;

    temp2_ptr->can_dlc = 8;
    dptr2 = (unsigned int *)temp2_ptr->data;
    temp2_ptr->can_id = 0x7E7;

    temp3_ptr->can_dlc = 8;
    dptr3 = (unsigned int *)temp3_ptr->data;
    temp3_ptr->can_id = 0x7E7;
    
   
    int i=0;
    ssize_t ret_in;
    unsigned char buffer[10]= {0};
    unsigned char signature;
    unsigned char flag = 0;
    int read_fd;
    key_t   shmkey;
    int     shmid;
    
    //7539

    /******************  UART PORT Opening & Initialisation ********************/
    uart_fd = open("/dev/ttymxc2",O_RDWR | O_NOCTTY);
    if (uart_fd == -1)
       printf("\nError in opening uart\n");
    else 
       printf("\nOpened succesfully - ttymxc2\n");
    

    tcgetattr(uart_fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings,B9600);   // O/P baud rate 9600
    cfsetospeed(&SerialPortSettings,B9600);   // I/P baud rate 9600
    SerialPortSettings.c_cflag &= ~PARENB;    // Clear Parity
    SerialPortSettings.c_cflag &= ~CSTOPB;    // Stop bit - 1
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;        // 8 bits data size
    SerialPortSettings.c_cflag |= (CREAD | CLOCAL); // Turn ON receiver of serial port
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);  // XON/XOFF

    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;
    
    tcsetattr(uart_fd,TCSANOW,&SerialPortSettings);
    //tcsetattr(uart_fd,TCIOFLUSH); 
    //close(uart_fd);
    /******************** END *********************************/

	/*
	printf("Checking the timer\n");
	system("echo start >> /home/root/date.txt");
	system("date >> /home/root/date.txt");
	system("date");
	*/
    
//for ip Address reading
	FILE *ip_fp;
    char line [50];
    struct in_addr addrptr;
    int retval;
    char *ipptr;
    unsigned short *sip_ptr;
    //unsigned char  buzz_cmd;
//End
/**********************memmap******************************/
    int j;
    int mem_fd = 0;
    int result;
/**********************shared memory*******************************/
    shmkey = ftok("shmfile",65);
    shmid  = shmget(shmkey,10,0600|IPC_CREAT);
    shmptr = shmat(shmid,(void*)0,0);
    
/**********************I2C for digital port*******************************/

    mem_fd = open(FILEPATH,O_RDWR|O_CREAT|O_TRUNC,0600);
    if(mem_fd == -1)
    {
      perror("mem file:");
      exit(EXIT_FAILURE);
    }
    result = lseek(mem_fd,FILESIZE-1,SEEK_SET);
    if(result == -1)
    {
      close(mem_fd);
      perror("error calling lseek");
      exit(EXIT_FAILURE);
    }
    result = write(mem_fd,"A",1);
    if(result != 1)
    {
      close(mem_fd);
      perror("error writing last byte of the file");
      exit(EXIT_FAILURE);
    }
    map = (int*)mmap(0,FILESIZE,PROT_READ|PROT_WRITE,MAP_SHARED,mem_fd,0);
    if(map == MAP_FAILED)
    {
      close(mem_fd);
      perror("Error mapping the file");
      exit(EXIT_FAILURE);
    }

/**********************I2C for digital port*******************************/
    nos_i2c_bus_open(0);
    usleep(100);
    nos_i2c_bus_open(1);
    usleep(100);

    nos_i2c_device_open(0,0x43);//I2C open
    usleep(100);
    nos_i2c_device_open(0,0x44);
    usleep(100);
    nos_i2c_device_open(1,0x43);
    usleep(100);
    nos_i2c_device_open(0,0x22);			// For Alarm

    printf("\nI2C Open finished\n");

    icos_io_init(0,0x43);//Device soft reset and impedence setting
    usleep(100);
    icos_io_init(1,0x43);//Device soft reset and impedence setting
    usleep(100);
    icos_io_init(0,0x22);                              // For Alarm

    //printf("\n*******Init finished*****\n");

    icos_io_set_direction(0,0x43,0x3F,0x00);       // 0011 1111   ('0' for input(DRYA,DRYB,DRYC,DRYD,DRYE,DRYF) and '1' for output(REPO and EPO))
    
    usleep(100);
    icos_io_set_direction(0,0x44,0x04,0x00);	   // 0000 0100   ('0' for input (TEMP_RT1,TEMP_RT2,Aux_Power_Fail,ZCD,CBSTAT_M,CBSTAT_P1) and '1' for output (Fan_PWM))
    //printf("\n*******set2 finished*****\n");
    usleep(100);
    usleep(100);
    buzz_cmd = 0x00;
    nos_i2c_write(0, 0x22, 0x06, &buzz_cmd, 1);//icos_io_set_direction(0,0x22,0x06,0x00);			// for Alarm
    usleep(100);
    buzz_cmd = 0x01;
    nos_i2c_write(0, 0x22, 0x02, &buzz_cmd, 1);//icos_io_set_direction(0,0x22,0x06,0x00);			// for Alarm
    usleep(100);
    buzz_cmd = 0x01;
    nos_i2c_write(0, 0x22, 0x02, &buzz_cmd, 1);//icos_io_set_direction(0,0x22,0x06,0x00);			// for Alarm
    usleep(100);
    
/**********************ADC Initialisation****************/
    
    usleep(200);
    printf("\nADC channels opened and Initialised\n");
/***********************END*******************************/
/************ Open the file corresponding to Fan_Limit_Parameters ***********************************************************************/
    j=0;
    fan_cntr_fd = open("/etc/fancntrparameters",O_RDONLY,S_IRUSR);
    if (fan_cntr_fd == -1)
    {
     fan_cntr_fd = open("/etc/fancntrparameters",O_RDWR|O_CREAT,S_IRUSR);
     if (fan_cntr_fd == -1)
     {     
      perror("Error in fan_cntr_parameters openning:");
     }
     fan_cntr_Parameters[0] = 8;                  // (AdcFanCurrent.array[i] < limit)
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[0]);
     fan_cntr_Parameters[1] = 1;                 // fan_err_cntr[i] set
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[1]);
     fan_cntr_Parameters[2] = 2;					// fan_no_err_cntr[i] set
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[2]);    
     fan_cntr_Parameters[3] = 100;					//  fan_rec_cntr set changed to //fan failure limit
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[3]);
	 fan_cntr_Parameters[4] = 9100;					// Fan_alarm_cntr set changed to //fan struck limit
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[4]);
	 fan_cntr_Parameters[5] = 8900;					// Fan_alarm_cntr set changed to //fan struck limit
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[5]);
	 fan_cntr_Parameters[6] = 10;					// Fan_alarm_cntr set changed to //fan struck limit
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[6]);
	 fan_cntr_Parameters[7] = 10;					// Fan_alarm_cntr set changed to //fan struck limit
     dprintf(fan_cntr_fd,"%d\n",fan_cntr_Parameters[7]);

    }
    read(fan_cntr_fd ,fan_cntr_buffer,100);
    cptr_fan_cntr_dummy = strtok(fan_cntr_buffer,"\n");
    while(cptr_fan_cntr_dummy != NULL)
    {
      fan_cntr_Parameters[j] = atoi(cptr_fan_cntr_dummy);

      cptr_fan_cntr_dummy = strtok(NULL,"\n");
      j++;
    }
    j=0;
    close(fan_cntr_fd);

	/************************* Fan number file ***************************************/

	fan_num_fd = open("/etc/fan_num",O_RDONLY,S_IRUSR);  
	if(fan_num_fd == -1)
	{
		fan_num_fd = open("/etc/fan_num",O_RDWR|O_CREAT,S_IRUSR);
		if ( fan_num_fd == -1 )
		{
			perror("Error in fan number file open:");
		} 
		fan_num = 6;
		dprintf(fan_num_fd,"%d\n",fan_num);  
	} 
	else
	{
		read(fan_num_fd,fannum_buff,4);
		fan_num = atoi(fannum_buff);
		Data.word.Sys.fan_num = ((0x10B << 16) | (fan_num));
	}
	close(fan_num_fd);
	fan_bits = (1 << fan_num) - 1;

/************************************ END ***********************************************************************************/  

fancalib_flagfd = open("/tmp/fancalib_flag",O_RDONLY,S_IRUSR);
if (fancalib_flagfd == -1)
{
	fancalib_flagfd = open("/tmp/fancalib_flag",O_RDWR|O_CREAT,S_IRUSR);
	if (fancalib_flagfd == -1)
	{
		perror("Error in fan offset flag file opening:");
	}
	dprintf(fancalib_flagfd,"%d\n",0);
}
read(fancalib_flagfd,fanflag_buffer,4);
fancalib_flag = atoi(fanflag_buffer);
//printf("Fan Calibration Offset : %d\n",fancalib_flag);

if(!fancalib_flag)
{
	get_fan_offset();
	fan_offsetfd = open("/tmp/fanoffset",O_RDWR|O_CREAT,S_IRUSR);
	if (fan_offsetfd == -1)
	{
		perror("Error in fan offset file opening:");
	}
	for (j = 0; j < fan_num; j++)
	{
		//printf("Into file : Fan Offset[%d] :  %d\n",j,z_Offset[j]);
		dprintf(fan_offsetfd,"%d\n",z_Offset[j]);
	}
	//fancalib_flagfd = open("/tmp/fancalib_flag",O_RDWR|O_TRUNC,S_IRUSR);
	lseek(fancalib_flagfd,0,SEEK_SET);
	dprintf(fancalib_flagfd,"%d\n",1);	
}
else
{
	j = 0;
	fan_offsetfd = open("/tmp/fanoffset",O_RDONLY,S_IRUSR);
	read(fan_offsetfd,fanoffset_buffer,100);
    cptr_offset_dummy = strtok(fanoffset_buffer,"\n");
    while(cptr_offset_dummy != NULL)
    {
      z_Offset[j] = atoi(cptr_offset_dummy);
      //printf("From file : Fan Offset[%d] :  %d\n",j,z_Offset[j]);
      cptr_offset_dummy = strtok(NULL,"\n");
      j++;
	}
}
close(fan_offsetfd);
close(fancalib_flagfd); 

/*************************** Configure ntp sync ***************************/

	ntp_fd = open("/etc/ntpsync_flag",O_RDONLY,S_IRUSR);
	if (ntp_fd == -1)
	{
		ntp_fd = open("/etc/ntpsync_flag",O_RDWR|O_CREAT,S_IRUSR);
		if (ntp_fd == -1)
		{
			perror("Error in ntp sync flag file opening:");
		}
		dprintf(ntp_fd,"%d\n",0);
	}
	read(ntp_fd,ntpflag_buffer,4);
	ntpsync_flag = atoi(ntpflag_buffer);

	if(ntpsync_flag)
	{
		epochtime_cntr = 1500;
		jack = 0;
	}
	close(ntp_fd);

/************* End **************************************************/

/***********************PWM Initialisation****************/
    #if 1
    icos_pwm_init(0);
    usleep(100);
    icos_set_pwm_period(0,1000000);
    usleep(100);
    icos_set_pwm_duty_cycle(0, 900000);
    usleep(100);
    icos_pwm_channel_enable(0,1);
    #endif

/********** Parameter ID setting for System Status******************/
    Data.word.System_Status.paraID = 0x02;
    j=0;
/************* Open the file corresponding to Dynamic Mapping of CT**/
    dynamic_mappingfd = open("/etc/dynamic_mapping",O_RDONLY,S_IRUSR);
    if (dynamic_mappingfd == -1)
    {
      dynamic_mappingfd = open("/etc/dynamic_mapping",O_RDWR|O_CREAT,S_IRUSR);
      if (dynamic_mappingfd == -1)
      {
	perror("Error in dynamic_mapping opening:");
      }
      for (j=1; j <=(84*NO_OF_SBOARDS); j++)
          {
          Dynamic_Buffer[j-1] = j;
          dprintf(dynamic_mappingfd,"%d\n",j);
          }
    }
    else
    {
		read(dynamic_mappingfd,dynamic_buffer,1500);
		cptr_dynamic_dummy = strtok(dynamic_buffer,"\n");
		while(cptr_dynamic_dummy != NULL)
		{
		Dynamic_Buffer[j] = atoi(cptr_dynamic_dummy);
		//printf("Dynamic_Buffer[%d] %d\n",j,Dynamic_Buffer[j]);
		cptr_dynamic_dummy = strtok(NULL,"\n");
		j++;
		}
    }
    close(dynamic_mappingfd); 


/*********************** NTP file *****************************************/

/*
	ntpserverfd = open("/etc/ntp_server",O_RDONLY,S_IRUSR);
    if (ntpserverfd == -1)
    {
		ntpserverfd = open("/etc/ntp_server",O_RDWR|O_CREAT,S_IRUSR);
		if (ntpserverfd == -1)
		{
			perror("Error in ntp_server opening:");
		}
		dprintf(ntpserverfd,"/usr/sbin/ntpdate 10.152.156.1\n");
    }
    else
    {
		read(ntpserverfd,ntp_buffer,40);
		cptr_ntp_dummy = strtok(ntp_bufferr,"\n");
		while(cptr_dynamic_dummy != NULL)
		{
			read(ntpserverfd,ntp_buffer,40);
			cptr_ntp_dummy = strtok(ntpbuffer,"\n");
			strcpy(ntp_ip,ntpbuffer);
			cptr_ntp_dummy = strtok(NULL,"\n");
			strcpy(ntp_port,cptr_ntp_dummy);
		}
    }
    close(ntpserverfd); 
*/

/************* End **************************************************/


  for (j = 0; j<12; j++)   //12345
  {
  fan_err_cntr[j] = 0;
  fan_no_err_cntr[j] = 0;
  }
  j=0;

/************ Open the file corresponding to PANEL_OL ***********************************************************************/
    j=0;
    panel_fd = open("/etc/panelconfig",O_RDONLY,S_IRUSR);
    if (panel_fd == -1)
    {
     panel_fd = open("/etc/panelconfig",O_RDWR|O_CREAT,S_IRUSR);
     if (panel_fd == -1)
     {     
      perror("Error in panelconfig openning:");
     }
     wPDU_Parameters[0] = 300;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[0]);
     wPDU_Parameters[1] = 1389;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[1]);
     wPDU_Parameters[2] = 1389;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[2]);
     wPDU_Parameters[3] = 1389;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[3]);
     wPDU_Parameters[4] = 1667;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[4]);
     wPDU_Parameters[5] = 1667;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[5]);
     wPDU_Parameters[6] = 1667;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[6]);
     wPDU_Parameters[7] = 1;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[7]);
     wPDU_Parameters[8] = 63;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[8]);
     wPDU_Parameters[9] = 64;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[9]);
     wPDU_Parameters[10] = 126;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[10]);
     wPDU_Parameters[11] = 127;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[11]);
     wPDU_Parameters[12] = 189;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[12]);
     wPDU_Parameters[13] = 0;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[13]);
     wPDU_Parameters[14] = 3;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[14]);
     wPDU_Parameters[15] = 3;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[15]);
     wPDU_Parameters[16] = 3;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[16]);
     wPDU_Parameters[17] = 120;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[17]);
     wPDU_Parameters[18] = 2400;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[18]);
     wPDU_Parameters[19] = 2400;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[19]); 
     wPDU_Parameters[20] = 5000;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[20]);
     wPDU_Parameters[21] = 150;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[21]);
     wPDU_Parameters[22] = 2083;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[22]);   
     wPDU_Parameters[23] = 2083;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[23]);   
     wPDU_Parameters[24] = 2083;
     dprintf(panel_fd,"%d\n",wPDU_Parameters[24]);
    }
    read(panel_fd,panel_buffer,100);
    cptr_panel_dummy = strtok(panel_buffer,"\n");
    while(cptr_panel_dummy != NULL)
    {
      wPDU_Parameters[j] = atoi(cptr_panel_dummy);
      //printf("wPDU_Parameters[%d] %d\n",j,wPDU_Parameters[j]);
      cptr_panel_dummy = strtok(NULL,"\n");
      j++;
    }
    j=0;

    Data.word.System_Parameter.Panel1_CT_Start = ((0x603 << 16) | (wPDU_Parameters[7]));
    Data.word.System_Parameter.Panel1_CT_End = ((0x604 << 16) | (wPDU_Parameters[8]));
    Data.word.System_Parameter.Panel2_CT_Start = ((0x605 << 16) | (wPDU_Parameters[9]));
    Data.word.System_Parameter.Panel2_CT_End = ((0x606 << 16) | (wPDU_Parameters[10]));
    Data.word.System_Parameter.Panel3_CT_Start = ((0x607 << 16) | (wPDU_Parameters[11]));
    Data.word.System_Parameter.Panel3_CT_End = ((0x608 << 16) | (wPDU_Parameters[12]));
    
    ///Sending always zero AK for testing
    if(wPDU_Parameters[13]==1)
      wPDU_Parameters[13] = 0;

    Data.word.System_Parameter.PanelNo = ((0x60B << 16) | (wPDU_Parameters[13]));//changed to 0 from parameter
    //End
    
    Data.word.System_Parameter.CTPanel_No = ((0x60C << 16) | (((wPDU_Parameters[16] & 0xFF) << 8) | ((wPDU_Parameters[15] & 0xFF) << 4) | (wPDU_Parameters[14] & 0xFF)));
    Data.word.Sys.wPowerCapacity = ((0x103 << 16) | (wPDU_Parameters[0]));
    Data.word.Sys.wNominal_IPVolt = ((0x106 << 16) | (wPDU_Parameters[18]));
    Data.word.Sys.wNominal_OPVolt = ((0x107 << 16) | (wPDU_Parameters[19]));
    wMax_Panel_Limit = (unsigned int)((unsigned long long)(wPDU_Parameters[1] * wPDU_Parameters[21]))/100;
    

/*********************** Create/Open the file corresponding to No_XFMR ******************************************************/

   no_xfmr_fd =open("/etc/no_xfmr",O_RDONLY,S_IRUSR);
	  
   if(no_xfmr_fd == -1)
   {
     no_xfmr_fd = open("/etc/no_xfmr",O_RDWR|O_CREAT,S_IRUSR);

     if ( no_xfmr_fd == -1 )
     {
       		perror("Error in no xfmr file open:");
     } 

     dprintf(no_xfmr_fd,"%d\n",1);
      
   } 

   close(no_xfmr_fd);

   no_xfmr_fd = open("/etc/no_xfmr",O_RDONLY,S_IRUSR);

    if (no_xfmr_fd == -1)
    {
       		perror("Error in no xfmr file open:");
    }

    read(no_xfmr_fd,buffer1,3);
    no_xfmr = atoi(buffer1);
    Data.word.System_Parameter.no_xfmr = ((0x602 << 16) | (no_xfmr));
    close(no_xfmr_fd);

/*************************************************END*************************************************************************/          
/***Start Signature reading*****/
    do
    {
      read_fd =open("/etc/signature",O_RDONLY|S_IRUSR);
	  
      if(read_fd == -1) 
      {
		if(!(flag & 1))
		{
		perror("log file open\n");
		printf("\n*******signature byte is missing*****\n");
		flag |= 1;

		}
      }
      else
      {
		flag &= ~(1);
		ret_in = read(read_fd,buffer,3);
		signature = atoi(buffer);
		printf("signature is :%d\n",signature);  
      }

      close(read_fd);
	}while(flag);
	   
 /***Start modbus ID reading*****/
	buffer[0]=0;
    buffer[1]=0;
    buffer[2]=0;
    buffer[3]=0;
    buffer[4]=0;
    buffer[5]=0;
    do
    {
      read_fd =open("/etc/slaveid",O_RDONLY|S_IRUSR);
      if(read_fd == -1) 
      {
		if(!(flag & 1))
		{
		  perror("log file open\n");
		  printf("\n*******slaveid is missing*****\n");
		  flag |= 1;
		}
      }
      else
      {
		flag &= ~(1);
		ret_in = read(read_fd,buffer,3);
		slave_id = atoi(buffer);
		Data.word.System_Parameter.SlaveID = ((0x60E << 16) | (slave_id));
		printf("slaveid is :%d\n",slave_id);       
      }

        close(read_fd);
    }while(flag);

    create_pkt_pool(100,Rx_PKT);
    sem_init(&semaphore_1,0,0);

    if ( p_db1 == NULL )
    {
      p_db1 = icos_mbusdb_init (1);//Shared Memory
    }

    create_db_field(p_db1,SHM_FIELD);//Shared memory
    system("sleep 30s");


/**************Copying FW Version*********/
    Data.word.Sys.wSys_Fw_Ver[0] = (0x12C0000 | ((unsigned int)wMboard_Fw_Ver[0]<<8) | ((unsigned int)wMboard_Fw_Ver[1]));
    Data_nv.word.Sys.wSys_Fw_Ver[0] = (0x12C0000 | ((unsigned int)wMboard_Fw_Ver[0]<<8) | ((unsigned int)wMboard_Fw_Ver[1]));
    Reg[STATUS_UPDATE + 44].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[0] & 0x0000FFFF);
    Data.word.Sys.wSys_Fw_Ver[1] = (0x12D0000 | ((unsigned int)wMboard_Fw_Ver[2]<<8) | ((unsigned int)wMboard_Fw_Ver[3]));
    Data_nv.word.Sys.wSys_Fw_Ver[1] = (0x12D0000 | ((unsigned int)wMboard_Fw_Ver[2]<<8) | ((unsigned int)wMboard_Fw_Ver[3]));
    Reg[STATUS_UPDATE + 45].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[1] & 0x0000FFFF);
    Data.word.Sys.wSys_Fw_Ver[2] = (0x12E0000 | ((unsigned int)wMboard_Fw_Ver[4]<<8) | ((unsigned int)wMboard_Fw_Ver[5]));
    Data_nv.word.Sys.wSys_Fw_Ver[2] = (0x12E0000 | ((unsigned int)wMboard_Fw_Ver[4]<<8) | ((unsigned int)wMboard_Fw_Ver[5]));
    Reg[STATUS_UPDATE + 46].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[2] & 0x0000FFFF);
    Data.word.Sys.wSys_Fw_Ver[3] = (0x12F0000 | ((unsigned int)wMboard_Fw_Ver[6]<<8) | ((unsigned int)wMboard_Fw_Ver[7]));
    Data_nv.word.Sys.wSys_Fw_Ver[3] = (0x12F0000 | ((unsigned int)wMboard_Fw_Ver[6]<<8) | ((unsigned int)wMboard_Fw_Ver[7]));
    Reg[STATUS_UPDATE + 47].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[3] & 0x0000FFFF);
    Data.word.Sys.wSys_Fw_Ver[4] = (0x1300000 | ((unsigned int)wMboard_Fw_Ver[8]<<8) | ((unsigned int)wMboard_Fw_Ver[9]));
    Data_nv.word.Sys.wSys_Fw_Ver[4] = (0x1300000 | ((unsigned int)wMboard_Fw_Ver[8]<<8) | ((unsigned int)wMboard_Fw_Ver[9]));
    Reg[STATUS_UPDATE + 48].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[4] & 0x0000FFFF);   
    Data.word.Sys.wSys_Fw_Ver[5] = (0x1310000 | ((unsigned int)wMboard_Fw_Ver[10]<<8) | ((unsigned int)wMboard_Fw_Ver[11]));
    Data_nv.word.Sys.wSys_Fw_Ver[5] = (0x1310000 | ((unsigned int)wMboard_Fw_Ver[10]<<8) | ((unsigned int)wMboard_Fw_Ver[11]));
    Reg[STATUS_UPDATE + 49].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[5] & 0x0000FFFF);
    Data.word.Sys.wSys_Fw_Ver[6] = (0x1320000 | ((unsigned int)wMboard_Fw_Ver[12]<<8) | ((unsigned int)wMboard_Fw_Ver[13]));
    Data_nv.word.Sys.wSys_Fw_Ver[6] = (0x1320000 | ((unsigned int)wMboard_Fw_Ver[12]<<8) | ((unsigned int)wMboard_Fw_Ver[13]));
    Reg[STATUS_UPDATE + 50].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[6] & 0x0000FFFF);
    Data.word.Sys.wSys_Fw_Ver[7] = (0x1330000 | ((unsigned int)wMboard_Fw_Ver[14]<<8) | ((unsigned int)wMboard_Fw_Ver[15]));
    Data_nv.word.Sys.wSys_Fw_Ver[7] = (0x1330000 | ((unsigned int)wMboard_Fw_Ver[14]<<8) | ((unsigned int)wMboard_Fw_Ver[15]));
    Reg[STATUS_UPDATE + 51].reg_d.reg_value = (Data.word.Sys.wSys_Fw_Ver[7] & 0x0000FFFF);

/**************Copying IP Number*********/
    ip_fd = socket(AF_INET,SOCK_DGRAM,0);
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name,"eth0",IFNAMSIZ-1);
    ioctl(ip_fd,SIOCGIFADDR,&ifr);
    close(ip_fd);
    ipv4_addr =  (unsigned int)(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr);
    sip_ptr = (unsigned short*)&ipv4_addr;
    Data.word.Sys.IPAddress[0] = ((0x136 << 16 ) | (*sip_ptr));
    printf("\nIPAdress1 : %x\n",Data.word.Sys.IPAddress[0]);
    Data_nv.word.Sys.IPAddress[0] = ((0x136 << 16 ) | (*sip_ptr));
    Reg[STATUS_UPDATE + 53].reg_d.reg_value = (Data.word.Sys.IPAddress[0] & 0x0000FFFF);
    
    *++sip_ptr;
    Data.word.Sys.IPAddress[1] = ((0x137 << 16 ) | (*sip_ptr));
    printf("\nIPAdress2 : %x\n",Data.word.Sys.IPAddress[1]);
    Data_nv.word.Sys.IPAddress[0] = ((0x136 << 16 ) | (*sip_ptr));
    Reg[STATUS_UPDATE + 54].reg_d.reg_value = (Data.word.Sys.IPAddress[1] & 0x0000FFFF);

	printf("before printing\n");

/*
	unsigned char bytes[4];
	bytes[0]= *sip_ptr & 0xFF;
	bytes[1]=(*sip_ptr >> 8) & 0xFF;
	
	bytes[2]=(*sip_ptr >> 16) & 0xFF;
	bytes[3]=(*sip_ptr >> 24) & 0xFF;
	printf("IP Address : %d.%d.%d.%d\n",bytes[2],bytes[3],bytes[0],bytes[1]);

	printf("After printing\n");
	*/

  /*********************END*****************/ 
  
  
  /***************** creating a new /etc/kwhdata file if it doesn't exit ************/

	int result_kwh;
	int kwhdata_check_fd;
	int kwhdata_fd;
	int kwh_fd;
	int kwhtxt_fd;

    kwhdata_fd = open("/etc/kwhvaldata",O_RDONLY,S_IRUSR);
	if(kwhdata_fd == -1)
	{
		kwhdata_fd = open("/etc/kwhvaldata",O_RDWR|O_CREAT|O_TRUNC,0600);
		if(kwhdata_fd == -1)
		{
		  perror("mem file:");
		  exit(EXIT_FAILURE);
		}
		result_kwh = lseek(kwhdata_fd,FILESIZE_KWHDATA-1,SEEK_SET);
		if(result_kwh == -1)
		{
		  close(kwhdata_fd);
		  perror("error calling lseek");
		  exit(EXIT_FAILURE);
		}
		result_kwh = write(kwhdata_fd,"A",1);
		if(result_kwh != 1)
		{
		  close(kwhdata_fd);
		  perror("error writing last byte of the file");
		  exit(EXIT_FAILURE);
		}
		map_etc = (int*)mmap(0,FILESIZE_KWHDATA,PROT_READ|PROT_WRITE,MAP_SHARED,kwhdata_fd,0);  
		if(map_etc == MAP_FAILED)
		{
		  close(kwhdata_fd);
		  perror("Error mapping the file");
		  exit(EXIT_FAILURE);
		}
		close(kwhdata_fd);    
  	
  /********* kwh values are taken from text file and written to map_etc hence /etc/kwhdata *********/	
    
		i = 0;
		kwhtxt_fd = open("/etc/sysIPkwhvalue",O_RDWR|S_IRUSR);
		if(kwhtxt_fd == -1)
		{
			while(i < 2)
			{
				*(map_etc + i) = 0;
				i++;
			}	
		}
		else
		{
			read(kwhtxt_fd,kwh_buffer,4500);   
			kwh_dummy = strtok(kwh_buffer,"\n");
			while( (kwh_dummy != NULL)&& (i<2) )
			{
				*(map_etc + i) = atoi(kwh_dummy);
				i++;
				kwh_dummy = strtok(NULL,"\n");
			}
		}
   		close(kwhtxt_fd);
   
		kwhtxt_fd = open("/etc/sysOPkwhvalue",O_RDWR|S_IRUSR);
		if(kwhtxt_fd == -1)
		{
			while(i < 8)
			{
				*(map_etc + i) = 0;
				i++;
			}	
		}
		else
		{
			read(kwhtxt_fd,kwh_buffer,5000);   
			kwh_dummy = strtok(kwh_buffer,"\n");
			while( (kwh_dummy != NULL)&& (i<8) )
			{
				*(map_etc + i) = atoi(kwh_dummy);
				i++;
				kwh_dummy = strtok(NULL,"\n");
			}
		}
		close(kwhtxt_fd);

		kwhtxt_fd = open("/etc/kwhvalue",O_RDWR|S_IRUSR);
		if(kwhtxt_fd == -1)
		{
			while(i < 344)
			{
				*(map_etc + i) = 0;
				i++;
			}	
		}
		else
		{	
			read(kwhtxt_fd,kwh_buffer,5000);   
			kwh_dummy = strtok(kwh_buffer,"\n");
			while( (kwh_dummy != NULL)&& (i<344) )
			{
				*(map_etc + i) = atoi(kwh_dummy);
				i++;
				kwh_dummy = strtok(NULL,"\n");
			}
		}
		close(kwhtxt_fd);   
   
		kwhtxt_fd = open("/etc/sysIPkwherrvalue",O_RDWR|S_IRUSR);
		if(kwhtxt_fd == -1)
		{
			while(i < 345)
			{
				*(map_etc + i) = 0;
				i++;
			}	
		}
		else
		{
			read(kwhtxt_fd,kwh_buffer,5000);   
			kwh_dummy = strtok(kwh_buffer,"\n");
			while( (kwh_dummy != NULL)&& (i<345) )
			{
				*(map_etc + i) = atoi(kwh_dummy);
				i++;
				kwh_dummy = strtok(NULL,"\n");
			}
		}
		close(kwhtxt_fd);
   
		kwhtxt_fd = open("/etc/sysOPkwherrvalue",O_RDWR|S_IRUSR);
		if(kwhtxt_fd == -1)
		{
			while(i < 348)
			{
				*(map_etc + i) = 0;
				i++;
			}
		}
		else
		{
			read(kwhtxt_fd,kwh_buffer,5000);   
			kwh_dummy = strtok(kwh_buffer,"\n");
			while( (kwh_dummy != NULL)&& (i<348) )
			{
				*(map_etc + i) = atoi(kwh_dummy);
				i++;
				kwh_dummy = strtok(NULL,"\n");
			}
		}
		close(kwhtxt_fd);
   
		kwhtxt_fd = open("/etc/kwherrvalue",O_RDWR|S_IRUSR);
		if(kwhtxt_fd == -1)
		{
			while(i < 516)
			{
				*(map_etc + i) = 0;
				i++;
			}
		}
		else
		{
			read(kwhtxt_fd,kwh_buffer,5000);   
			kwh_dummy = strtok(kwh_buffer,"\n");
			while( (kwh_dummy != NULL)&& (i<516) )
			{
				*(map_etc + i) = atoi(kwh_dummy);
				i++;
				kwh_dummy = strtok(NULL,"\n");
			}
		}
		i = 0;
		close(kwhtxt_fd);

        //Writing signature byte
		*(map_etc + 518 - 2) = 0xAA55;
  
		if (msync((void *)map_etc, FILESIZE_KWHDATA, MS_SYNC) < 0)
		{
			perror("msync");
			exit(1);
		}
	}

	/****************** end ***********************************/ 

 	/**********New process******************/
 	
    printf("\nNew process starts\n");
    if (pipe(fd1)==-1) 
    { 
      fprintf(stderr, "Pipe1 Failed" ); 
      return 1; 
    }
    if (pipe(fd2)==-1) 
    { 
      fprintf(stderr, "Pipe2 Failed" ); 
      return 1; 
    }  
    if (pipe(fd3)==-1) 
    { 
	  fprintf(stderr, "Pipe3 Failed" ); 
      return 1; 
    } 
    if (pipe(fd4)==-1) 
    { 
      fprintf(stderr, "Pipe4 Failed" ); 
      return 1; 
    }
    if (pipe(fd5) == -1)
    {
      fprintf(stderr, "Pipe5 Failed");
      return 1;
    }
    if (pipe(fd7)==-1) 
    { 
      fprintf(stderr, "Pipe7 Failed" ); 
      return 1; 
    }   
    if (pipe(fd8)==-1) 
    { 
      fprintf(stderr, "Pipe8 Failed" ); 
      return 1; 
    }
    if (pipe(fd9)==-1) 
    { 
      fprintf(stderr, "Pipe9 Failed" ); 
      return 1; 
    }   
    if (pipe(fd10)==-1) 
    { 
      fprintf(stderr, "Pipe10 Failed" ); 
      return 1; 
    }   

    /*************added for creating process pid file********************/
    
    int processpid_fd;
    
	processpid_fd = open("/etc/process_pid",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR);
	if (processpid_fd == -1)
	{     
	   perror("Error in process_id openning:");
	}
    adc_pid = fork();
    if(adc_pid == 0)
    {
      dprintf(processpid_fd,"ADC process PID : %d\n",getpid());
      printf("ADC process PID : %d\n", getpid());
      ADC_process();
    }
    config_pid = fork();
    if(config_pid == 0)
    {
      dprintf(processpid_fd,"Config process PID : %d\n",getpid());
      printf("Config process PID : %d\n", getpid());
      config_process();
    }
    event_pid = fork();
    if(event_pid == 0)
    {
      dprintf(processpid_fd,"Event process PID : %d\n",getpid());
      printf("Event process PID : %d\n", getpid());   
      event_process();
    }
    if(signature & (1<<4))
    {
      BACnet_pid = fork();
      if(BACnet_pid == 0)
      {
		dprintf(processpid_fd,"BACnet process PID : %d\n",getpid());
		printf("BACnet process PID : %d\n", getpid());
        BACnet_process();
      }
    }
    close(processpid_fd);


	for (i=0; i<64; i++)
	  event_flag[i] = 0;
/**********End ***************/  
    maketimer(&secondTimerID); //10ms
    maketimer(&firstTimerID); //10ms
    maketimer(&thirdTimerID); //10ms
    maketimer(&fourthTimerID); // 520us
    maketimer(&sixthTimerID);
	maketimer(&seventhTimerID);
	maketimer(&eighthTimerID);
    //maketimer(&calibration_timerID);   // 1s
    //maketimer(&S_Board_Update_timer);  // 5s

	if(pthread_create(&thread_w,NULL,write_message,NULL))
	{
	  printf("thread create error\n");
	  return -1;
	}
	if ( pthread_create(&p_thread, NULL, handler_func, (void*)&p_ctx) != 0 )
	{
	  return (-1);
	}
	/*if (pthread_create(&uart_thread, NULL, uart_handler, (void*)&p_ctx1) != 0)
	{
	  return -1;
	}
	if (pthread_create(&db_thread, NULL, db_filling, NULL))
	{
	 printf("DB thread failed creation\n");
	 return -1;
	}*///DK removed for testing
    if (pthread_create(&shm_thread, NULL, shm_filling, NULL))
	{
	 printf("DB thread failed creation\n");
	 return -1;
	}
	pthread_join(thread_w,NULL);
	pthread_join(p_thread,NULL);
    pthread_join(shm_thread,NULL);
    //pthread_join(uart_thread,NULL);
	//pthread_join(db_thread,NULL);//DK removed for testing
    return 0;
}


void DB_entry_creation_for(unsigned int base,unsigned int length,unsigned short reg_type,NOS_DB_HANDLE  *p_db_dummy)
{
    unsigned int i;
    printf("\nEntry : %d\n",if_index_start);
    for (i=0;i<=(length);i++)
    {
	  icos_mbusdb_create_reg_interface( p_db_dummy, if_index_start,
	  modbus_bus_no, slave_id, base,  reg_type);
	  if_index_start++;
	  base++;
    }       

}
void create_db_field(NOS_DB_HANDLE  *p_db,int filed)
{
    int i;
    if (!icos_mbusdb_if_index_exists(p_db, if_index_cur)) 
    {
      printf("if index not existing - bus = %d\n", modbus_bus_no);

     	
      DB_entry_creation_for(0x0800,FIELD_TOTALENTRY,INPUT_REGISTER,p_db);//989 entries
#if 0
      DB_entry_creation_for(0x0900,0x2B,INPUT_REGISTER);
      DB_entry_creation_for(0x0002,0x01,INPUT_REGISTER);
      DB_entry_creation_for(0x0100,0x37,INPUT_REGISTER);
      DB_entry_creation_for(0x10A0,0x07,INPUT_REGISTER);
      DB_entry_creation_for(0x0600,0x0E,INPUT_REGISTER);	
      //DB_entry_creation_for(0x0A00,0x15,INPUT_REGISTER);

      DB_entry_creation_for(0x0D00,0x55,INPUT_REGISTER);      // Branch Current
      DB_entry_creation_for(0x4D00,0x55,INPUT_REGISTER);
      /*DB_entry_creation_for(0x8D00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xCD00,0x55,INPUT_REGISTER);*/

      DB_entry_creation_for(0x1700,0x55,INPUT_REGISTER);     // Branch Kw
      DB_entry_creation_for(0x5700,0x55,INPUT_REGISTER);
      /*DB_entry_creation_for(0x9700,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xD700,0x55,INPUT_REGISTER);*/

      DB_entry_creation_for(0x1900,0x55,INPUT_REGISTER);     // Branch Load %
      DB_entry_creation_for(0x5900,0x55,INPUT_REGISTER);
      /*DB_entry_creation_for(0x9900,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xD900,0x55,INPUT_REGISTER);*/

      DB_entry_creation_for(0x2D00,0xA8,INPUT_REGISTER);     // Branch KWH
      DB_entry_creation_for(0x6D00,0xA8,INPUT_REGISTER);
      /*DB_entry_creation_for(0xAD00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xED00,0x55,INPUT_REGISTER);*/

      /*DB_entry_creation_for(0x0A00,0x16,INPUT_REGISTER);     // Max_Min_Parameters

      DB_entry_creation_for(0x1600,0x55,INPUT_REGISTER);     // Branch Max Current Demand
      DB_entry_creation_for(0x5600,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0x9600,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xD600,0x55,INPUT_REGISTER);

      DB_entry_creation_for(0x0700,0x55,INPUT_REGISTER);     // Branch Max KW Demand
      DB_entry_creation_for(0x4700,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0x8700,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xC700,0x55,INPUT_REGISTER);

      DB_entry_creation_for(0x1800,0x55,INPUT_REGISTER);     // Branch Max Current 
      DB_entry_creation_for(0x5800,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0x9800,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xD800,0x55,INPUT_REGISTER);

      DB_entry_creation_for(0x0E00,0x55,INPUT_REGISTER);     // Branch Min Current
      DB_entry_creation_for(0x4E00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0x8E00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xCE00,0x55,INPUT_REGISTER);

      DB_entry_creation_for(0x0B00,0x55,INPUT_REGISTER);     // Branch current Demand 
      DB_entry_creation_for(0x4B00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0x8B00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xCB00,0x55,INPUT_REGISTER);

      DB_entry_creation_for(0x0C00,0x55,INPUT_REGISTER);     // Branch KW Demand
      DB_entry_creation_for(0x4C00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0x8C00,0x55,INPUT_REGISTER);
      DB_entry_creation_for(0xCC00,0x55,INPUT_REGISTER);*/
#endif
      /*if(filed == DB_FIELD )
      {
        icos_mbusdb_perform_cmd(p_db,"CREATE INDEX ifindex_ix ON object_data_table (object_index)");
      }*///DK removed

      printf("\n DB Entry created\n");
    }
    else {
        printf("if index existing\n");
    }
}

/*
 * Thread handler function
 * Passed as an argument while creating the thread
 */ 
#if 0
void *uart_handler (void *ctx1)
{
	char read_buffer[5];
	THREAD_CTX thread1_ctx;
	int ret1 = -1;
	fd_set temp1_RD_fd;

	FD_SET(uart_fd,&(p_ctx1.read_fds));
	p_ctx1.max_fd = uart_fd + 1;
	p_ctx1.uart_fd = uart_fd;

	if ( ctx1 == NULL )
	{
	return (NULL);
	}

	thread1_ctx = *((THREAD_CTX *)ctx1);

	while(1)
	{
	  temp1_RD_fd = thread1_ctx.read_fds;
	  ret1 = select(thread1_ctx.max_fd,&temp1_RD_fd,NULL,NULL,NULL);

	  if ( ret1 == -1 )
	  {

	    continue;
	  } 
	  else if ( ret1 == 0 )
	  {
	    printf("no fd set\n");
	    continue;
	  } 
	  else
	  {
	    if ( FD_ISSET(thread1_ctx.uart_fd,&temp1_RD_fd))
	    {
		settimer(&S_Board_Update_timer,2,0);
		ret1 = read(uart_fd, &read_buffer,5);
               if (ret1 == -1)
                  perror("Error in reading");
               else
                  {
		     if (read_buffer[0] == 0x00)
	             {
			if ((read_buffer[1] >= 0x1) && (read_buffer[1] <= 0x54))
			{
			   SBoard1[read_buffer[1] - 1] = (short)((read_buffer[2] << 8) | (read_buffer[3]));
			}
			   
		        /*else if ((read_buffer[1] >= 0x55)&&(read_buffer[1] <= 0xA8))
			{
			   SBoard2[read_buffer[1] - 1] = (short)((read_buffer[2] << 8) | (read_buffer[3]));
			}*/  
	             }
	             else if (read_buffer[0] == 0x01)
		     {
		        if ((read_buffer[1] >= 0x1) && (read_buffer[1] <= 0x54))
			   SBoard1[(read_buffer[1] - 1) + 0x54] = (short)((read_buffer[2] << 8) | (read_buffer[3]));
		        /*else if ((read_buffer[1] >= 0x55)&&(read_buffer[1] <= 0xA8))
			   SBoard2[(read_buffer[1] - 1) + 0x54] = (short)((read_buffer[2] << 8) | (read_buffer[3]));*/
		     }
		     else if (read_buffer[0] == 0x02)
		     {
			System_gain[read_buffer[1] - 1] = (short)((read_buffer[2] << 8) | (read_buffer[3]));
			/*printf("\nOffset : %d\tValue : %d\n",(read_buffer[1] - 1),((read_buffer[2] << 8) | (read_buffer[3])));*/
		     }
		  }
	     }
            else 
	     {
		printf("\nUnkown FD\n");
	     }
	  	
	   }
	}

	
}
#endif

void *handler_func (void *ctx )
{
    unsigned int dummy;
	THREAD_CTX  thread_ctx;
        //char read_buffer[5];
	fd_set temp_RD_fd,temp_WR_fd;
	int ret = -1;
	int nbytes;
	sensor_PKT *p_ptr = NULL;
	int len,len1,len_dummy ;//DK for checking;
	int s =-1;
	int *dat;
	int value = -1;
	int i;
    int uart;
    int j = 0;
	int dry = 0;
	int dry1 = 0;
	static int fd2_count = 0;
	static int fd3_count = 0;
	static int fd4_count = 0;
	static int fd8_count = 0;
	static int fd9_count = 0;
	static int fd10_count = 0;
	unsigned char buffer2[4] = {0,0,0,0};
	unsigned char dry_buffer[50];
	unsigned char dry_buffer1[24]; 
	unsigned char *cptr_dry_dummy;

	sock_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	printf("CAN socket is %d\n",sock_can);
	if ( sock_can == -1 )
	{
	  perror("Error can raw socket creation:");
	}
	strcpy(ifr.ifr_name, "can1" );
	ioctl(sock_can, SIOCGIFINDEX, &ifr);
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(sock_can, (struct sockaddr *)&addr, sizeof(addr));

	if(st_can_filter(0x7F7,CAN_SFF_MASK,3) == -1)
	{
	perror("Error in can filter setting:");
	}
	  
	FD_ZERO(&(p_ctx.read_fds));
	FD_SET(fd1[0],&(p_ctx.read_fds));
	FD_SET(fd2[0],&(p_ctx.read_fds));
	FD_SET(fd3[0],&(p_ctx.read_fds));
	FD_SET(fd4[0],&(p_ctx.read_fds));
	/***************For system KWH - start*******************/
	FD_SET(fd7[0],&(p_ctx.read_fds));
	FD_SET(fd8[0],&(p_ctx.read_fds));
	FD_SET(fd9[0],&(p_ctx.read_fds));
	FD_SET(fd10[0],&(p_ctx.read_fds));
	/***************End**************************************/
        //FD_SET(uart_fd,&(p_ctx.read_fds));
	FD_SET(sock_can,&(p_ctx.read_fds));
	p_ctx.max_fd = sock_can+1;
	p_ctx.sock_adc = fd1[0];
	p_ctx.sock_config = fd2[0];
	p_ctx.sock_initKWHerr = fd3[0];
	p_ctx.sock_initKWH = fd4[0];
	/***************For system KWH - start*******************/
	p_ctx.sock_initIPsysKWHerr = fd7[0];
	p_ctx.sock_initIPsysKWH    = fd8[0];
	p_ctx.sock_initOPsysKWHerr = fd9[0];
	p_ctx.sock_initOPsysKWH    = fd10[0];

	/***************End *************************************/
	p_ctx.sock_can = sock_can;
        //p_ctx.uart_fd  = uart_fd;

	if ( ctx == NULL )
	{
	return (NULL);
	}

	thread_ctx = *((THREAD_CTX *)ctx);

	SetDefault();
	settimer(&fourthTimerID,0,10); //
	settimer(&seventhTimerID,0,30);
	settimer(&sixthTimerID,0,5);
	settimer(&eighthTimerID,0,60000);
	//settimer(&calibration_timerID,1,0);
	 
	while(1)
	{

	  /******************* Reading the DRY Contact Configuration ****/
		j=0;
	  dry_contact_fd = open("/etc/dry_contact",O_RDONLY,S_IRUSR);
	  if (dry_contact_fd == -1)
	  {
            dry_contact_fd = open("/etc/dry_contact",O_RDONLY|O_CREAT,S_IRUSR);
	    if(dry_contact_fd == -1)
            {
	      perror("Error opening dry_contact file");
            }
	  }
	  read(dry_contact_fd,dry_buffer,50);
	  cptr_dry_dummy = strtok(dry_buffer,"\n");
	  while(cptr_dry_dummy != NULL)
	   {
	     dry_contact_flag[dry] = atoi(cptr_dry_dummy);
	     //printf("\nThe details of Dry contact %d : %d\n",dry+1,dry_contact_flag[dry]);
	     cptr_dry_dummy = strtok(NULL,"\n");
	     dry++;
	   }
	  dry=0;
	  close(dry_contact_fd);
	  

	  dry_contact_on_off_fd = open("/etc/dry_contact_ON_OFF",O_RDONLY,S_IRUSR);
	  if (dry_contact_on_off_fd == -1)
	  {
		dry_contact_on_off_fd = open("/etc/dry_contact_ON_OFF",O_RDONLY|O_CREAT,S_IRUSR);
		if (dry_contact_on_off_fd == -1)
                {
	  	   perror("Error in dry_contact_ON_OFF file open:");
                }	
	  }
	  read(dry_contact_on_off_fd,dry_buffer1,24);
	  cptr_dry_dummy = strtok(dry_buffer1,"\n");
	  while(cptr_dry_dummy != NULL)
	  {
	     dry_on_off_flag[dry1] = atoi(cptr_dry_dummy);
	     //printf("\nThe details of Dry contact %d : %d\n",dry1+1,dry_on_off_flag[dry1]);
	     cptr_dry_dummy = strtok(NULL,"\n");
	     dry1++;
	  }
	  dry1 = 0;
	  close(dry_contact_on_off_fd);
    

	  /******************* END **************************************/

	  /************ Open the file corresponding to THD_Parameters ***********************************************************************/
	j=0;
    thd_fd = open("/etc/thdparameters",O_RDONLY,S_IRUSR);
    if (thd_fd == -1)
    {
     thd_fd = open("/etc/thdparameters",O_RDWR|O_CREAT,S_IRUSR);
     if (thd_fd == -1)
     {     
      perror("Error in thdparameters openning:");
     }
     wTHD_Parameters[0] = 50;
     dprintf(thd_fd,"%d\n",wTHD_Parameters[0]);
     wTHD_Parameters[1] = 150;
     dprintf(thd_fd,"%d\n",wTHD_Parameters[1]);
     wTHD_Parameters[2] = 75;
     dprintf(thd_fd,"%d\n",wTHD_Parameters[2]);
          
    }
    read(thd_fd,thd_buffer,100);
    cptr_thd_dummy = strtok(thd_buffer,"\n");
    while(cptr_thd_dummy != NULL)
    {
      wTHD_Parameters[j] = atoi(cptr_thd_dummy);
      //printf("wTHD_Parameters[%d] %d\n",j,wTHD_Parameters[j]);
      cptr_thd_dummy = strtok(NULL,"\n");
      j++;
    }
    j=0;
    close(thd_fd);

	wSec_Thd.word.wOverVoltTHD = (wTHD_Parameters[0] & 0xFFFF) ;  //*10
	wSec_Thd.word.wOverCurrTHD = (wTHD_Parameters[1] & 0xFFFF) ;   //*10
	wSec_Thd.word.wPowerfactor = (wTHD_Parameters[2] & 0xFFFF) ;   //*100
    

/************************************ END ***********************************************************************************/ 
	  /******************* Reading Buzzer Silence File *************/

	  buzz_silence_fd = open("/tmp/buzz_silence",O_RDONLY,S_IRUSR);

	  if (buzz_silence_fd == -1)
	  {
		perror("Error in buzz silence file open:");
	  }
	  read(buzz_silence_fd,buffer2,3);
	  buzz_status = atoi(buffer2);
	  close(buzz_silence_fd);

	  /****************************** END *************************/
	  
	  temp_RD_fd = thread_ctx.read_fds;
	  ret = select(thread_ctx.max_fd,&temp_RD_fd,NULL,NULL,NULL);

	  if ( ret == -1 )
	  {

	    continue;
	  } else if ( ret == 0 )
	  {
	    printf("no fd set\n");
	    continue;
	  } else
	  {
	    if ( FD_ISSET(thread_ctx.sock_adc,&temp_RD_fd) ) 
	    {
		  read(fd1[0], &status_result, 4); 
		  Data.word.System_Status.Fan_status = (unsigned short) ((status_result) & fan_bits);  //0x00ff changes to fan bits
                  
			if (no_xfmr)
			{
				if ((Data.word.System_Status.Fan_status == 0))
				{ 
					if(event_flag[3])
					{
						dummy = 4;
						write(fd5[1],&dummy,1);
						event_flag[3] = 0;
						alarm_status &= ~(1 << FAN_FAILURE);
						alarm_flag = 0;
					}
				}
				else if (Data.word.System_Status.Fan_status != 0)
				{
					
					if(!event_flag[3])
					{
						dummy = 3;
						write(fd5[1],&dummy,1);
						event_flag[3] = 1;
						alarm_status |= (1 << FAN_FAILURE);
						alarm_flag = 0;
					}
				
				}
			}	 
	    }
	    else if ( FD_ISSET(thread_ctx.sock_config,&temp_RD_fd) ) 
	    {
	      ret = read(fd2[0], &Data_In.word.Max_Min_Limit[0][(fd2_count*21)], (21*(sizeof(unsigned int))));
		  fd2_count++;
		  if(fd2_count == 8)
		    fd2_count = 0;
	    }
	    else if ( FD_ISSET(thread_ctx.sock_initKWHerr,&temp_RD_fd) ) 
	    {
	      ret = read(fd3[0], &Data_nv.array[SYSTEM_GAINS + fd3_count*21], (21*(sizeof(unsigned int))));
	      fd3_count++;
		  if(fd3_count == 8)
		    fd3_count = 0;
	    }
	    else if ( FD_ISSET(thread_ctx.sock_initKWH,&temp_RD_fd) ) 
	    {
		  ret = read(fd4[0], &Data.array[PANEL24_OVERCURR_STATUS_FLAG + fd4_count*21], (21*(sizeof(unsigned int))));
		  fd4_count++;
		  if(fd4_count == 16)
		    fd4_count = 0;
	    }
		  /***************For system KWH - start*******************/
	    else if ( FD_ISSET(thread_ctx.sock_initIPsysKWHerr,&temp_RD_fd) ) 
	    {
		  ret = read(fd7[0], &Data_nv.array[KWH1_ERROR], ((sizeof(unsigned int))));

	    }
	    else if ( FD_ISSET(thread_ctx.sock_initIPsysKWH,&temp_RD_fd) ) 
	    {
		  ret = read(fd8[0], &Data.array[10 + fd8_count], (2*(sizeof(unsigned int))));
		  fd8_count++;
		  if (fd8_count == 2)
		  fd8_count = 0;
	    }
	    else if ( FD_ISSET(thread_ctx.sock_initOPsysKWHerr,&temp_RD_fd) ) 
	    {
		  ret = read(fd9[0], &Data_nv.array[KWH1_ERROR + 1 + fd9_count], (3*(sizeof(unsigned int))));

		  fd9_count++;
		  if (fd9_count == 3)
		  fd9_count =0;
	    }
	    else if ( FD_ISSET(thread_ctx.sock_initOPsysKWH,&temp_RD_fd) ) 
	    {
		  ret = read(fd10[0], &Data.array[PRI_OFFSET + 10 + fd10_count], (6*(sizeof(unsigned int))));

		  fd10_count++;
		  if (fd10_count == 6)
		  fd10_count = 0;
	    }
		  /***************End**************************************/
	    else if ( FD_ISSET(thread_ctx.sock_can,&temp_RD_fd) ) 
	    {

		  p_ptr = copy_CAN_Rx_to_buffer_2(p_ptr);
		  if (p_ptr == NULL) {
		    continue;
	      }

		  nbytes = read(sock_can, 
		  &p_ptr->sensor_data[p_ptr->frame_no], 
		  sizeof(struct can_frame));

		  if (nbytes < 0) {
		    perror("can raw socket read");
		    continue;
		  }

		/* paranoid check ... */
		  if (nbytes < sizeof(struct can_frame)) 
		  {
		    fprintf(stderr, "read: incomplete CAN frame\n");
		    continue;
		  }
	
		  dat = (unsigned int *)p_ptr->sensor_data[p_ptr->frame_no].data;
		  if (*dat == 1)
		  {
		    dummy = 7;
		    write(fd5[1],&dummy,1);
		    alarm_flag = 1;                 // Buzzer disable activated here  
		  }
		  p_ptr->frame_no += 1;
		  if(p_ptr->frame_no >= FRAME_LIMIT)
		  {
		    cleartimer(&secondTimerID);

		    pthread_mutex_lock(&lock1);
		    enqueue_pkt(p_ptr,Tx_PKT);

		    pthread_mutex_unlock(&lock1);
		    p_ptr = NULL;   
		    ret = sem_post(&semaphore_1);
		    if(ret == -1) 
		    {
		      continue;
		    }
	      }
	    } 
	    else
	    {
	      printf("unknown fd\n");
	    }
	  }
	}
	return NULL;   
}

int st_can_filter(canid_t id,canid_t mask_t,int filter_no)
{
	if(filter_no > 3)
	{
	return -1;
	}
	board_filter[0].can_id = id;
	board_filter[0].can_mask = mask_t; //CAN_SFF_MASK;//0x7FF;
	board_filter[1].can_id = 0x7FB;
	board_filter[1].can_mask = mask_t;//0x7FF;
	board_filter[2].can_id = 0x7FD;
	board_filter[2].can_mask = mask_t;//0x7FF;
	setsockopt(sock_can,SOL_CAN_RAW,CAN_RAW_FILTER,&board_filter,sizeof(board_filter));

}

int copy_CAN_Rx_to_buffer(void)
{
	if(!flag)
	{

	  pthread_mutex_lock(&lock2);
	  pkt_ptr = release_pkt_from_free_pool(Rx_PKT);
	  pthread_mutex_unlock(&lock2);
	  settimer(&secondTimerID, 0, 10);
	  flag =1;
	  pkt_ptr->frame_no = 0;
	}
	return 0;
}

sensor_PKT *copy_CAN_Rx_to_buffer_2(sensor_PKT *p_ptr)
{

	if((p_ptr == NULL))
	{

	  pthread_mutex_lock(&lock2);
	  p_ptr = release_pkt_from_free_pool(Rx_PKT);
	  pthread_mutex_unlock(&lock2);

	  flag = 1;

	  settimer(&secondTimerID, 0, 10);
	  if (p_ptr != NULL) {
	    p_ptr->frame_no = 0;
	  }

	}
	return p_ptr;
}

int initiate_timer(unsigned int sec, unsigned int msec)
{
	struct itimerspec its;
	its.it_value.tv_sec  = sec;
	its.it_value.tv_nsec = msec*1000000;
	its.it_interval.tv_sec  = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	if ( timer_settime(timerid, 0, &its, NULL) == -1 )
	{
	  return (-1);
	}
	return 0;
}

int initiate_timer_type(struct itimerspec *dummy,unsigned int sec, unsigned int msec)
{

	dummy->it_value.tv_sec  = sec;
	dummy->it_value.tv_nsec = msec*1000000;
	dummy->it_interval.tv_sec  = dummy->it_value.tv_sec;
	dummy->it_interval.tv_nsec = dummy->it_value.tv_nsec;

	if ( timer_settime(timerid, 0, dummy, NULL) == -1 )
	{
	return (-1);
	}
	return 0;
}


int initiate_timer_type1(timer_t *timerID,unsigned int sec, unsigned int msec)
{
	struct itimerspec its;
	its.it_value.tv_sec  = sec;
	its.it_value.tv_nsec = msec*1000000;
	its.it_interval.tv_sec  = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	if ( timer_settime(*timerID, 0, &its, NULL) == -1 )
	{
	  return (-1);
	}
	return 0;
}

int clear_timer_type(struct itimerspec *dummy)
{
	dummy->it_value.tv_sec  = 0;
	dummy->it_value.tv_nsec = 0;
	dummy->it_interval.tv_sec  = dummy->it_value.tv_sec;
	dummy->it_interval.tv_nsec = dummy->it_value.tv_nsec;

	if ( timer_settime(timerid, 0, dummy, NULL) == -1 )
	{
	  return (-1);
	}
	return 0;
}

void *  write_message(void* arg )
{
	sensor_PKT  *temp_p_pkt_dummy;
	int ret;
	while(1)
	{

	  ret = sem_wait(&semaphore_1);
	  if(ret == -1) 
	  {
	  	continue;
	  }

	  set_address();
	  transmit_pkt();
	  temp_p_pkt_dummy = NULL;
	  pthread_mutex_lock(&lock1);
	  temp_p_pkt_dummy = (dequeue_pkt(Tx_PKT));
	  pthread_mutex_unlock(&lock1);

	  if(temp_p_pkt_dummy)
	  {
		pthread_mutex_lock(&lock2);
		append_pkt_to_free_pool(temp_p_pkt_dummy,Rx_PKT);
		pthread_mutex_unlock(&lock2);
		flag = 0;
	  }

	}
}

#define DB_DELAY  10
#define DB_uDELAY 20000
#define ERR_COUNT_LIMIT 20
void mysleep(int );
void mysleep(int sec)
{
    int ret = 0;
    while(sec )
    {
      ret = sleep(sec);
      sec = ret;
      printf("\n return is : %d \n",sec);
    }
}

#if 0
void * db_filling(void* arg)
{
    unsigned int dummy_cntr=0;
    unsigned int count_limit=0;
    int ret = 0;
    unsigned char count = 0;
    unsigned int Arr_ifindex[3188];
    unsigned int db_reg_value[3188];
    unsigned int dbcount=0;
    unsigned int dbflag = 0;
    int errflag =0;
    int errcount =0;
    unsigned loop = 0;
    int i =0;

    time_t rawtime1;
    time(&rawtime1);

    sleep(5);

    time_t rawtime;
    struct tm *info;

    dummy_cntr = 0;

    while(1)
    {
  
      dbcount = 0;
      for (dummy_cntr =0;dummy_cntr<(989);dummy_cntr++)                // chnaged from 1100 to 3150
      {
        {
          db_reg_value[dbcount] = dummy_cntr;//Reg[dummy_cntr].reg_d.reg_value;
          Arr_ifindex[dbcount] = (0x7000 + dummy_cntr);//Reg[dummy_cntr].if_index;
 
          dbcount++;
          dbflag  =1;
        }

      }
  
            
      time(&rawtime);
      printf("\n Db Time is : %s\n",ctime(&rawtime));
    
      for(i=0;i<10;i++)
      {
        //icos_mbusdb_perform_cmd(p_db,"BEGIN TRANSACTION");
        for(loop = 0;loop<99;loop++)
        {
          errcount = 0;
          do
          {
            errflag = icos_mbusdb_update_reg(p_db,Arr_ifindex[((i*99)+loop)], db_reg_value[((i*99)+loop)]);
 printf("\n  value is:%d\t register is :%d\n",db_reg_value[((i*99)+loop)],Arr_ifindex[((i*99)+loop)]);
            //errflag = icos_mbusdb_update_reg(p_db,Reg[((i*99)+loop)].if_index, Reg[((i*99)+loop)].reg_d.reg_value);
            if(errflag)
            {
              printf("\n Error1 value is:%d\t err count is :%d\t register is :%d\n",errflag,errcount,loop);
              errflag = 0;  
            }
          }while(errflag);
          
        }// first for loop end
        //icos_mbusdb_perform_cmd(p_db,"END TRANSACTION");
        system("sleep 2s");
      }//second for loop end
    }//while end
}
#endif
#if 0
void * shm_filling(void* arg)
{
    unsigned int dummy_cntr=0;
    unsigned int count_limit=0;
    int ret = 0;
    unsigned char count = 0;
    unsigned int Arr_ifindex[3188];
    unsigned int db_reg_value[3188];
    unsigned int dbcount=0;
    unsigned int dbflag = 0;
    int errflag =0;
    int errcount =0;
    unsigned loop = 0;
    int i =0;
    time_t rawtime1;
    time(&rawtime1);
    sleep(5);
    time_t rawtime;
    struct tm *info;
    dummy_cntr = 0;

    while(1)
    {
      dbcount = 0;
      for (dummy_cntr =0;dummy_cntr<(989);dummy_cntr++)                // chnaged from 1100 to 3150
      {
   
        if(Reg[dummy_cntr].reg_flag == 1)
        {
          db_reg_value[dbcount] = Reg[dummy_cntr].reg_d.reg_value;
          Arr_ifindex[dbcount] = Reg[dummy_cntr].if_index;
      
          dbcount++;
          dbflag  =1;
        }
      }
  
  
      time(&rawtime);
      printf("\n Shm Time is : %s\n",ctime(&rawtime));
    
      for(i=0;i<10;i++)
      {
        for(loop = 0;loop<99;loop++)
        {  
          errflag = icos_mbusdb_update_reg(p_db1,Arr_ifindex[((i*99)+loop)], db_reg_value[((i*99)+loop)]);
          //errflag = icos_mbusdb_update_reg(p_db,Reg[((i*99)+loop)].if_index, Reg[((i*99)+loop)].reg_d.reg_value);
          if(errflag)
          {
            printf("\n Error1 value is:%d\t err count is :%d\t register is :%d\n",errflag,errcount,loop);
            errflag = 0;  
          } 

        }// first for loop end
    
        system("sleep 2s");
      }//second for loop end
  
    }//while end
}

#endif
//#if 0//This function for checking purpose only
void * shm_filling(void* arg)
{
    unsigned int dummy_cntr=0;
    unsigned int count_limit=0;
    int ret = 0;
    unsigned char count = 0;
    unsigned int Arr_ifindex[3188];
    unsigned int db_reg_value[3188];
    unsigned int dbcount=0;
    unsigned int dbflag = 0;
    int errflag =0;
    int errcount =0;
    unsigned loop = 0;
    int i =0;
    unsigned short dummyvalue = 0;
    time_t rawtime1;
    time(&rawtime1);
    sleep(5);
    time_t rawtime;
    struct tm *info;
    int dhee = 0;
     
    while(1)
    {
      for (dummy_cntr =0;dummy_cntr<(FIELD_TOTALENTRY);dummy_cntr++)                
      {
          db_reg_value[dummy_cntr] = Reg[dummy_cntr].reg_d.reg_value;
          Arr_ifindex[dummy_cntr] = (0x7000 + dummy_cntr);   
                 
      }
      
    
      
        for(loop = 0;loop<FIELD_TOTALENTRY;loop++)
        {  
          errflag = icos_mbusdb_update_reg(p_db1,Arr_ifindex[loop], db_reg_value[loop]);
          //errflag = icos_mbusdb_update_reg(p_db1,Reg[loop].if_index, Reg[loop].reg_d.reg_value);
         
          if(errflag)
          {
            printf("\n Error1 value is:%d\t err count is :%d\t register is :%d\n",errflag,errcount,loop);
            errflag = 0;  
          } 

        }// first for loop end
      // time(&rawtime);
      //printf("\n Shm Time2 is : %s\n",ctime(&rawtime));
        system("sleep 10s");
      
    }//while end
}
//#endif
void log_init(void)
{

	log_fd =open("/tmp/paramterlog",O_CREAT|O_APPEND|O_WRONLY|S_IRUSR);
	if(log_fd == -1) 
	{
	  perror("log file open\n");
	}

}

void stamp_time(void)
{
	time_t rawtime;
	struct tm *info;

	time(&rawtime);
	info =localtime(&rawtime);
	write(log_fd,asctime(info),20);
}

void add_to_log( unsigned long dptr)
{
	unsigned char newline = '\n';
	stamp_time();
	dprintf(log_fd,"\n%x\n",dptr);
 
}


void timerHandler( int sig, siginfo_t *si, void *uc )
{
  
	timer_t *tidp;
    int      update_count;
	unsigned short    temp_dummy = 0;
	unsigned short    temp_dummy1 = 0;
	unsigned short value = 0;
	unsigned int  result = 0;

	tidp = si->si_value.sival_ptr;

	if ( *tidp == firstTimerID )
	{
	  time_stamp_flag = 1;
	}
	else if ( *tidp == secondTimerID )
	{
	  cleartimer(&secondTimerID);
	  sem_post(&semaphore_1);
	}
	else if ( *tidp == thirdTimerID )
	{
	  cleartimer(&thirdTimerID);
	  time_stamp_flag = 0;
	}  
	else if (*tidp == fourthTimerID)
	{
	  cleartimer(&fourthTimerID);
	   
	  wCalc_Cntr++;
	  if (wCalc_Cntr >= 100) 
	  {
	    if(!sFlag.kwh_calc_in_process)
	    {
		  wCalc_Cntr -= 100;
		  sFlag.kwh_calc_in_process = 1;
		  wKwh_Calc_Cntr = 0;	
	    }
	    else
	    {
		  wKwh_Calc_Cntr++;
		  if (wKwh_Calc_Cntr == PANEL4){
		    sFlag.kwh_calc_in_process = 0;
          }
	   	}
		KWH_Calc(wKwh_Calc_Cntr);
	  }
	  if (wDemandChkCntr1s_10ms >= 100)
	  {
		sFlag.pnl1_demand_chk_1sec = 1;
		sFlag.pnl2_demand_chk_1sec = 1;
		sFlag.pnl3_demand_chk_1sec = 1;
		sFlag.pnl4_demand_chk_1sec = 1;
		wDemandChkCntr1s_10ms = 0;
		wDemandChkCntr1hr_10ms++;
	  }
	  else
	    wDemandChkCntr1s_10ms++;
	  if (wDemandChkCntr1hr_10ms >= 36)
	  {
		sFlag.pnl1_demand_chk_1hr = 1;
		sFlag.pnl2_demand_chk_1hr = 1;
		sFlag.pnl3_demand_chk_1hr = 1;
		sFlag.pnl4_demand_chk_1hr = 1;
		wDemandChkCntr1hr_10ms = 0;
		wDemandChkCntr24hr_10ms++;
	  }
	  if (wDemandChkCntr24hr_10ms >= 24)
	  {
		sFlag.pnl1_demand_chk_24hr = 1;
		sFlag.pnl2_demand_chk_24hr = 1;
		sFlag.pnl3_demand_chk_24hr = 1;
		sFlag.pnl4_demand_chk_24hr = 1;

		if(ntpsync_flag)
		{
			epochtime_cntr = 1500;
		}

		wDemandChkCntr24hr_10ms = 0;
	  }
	  settimer(&fourthTimerID,0,10);
	}
	else if (*tidp == sixthTimerID)
	{
	  cleartimer(&sixthTimerID);
	  Dynammic_sorting();
	  Pri_Winding_Calculation();
	  Sec_Winding_Calculation();
	  Branch_CT_Calculation();
	  AmbientTempCalc();
	  settimer(&sixthTimerID,0,5);
	}
	else if (*tidp == seventhTimerID)
	{
		cleartimer(&seventhTimerID);
		System_Status_Update();
	    settimer(&seventhTimerID,0,30);
	}
	else if (*tidp == copyTimerID)
	{
	  cleartimer(&copyTimerID);
	  copy_flag = 1;
   	  settimer(&copyTimerID ,60,0);
	}
	else if (*tidp == eighthTimerID)
	{
	  cleartimer(&eighthTimerID);
	  //counter for sending epochtime flag as 1
	  if(ntpsync_flag)
	  	epochtime_cntr = 1500;
	}
	
	
	#if 0
	}
	else if (*tidp == calibration_timerID)
	{
	  if (calibration_flag)
	  {
	   cleartimer(&calibration_timerID);
	   nbytes1 = write(sock_can, &temp1_frame, sizeof(struct can_frame));
	   //printf("\nSuccesful write : %d\tCan Data : %x\n",nbytes1,*dptr1);
	   //calibration_count++;
	   /*temp1_ptr->can_id += 0x004;
	   if (temp1_ptr->can_id == 0x7EF)
	      temp1_ptr->can_id = 0x7E7;*/
	   /*if (calibration_count == 5)*/
	      //calibration_flag = 0;
	   settimer(&calibration_timerID,1,0);	
	  }
	}
	else if (*tidp == S_Board_Update_timer)
	{
	   cleartimer(&S_Board_Update_timer);
	   for (update_count = 0;update_count < PANEL_GAIN; update_count++)
	   {
		if (SBoard1[update_count] != 0)
	        {
		   temp_dummy = ((0x1 << 15) | (SBoard1[update_count]));
		   temp_dummy1 = ((temp_dummy >> 8) | (temp_dummy << 8));
		   *dptr2 = (unsigned int)((((update_count << 8) | (0x5B)) << 16) | temp_dummy1);
		   write(sock_can, &temp2_frame, sizeof(struct can_frame));
		   SBoard1[update_count] = 0;
	        }
		if (System_gain[update_count] != 0)
		{
		   temp_dummy = ((0x1 << 15) | (System_gain[update_count]));
		   temp_dummy1 = ((temp_dummy >> 8) | (temp_dummy << 8));
		   *dptr3 = (unsigned int)((((update_count + 1) << 24) | (0x05 << 16)) | temp_dummy1);

		   //printf("\nCan Message : %x\n",*dptr3);
		   write(sock_can, &temp3_frame, sizeof(struct can_frame));
		   System_gain[update_count] = 0;
	        }
	   }
	   *dptr2 = 0x0301;
	   write(sock_can, &temp2_frame, sizeof(struct can_frame));
	   *dptr3 = 0x0301;
	   write(sock_can, &temp3_frame, sizeof(struct can_frame));
	}
	#endif	
}

int maketimer( timer_t *timerID)
{
	struct sigevent te;
	struct itimerspec its;
	struct sigaction sa;
	int sigNo = SIGRTMIN;

	/* Set up signal handler. */
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = timerHandler;
	sigemptyset(&sa.sa_mask);
	if (sigaction(sigNo, &sa, NULL) == -1) {
	  perror("sigaction");
	}

	/* Set and enable alarm */
	te.sigev_notify = SIGEV_SIGNAL;
	te.sigev_signo = sigNo;
	te.sigev_value.sival_ptr = timerID;
	timer_create(CLOCK_REALTIME, &te, timerID);

	return 1;
}

int cleartimer(timer_t *timerID)
{
	struct itimerspec its;
	its.it_value.tv_sec  = 0;
	its.it_value.tv_nsec = 0;
	its.it_interval.tv_sec  = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	timer_settime(*timerID, 0, &its, NULL);
	return 0;

}
int settimer( timer_t *timerID, int time_sec, int time_msec )
{
	struct itimerspec its;
	its.it_value.tv_sec  = time_sec;
	its.it_value.tv_nsec = time_msec * 1000000;
	its.it_interval.tv_sec  = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	timer_settime(*timerID, 0, &its, NULL);
	return 0;
}
int init_timer_routine(timer_t *timerID)
{
	struct sigevent sev;
	struct itimerspec its;

	sev.sigev_notify          = SIGEV_SIGNAL;
	sev.sigev_signo           = SIGRTMIN;
	sev.sigev_value.sival_ptr = timerID;
	if ( timer_create(CLOCK_REALTIME, &sev, timerID) == -1 )
	{
	  return (-1);
	}
	sigemptyset(&mask);
	sigaddset(&mask, SIGRTMIN);

	if ( sigprocmask(SIG_BLOCK, &mask, NULL) == -1 )
	{
	  return (-1);
	}
	return 0;
}
unsigned int set_address(void )
{
	unsigned int loop;
	unsigned int nbytes;
	unsigned int* pptr,*dptr;
	sensor_PKT *temp_ptr  = sensor_Tx_pkt_hdr;
	unsigned int* iptr = NULL;
	unsigned char id;
	unsigned char *cptr = NULL;
	unsigned short *sptr = NULL;
	unsigned char paraID,Offset;
	unsigned short parameter_offset = 0;
	unsigned short dummy_parameter_offset = 0;
	unsigned short parameter_data =0;
	unsigned int result = 0;
    static unsigned int panel_ctr = 0;

	iptr = Data.array;

	if(temp_ptr)
	{
      	for(loop = 0;loop < temp_ptr->frame_no;loop++)
      	{

		if(temp_ptr->sensor_data[loop].can_id == 0x07f7)
		{
		  id = 0;
		}
		else if(temp_ptr->sensor_data[loop].can_id == 0x07fB)
		{
		  id = 1;
		}
		else if(temp_ptr->sensor_data[loop].can_id == 0x07fD)
		{
		  id = 2;
		}
		else if(temp_ptr->sensor_data[loop].can_id == 0x07fE)
		{
		  id = 3;
		}

		dptr = (unsigned int*)temp_ptr->sensor_data[loop].data;
		sptr = (unsigned short *)(dptr);
		cptr = (unsigned char *)(dptr);
		paraID = *cptr++;
		Offset = *cptr;	
		parameter_offset = ((*sptr >> 8 )|(*sptr << 8));
		dummy_parameter_offset = parameter_offset;
		sptr++;
		parameter_data = ((*sptr >> 8 )|(*sptr << 8));
		/*if(temp_ptr->sensor_data[loop].can_id == 0x7f7)
		{
	         if (paraID == 0x50 || paraID == 0x51 || paraID == 0x60)
		 {
		   if (Offset >=0 && Offset <=3)
		    printf("ParaID : %x\tData : %d\n",paraID,parameter_data);
		 }
		}
		else if (temp_ptr->sensor_data[loop].can_id == 0x7fb)
		{
		 if (paraID == 0x50 || paraID == 0x51 || paraID == 0x60)
		 {
		   if (Offset >=0 && Offset <=3)
		    printf("ParaID : %x\tData : %d\n",paraID,parameter_data);
		 }
		}*/
		if (temp_ptr->sensor_data[loop].can_id == 0x7f7)
		{
		  if ((paraID == 0x50) && (Offset >= 0) && (Offset <=3))
		  {
		     wPhaseA_Board_I_PanelCurrent[Offset] = parameter_data;
		     //printf("\nwPhaseA_Board_I_PanelCurrent[%x] : %d\n",Offset,wPhaseA_Board_I_PanelCurrent[Offset]);
		  }
		  else if ((paraID == 0x51) && (Offset >= 0) && (Offset <= 3))
		  {
		     wPhaseB_Board_I_PanelCurrent[Offset] = parameter_data;
		     //printf("\nPhaseB_Board_I_PanelCurrent[%x] : %d\n",Offset,wPhaseB_Board_I_PanelCurrent[Offset]);
		  }
		  else if ((paraID == 0x60) && (Offset >= 0) && (Offset <= 3))
		  {
		     wPhaseC_Board_I_PanelCurrent[Offset] = parameter_data;
		     //printf("\nPhaseC_Board_I_PanelCurrent[%x] : %d\n",Offset,wPhaseC_Board_I_PanelCurrent[Offset]);
		  }
		}
		else if (temp_ptr->sensor_data[loop].can_id == 0x7fb)
		{
		  if ((paraID == 0x50) && (Offset >= 0) && (Offset <= 3))
		  {
		     wPhaseA_Board_I_PanelCurrent[Offset + 4] = parameter_data;
		     //printf("\nPhaseA_Board_II_PanelCurrent[%x] : %d\n",Offset,wPhaseA_Board_II_PanelCurrent[Offset]);
		  }
		  else if ((paraID == 0x51) && (Offset >= 0) && (Offset <= 3))
		  {
		     wPhaseB_Board_I_PanelCurrent[Offset + 4] = parameter_data;
		     //printf("\nPhaseB_Board_II_PanelCurrent[%x] : %d\n",Offset,wPhaseB_Board_II_PanelCurrent[Offset]);
		  }
		  else if ((paraID == 0x60) && (Offset >=0) && (Offset <= 3))
		  {
		     wPhaseC_Board_I_PanelCurrent[Offset +4] = parameter_data;	 
		     //printf("\nPhaseC_Board_II_PanelCurrent[%x] : %d\n",Offset,wPhaseC_Board_II_PanelCurrent[Offset]);
		  } 
		} 


		if((parameter_offset>= 0x5000)&&(parameter_offset<= 0x53ff))
		{
		  parameter_offset = ((parameter_offset&0x0fff)+0x0B00)|((id)<<14);	

		}
		else if((parameter_offset>= 0x5700)&&(parameter_offset<= 0x5fff))
		{
		  parameter_offset = ((parameter_offset&0x0fff)+0x0800)|((id)<<14);

		}
		else if((parameter_offset>= 0x0100)&&(parameter_offset<= 0x0A15))
		{
		  parameter_offset = ((parameter_offset&0x0fff)+0x0000)|((id)<<14);

		}

		result =(unsigned long) ((parameter_offset<<16)|(parameter_data));

		if ( paraID == DSP_PRIMARY_HI_ADDR)
		{
		  *(iptr + Offset) = result;
		  *(map + Offset) = (result & 0x0000FFFF);
		  
		  Reg[Offset].reg_d.reg_value = (result & 0x0000FFFF);
		  

		}
		else if (paraID == DSP_SEC_HI_ADDR)
		{
		  *(iptr + PRI_OFFSET + Offset) = result;
		  *(map + Offset + PRIMARY_OFFSET) = (result & 0x0000FFFF);
		  
		  Reg[PRIMARY_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
		  

		}
		else if (paraID == BOARDI_CT_RMS_HI_ADDR)
		{
                 if (temp_ptr->sensor_data[loop].can_id == 0x7f7)
		 {
		     Current_Buffer[Offset] = (result & 0x0000FFFF);
		     //Reg[SYSTEMPARAMETER + Offset].reg_d.reg_value = (result & 0x0000FFFF);
		 } 
		else if (temp_ptr->sensor_data[loop].can_id == 0x7fb)
		 {
		     Current_Buffer[Offset + 84] = (result & 0x000FFFF);
		     //Reg[RMS0_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
		 }
	        else if (temp_ptr->sensor_data[loop].can_id == 0x7fd)
		 {
		     Current_Buffer[Offset + 168] = (result & 0x000FFFF);
		     //Reg[RMS1_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
		 }
	        else if (temp_ptr->sensor_data[loop].can_id == 0x7fe)
		 {
		     Current_Buffer[Offset + 252] = (result & 0x000FFFF);
		     //Reg[RMS2_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
		 }
		}
		else if (paraID == BOARDI_KW_HI_ADDR)
		{
		  if(temp_ptr->sensor_data[loop].can_id == 0x7f7)
		  {
		     KW_Buffer[Offset] = (result & 0x0000FFFF);
		  }
		  else if(temp_ptr->sensor_data[loop].can_id == 0x7fb)
		  {
	             KW_Buffer[Offset + 84] = (result & 0x0000FFFF);
		  }
		  else if(temp_ptr->sensor_data[loop].can_id == 0x7fd)
		  {
		     KW_Buffer[Offset + 168] = (result & 0x0000FFFF);
		  }
		  else if(temp_ptr->sensor_data[loop].can_id == 0x7fe)
		  {
		     KW_Buffer[Offset + 252] = (result & 0x0000FFFF);
		  }	
		}
		else if (paraID == FIRMWARE_VERSION)
		{
		  if(temp_ptr->sensor_data[loop].can_id == 0x7f7)
		  {
			if((Offset >= 0xA0) || (Offset <= 0xA7))
			  *(iptr + SYS + (Offset - 0xA0)) = result;
			  Reg[SYS_NV + (Offset - 0xA0)].reg_d.reg_value = (result & 0x0000FFFF);
                          //printf("\n Firmware version %d : %x\n",Offset - 0xA0,result);
		  }
		  else if(temp_ptr->sensor_data[loop].can_id == 0x7fb)
		  {
			if((Offset >= 0xA0) || (Offset <= 0xA7))
			  *(iptr + FIRMWARE_0 + (Offset - 0xA0)) = result;
		  }
		  else if(temp_ptr->sensor_data[loop].can_id == 0x7fd)
		  {
			if((Offset >= 0xA0) || (Offset <= 0xA7))
		      *(iptr + FIRMWARE_1 + (Offset - 0xA0)) = result;
		  }
		  else if(temp_ptr->sensor_data[loop].can_id == 0x7fe)
		  {
			if((Offset >= 0xA0) || (Offset <= 0xA7))
			  *(iptr + FIRMWARE_2 + (Offset - 0xA0)) = result;
		  }	
		}
		else if (paraID == DSP_SYS_PARAMETER_HI_ADDR)
		{
			Data.word.System_Parameter.Ground_Curr = result;
		}
	        else if (paraID == BRANCH_GAIN)
	        {
		  if (temp_ptr->sensor_data[loop].can_id == 0x7F7)
		  {
		      if ((Offset >= 0x00) && (Offset <= 0x53))
	              {
			   Reg[KWH1_OFFSET + Offset].reg_d.reg_value = parameter_data;
			   *(map + KWH1_OFFSET + Offset) = parameter_data;
	              }
	              else if ((Offset >= 0x54) && (Offset <= 0xA7))
	              {
			 Reg[CT_GAIN +  (Offset - 0x54)].reg_d.reg_value = parameter_data;
		         *(map + CT_GAIN + (Offset - 0x54)) = parameter_data;
		      }
		  }
		 
	        }
		else if (paraID == DSP_PRI_SEC_GAIN_HI_ADDR)
	        {
		   if (temp_ptr->sensor_data[loop].can_id == 0x7F7)
		   {
			if ((Offset >= 0x00) && (Offset <= 0x13))
			{
			   Reg[KW_GAIN + Offset].reg_d.reg_value = parameter_data;
			   *(map + KW_GAIN + Offset) = parameter_data;
			}
		   }
	        }

		 		 	
        }
	}
    else
    {
    //printf("No Tx packet\n");
    }

}



void transmit_pkt(void )
{
	unsigned int loop;
	unsigned int nbytes;
	//unsigned int nbytes_uart;
	unsigned int* pptr,*dptr;
	unsigned int dummy_Tx_frame_count;
	char write_buffer[] = "A";
	struct can_frame temp_frame;
	struct can_frame *temp_ptr = &temp_frame;
	unsigned char buzzer_stat;

	/*******Sending Epochtime***********************************************************************///3216212 Aswin Krishna(16/11/21)

	epochtime = (unsigned int)time(NULL);  
	
	/*                      
    if(epochtime_cntr == 1000)
    {
    	printf("Start Epoch Counter\n");
    	system("date");
    }
    if(epochtime_cntr == 0)
    {
    	printf("End Epoch Counter\n");
		system("date");
		epochtime_cntr = -1;
		epochtime_flag = 0;
	}
	*/
	
	//printf("Epochtime_cntr : %d\n",epochtime_cntr);

    if(epochtime_cntr <= 0)
    {
    	epochtime_flag = 0;
	}
	else if(epochtime_cntr > 0)
	{
		epochtime_flag = 1;
		epochtime_cntr--;
	}
	
	//printf("Epochtime flag : %d\n",epochtime_flag);
	//printf("Epochtime counter : %d\n",epochtime_cntr);
	Data.word.Sys.wEpochtimeL = ((0x10C << 16) | (epochtime & 0xFFFF));            
    Data.word.Sys.wEpochtimeH = ((0x10D << 16) | ((epochtime>>16) & 0xFFFF));       
    Data.word.Sys.wEpochtime_flag = ((0x10E << 16 ) | epochtime_flag);
    
    /**********Sending buzzer status*******************************************************************///
   
    nos_i2c_read(0, 0x22, 0x02, &buzzer_stat, 4);
	buzzer_stat = (buzzer_stat >> 3) & 0x01;
	Data.word.Sys.buzzer_stat = ((0x10F << 16) | ((buzzer_stat & 0x0F) | ((buzz_status << 4) & 0xF0)));
 

	/************** Checking the time between each transmission **************/
	//753
	#if 0

	if(jack == 1)
	{
		gettimeofday(&stop_t, NULL);
		printf("Time between 2 transitions to TP %lu us\n", (stop_t.tv_sec - start_t.tv_sec) * 1000000 + stop_t.tv_usec - start_t.tv_usec);
		jack++;
		
	}

	if(jack == 0)
	{
		gettimeofday(&start_t, NULL);
		jack = 1;
	}
	#endif
	
	/************************** End **************************/	
			
	if(Tx_frame_count >= (sizeof(Data)/4))
    {
	  Tx_frame_count = 0;
    }
	dummy_Tx_frame_count = Tx_frame_count;
	for(loop= dummy_Tx_frame_count;loop < (dummy_Tx_frame_count+FRAME_LIMIT);loop++,Tx_frame_count++)
	{
	  if(Data.array[loop])
	  {
	  	temp_ptr->can_dlc = 4;		
		dptr = (unsigned int*)temp_ptr->data;
		*dptr = Data.array[loop];
		temp_ptr->can_id = 0x0200;
		/*if (((*dptr & 0xFF000000) >> 24) == 0x16)
		{ 
		printf("\nCan data : %x\n",*dptr);
		}*/
		nbytes = write(sock_can, &temp_frame, sizeof(struct can_frame));
		usleep(100);

               
                
	  }
	}
}


void Pri_Winding_Calculation()
{
	LoadPer_Calc(PRIMARY);
	Max_Min_Parameter(PRIMARY);
	Status_Update(PRIMARY);
}
void Sec_Winding_Calculation()
{
	LoadPer_Calc(SEC);
	Max_Min_Parameter(SEC);
	Status_Update(SEC);

}
void Branch_CT_Calculation()
{
	Demand_Calc(PANEL1);
	LoadPer_Calc(PANEL1);
	Max_Min_Parameter(PANEL1);
	Status_Update(PANEL1);
 
	Demand_Calc(PANEL2);
	LoadPer_Calc(PANEL2);
	Max_Min_Parameter(PANEL2);
    Status_Update(PANEL2);
	
}	

void LoadPer_Calc( unsigned int reg)
{
	unsigned int i;
        unsigned int j;
	unsigned int Panel_PhaseA[3] = {0,0,0};
        unsigned int Panel_PhaseB[3] = {0,0,0};
	unsigned int Panel_PhaseC[3] = {0,0,0};
	switch (reg)
	{
	  case PRIMARY:
	  {
		i = (Data.word.Primary.RMS_Curr_Phase_A&0xFFFF);
		dwMathBuff = ((unsigned long)i * 1000L);
		(Data.word.Primary.LoadPer_Phase_A) = ((unsigned long)(0x814 << 16))|(((unsigned long)dwMathBuff / wPri_Thd.word.wPhaseA_OverCurr));
		*(map + 19) = (Data.word.Primary.LoadPer_Phase_A & 0x0000FFFF);
		
		Reg[20].reg_d.reg_value = (Data.word.Primary.LoadPer_Phase_A & 0x0000FFFF);
		

		i = (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF);
		dwMathBuff = ((unsigned long)i * 1000L);
		(Data.word.Primary.LoadPer_Phase_B) = ((unsigned long)(0x815 << 16))|(((unsigned long)dwMathBuff / wPri_Thd.word.wPhaseB_OverCurr));
		*(map + 20) = (Data.word.Primary.LoadPer_Phase_B & 0x0000FFFF);
		
		Reg[21].reg_d.reg_value = (Data.word.Primary.LoadPer_Phase_B & 0x0000FFFF);
		



		i = (Data.word.Primary.RMS_Curr_Phase_C&0xFFFF);
		dwMathBuff = ((unsigned long)i * 1000L);
		(Data.word.Primary.LoadPer_Phase_C) = ((unsigned long)(0x816 << 16))|(((unsigned long)dwMathBuff / wPri_Thd.word.wPhaseC_OverCurr));
		*(map + 21) = (Data.word.Primary.LoadPer_Phase_C & 0x0000FFFF);
		
		Reg[22].reg_d.reg_value = (Data.word.Primary.LoadPer_Phase_C & 0x0000FFFF);
		

		break;
	  }
	  case SEC1:
	  {
		i = Secondary1.word.RMS_Curr_Phase_A;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary1.word.LoadPer_Phase_A = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr);

		i = Secondary1.word.RMS_Curr_Phase_B;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary1.word.LoadPer_Phase_B = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[0].word.wPanel1PhaseB_OverCurr);

		i = Secondary1.word.RMS_Curr_Phase_C;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary1.word.LoadPer_Phase_C = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[0].word.wPanel1PhaseC_OverCurr);

		break;
	  }
	  case SEC2:
	  {
		i = Secondary2.word.RMS_Curr_Phase_A;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary2.word.LoadPer_Phase_A = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr);

		i = Secondary2.word.RMS_Curr_Phase_B;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary2.word.LoadPer_Phase_B = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[0].word.wPanel2PhaseB_OverCurr);

		i = Secondary2.word.RMS_Curr_Phase_C;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary2.word.LoadPer_Phase_C = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[0].word.wPanel2PhaseC_OverCurr);	
		break;
	  }
	  case SEC3: 
	  {
		i = Secondary3.word.RMS_Curr_Phase_A;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary3.word.LoadPer_Phase_A = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr);

		i = Secondary3.word.RMS_Curr_Phase_B;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary3.word.LoadPer_Phase_B = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[1].word.wPanel1PhaseB_OverCurr);

		i = Secondary3.word.RMS_Curr_Phase_C;
		dwMathBuff = ((unsigned long)i * 1000L);
		Secondary3.word.LoadPer_Phase_C = ((unsigned long)dwMathBuff / Data_In.word.wPanelThd[1].word.wPanel1PhaseC_OverCurr);
		break;
	  }
	  case SEC:
	  {
		i =(Data.word.Secondary.RMS_Curr_Phase_A&0xFFFF);
		dwMathBuff = ((unsigned long)i * 1000L);
		(Data.word.Secondary.LoadPer_Phase_A) = ((unsigned long)(0x922 << 16))|(((unsigned long)dwMathBuff / wSec_Thd.word.wPhaseA_OverCurr));
		*(map + PRIMARY_OFFSET + 34) = (Data.word.Secondary.LoadPer_Phase_A & 0x0000FFFF);
		
		Reg[PRIMARY_OFFSET + 34].reg_d.reg_value = (Data.word.Secondary.LoadPer_Phase_A & 0x0000FFFF);
		

		i = (Data.word.Secondary.RMS_Curr_Phase_B&0xFFFF);
		dwMathBuff = ((unsigned long)i * 1000L);
		(Data.word.Secondary.LoadPer_Phase_B) = ((unsigned long)(0x923 << 16))|(((unsigned long)dwMathBuff / wSec_Thd.word.wPhaseB_OverCurr));
		*(map + PRIMARY_OFFSET + 35) = (Data.word.Secondary.LoadPer_Phase_B & 0x0000FFFF);
		
		Reg[PRIMARY_OFFSET + 35].reg_d.reg_value = (Data.word.Secondary.LoadPer_Phase_B & 0x0000FFFF);
		

		i = (Data.word.Secondary.RMS_Curr_Phase_C&0xFFFF);
		dwMathBuff = ((unsigned long)i * 1000L);
		(Data.word.Secondary.LoadPer_Phase_C) = ((unsigned long)(0x924 << 16))|(((unsigned long)dwMathBuff / wSec_Thd.word.wPhaseC_OverCurr));
		*(map + PRIMARY_OFFSET + 36) = (Data.word.Secondary.LoadPer_Phase_C & 0x0000FFFF);
		
		Reg[PRIMARY_OFFSET + 36].reg_d.reg_value = (Data.word.Secondary.LoadPer_Phase_C & 0x0000FFFF);

               
		for (j= 0;j<(wPDU_Parameters[14]);j++)
		{
		    Panel_PhaseA[0] += wPhaseA_Board_I_PanelCurrent[j];
		    Panel_PhaseB[0] += wPhaseB_Board_I_PanelCurrent[j];
		    Panel_PhaseC[0] += wPhaseC_Board_I_PanelCurrent[j];
		}

                //AK added for panel disable
                if(wPDU_Parameters[13] > 1)
                {
			wPhaseA_PanelCurrent[0] = Panel_PhaseA[0];
			Data.word.Secondary.Panel1_PhaseA_Load = (unsigned int)(0x092C0000 | (((unsigned long long)(wPhaseA_PanelCurrent[0])*1000)/(wPDU_Parameters[1])));
			Reg[PRIMARY_OFFSET + 44].reg_d.reg_value = (Data.word.Secondary.Panel1_PhaseA_Load & 0x0000FFFF);
			wPhaseB_PanelCurrent[0] = Panel_PhaseB[0];
			Data.word.Secondary.Panel1_PhaseB_Load = (unsigned int)(0x092D0000 | (((unsigned long long)(wPhaseB_PanelCurrent[0])*1000)/(wPDU_Parameters[2])));
			Reg[PRIMARY_OFFSET + 45].reg_d.reg_value = (Data.word.Secondary.Panel1_PhaseB_Load & 0x0000FFFF);
			wPhaseC_PanelCurrent[0] = Panel_PhaseC[0];
			Data.word.Secondary.Panel1_PhaseC_Load = (unsigned int)(0x092E0000 | (((unsigned long long)(wPhaseC_PanelCurrent[0])*1000)/(wPDU_Parameters[3])));
			Reg[PRIMARY_OFFSET + 46].reg_d.reg_value = (Data.word.Secondary.Panel1_PhaseC_Load & 0x0000FFFF);
                }
                //end		
		
                for (j= wPDU_Parameters[14];j<(wPDU_Parameters[14] + wPDU_Parameters[15]);j++)
		{
		    Panel_PhaseA[1] += wPhaseA_Board_I_PanelCurrent[j];
		    Panel_PhaseB[1] += wPhaseB_Board_I_PanelCurrent[j];
		    Panel_PhaseC[1] += wPhaseC_Board_I_PanelCurrent[j];
		}

                //AK added for panel disable 
                if(wPDU_Parameters[13] > 1) 
                {              
			wPhaseA_PanelCurrent[1] = Panel_PhaseA[1];
			Data.word.Secondary.Panel2_PhaseA_Load = (unsigned int)(0x092F0000 | (((unsigned long long)(wPhaseA_PanelCurrent[1])*1000)/(wPDU_Parameters[1])));
			Reg[PRIMARY_OFFSET + 47].reg_d.reg_value = (Data.word.Secondary.Panel2_PhaseA_Load & 0x0000FFFF);
			*(map + PRIMARY_OFFSET + 47) = (Data.word.Secondary.Panel2_PhaseA_Load & 0x0000FFFF);
			wPhaseB_PanelCurrent[1] = Panel_PhaseB[1];
			Data.word.Secondary.Panel2_PhaseB_Load = (unsigned int)(0x09300000 | (((unsigned long long)(wPhaseB_PanelCurrent[1])*1000)/(wPDU_Parameters[2])));
			Reg[PRIMARY_OFFSET + 48].reg_d.reg_value = (Data.word.Secondary.Panel2_PhaseB_Load & 0x0000FFFF);
			*(map + PRIMARY_OFFSET + 48) = (Data.word.Secondary.Panel2_PhaseB_Load & 0x0000FFFF);
			wPhaseC_PanelCurrent[1] = Panel_PhaseC[1];
			Data.word.Secondary.Panel2_PhaseC_Load = (unsigned int)(0x09310000 | (((unsigned long long)(wPhaseC_PanelCurrent[1])*1000)/(wPDU_Parameters[3])));
			Reg[PRIMARY_OFFSET + 49].reg_d.reg_value = (Data.word.Secondary.Panel2_PhaseC_Load & 0x0000FFFF);
			*(map + PRIMARY_OFFSET + 48) = (Data.word.Secondary.Panel2_PhaseC_Load & 0x0000FFFF);
                }
                //end

		for ((j= wPDU_Parameters[14] + wPDU_Parameters[15]);j<(wPDU_Parameters[14] + wPDU_Parameters[15] + wPDU_Parameters[16]);j++)
		{
		    Panel_PhaseA[2] += wPhaseA_Board_I_PanelCurrent[j];
		    Panel_PhaseB[2] += wPhaseB_Board_I_PanelCurrent[j];
		    Panel_PhaseC[2] += wPhaseC_Board_I_PanelCurrent[j];
		}
 
                //AK added for panel disable 
                if(wPDU_Parameters[13] > 1) 
                {     
			wPhaseA_PanelCurrent[2] = Panel_PhaseA[2];
			Data.word.Secondary.Panel3_PhaseA_Load = (unsigned int)(0x09320000 | (((unsigned long long)(wPhaseA_PanelCurrent[2])*1000)/(wPDU_Parameters[1])));
			Reg[PRIMARY_OFFSET + 50].reg_d.reg_value = (Data.word.Secondary.Panel3_PhaseA_Load & 0x0000FFFF);
			*(map + PRIMARY_OFFSET + 50) = (Data.word.Secondary.Panel3_PhaseA_Load & 0x0000FFFF);
			wPhaseB_PanelCurrent[2] = Panel_PhaseB[2];
			Data.word.Secondary.Panel3_PhaseB_Load = (unsigned int)(0x09330000 | (((unsigned long long)(wPhaseB_PanelCurrent[2])*1000)/(wPDU_Parameters[2])));
			Reg[PRIMARY_OFFSET + 51].reg_d.reg_value = (Data.word.Secondary.Panel3_PhaseB_Load & 0x0000FFFF);
			*(map + PRIMARY_OFFSET + 51) = (Data.word.Secondary.Panel3_PhaseB_Load & 0x0000FFFF);
			wPhaseC_PanelCurrent[2] = Panel_PhaseC[2];
			Data.word.Secondary.Panel3_PhaseC_Load = (unsigned int)(0x09340000 | (((unsigned long long)(wPhaseC_PanelCurrent[2])*1000)/(wPDU_Parameters[3])));
			Reg[PRIMARY_OFFSET + 52].reg_d.reg_value = (Data.word.Secondary.Panel3_PhaseC_Load & 0x0000FFFF);
			*(map + PRIMARY_OFFSET + 52) = (Data.word.Secondary.Panel3_PhaseC_Load & 0x0000FFFF);

			Data.word.Secondary.Panel1_Load = (unsigned int)(0x09290000 | (((unsigned long long)(Panel_PhaseA[0] + Panel_PhaseB[0] + Panel_PhaseC[0])*1000)/(wPDU_Parameters[1] + wPDU_Parameters[2] + wPDU_Parameters[3])));
	    Reg[PRIMARY_OFFSET + 41].reg_d.reg_value = (Data.word.Secondary.Panel1_Load & 0x0000FFFF);
	    *(map + PRIMARY_OFFSET + 41) = (Data.word.Secondary.Panel1_Load & 0x0000FFFF);

	    		Data.word.Secondary.Panel2_Load = (unsigned int)(0x092A0000 | (((unsigned long long)(Panel_PhaseA[1] + Panel_PhaseB[1] + Panel_PhaseC[1])*1000)/(wPDU_Parameters[1] + wPDU_Parameters[2] + wPDU_Parameters[3])));
	    Reg[PRIMARY_OFFSET + 42].reg_d.reg_value = (Data.word.Secondary.Panel2_Load & 0x0000FFFF);
	    *(map + PRIMARY_OFFSET + 42) = (Data.word.Secondary.Panel2_Load & 0x0000FFFF);

	   		Data.word.Secondary.Panel3_Load = (unsigned int)(0x092B0000 | (((unsigned long long)(Panel_PhaseA[2] + Panel_PhaseB[2] + Panel_PhaseC[2])*1000)/(wPDU_Parameters[1] + wPDU_Parameters[2] + wPDU_Parameters[3])));
	   Reg[PRIMARY_OFFSET + 43].reg_d.reg_value = (Data.word.Secondary.Panel3_Load & 0x0000FFFF);
	   *(map + PRIMARY_OFFSET + 43) = (Data.word.Secondary.Panel3_Load & 0x0000FFFF);
                 }
		
		
		break;
	  }
	  case PANEL1:
	  {
		for(i=0; i<84; i++)
		{
		  dwMathBuff = (unsigned long)((unsigned long)((Data.array[SEC_OFFSET + i]&0xFFFF)) * 1000);
		  if ((Data_In.word.Max_Min_Limit[0][i]) & 0xFFFF)
		  {

			Data.array[KW_3_OFFSET + i] = ((unsigned long)((0x1900|i)|(0<<14))<<16)|((unsigned long)(dwMathBuff/((Data_In.word.Max_Min_Limit[0][i]) & 0xFFFF)));
			*(map + KW1_OFFSET + i) = (Data.array[KW_3_OFFSET + i] & 0x0000FFFF);
			
			Reg[KW1_OFFSET + i].reg_d.reg_value = (Data.array[KW_3_OFFSET + i] & 0x0000FFFF);
			

		  }
		}
		break;
	  }
	  case PANEL2:
	  {
		for(i=0; i<84; i++)
		{
		  dwMathBuff = (unsigned long)((unsigned long)((Data.array[RMS_0_OFFSET + i]&0xFFFF)) * 1000);
		  if(((Data_In.word.Max_Min_Limit[1][i]) & 0xFFFF))
		  {
			Data.array[LOAD_0_OFFSET + i] = ((unsigned long)((0x1900|(i + 84))|(0<<14))<<16)|((unsigned long)(dwMathBuff / ((Data_In.word.Max_Min_Limit[1][i]) & 0xFFFF)));
			//printf("\nLoad Per : %x\n",Data.array[LOAD_0_OFFSET + i]);
			*(map + LOAD0_OFFSET + i) = (Data.array[LOAD_0_OFFSET + i] & 0x0000FFFF);
			
			Reg[LOAD0_OFFSET + i].reg_d.reg_value = (Data.array[LOAD_0_OFFSET + i] & 0x0000FFFF);
			

		  }
		}
		break;	
	  }
	  case PANEL3:
	  {
		for(i=0; i<84; i++)
		{
		  dwMathBuff = (unsigned long)((unsigned long)((Data.array[RMS_1_OFFSET + i]&0xFFFF)) * 1000);
		  if(((Data_In.word.Max_Min_Limit[2][i]) & 0xFFFF))
		  {
			Data.array[LOAD_1_OFFSET + i] = ((unsigned long)((0x1900|i)|(2<<14))<<16)|((unsigned long)(dwMathBuff / ((Data_In.word.Max_Min_Limit[2][i]) & 0xFFFF)));
			/*Reg[LOAD_1_OFFSET + i].if_index = (if_index_cur | (LOAD_0_OFFSET + i));
			Reg[LOAD_1_OFFSET + i].reg_addr = ((Data.array[LOAD_1_OFFSET + i] & 0xFFFF0000) >> 16);
			Reg[LOAD_1_OFFSET + i].reg_d.reg_value = (Data.array[LOAD_1_OFFSET + i] & 0x0000FFFF);
			Reg[LOAD_1_OFFSET + i].reg_type  = INPUT_REGISTER;
			Reg[LOAD_1_OFFSET + i].reg_flag  = 1;*/
		  }
		}
		break;
	  }
	  case PANEL4:
	  {
	    for(i=0; i<84; i++)
		{
		  dwMathBuff = (unsigned long)((unsigned long)((Data.array[RMS_2_OFFSET + i]&0xFFFF)) * 1000);
		  if(((Data_In.word.Max_Min_Limit[3][i]) & 0xFFFF))
		  {
			Data.array[LOAD_2_OFFSET + i] = ((unsigned long)((0x1900|i)|(3<<14))<<16)|((unsigned long)(dwMathBuff / ((Data_In.word.Max_Min_Limit[3][i]) & 0xFFFF)));
			/*Reg[LOAD_2_OFFSET + i].if_index = (if_index_cur | (LOAD_2_OFFSET + i));
			Reg[LOAD_2_OFFSET + i].reg_addr = ((Data.array[LOAD_2_OFFSET + i] & 0xFFFF0000) >> 16);
			Reg[LOAD_2_OFFSET + i].reg_d.reg_value = (Data.array[LOAD_2_OFFSET + i] & 0x0000FFFF);
			Reg[LOAD_2_OFFSET + i].reg_type  = INPUT_REGISTER;
			Reg[LOAD_2_OFFSET + i].reg_flag  = 1;*/
	  	  }
		}
		break;
	  }

	  default:
	  break;
	}
}

void Status_Update (unsigned int reg)
{
	unsigned char dummy;
	unsigned char dummy1;
	unsigned int *ptCTCurDemand;
	unsigned int *ptCTKwDemand;

	unsigned int i=0;
	unsigned int j=0;
	unsigned int alarm;
	unsigned char alarm1;

	
	
	switch (reg)
	{
	  case PRIMARY:
	  {

		Data.word.Pri_Status_Flag.paraID = 0x03;

		if ((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) < wPri_Thd.word.wPhaseA_UnderVolt)
		{
		  Data.word.Pri_Status_Flag.PhaseA_UnderVolt = 1;
		}
		else
		{
		  if((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) > (wPri_Thd.word.wPhaseA_UnderVolt + VOLT_HYSTER))
		  {
			Data.word.Pri_Status_Flag.PhaseA_UnderVolt = 0;
		  }
		}

		if ((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseB_UnderVolt)
		{

		  Data.word.Pri_Status_Flag.PhaseB_UnderVolt = 1;
		}
		else
		{
		  if((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_UnderVolt + VOLT_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseB_UnderVolt = 0;
		  }	
		}

		if ((Data.word.Primary.L2N_Volt_Phase_C & 0x0000FFFF) < wPri_Thd.word.wPhaseC_UnderVolt)
		{
		  Data.word.Pri_Status_Flag.PhaseC_UnderVolt = 1;	
		}
		else
		{
		  if((Data.word.Primary.L2N_Volt_Phase_C & 0x0000FFFF) > wPri_Thd.word.wPhaseC_UnderVolt + VOLT_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseC_UnderVolt = 0;
		  }	
		}
		if (no_xfmr)
		{
		if ((Data.word.Pri_Status_Flag.PhaseA_UnderVolt) || (Data.word.Pri_Status_Flag.PhaseB_UnderVolt) || (Data.word.Pri_Status_Flag.PhaseC_UnderVolt))
		{
		 input_undervolt_count++;
	         if (input_undervolt_count >= 10)
		 {
		  if (!event_flag[33])
		  {
			dummy1 = 33;
			write(fd5[1],&dummy1,1);
			event_flag[33] = 1;
		        alarm_status |= (1 << IP_UNDER_VOLTAGE);
			alarm_flag = 0;
		  }
		 }
		}
		else if  ((!Data.word.Pri_Status_Flag.PhaseA_UnderVolt) || (!Data.word.Pri_Status_Flag.PhaseB_UnderVolt) || (!Data.word.Pri_Status_Flag.PhaseC_UnderVolt))
		{
		  input_undervolt_count = 0;
		  if (event_flag[33])
		  {
			dummy1 = 34;
			write(fd5[1],&dummy1,1);
			event_flag[33] = 0;
			alarm_status &= ~(1 << IP_UNDER_VOLTAGE);
			alarm_flag = 0;
		  }
		}
		}
		if ((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) > wPri_Thd.word.wPhaseA_OverVolt)
		{
		  Data.word.Pri_Status_Flag.PhaseA_OverVolt = 1;
		}
		else
		{
		  if((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) < wPri_Thd.word.wPhaseA_OverVolt - VOLT_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseA_OverVolt = 0;
		  }
		}

		if ((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_OverVolt)
		{
		  Data.word.Pri_Status_Flag.PhaseB_OverVolt = 1;
		}
		else
		{
		  if((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseB_OverVolt - VOLT_HYSTER)
		  {
		    Data.word.Pri_Status_Flag.PhaseB_OverVolt = 0;
		  }
		}

		if ((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseC_OverVolt)
		{
		  Data.word.Pri_Status_Flag.PhaseC_OverVolt = 1;
		}
		else
		{
		  if((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseC_OverVolt - VOLT_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseC_OverVolt = 0;
		  }
		}
		if (no_xfmr)
		{
		if ((Data.word.Pri_Status_Flag.PhaseA_OverVolt) || (Data.word.Pri_Status_Flag.PhaseB_OverVolt) || (Data.word.Pri_Status_Flag.PhaseC_OverVolt))
		{
		  if (!event_flag[39])
		  {
			dummy1 = 39;
			write(fd5[1],&dummy1,1);
			event_flag[39] = 1;
			alarm_status |= (1 << IP_OVER_VOLTAGE);
			alarm_flag = 0;
		  }
		}
		else if ((!Data.word.Pri_Status_Flag.PhaseA_OverVolt) || (!Data.word.Pri_Status_Flag.PhaseB_OverVolt) || (!Data.word.Pri_Status_Flag.PhaseC_OverVolt))
		{
		  if (event_flag[39])
		  {
			dummy1 = 40;
			write(fd5[1],&dummy1,1);
			event_flag[39] = 0;
			alarm_status &= ~(1 << IP_OVER_VOLTAGE);
			alarm_flag = 0;
		  }
		}
		}
		// Current Status
		if ((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) < wPri_Thd.word.wPhaseA_UnderCurr)
		{
		  Data.word.Pri_Status_Flag.PhaseA_UnderCurr = 1;
		}
		else
		{
		  if((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) > wPri_Thd.word.wPhaseA_UnderCurr + IP_UC_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseA_UnderCurr = 0;
		  }
		}

		if ((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseB_UnderCurr)
		{
		  Data.word.Pri_Status_Flag.PhaseB_UnderCurr = 1;
		}
		else
		{
		  if((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_UnderCurr + IP_UC_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseB_UnderCurr = 0;
		  }
		}

		if ((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) < wPri_Thd.word.wPhaseC_UnderCurr)
		{
		  Data.word.Pri_Status_Flag.PhaseC_UnderCurr = 1;
		}
		else
		{
		  if((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) > wPri_Thd.word.wPhaseC_UnderCurr + IP_UC_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseC_UnderCurr = 0;
		  }
		}
		if (no_xfmr)
		{
		if ((Data.word.Pri_Status_Flag.PhaseA_UnderCurr) || (Data.word.Pri_Status_Flag.PhaseB_UnderCurr) || (Data.word.Pri_Status_Flag.PhaseC_UnderCurr))
		{
		  if (!event_flag[35])
		  {
			dummy1 = 35;
			write(fd5[1],&dummy1,1);
			event_flag[35] = 1;
			alarm_status |= (1 << IP_UNDER_CURRENT); 
			alarm_flag = 0;
		  }
		}
		else if ((!Data.word.Pri_Status_Flag.PhaseA_UnderCurr) || (!Data.word.Pri_Status_Flag.PhaseB_UnderCurr) || (!Data.word.Pri_Status_Flag.PhaseC_UnderCurr))
		{
		  if (event_flag[35])
		  {
			dummy1 = 36;
			write(fd5[1],&dummy1,1);
			event_flag[35] = 0;
			alarm_status &= ~(1 << IP_UNDER_CURRENT);
			alarm_flag = 0;
		  }
		}
		}
		if (((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) > (wPri_Thd.word.wPhaseA_OverCurr * 7))
		{
		  Data.word.Pri_Status_Flag.PhaseA_Curr_High =1;
		  if (((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF)*10) > (wPri_Thd.word.wPhaseA_OverCurr*8))
		  {
			Data.word.Pri_Status_Flag.PhaseA_OverCurr = 1;
		  }
		  else 
		  {
		    if (((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF)*10) < ((wPri_Thd.word.wPhaseA_OverCurr*8) - IP_OC_CURR_HYSTER))
		    {
			  Data.word.Pri_Status_Flag.PhaseA_OverCurr = 0;
			}
		  }
		}
		else 
		{
		  if(((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF)*10) < ((wPri_Thd.word.wPhaseA_OverCurr * 7) - IP_HIGH_CURR_HYSTER))
		  {
			Data.word.Pri_Status_Flag.PhaseA_OverCurr = 0;
			Data.word.Pri_Status_Flag.PhaseA_Curr_High = 0;
		  }
		}

		if (((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) > (wPri_Thd.word.wPhaseB_OverCurr * 7))
		{
		  Data.word.Pri_Status_Flag.PhaseB_Curr_High =1;
		  if (((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF)*10) > wPri_Thd.word.wPhaseB_OverCurr*8)
		  {
			Data.word.Pri_Status_Flag.PhaseB_OverCurr = 1;
		  }
		  else 
		  {
		    if (((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF)*10) < ((wPri_Thd.word.wPhaseB_OverCurr*8) - IP_OC_CURR_HYSTER))
		    {
			  Data.word.Pri_Status_Flag.PhaseB_OverCurr = 0;
		    }
		  }
		}
		else {
		  if(((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) < ((wPri_Thd.word.wPhaseB_OverCurr * 7) - IP_HIGH_CURR_HYSTER))
		  {
			Data.word.Pri_Status_Flag.PhaseB_OverCurr = 0;
			Data.word.Pri_Status_Flag.PhaseB_Curr_High = 0;
		  }
		}

		if (((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) > wPri_Thd.word.wPhaseC_OverCurr * 7)
		{
		  Data.word.Pri_Status_Flag.PhaseC_Curr_High =1;
		  if (((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF)*10) > (wPri_Thd.word.wPhaseC_OverCurr * 8))
		  {
			Data.word.Pri_Status_Flag.PhaseC_OverCurr = 1;
		  }
		  else 
		  {
		    if (((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF)*10) < ((wPri_Thd.word.wPhaseC_OverCurr*8) - IP_OC_CURR_HYSTER))
		    {
			  Data.word.Pri_Status_Flag.PhaseC_OverCurr = 0;
			}
		  }
		}
		else 
		{
		  if(((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) < (wPri_Thd.word.wPhaseC_OverCurr * 7) - IP_HIGH_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseC_OverCurr = 0;
			Data.word.Pri_Status_Flag.PhaseC_Curr_High = 0;
		  }
		}

		if(((Data_In.word.wSysInfo.word.wSysConfig & 0x0000FFFF) & 0x0040) == 0x0040)
		{
		  if(((Data.word.Primary.RMS_Curr_Neutral & 0x0000FFFF)*10) > wPri_Thd.word.wNeutral_OverCurr)
		  {
			Data.word.Pri_Status_Flag.Neutral_OverCurr = 1;
		  }
		  else 
		  {
			if(((Data.word.Primary.RMS_Curr_Neutral & 0x0000FFFF)*10) < (wPri_Thd.word.wNeutral_OverCurr - IP_NEUTRAL_CURR_HYSTER))
			{
			  Data.word.Pri_Status_Flag.Neutral_OverCurr = 0;
			}
		  }
		}
		else
		{
		  Data.word.Pri_Status_Flag.Neutral_OverCurr = 0;
		}
		if (no_xfmr)
		{
		if ((Data.word.Pri_Status_Flag.PhaseA_OverCurr) || (Data.word.Pri_Status_Flag.PhaseB_OverCurr) || (Data.word.Pri_Status_Flag.PhaseC_OverCurr))
		{
		  if (!event_flag[36])
		  {
			dummy1 = 41;
			write(fd5[1],&dummy1,1);
			event_flag[36] = 1;
			alarm_status |= (1 << IP_OVER_CURRENT);
			alarm_flag = 0;
		  }
		}
		else if ((!Data.word.Pri_Status_Flag.PhaseA_OverCurr) || (!Data.word.Pri_Status_Flag.PhaseB_OverCurr) || (!Data.word.Pri_Status_Flag.PhaseC_OverCurr))
		{
		  if (event_flag[36])
		  {
			dummy1 = 42;
			write(fd5[1],&dummy1,1);
			event_flag[36] = 0;
			alarm_status &= ~(1 << IP_OVER_CURRENT);
			alarm_flag = 0;
		  }
		}
		if ((Data.word.Pri_Status_Flag.PhaseA_Curr_High) || (Data.word.Pri_Status_Flag.PhaseB_Curr_High) || (Data.word.Pri_Status_Flag.PhaseC_Curr_High))
		{
		  if (!event_flag[37])
		  {
			dummy1 = 49;
			write(fd5[1],&dummy1,1);
			event_flag[37] = 1;
		  }
		}
		else if ((!Data.word.Pri_Status_Flag.PhaseA_Curr_High) || (!Data.word.Pri_Status_Flag.PhaseB_Curr_High) || (!Data.word.Pri_Status_Flag.PhaseC_Curr_High))
		{
		  if (event_flag[37])
		  {
			dummy1 = 50;
			write(fd5[1],&dummy1,1);
			event_flag[37] = 0;
		  }
		}
		}
		break;
	  }
	  
	  case SEC:
	  {

		Data.word.Sec_Status_Flag.paraID = 0x0F;
		if ((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) < wSec_Thd.word.wPhaseA_UnderVolt)
		{
		  Data.word.Sec_Status_Flag.PhaseA_UnderVolt = 1;
		}
		else
		{
		  if((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_UnderVolt + VOLT_HYSTER)
		   {
		  	 Data.word.Sec_Status_Flag.PhaseA_UnderVolt = 0;
		   }
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) < wSec_Thd.word.wPhaseB_UnderVolt)
		{
		  Data.word.Sec_Status_Flag.PhaseB_UnderVolt = 1;
		}
		else
		{
		  if((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_UnderVolt + VOLT_HYSTER)
		  {
		    Data.word.Sec_Status_Flag.PhaseB_UnderVolt = 0;
		  }
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) < wSec_Thd.word.wPhaseC_UnderVolt)
		{
		  Data.word.Sec_Status_Flag.PhaseC_UnderVolt = 1;
		}
		else
		{
		  if((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_UnderVolt + VOLT_HYSTER)
		  {
		 	Data.word.Sec_Status_Flag.PhaseC_UnderVolt = 0;
		  }
		}
	    if ((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_OverVolt)
		{
		  Data.word.Sec_Status_Flag.PhaseA_OverVolt = 1;
		}
		else
		{
		  if((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) < wSec_Thd.word.wPhaseA_OverVolt - VOLT_HYSTER)
		  {
		    Data.word.Sec_Status_Flag.PhaseA_OverVolt = 0;
		  }
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_OverVolt)
		{
		  Data.word.Sec_Status_Flag.PhaseB_OverVolt = 1;
		}
		else
		{
		  if((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) < wSec_Thd.word.wPhaseB_OverVolt - VOLT_HYSTER)
		  {
		    Data.word.Sec_Status_Flag.PhaseB_OverVolt = 0;
		  }
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_OverVolt)
		{
		  Data.word.Sec_Status_Flag.PhaseC_OverVolt = 1;
		}
		else
		{
		  if((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) < wSec_Thd.word.wPhaseC_OverVolt - VOLT_HYSTER)
		  {
		    Data.word.Sec_Status_Flag.PhaseC_OverVolt = 0;
		  }
		}
		if ((Data.word.Sec_Status_Flag.PhaseA_UnderVolt) || (Data.word.Sec_Status_Flag.PhaseB_UnderVolt) || (Data.word.Sec_Status_Flag.PhaseC_UnderVolt))
		{
		  if (!event_flag[51])
		  {
	             dummy1 = 51;
	             write(fd5[1],&dummy1,1);
	             event_flag[51] = 1;
		     alarm_status |= (1 << OP_UNDER_VOLTAGE);
		     alarm_flag = 0;
		  }
		}
	    else if  ((!Data.word.Sec_Status_Flag.PhaseA_UnderVolt) || (Data.word.Sec_Status_Flag.PhaseB_UnderVolt) || (Data.word.Sec_Status_Flag.PhaseC_UnderVolt))
		{
		  if (event_flag[51])
		  {
		     dummy1 = 52;
	             write(fd5[1],&dummy1,1);
		     event_flag[51] = 0;
		     alarm_status &= ~(1 << OP_UNDER_VOLTAGE);
		     alarm_flag = 0;
		  }
		}
		if ((Data.word.Sec_Status_Flag.PhaseA_OverVolt) || (Data.word.Sec_Status_Flag.PhaseB_OverVolt) || (Data.word.Sec_Status_Flag.PhaseC_OverVolt))
		{
		  if (!event_flag[53])
		  {
	             dummy1 = 53;
	             write(fd5[1],&dummy1,1);
	             event_flag[53] = 1;
		     alarm_status |= (1 << OP_OVER_VOLTAGE);
		     alarm_flag = 0;
		  }
		}
	    else if  ((!Data.word.Sec_Status_Flag.PhaseA_OverVolt) || (Data.word.Sec_Status_Flag.PhaseB_OverVolt) || (Data.word.Sec_Status_Flag.PhaseC_OverVolt))
		{
		  if (event_flag[53])
		  {
			dummy1 = 54;
			write(fd5[1],&dummy1,1);
			event_flag[53] = 0;
			alarm_status &= ~(1 << OP_OVER_VOLTAGE);
			alarm_flag = 0;
		  }
	    }
		
		// Current Status       
		if ((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) < wSec_Thd.word.wPhaseA_UnderCurr)
		{
		  Data.word.Sec_Status_Flag.PhaseA_UnderCurr = 1;
		}
		else
		{
		  if((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_UnderCurr + OP_UC_CURR_HYSTER)
		  {
		    Data.word.Sec_Status_Flag.PhaseA_UnderCurr = 0;
		  }
		}
                
		if ((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) < wSec_Thd.word.wPhaseB_UnderCurr)
		{
		  Data.word.Sec_Status_Flag.PhaseB_UnderCurr = 1;
		}
		else
		{
		  if((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_UnderCurr + OP_UC_CURR_HYSTER)
		  {
		    Data.word.Sec_Status_Flag.PhaseB_UnderCurr = 0;
		  }
		}
		
		if ((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) < wSec_Thd.word.wPhaseC_UnderCurr)
		{
			Data.word.Sec_Status_Flag.PhaseC_UnderCurr = 1;
		}
		else
		{
		  if((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_UnderCurr + OP_UC_CURR_HYSTER)
		   {
		  	Data.word.Sec_Status_Flag.PhaseC_UnderCurr = 0;
		   }
		}

		if ((Data.word.Sec_Status_Flag.PhaseA_UnderCurr) || (Data.word.Sec_Status_Flag.PhaseB_UnderCurr) || (Data.word.Sec_Status_Flag.PhaseC_UnderCurr))
		{
		  if (!event_flag[55])
		  {
			dummy1 = 55;
			write(fd5[1],&dummy1,1);
			event_flag[55] = 1;
		        alarm_status |= (1 << OP_UNDER_CURRENT);
			alarm_flag = 0;
		  }
		}
		else if ((!Data.word.Sec_Status_Flag.PhaseA_UnderCurr) || (Data.word.Sec_Status_Flag.PhaseB_UnderCurr) || (Data.word.Sec_Status_Flag.PhaseC_UnderCurr))
		{
		  if (event_flag[55])
		  {
			dummy1 = 56;
			write(fd5[1],&dummy1,1);
			event_flag[55] = 0;
		        alarm_status &= ~(1 << OP_UNDER_CURRENT);
			alarm_flag = 0;
		  }
		}

		if (((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseA_OverCurr * 7)
		{
		  //Data.word.Sec_Status_Flag.PhaseA_Curr_High =1;
		  if (((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseA_OverCurr * 8)
		  {
			Data.word.Sec_Status_Flag.PhaseA_OverCurr = 1;
		  }
		  else 
		  {
			if (((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF)*10) < ((wSec_Thd.word.wPhaseA_OverCurr*8) - OP_OC_CURR_HYSTER))
		    {
			  Data.word.Sec_Status_Flag.PhaseA_OverCurr = 0;
			}
		  }
		}
		else {
		  if(((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) < (wSec_Thd.word.wPhaseA_OverCurr * 7) - OP_OC_CURR_HYSTER)
		  {
			Data.word.Sec_Status_Flag.PhaseA_OverCurr = 0;
			//Data.word.Sec_Status_Flag.PhaseA_Curr_High = 0;
		  }
		}
		
		if (((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseB_OverCurr * 7)
		{
		  //Data.word.Sec_Status_Flag.PhaseB_Curr_High =1;
		  if (((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseB_OverCurr*8)
		  {
			Data.word.Sec_Status_Flag.PhaseB_OverCurr = 1;
		  }
		  else 
		  {
			if (((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) < ((wSec_Thd.word.wPhaseB_OverCurr*8) - OP_OC_CURR_HYSTER))
		    {
			  Data.word.Sec_Status_Flag.PhaseB_OverCurr = 0;
			}
		  }
		}
		else 
		{
		  if(((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) < ((wSec_Thd.word.wPhaseB_OverCurr * 7) - OP_HIGH_CURR_HYSTER))
		  {
			Data.word.Sec_Status_Flag.PhaseB_OverCurr = 0;
			//Data.word.Sec_Status_Flag.PhaseB_Curr_High = 0;
		  }
		}

		if (((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseC_OverCurr * 7)
		{
		  //Data.word.Sec_Status_Flag.PhaseC_Curr_High =1;
		  if (((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) > (wSec_Thd.word.wPhaseC_OverCurr * 8))
		  {
			Data.word.Sec_Status_Flag.PhaseC_OverCurr = 1;
		  }
		  else 
		  {
			if (((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) < ((wSec_Thd.word.wPhaseC_OverCurr * 8) - OP_OC_CURR_HYSTER))
			{
			  Data.word.Sec_Status_Flag.PhaseC_OverCurr = 0;
			}
		  }
		}
		else {
		  if(((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) < ((wSec_Thd.word.wPhaseC_OverCurr * 7) - OP_HIGH_CURR_HYSTER))
		  {
			Data.word.Sec_Status_Flag.PhaseC_OverCurr = 0;
			//Data.word.Sec_Status_Flag.PhaseC_Curr_High = 0;
		  }
		}
		if ((Data.word.Sec_Status_Flag.PhaseA_OverCurr) || (Data.word.Sec_Status_Flag.PhaseB_OverCurr) || (Data.word.Sec_Status_Flag.PhaseC_OverCurr))
		{
		  if (!event_flag[66])
		  {
			dummy1 = 66;
			write(fd5[1],&dummy1,1);
			event_flag[66] = 1;
			alarm_status |= (1 << OP_OVER_CURRENT);
			alarm_flag = 0;
		  }
		}
		else if ((!Data.word.Sec_Status_Flag.PhaseA_OverCurr) || (Data.word.Sec_Status_Flag.PhaseB_OverCurr) || (Data.word.Sec_Status_Flag.PhaseC_OverCurr))
		{
		  if (event_flag[66])
		  {
			dummy1 = 67;
			write(fd5[1],&dummy1,1);
			event_flag[66] = 0;
			alarm_status &= ~(1 << OP_OVER_CURRENT);
			alarm_flag = 0;
		  }
		}
		/*if ((Data.word.Sec_Status_Flag.PhaseA_Curr_High) || (Data.word.Sec_Status_Flag.PhaseB_Curr_High) || (Data.word.Sec_Status_Flag.PhaseC_Curr_High))
		{
		  if (!event_flag[64])
		  {
			dummy1 = 64;
			write(fd5[1],&dummy1,1);
			event_flag[64] = 1;
		  }
		}
		else if ((!Data.word.Sec_Status_Flag.PhaseA_Curr_High) || (!Data.word.Sec_Status_Flag.PhaseB_Curr_High) || (!Data.word.Sec_Status_Flag.PhaseC_Curr_High))
		{
		  if (event_flag[64])
		  {
			dummy1 = 65;
			write(fd5[1],&dummy1,1);
			event_flag[64] = 0;
		  }
		}*/
		if((Data.word.Secondary.RMS_Curr_Neutral & 0x0000FFFF) > wSec_Thd.word.wNeutral_OverCurr)
		  Data.word.Sec_Status_Flag.Neutral_OverCurr = 1;
	    
		else {
		  if((Data.word.Secondary.RMS_Curr_Neutral & 0x0000FFFF) < (wSec_Thd.word.wNeutral_OverCurr - OP_NEUTRAL_CURR_HYSTER))
			Data.word.Sec_Status_Flag.Neutral_OverCurr = 0;
		}
		if (Data.word.Sec_Status_Flag.Neutral_OverCurr)
		{
		  if (!event_flag[38])
		  {

			dummy1 = 37;
			write(fd5[1],&dummy1,1);
			event_flag[38] = 1;
	                alarm_status |= (1 << NEUTRAL_CURRENT);
			alarm_flag = 0;
		  }
		}
		else if (!Data.word.Sec_Status_Flag.Neutral_OverCurr)
		{
		  if (event_flag[38])
		  {

			dummy1 = 38;
			write(fd5[1],&dummy1,1);
			event_flag[38] = 0;
			alarm_status &= ~(1 << NEUTRAL_CURRENT);
			alarm_flag = 0;
		  }
		}

		if (((Data.word.Secondary.Volt_THD_Phase_A & 0xFFFF) > wSec_Thd.word.wOverVoltTHD) && ((Data.word.Secondary.Volt_THD_Phase_A & 0xFFFF) != 0xFFFF))
		{

		  Data.word.Sec_Status_Flag.PhaseA_OverTHDVolt = 1;
		}
		else
		{
		  if ((Data.word.Secondary.Volt_THD_Phase_A & 0xFFFF) < (wSec_Thd.word.wOverVoltTHD - OP_VTHD_HYSTER))
		  {

			Data.word.Sec_Status_Flag.PhaseA_OverTHDVolt = 0;
		  }
		}
	      
		if (((Data.word.Secondary.Volt_THD_Phase_B & 0xFFFF) > wSec_Thd.word.wOverVoltTHD) && ((Data.word.Secondary.Volt_THD_Phase_B & 0xFFFF) != 0xFFFF))
		{

		  Data.word.Sec_Status_Flag.PhaseB_OverTHDVolt = 1;
		}
		else
		{
		  if ((Data.word.Secondary.Volt_THD_Phase_B & 0xFFFF) < (wSec_Thd.word.wOverVoltTHD - OP_VTHD_HYSTER))
		  {
			Data.word.Sec_Status_Flag.PhaseB_OverTHDVolt = 0;
		  }
		}
		if (((Data.word.Secondary.Volt_THD_Phase_C & 0xFFFF) > wSec_Thd.word.wOverVoltTHD) && ((Data.word.Secondary.Volt_THD_Phase_C & 0xFFFF) != 0xFFFF))
		{

		  Data.word.Sec_Status_Flag.PhaseC_OverTHDVolt = 1;
		}
		else
		{
		  if ((Data.word.Secondary.Volt_THD_Phase_C & 0xFFFF) < (wSec_Thd.word.wOverVoltTHD - OP_VTHD_HYSTER))
		  {

			Data.word.Sec_Status_Flag.PhaseC_OverTHDVolt = 0;
		  }
		}

	        if ((Data.word.Secondary.RMS_Curr_Phase_A & 0xFFFF) ||
		    (Data.word.Secondary.RMS_Curr_Phase_B & 0xFFFF) || 
		    (Data.word.Secondary.RMS_Curr_Phase_C & 0xFFFF))
	        {
		if (((Data.word.Secondary.Curr_THD_Phase_A & 0xFFFF) > wSec_Thd.word.wOverCurrTHD) && ((Data.word.Secondary.Curr_THD_Phase_A & 0xFFFF)!= 0xFFFF))
		{
		  Data.word.Sec_Status_Flag.PhaseA_OverTHDCurr = 1;
		}
		else
		{
		  if ((Data.word.Secondary.Curr_THD_Phase_A & 0xFFFF) < (wSec_Thd.word.wOverCurrTHD - OP_ITHD_HYSTER))
		    Data.word.Sec_Status_Flag.PhaseA_OverTHDCurr = 0;
		}
		if (((Data.word.Secondary.Curr_THD_Phase_B & 0xFFFF) > wSec_Thd.word.wOverCurrTHD) && ((Data.word.Secondary.Curr_THD_Phase_B & 0xFFFF)!= 0xFFFF))
		{
		  Data.word.Sec_Status_Flag.PhaseB_OverTHDCurr = 1;
		}
		else
		{
		  if ((Data.word.Secondary.Curr_THD_Phase_B & 0xFFFF) < (wSec_Thd.word.wOverCurrTHD - OP_ITHD_HYSTER))
			Data.word.Sec_Status_Flag.PhaseB_OverTHDCurr = 0;
		}
		if (((Data.word.Secondary.Curr_THD_Phase_C & 0xFFFF) > wSec_Thd.word.wOverCurrTHD) && ((Data.word.Secondary.Curr_THD_Phase_C & 0xFFFF)!= 0xFFFF))
		{
		  Data.word.Sec_Status_Flag.PhaseC_OverTHDCurr = 1;
		}
		else
		{
		  if ((Data.word.Secondary.Curr_THD_Phase_C & 0xFFFF) < (wSec_Thd.word.wOverCurrTHD - OP_ITHD_HYSTER))
			Data.word.Sec_Status_Flag.PhaseC_OverTHDCurr = 0;
		}
	        }
	       else
		{
		   Data.word.Sec_Status_Flag.PhaseA_OverTHDCurr = 0;
		   Data.word.Sec_Status_Flag.PhaseB_OverTHDCurr = 0;
		   Data.word.Sec_Status_Flag.PhaseC_OverTHDCurr = 0;
		}


	        if ((Data.word.Sec_Status_Flag.PhaseA_OverTHDVolt)
		 ||(Data.word.Sec_Status_Flag.PhaseB_OverTHDVolt)
	         ||(Data.word.Sec_Status_Flag.PhaseC_OverTHDVolt))
		{
		   if (!event_flag[25])
		  {

			dummy1 = 25;
			write(fd5[1],&dummy1,1);
			event_flag[25] = 1;
	                alarm_status |= (1 << OUTPUT_VTHD);
			alarm_flag = 0;
		  }
		}
	       else
		{
		  if (event_flag[25])
		  {

			dummy1 = 26;
			write(fd5[1],&dummy1,1);
			event_flag[25] = 0;
	                alarm_status &= ~(1 << OUTPUT_VTHD);
			alarm_flag = 0;
		  }
		}

	      if ((Data.word.Sec_Status_Flag.PhaseA_OverTHDCurr)
		 ||(Data.word.Sec_Status_Flag.PhaseB_OverTHDCurr)
	         ||(Data.word.Sec_Status_Flag.PhaseC_OverTHDCurr))
		{
		   if (!event_flag[27])
		  {

			dummy1 = 27;
			write(fd5[1],&dummy1,1);
			event_flag[27] = 1;
	        alarm_status |= (1 << OUTPUT_CTHD);  
			alarm_flag = 0;
		  }
		}
	       else
		{
		  if (event_flag[27])
		  {

			dummy1 = 28;
			write(fd5[1],&dummy1,1);
			event_flag[27] = 0;
	                alarm_status &= ~(1 << OUTPUT_CTHD);
			alarm_flag = 0;
		  }
		}
		
	    break;
	  }	 	
	  case PANEL1:
	  {
		
		if ((((Data_In.word.wSysInfo.word.wConfiguration & 0x0000FFFF)&0xFF00)>>8) < 0x01)
		{
		  Data_In.word.wPanelAct[0].word.Active_Inactive[0] = 0;
		  Data_In.word.wPanelAct[0].word.Active_Inactive[1] = 0;
		  Data_In.word.wPanelAct[0].word.Active_Inactive[2] = 0;
		  Data_In.word.wPanelAct[0].word.Active_Inactive[3] = 0;
	      break;
	    }
		
		Data.word.Panel1_Status_Undercurr[0].paraID = 0x1C;
		Data.word.Panel1_Status_Undercurr[1].paraID = 0x5C;
		Data.word.Panel1_Status_Undercurr[2].paraID = 0x9C;
		Data.word.Panel1_Status_Undercurr[3].paraID = 0xDC;

		Data.word.Panel1_Status_Overcurr[0].paraID = 0x2C;
		Data.word.Panel1_Status_Overcurr[1].paraID = 0x6C;
		Data.word.Panel1_Status_Overcurr[2].paraID = 0xAC;
		Data.word.Panel1_Status_Overcurr[3].paraID = 0xEC;

		for (i=0; i<21; i++)
		{
				
		  alarm = 0x00000001 << i;
		//****************************UNDER CURRENT Panel 1 Strip 1  (1~21)**************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[0] & alarm)
		  {
			if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[0][i] >> 16) & 0xFFFF))
			{
			  Data.array[SEC_STATUS_FLAG] |= alarm;


			  *(map + LOAD1_OFFSET) |= alarm;
			}
		    else 
		    {
			  if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[0][i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
			  {
				Data.array[SEC_STATUS_FLAG] &= (~alarm);


				*(map + LOAD1_OFFSET) &= (~alarm);
			  }
			}
		  }
		  else
		  {
			Data.array[SEC_STATUS_FLAG] &= (~alarm);
			*(map + LOAD1_OFFSET) &= (~alarm);
		  }
		  //****************************UNDER CURRENT Panel 1 Strip 2  (21~42)**************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[1] & alarm)
		  {
			if((Data.array[SEC_OFFSET + 21+ i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[0][21+i] >> 16) & 0xFFFF))
			{
			  Data.array[PANEL11_UNDERCURR_STATUS_FLAG] |= alarm;
			  *(map + PANEL11_UNDERCURR_FLAG) |= alarm;
			}
			else 
			{
			  if((Data.array[SEC_OFFSET + 21+ i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[0][21+i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
			  {
				Data.array[PANEL11_UNDERCURR_STATUS_FLAG] &= (~alarm);
				*(map + PANEL11_UNDERCURR_FLAG) &= (~alarm);
			  }
			} 
		  }
		  else
	      {
			  Data.array[PANEL11_UNDERCURR_STATUS_FLAG] &= (~alarm);
			  *(map + PANEL11_UNDERCURR_FLAG) &= (~alarm);
		  }
		}
                 
		for (i=0; i<21; i++)
		{
		  alarm = 0x00000001 << i;
		  //********************************UNDER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[2] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[0][42+i] >> 16) & 0xFFFF))
			{
				Data.array[PANEL12_UNDERCURR_STATUS_FLAG] |= alarm;
				*(map + PANEL12_UNDERCURR_FLAG) |= alarm;
			}
		    else
			{
			  if ((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[0][42+i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
			  {
			    Data.array[PANEL12_UNDERCURR_STATUS_FLAG] &= (~alarm);
			    *(map + PANEL12_UNDERCURR_FLAG) &= (~alarm);
              }
			}
		  }
		  else
          {
		    Data.array[PANEL12_UNDERCURR_STATUS_FLAG] &= (~alarm);
		    *(map + PANEL12_UNDERCURR_FLAG) &= (~alarm);
		  } 
		  //********************************UNDER CURRENT Panel 1 Strip 4(63~84)******************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[3] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[0][63+i] >> 16) & 0xFFFF))
			{
			  Data.array[PANEL13_UNDERCURR_STATUS_FLAG] |= alarm;
			  *(map + PANEL13_UNDERCURR_FLAG) |= alarm;
            }
		    else
   			{
			  if ((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[0][63+i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
			  {
			    Data.array[PANEL13_UNDERCURR_STATUS_FLAG] &= (~alarm);
			    *(map + PANEL13_UNDERCURR_FLAG) &= (~alarm);
			  }
			}
		  }
		  else
		  {
		    Data.array[PANEL13_UNDERCURR_STATUS_FLAG] &= (~alarm);
		    *(map + PANEL13_UNDERCURR_FLAG) &= (~alarm);
		  }
		}

		for (i=0; i<21; i++)
		{
		  alarm = 1 << i;
	          //********************************OVER CURRENT Panel 1 Strip 1(1~21)*****************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[0] & alarm)
		  {
		    if ((Data.array[SEC_OFFSET + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[0][i]) & 0xFFFF))
		    {
			  Data.array[PANEL14_UNDERCURR_STATUS_FLAG] |= alarm;
			  *(map + PANEL14_UNDERCURR_FLAG) |= alarm;
     		}
		    else
		    {
		      if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[0][i]) & 0xFFFF) - BRANCH_OC_HYSTER))
		      {
		        Data.array[PANEL14_UNDERCURR_STATUS_FLAG] &= (~alarm);
		        *(map + PANEL14_UNDERCURR_FLAG) &= (~alarm);
			  }
		    }
		  }
		  else
		  {
			Data.array[PANEL14_UNDERCURR_STATUS_FLAG] &= (~alarm);
			*(map + PANEL14_UNDERCURR_FLAG) &= (~alarm);
		  }

		  //********************************OVER CURRENT Panel 1 Strip 2(21~42)*****************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[1] & alarm)
		  {
		    if ((Data.array[SEC_OFFSET + 21 + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[0][21+i]) & 0xFFFF))
			{
			  Data.array[PANEL11_OVERCURR_STATUS_FLAG] |= alarm;
			  *(map + PANEL11_OVERCURR_FLAG) |= alarm;
			}
		    else
		    {
			  if((Data.array[SEC_OFFSET + 21 + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[0][21+i]) & 0xFFFF) - BRANCH_OC_HYSTER))
			  {
				Data.array[PANEL11_OVERCURR_STATUS_FLAG] &= (~alarm);
				*(map + PANEL11_OVERCURR_FLAG) &= (~alarm);
			  }
			}
		  }
		  else{
			Data.array[PANEL11_OVERCURR_STATUS_FLAG] &= (~alarm);
			*(map + PANEL11_OVERCURR_FLAG) &= (~alarm);
		  }
		}
     		
		for (i=0;i<21;i++)
		{
		  alarm = 0x00000001 << i;
		//********************************OVER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[2] & alarm)
		  {

		    if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[0][42+i]) & 0xFFFF))
			{
			  Data.array[PANEL12_OVERCURR_STATUS_FLAG] |= alarm;
			  *(map + PANEL12_OVERCURR_FLAG) |= alarm;
			}
		    else
		    {
			  if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[0][42+i]) & 0xFFFF) - BRANCH_OC_HYSTER))
			  {
			    Data.array[PANEL12_OVERCURR_STATUS_FLAG] &= (~alarm);
				*(map + PANEL12_OVERCURR_FLAG) &= (~alarm);
			  }
		    }
		  }
		  else
		  {
			Data.array[PANEL12_OVERCURR_STATUS_FLAG] &= (~alarm);
			*(map + PANEL12_OVERCURR_FLAG) &= (~alarm);}

		//********************************OVER CURRENT Panel 1 Strip 2(12~21)*****************************//
			if (Data_In.word.wPanelAct[0].word.Active_Inactive[3] & alarm)
			{

			  if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[0][63+i]) & 0xFFFF))
			  {
				Data.array[PANEL13_OVERCURR_STATUS_FLAG] |= alarm;
				*(map + PANEL13_OVERCURR_FLAG) |= alarm;
			  }
			  else
			  {
			    if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[0][63+i]) & 0xFFFF) - BRANCH_OC_HYSTER))
				{
				  Data.array[PANEL13_OVERCURR_STATUS_FLAG] &= (~alarm);
				  *(map + PANEL13_OVERCURR_FLAG) &= (~alarm);
				}
			  }
			}
			else
			{
			  Data.array[PANEL13_OVERCURR_STATUS_FLAG] &= (~alarm);
			  *(map + PANEL13_OVERCURR_FLAG) &= (~alarm);
			}
 		  //********************************OVER CURRENT DEMAND Panel 1 Strip 1(12~21)*****************************//

		  }
		  if ((Data.array[PANEL14_UNDERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL11_OVERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL12_OVERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL13_OVERCURR_STATUS_FLAG] & 0x001FFFFF))
		  {

		    if(!event_flag[68])
			{
			  dummy = 68;
			  write(fd5[1],&dummy,1);
			  event_flag[68] = 1;
			  alarm_status |= (1 << BRANCH_OVER_CURRENT);
			  alarm_flag = 0;
			}
		  }
		  else
		  {
			if(event_flag[68])
			{
			  dummy = 69;
			  write(fd5[1],&dummy,1);
			  event_flag[68] = 0;
			  alarm_status &= ~(1 << BRANCH_OVER_CURRENT);
			  alarm_flag = 0;
			}
		  }
      
	       break;  
	    }
	    case PANEL2:
			
		  if ((((Data_In.word.wSysInfo.word.wConfiguration & 0x0000FFFF)&0xFF00)>>8) < 0x02)
		  {
			Data_In.word.wPanelAct[1].word.Active_Inactive[0] = 0;
			Data_In.word.wPanelAct[1].word.Active_Inactive[1] = 0;

			Data_In.word.wPanelAct[1].word.Active_Inactive[2] = 0;
			Data_In.word.wPanelAct[1].word.Active_Inactive[3] = 0;
		    break;
		  }
		  Data.word.Panel2_Status_Undercurr[0].paraID = 0x1B;
		  Data.word.Panel2_Status_Undercurr[1].paraID = 0x5B;
		  Data.word.Panel2_Status_Undercurr[2].paraID = 0x9B;
		  Data.word.Panel2_Status_Undercurr[3].paraID = 0xDB;

		  Data.word.Panel2_Status_Overcurr[0].paraID = 0x2B;
		  Data.word.Panel2_Status_Overcurr[1].paraID = 0x6B;
		  Data.word.Panel2_Status_Overcurr[2].paraID = 0xAB;
		  Data.word.Panel2_Status_Overcurr[3].paraID = 0xEB;

	      for (i=0; i<21; i++)
		  {
				
		    alarm = 0x00000001 << i;
		  //****************************UNDER CURRENT Panel 1 Strip 1  (1~21)**************************//
			if(Data_In.word.wPanelAct[1].word.Active_Inactive[0] & alarm)
			{
			  if((Data.array[RMS_0_OFFSET + i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[1][i] >> 16) & 0xFFFF))
			  {
				Data.array[PANEL14_OVERCURR_STATUS_FLAG] |= alarm;
				*(map + PANEL14_OVERCURR_FLAG) |= alarm;
			  }
			  else {
				if((Data.array[RMS_0_OFFSET + i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[1][i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
			    {
				  Data.array[PANEL14_OVERCURR_STATUS_FLAG] &= (~alarm);
				  *(map + PANEL14_OVERCURR_FLAG) &= (~alarm);
				}
			  }
			}
			else
			{
			  Data.array[PANEL14_OVERCURR_STATUS_FLAG] &= (~alarm);
			  *(map + PANEL14_OVERCURR_FLAG) &= (~alarm);}
			//****************************UNDER CURRENT Panel 1 Strip 2  (21~42)**************************//
			  if(Data_In.word.wPanelAct[1].word.Active_Inactive[1] & alarm)
			  {
				if((Data.array[RMS_0_OFFSET + 21+ i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[1][21+i] >> 16) & 0xFFFF))
				{
				  Data.array[PANEL21_UNDERCURR_STATUS_FLAG] |= alarm;
				  *(map + PANEL21_UNDERCURR_FLAG) |= alarm;
				}
				else 
				{
				  if((Data.array[RMS_0_OFFSET + 21+ i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[1][21+i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
				  {
					Data.array[PANEL21_UNDERCURR_STATUS_FLAG] &= (~alarm);
					*(map + PANEL21_UNDERCURR_FLAG) &= (~alarm);
				  }
				}
			  }
			  else
			  {
				Data.array[PANEL21_UNDERCURR_STATUS_FLAG] &= (~alarm);
				*(map + PANEL21_UNDERCURR_FLAG) &= (~alarm);
			  }
	  
		    }
                 
		  for (i=0; i<21; i++)
		  {
		    alarm = 0x00000001 << i;
		  //********************************UNDER CURRENT Panel 1 Strip 3(42~63)*****************************//
		    if (Data_In.word.wPanelAct[1].word.Active_Inactive[2] & alarm)
			{
			  if((Data.array[RMS_0_OFFSET + 42 + i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[1][42+i] >> 16) & 0xFFFF))
			  {
				Data.array[PANEL22_UNDERCURR_STATUS_FLAG] |= alarm;
				*(map + PANEL22_UNDERCURR_FLAG) |= alarm;
			  }
			  else
			  {
			    if ((Data.array[RMS_0_OFFSET + 42 + i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[1][42+i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
			    {
				  Data.array[PANEL22_UNDERCURR_STATUS_FLAG] &= (~alarm);
				  *(map + PANEL22_UNDERCURR_FLAG) &= (~alarm);
			    }
			  }
			}
		    else
            {
		      Data.array[PANEL22_UNDERCURR_STATUS_FLAG] &= (~alarm); 
		      *(map + PANEL22_UNDERCURR_FLAG) &= (~alarm);
			}
		  //********************************UNDER CURRENT Panel 1 Strip 4(63~84)******************************//
		    if (Data_In.word.wPanelAct[1].word.Active_Inactive[3] & alarm)
		    {
		      if((Data.array[RMS_0_OFFSET + 63 + i] & 0x0000FFFF) < ((Data_In.word.Max_Min_Limit[1][63+i] >> 16) & 0xFFFF))
			  {
				Data.array[PANEL23_UNDERCURR_STATUS_FLAG] |= alarm;
				*(map + PANEL23_UNDERCURR_FLAG) |= (alarm);
			  }
		      else
			  {
				if ((Data.array[RMS_0_OFFSET + 63 + i] & 0x0000FFFF) > (((Data_In.word.Max_Min_Limit[1][63+i] >> 16) & 0xFFFF) + BRANCH_UC_HYSTER))
				{
				  Data.array[PANEL23_UNDERCURR_STATUS_FLAG] &= (~alarm);
				  *(map + PANEL23_UNDERCURR_FLAG) &= (~alarm);
				}
			  }
		    }
		    else
			{
		      Data.array[PANEL23_UNDERCURR_STATUS_FLAG] &= (~alarm);
	          *(map + PANEL23_UNDERCURR_FLAG) &= (~alarm);
			}
		  
		  //********************************UNDER CURRENT DEMAND Panel 1 Strip 1(17~21)*****************************//
		  
		}
		
		
		
		for (i=0; i<21; i++)
		{
		  alarm = 0x00000001 << i;
	          //********************************OVER CURRENT Panel 1 Strip 1(1~21)*****************************//
		  if(Data_In.word.wPanelAct[1].word.Active_Inactive[0] & alarm)
		  {
		    //printf("\nPanel21_Overcurrent_alarm\n");
		    if ((Data.array[RMS_0_OFFSET + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[1][i]) & 0xFFFF))
			{
			  Data.array[PANEL24_UNDERCURR_STATUS_FLAG] |= alarm;
			}
		    else
		    {
		      if((Data.array[RMS_0_OFFSET + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[1][i]) & 0xFFFF) - BRANCH_OC_HYSTER))
			  {
		       Data.array[PANEL24_UNDERCURR_STATUS_FLAG] &= (~alarm);
			  }
		    }
		  }
		  else
		  {
		    Data.array[PANEL24_UNDERCURR_STATUS_FLAG] &= (~alarm);
		  }

		  //********************************OVER CURRENT Panel 1 Strip 2(21~42)*****************************//
		  if(Data_In.word.wPanelAct[1].word.Active_Inactive[1] & alarm)
		  {
		    
		    if ((Data.array[RMS_0_OFFSET + 21 + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[1][21+i]) & 0xFFFF))
			{
			  Data.array[PANEL21_OVERCURR_STATUS_FLAG] |= alarm;
			}
		    else
		    {
			  if((Data.array[RMS_0_OFFSET + 21 + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[1][21+i]) & 0xFFFF) - BRANCH_OC_HYSTER))
			  {
				Data.array[PANEL21_OVERCURR_STATUS_FLAG] &= (~alarm);
			  }
			}
		  }
		  else
		  {
			Data.array[PANEL21_OVERCURR_STATUS_FLAG] &= (~alarm);
		  }

		}
     		
		for (i=0;i<21;i++)
		{
		  alarm = 0x00000001 << i;
		  //********************************OVER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[1].word.Active_Inactive[2] & alarm)
		  {
			if((Data.array[RMS_0_OFFSET + 42 + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[1][42+i]) & 0xFFFF))
			{
			  Data.array[PANEL22_OVERCURR_STATUS_FLAG] |= alarm;
			  *(map + PANEL22_OVERCURR_FLAG) |= (alarm);
			}
			else
			{
			  if((Data.array[RMS_0_OFFSET + 42 + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[1][42+i]) & 0xFFFF) - BRANCH_OC_HYSTER))
			  {
				Data.array[PANEL22_OVERCURR_STATUS_FLAG] &= (~alarm);
				*(map + PANEL22_OVERCURR_FLAG) &= (~alarm);}
			  }
			}
			else
			{
			  Data.array[PANEL22_OVERCURR_STATUS_FLAG] &= (~alarm);
			  *(map + PANEL22_OVERCURR_FLAG) &= (~alarm);}
		
		  //********************************OVER CURRENT Panel 1 Strip 2(12~21)*****************************//
			  if (Data_In.word.wPanelAct[1].word.Active_Inactive[3] & alarm)
			  {
			    if((Data.array[RMS_0_OFFSET + 63 + i] & 0x0000FFFF) > ((Data_In.word.Max_Min_Limit[1][63+i]) & 0xFFFF))
				{
				  Data.array[PANEL23_OVERCURR_STATUS_FLAG] |= alarm;
				  *(map + PANEL23_OVERCURR_FLAG) |= (alarm);
				}
			  else
			  {
				if((Data.array[RMS_0_OFFSET + 63 + i] & 0x0000FFFF) < (((Data_In.word.Max_Min_Limit[1][63+i]) & 0xFFFF) - BRANCH_OC_HYSTER))
				{
			  	  Data.array[PANEL23_OVERCURR_STATUS_FLAG] &= (~alarm);
			  	  *(map + PANEL23_OVERCURR_FLAG) &= (~alarm);
				}
			  }
			}
			else
			{
			  Data.array[PANEL23_OVERCURR_STATUS_FLAG] &= (~alarm);
			  *(map + PANEL23_OVERCURR_FLAG) &= (~alarm);}
            }
		    
		    if ((Data.array[PANEL24_UNDERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL21_OVERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL22_OVERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL23_OVERCURR_STATUS_FLAG] & 0x001FFFFF))
		     {
				    
			  if(!event_flag[69])
			  {
				 alarm_status |= (1 << BRANCH_OVER_CURRENT);
				 
				 alarm_flag = 0;
     				 dummy = 68;
				 write(fd5[1],&dummy,1);
				 event_flag[69] = 1;
				 
				 
				 
				 	
			  }
		     }
		    else 
		     {
			   if(event_flag[69])
			   {
				alarm_status &= ~(1 << BRANCH_OVER_CURRENT);
				alarm_flag = 0;
				dummy = 69;
				write(fd5[1],&dummy,1);
				event_flag[69] = 0;
				
				
				
			   }
			  //}
		     }
	       
	        break;  
		default:
		break;
	  }
}

void Max_Min_Parameter(unsigned int reg)
{
	unsigned int i;
	unsigned int *ptCTMaxCurDemand;
	unsigned int *ptCTMaxKWDemand;
	unsigned int *ptMinCurrent;
	unsigned int *ptMaxCurrent;
	unsigned int *ptCTCurDemand;
	unsigned int *ptCTKwDemand;
	unsigned int *ptCTMaxCurDemand24hr;
	unsigned int *ptCTMaxKWDemand24hr;
	
	switch (reg)
	  {
		case PRIMARY:
		{
		  if (((Data.word.Max_Min_Para[0].Max_Volt_PhaseA & 0xFFFF) < (Data.word.Primary.L2N_Volt_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Volt_PhaseA & 0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Volt_PhaseA = ((unsigned long)(0xA00 << 16))|((Data.word.Primary.L2N_Volt_Phase_A&0xFFFF));
			
			
		  }
		  if (((Data.word.Max_Min_Para[0].Max_Volt_PhaseB&0xFFFF) < (Data.word.Primary.L2N_Volt_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Volt_PhaseB&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Volt_PhaseB = ((unsigned long)(0xA01 << 16))|((Data.word.Primary.L2N_Volt_Phase_B&0xFFFF));
			
			
			
		  }
		  if (((Data.word.Max_Min_Para[0].Max_Volt_PhaseC&0xFFFF) < (Data.word.Primary.L2N_Volt_Phase_C&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Volt_PhaseC&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Volt_PhaseC = ((unsigned long)(0xA02 << 16))|((Data.word.Primary.L2N_Volt_Phase_C&0xFFFF));
			
			
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Volt_PhaseA&0xFFFF) > (Data.word.Primary.L2N_Volt_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Volt_PhaseA&0xFFFF) == 0))
	  	  {
		    Data.word.Max_Min_Para[0].Min_Volt_PhaseA = ((unsigned long)(0xA03 << 16))|((Data.word.Primary.L2N_Volt_Phase_A&0xFFFF));
		    
		    
		    
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Volt_PhaseB&0xFFFF) > (Data.word.Primary.L2N_Volt_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Volt_PhaseB&0xFFFF) == 0))
		  {
			Data.word.Max_Min_Para[0].Min_Volt_PhaseB = ((unsigned long)(0xA04 << 16))|((Data.word.Primary.L2N_Volt_Phase_B&0xFFFF));
			
			
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Volt_PhaseC&0xFFFF) > (Data.word.Primary.L2N_Volt_Phase_C&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Volt_PhaseC&0xFFFF) == 0))
		  {
			Data.word.Max_Min_Para[0].Min_Volt_PhaseC = ((unsigned long)(0xA05 << 16))|((Data.word.Primary.L2N_Volt_Phase_C&0xFFFF));
			

		  }
		  if (((Data.word.Max_Min_Para[0].Max_Curr_PhaseA&0xFFFF) < (Data.word.Primary.RMS_Curr_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_PhaseA&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Curr_PhaseA = ((unsigned long)(0xA06 << 16))|((Data.word.Primary.RMS_Curr_Phase_A&0xFFFF));
			
			
		  }
		  if (((Data.word.Max_Min_Para[0].Max_Curr_PhaseB&0xFFFF) < (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_PhaseB&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Curr_PhaseB = ((unsigned long)(0xA07 << 16))|((Data.word.Primary.RMS_Curr_Phase_B&0xFFFF));
			
		  }
		  if (((Data.word.Max_Min_Para[0].Max_Curr_PhaseC&0xFFFF) < (Data.word.Primary.RMS_Curr_Phase_C&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_PhaseC&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Curr_PhaseC = ((unsigned long)(0xA08 << 16))|((Data.word.Primary.RMS_Curr_Phase_C&0xFFFF));
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Curr_PhaseA&0xFFFF) > (Data.word.Primary.RMS_Curr_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_PhaseA&0xFFFF) == 0))
		  {
			Data.word.Max_Min_Para[0].Min_Curr_PhaseA = ((unsigned long)(0xA0A << 16))|((Data.word.Primary.RMS_Curr_Phase_A&0xFFFF));
			
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Curr_PhaseB&0xFFFF) > (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_PhaseB&0xFFFF) == 0))
		  {
			Data.word.Max_Min_Para[0].Min_Curr_PhaseB = ((unsigned long)(0xA0B << 16))|((Data.word.Primary.RMS_Curr_Phase_B&0xFFFF));
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Curr_PhaseC&0xFFFF) > (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_PhaseC&0xFFFF) == 0))	
		  {
			Data.word.Max_Min_Para[0].Min_Curr_PhaseC = ((unsigned long)(0xA0C << 16))|((Data.word.Primary.RMS_Curr_Phase_C&0xFFFF));
			
		  }
		  if (((Data.word.Max_Min_Para[0].Max_Curr_Neutral&0xFFFF) < (Data.word.Primary.RMS_Curr_Neutral&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_Neutral&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Curr_Neutral = ((unsigned long)(0xA09 << 16))|((Data.word.Primary.RMS_Curr_Neutral&0xFFFF)); 
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Curr_Neutral&0xFFFF) > (Data.word.Primary.RMS_Curr_Neutral&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_Neutral&0xFFFF) == 0))
		  {
			Data.word.Max_Min_Para[0].Min_Curr_Neutral = ((unsigned long)(0xA0D << 16))|((Data.word.Primary.RMS_Curr_Neutral&0xFFFF));
			
		  }
		  if (((Data.word.Max_Min_Para[0].Max_Freq_PhaseA&0xFFFF) < (Data.word.Secondary.Frequency&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Freq_PhaseA&0xFFFF) == 0xFFFF))
		  {
			Data.word.Max_Min_Para[0].Max_Freq_PhaseA = ((unsigned long)(0xA0E << 16))|((Data.word.Secondary.Frequency&0xFFFF)); 
			
		  }
		  if (((Data.word.Max_Min_Para[0].Min_Freq_PhaseA&0xFFFF) > (Data.word.Secondary.Frequency&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Freq_PhaseA&0xFFFF) == 0))
		  {
			Data.word.Max_Min_Para[0].Min_Freq_PhaseA = ((unsigned long)(0xA11 << 16))|((Data.word.Secondary.Frequency&0xFFFF));	
			
		  }	
	      break;
	    }
	    case SEC1:
		break;
	    case SEC2:
		break;
	    case SEC3:
		break;
    //Data.word.Max_Min_Para.Min_Curr_PhaseA
	    case PANEL1:
		{
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[MAX_MIN_PRIMARY];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_3];
		  ptMinCurrent = (unsigned int *)&Data.array[MAXIMUM_3_OFFSET];
		  ptMaxCurrent = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_3];
		  ptCTCurDemand	= (unsigned int *)&Data.array[MINIMUM_3_OFFSET];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_CURR_DEMAND_3];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_KW_DEMAND_3];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_3];
		  for (i=0; i < 84; i++)
		  {

		    if ((((*(ptMaxCurrent + i)&0x0000FFFF) < (Data.array[SEC_OFFSET + i]&0x0000FFFF)) || ((*(ptMaxCurrent + i)&0x0000FFFF) == 0xFFFF)) && ((Data.array[SEC_OFFSET + i]&0x0000FFFF) != 0xFFFF))
			{
			  *(ptMaxCurrent + i) = ((unsigned long)((0x1800|i)|(0<<14))<<16)|(Data.array[SEC_OFFSET + i]&0x0000FFFF);
			  //Reg[CT_MAX_KW_DEMAND_3 + i].reg_d.reg_value = ((*(ptMaxCurrent + i))  & 0x0000FFFF);
			  		
			}

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[SEC_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[SEC_OFFSET + i]&0xFFFF) != 0x0000)) 
			{

		      *(ptMinCurrent + i) = ((unsigned long)((0x0E00|i)|(0<<14))<<16)|(Data.array[SEC_OFFSET + i]&0x0000FFFF);

			  //Reg[MAXIMUM_3_OFFSET + i].reg_d.reg_value = ((*(ptMinCurrent + i))  & 0x0000FFFF);

			  
			}

			if ((((*(ptCTMaxCurDemand + i)&0x0000FFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
			{
			  *(ptCTMaxCurDemand + i) = ((unsigned long)((0x1600|i)|(0<<14))<<16)|((*(ptCTCurDemand + i)&0x0000FFFF));
			  Reg[KW1_DEMAND + i].reg_d.reg_value = ((*(ptCTMaxCurDemand + i))  & 0x0000FFFF);
			}
			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
			{
			  *(ptCTMaxKWDemand + i) = ((unsigned long)((0x7000|i)|(0<<14))<<16)|((*(ptCTKwDemand + i)&0x0000FFFF));  //206855
			  Reg[MAX_CURR_DEMAND1 + i].reg_d.reg_value = ((*(ptCTMaxKWDemand + i))  & 0x0000FFFF);
			  
			} 
		  }

		  if (sFlag.pnl1_demand_chk_24hr)
		  {
			
			for (i=0; i < 84; i++)
		    {
			  (*(ptCTMaxCurDemand24hr +i)) = (*(ptCTMaxCurDemand + i)&0xFFFF);
			  (*(ptCTMaxKWDemand24hr + i)) = (*(ptCTMaxKWDemand + i)&0xFFFF);

			  (*(ptCTMaxKWDemand + i)) = 0;
			  (*(ptCTMaxCurDemand + i)) = 0;
			}
		    //P1_CAPTURE_MAX_DEMAND_DONE;
		    sFlag.pnl1_demand_chk_24hr = 0;
		  }	
		  break;
		}
	    case PANEL2:
	    {
		  
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_0];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_0];
		  ptMinCurrent = (unsigned int *)&Data.array[MINIMUM_0_OFFSET];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[MAXIMUM_0_OFFSET];
		  ptCTCurDemand	= (unsigned int *)&Data.array[CT_CURR_DEMAND_0];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_0];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_0];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_24HR_0];
		
		  for (i=0; i < 84; i++)
		  {
			
			if ((((*(ptMaxCurrent + i)&0xFFFF) < (Data.array[RMS_0_OFFSET + i]&0xFFFF)) || ((*(ptMaxCurrent + i)&0xFFFF) == 0xFFFF)) && ((Data.array[RMS_0_OFFSET + i]&0xFFFF) != 0xFFFF))
			{
			  *(ptMaxCurrent + i) = ((unsigned long)((0x1800|(i+84)))<<16)|(Data.array[RMS_0_OFFSET + i]&0x0000FFFF);
			  //Reg[MAXIMUM_0_OFFSET + i].reg_d.reg_value = ((*(ptMaxCurrent + i))  & 0x0000FFFF);	
			}

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[RMS_0_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[RMS_0_OFFSET + i]&0xFFFF) != 0x0000))
			{
			  *(ptMinCurrent + i) = ((unsigned long)((0x0E00|(i+84)))<<16)|((Data.array[RMS_0_OFFSET + i]&0x0000FFFF));
			  //Reg[MINIMUM_0_OFFSET + i].reg_d.reg_value = ((*(ptMinCurrent + i))  & 0x0000FFFF);
			}
			if ((((*(ptCTMaxCurDemand + i)&0xFFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
			{
			  *(ptCTMaxCurDemand + i) = ((unsigned long)((0x1600|(i+84)))<<16)|((*(ptCTCurDemand + i)&0x0000FFFF));
			  Reg[MAX_CURR_DEMAND0 + i].reg_d.reg_value = ((*(ptCTMaxCurDemand + i))  & 0x0000FFFF);
			}
			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
			{
			  *(ptCTMaxKWDemand + i) = ((unsigned long)((0x7000|(i+84)))<<16)|((*(ptCTKwDemand + i)&0x0000FFFF));
			  Reg[MAX_KW_DEMAND0 + i].reg_d.reg_value = ((*(ptCTMaxKWDemand + i))  & 0x0000FFFF);
			}
		  }

		  if (sFlag.pnl2_demand_chk_24hr)
		  {
			
			for (i=0; i < 84; i++)
			{
			  (*(ptCTMaxCurDemand24hr +i)) = (*(ptCTMaxCurDemand + i)&0xFFFF);
			  (*(ptCTMaxKWDemand24hr + i)) = (*(ptCTMaxKWDemand + i)&0xFFFF);

			  (*(ptCTMaxKWDemand + i)) = 0;
			  (*(ptCTMaxCurDemand + i)) = 0;
			}
			//P2_CAPTURE_MAX_DEMAND_DONE;
		        sFlag.pnl2_demand_chk_24hr = 0;
		  }	
			break;
	    }
	    case PANEL3:
	    {
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_1];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_1];
		  ptMinCurrent = (unsigned int *)&Data.array[MINIMUM_1_OFFSET];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[MAXIMUM_1_OFFSET];
		  ptCTCurDemand	= (unsigned int *)&Data.array[CT_CURR_DEMAND_1];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_1];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_1];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_24HR_1];
		
		  for (i=0; i < 84; i++)
		  {
		 	if ((((*(ptMaxCurrent + i)&0xFFFF) < (Data.array[RMS_1_OFFSET + i]&0xFFFF)) || ((*(ptMaxCurrent + i)&0xFFFF) == 0xFFFF)) && ((Data.array[RMS_1_OFFSET + i]&0xFFFF) != 0xFFFF))
				*(ptMaxCurrent + i) = ((unsigned long)((0x5800|i)|(0<<14))<<16)|((Data.array[RMS_1_OFFSET + i]&0x0000FFFF));

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[RMS_1_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[RMS_1_OFFSET + i]&0xFFFF) != 0x0000))
				*(ptMinCurrent + i) = ((unsigned long)((0x4E00|i)|(0<<14))<<16)|((Data.array[RMS_1_OFFSET + i]&0x0000FFFF));

			if ((((*(ptCTMaxCurDemand + i)&0xFFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxCurDemand + i) = ((unsigned long)((0x5600|i)|(0<<14))<<16)|((*(ptCTCurDemand + i)&0x0000FFFF));

			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxKWDemand + i) = ((unsigned long)((0x8000|i)|(2<<14))<<16)|((*(ptCTKwDemand + i)&0x0000FFFF));
		  }

		  if (P3_TEST_CAPTURE_MAX_DEMAND)
		  {
			for (i=0; i < 84; i++)
			{
			  (*(ptCTMaxCurDemand24hr +i)) = (*(ptCTMaxCurDemand + i)&0xFFFF);
			  (*(ptCTMaxKWDemand24hr + i)) = (*(ptCTMaxKWDemand + i)&0xFFFF);

			  (*(ptCTMaxKWDemand + i)) = 0;
			  (*(ptCTMaxCurDemand + i)) = 0;
			}
		    P3_CAPTURE_MAX_DEMAND_DONE;
		  }	
		  break;
		}
	    case PANEL4:
	    {
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_2];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_2];
		  ptMinCurrent = (unsigned int *)&Data.array[MINIMUM_2_OFFSET];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[MAXIMUM_2_OFFSET];
		  ptCTCurDemand	=(unsigned int *)&Data.array[CT_CURR_DEMAND_2];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_2];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_2];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_24HR_2];
		
		  for (i=0; i < 84; i++)
		  {
		 	if ((((*(ptMaxCurrent + i)&0xFFFF) < (Data.array[RMS_2_OFFSET + i]&0xFFFF)) || ((*(ptMaxCurrent + i)&0xFFFF) == 0xFFFF)) && ((Data.array[RMS_2_OFFSET + i]&0xFFFF) != 0xFFFF))
				*(ptMaxCurrent + i) = ((unsigned long)((0x5800|(i+84))|(0<<14))<<16)|((Data.array[RMS_2_OFFSET + i]&0x0000FFFF));

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[RMS_2_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[RMS_2_OFFSET + i]&0xFFFF) != 0x0000))
				*(ptMinCurrent + i) = ((unsigned long)((0x4E00|(i+84))|(0<<14))<<16)|((Data.array[RMS_2_OFFSET + i]&0x0000FFFF));

			if ((((*(ptCTMaxCurDemand + i)&0xFFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxCurDemand + i) = ((unsigned long)((0x5600|(i+84))|(0<<14))<<16)|((*(ptCTCurDemand + i)&0x0000FFFF));

			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxKWDemand + i) = ((unsigned long)((0x8000|i)|(3<<14))<<16)|((*(ptCTKwDemand + i)&0x0000FFFF));
		  }

		  if (P4_TEST_CAPTURE_MAX_DEMAND)
		  {
			for (i=0; i < 84; i++)
			{
			  (*(ptCTMaxCurDemand24hr +i)) = (*(ptCTMaxCurDemand + i)&0xFFFF);
			  (*(ptCTMaxKWDemand24hr + i)) = (*(ptCTMaxKWDemand + i)&0xFFFF);

			  (*(ptCTMaxKWDemand + i)) = 0;
			  (*(ptCTMaxCurDemand + i)) = 0;
			}
			P4_CAPTURE_MAX_DEMAND_DONE;
		  }	
		  break;
	    }
	    default:
	    break;
	  }
}


void Demand_Calc(unsigned int reg)
{
	unsigned int i;

	unsigned long *ptCurrDemandSum;
	unsigned long *ptKwDemandSum;
	unsigned int *ptCTCurDemand;
	unsigned int *ptCTKwDemand;

	//wMathBuff = 0;

	switch (reg)
	{
	  case PRIMARY:
	    break;
	  case SEC1:
	    break;
	  case SEC2:
	    break;
	  case SEC3:
	    break;
	  case SEC4:
		break;
	  case SEC:
		break;
	  case PANEL1:
	  {
		ptCurrDemandSum = &strCurrDemandSum[0].array[0];
		ptKwDemandSum = &strKwDemandSum[0].array[0];
		ptCTCurDemand = (unsigned int *)&Data.array[MINIMUM_3_OFFSET];
		ptCTKwDemand = (unsigned int *)&Data.array[CT_CURR_DEMAND_3];

		if (sFlag.pnl1_demand_chk_1sec)
		{	
		  for (i=0; i<84; i++)					// Every 1 sec
		  {
			(*(ptCurrDemandSum + i))+=(Data.array[SEC_OFFSET + i] & 0x0000FFFF);
			(*(ptKwDemandSum + i))+= (Data.array[RMS_3_OFFSET + i] & 0x0000FFFF);
		  }	
		  sFlag.pnl1_demand_chk_1sec = 0;
		  wDemandSumCntr[0]++;
		}

		if (sFlag.pnl1_demand_chk_1hr)
		{
		  for (i=0; i<84; i++)
		  {      

			*(ptCTCurDemand + i) = ((unsigned int)((0x0B00|i)|(0<<14))<<16) | ((unsigned int)(*(ptCurrDemandSum + i) / wDemandSumCntr[0]));

			*(ptCTKwDemand + i) = ((unsigned int)((0x0C00|i)|(0<<14))<<16) | ((unsigned int)(*(ptKwDemandSum + i) / wDemandSumCntr[0]));  //206855

			Reg[SYS_KWH_ERROR + i].reg_d.reg_value = (*(ptCTCurDemand +i) & 0x0000FFFF);
                        *(map + SYS_KWH_ERROR) = (*(ptCTCurDemand +i) & 0x0000FFFF);

			Reg[CURR1_DEMAND + i].reg_d.reg_value = (*(ptCTKwDemand +i) & 0x0000FFFF);
		        *(map + CURR1_DEMAND) = (*(ptCTKwDemand +i) & 0x0000FFFF);

			*(ptCurrDemandSum +i) = 0;
			*(ptKwDemandSum + i) = 0;
		  }
		  sFlag.pnl1_demand_chk_1hr = 0;
		  wDemandSumCntr[0] = 0;
		}	
		break;
	  }
	  case PANEL2: 
	  {
		ptCurrDemandSum = &strCurrDemandSum[1].array[0];
		ptKwDemandSum = &strKwDemandSum[1].array[0];
		ptCTCurDemand = (unsigned int *)&Data.array[CT_CURR_DEMAND_0];
		ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_0];

		if (sFlag.pnl2_demand_chk_1sec)
		{	
		  for (i=0; i<84; i++)					// Every 1 sec
		  {
			(*(ptCurrDemandSum + i))+=(Data.array[RMS_0_OFFSET + i] & 0x0000FFFF);
			(*(ptKwDemandSum + i))+= (Data.array[KW_0_OFFSET + i] & 0x0000FFFF);
		  }
		  sFlag.pnl2_demand_chk_1sec = 0;
		  wDemandSumCntr[1]++;
		}

		if (sFlag.pnl2_demand_chk_1hr)
		{ 
		  for (i=0; i<84; i++)
		  {
			*(ptCTCurDemand + i) = ((unsigned int)((0x0B00|i)|(1<<14))<<16) | ((unsigned int)(*(ptCurrDemandSum + i) / wDemandSumCntr[1]));
			*(ptCTKwDemand + i) = ((unsigned int)((0x0C00|i)|(1<<14))<<16) | ((unsigned int)(*(ptKwDemandSum + i) / wDemandSumCntr[1]));
			
			Reg[CURR0_DEMAND + i].reg_d.reg_value = (*(ptCTCurDemand +i) & 0x0000FFFF);
		        *(map + CURR0_DEMAND) = (*(ptCTCurDemand +i) & 0x0000FFFF);
			

			
			Reg[KW0_DEMAND + i].reg_d.reg_value = (*(ptCTKwDemand +i) & 0x0000FFFF);
		        *(map + KW0_DEMAND) = (*(ptCTKwDemand +i) & 0x0000FFFF);
			

			*(ptCurrDemandSum +i) = 0;
			*(ptKwDemandSum + i) = 0;
		  }
		  sFlag.pnl2_demand_chk_1hr = 0;
		  wDemandSumCntr[1] = 0;
	    }
	    break;
	  }
	  case PANEL3: 
	  {
		ptCurrDemandSum = &strCurrDemandSum[2].array[0];
		ptKwDemandSum = &strKwDemandSum[2].array[0];
		ptCTCurDemand = (unsigned int *)&Data.array[CT_CURR_DEMAND_1];
		ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_1];

		if (sFlag.pnl3_demand_chk_1sec)
		{	
		  for (i=0; i<84; i++)					// Every 1 sec
		  {
			(*(ptCurrDemandSum + i))+=(Data.array[RMS_1_OFFSET + i] & 0x0000FFFF);
			(*(ptKwDemandSum + i))+= (Data.array[KW_1_OFFSET + i] & 0x0000FFFF);
		  }
		  sFlag.pnl3_demand_chk_1sec = 0;
		  wDemandSumCntr[2]++;
		}

		if (sFlag.pnl3_demand_chk_1hr)
		{ 
		  for (i=0; i<84; i++)
		  {
			*(ptCTCurDemand + i) = ((unsigned int)((0x4B00|i)|(0<<14))<<16) | ((unsigned int)(*(ptCurrDemandSum + i) / wDemandSumCntr[2]));
			*(ptCTKwDemand + i) = ((unsigned int)((0x4C00|i)|(0<<14))<<16) | ((unsigned int)(*(ptKwDemandSum + i) / wDemandSumCntr[2]));

		 	Reg[CURR1_DEMAND + i].reg_d.reg_value = (*(ptCTCurDemand +i) & 0x0000FFFF);
		        *(map + CURR1_DEMAND) = (*(ptCTCurDemand +i) & 0x0000FFFF);
			

			
			Reg[KW1_DEMAND + i].reg_d.reg_value = (*(ptCTKwDemand +i) & 0x0000FFFF);
		        *(map + KW1_DEMAND) = (*(ptCTKwDemand +i) & 0x0000FFFF);
			

			*(ptCurrDemandSum +i) = 0;
			*(ptKwDemandSum + i) = 0;
		  }
		  sFlag.pnl3_demand_chk_1hr = 0;
		  wDemandSumCntr[2] = 0;
	    }
	    break;
	  }	
	  case PANEL4:
	  {
		ptCurrDemandSum = &strCurrDemandSum[3].array[0];
		ptKwDemandSum = &strKwDemandSum[3].array[0];
		ptCTCurDemand = (unsigned int *)&Data.array[CT_CURR_DEMAND_2];
		ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_2];

		if (sFlag.pnl4_demand_chk_1sec)
		{	
		  for (i=0; i<84; i++)					// Every 1 sec
		  {
			(*(ptCurrDemandSum + i))+=(Data.array[RMS_2_OFFSET + i] & 0x0000FFFF);
			(*(ptKwDemandSum + i))+= (Data.array[KW_2_OFFSET + i] & 0x0000FFFF);
		  }
		  sFlag.pnl4_demand_chk_1sec = 0;
		  wDemandSumCntr[3]++;
		}

		if (sFlag.pnl4_demand_chk_1hr)
		{ 
		  for (i=0; i<84; i++)
		  {
			*(ptCTCurDemand + i) = ((unsigned int)((0x4B00|(i+84))|(0<<14))<<16) | ((unsigned int)(*(ptCurrDemandSum + i) / wDemandSumCntr[3]));
			*(ptCTKwDemand + i) = ((unsigned int)((0x4C00|(i+84))|(0<<14))<<16) | ((unsigned int)(*(ptKwDemandSum + i) / wDemandSumCntr[3]));

			//Reg[CURR2_DEMAND + i].reg_d.reg_value = (*(ptCTCurDemand +i) & 0x0000FFFF);
		        //*(map + CURR2_DEMAND) = (*(ptCTCurDemand +i) & 0x0000FFFF);
			
			//Reg[CT_CURR_DEMAND_2 + i].reg_d.reg_value = (*(ptCTCurDemand +i) & 0x0000FFFF);
			
			//Reg[KW2_DEMAND + i].reg_d.reg_value = (*(ptCTKwDemand +i) & 0x0000FFFF);
		        //*(map + KW2_DEMAND) = (*(ptCTKwDemand +i) & 0x0000FFFF);
			

			*(ptCurrDemandSum +i) = 0;
			*(ptKwDemandSum + i) = 0;

		  }
		  sFlag.pnl4_demand_chk_1hr = 0;
		  wDemandSumCntr[3] = 0;
		}
		break;
	  }
	  default:
		break;
	}
}

void SetDefault()
{
	unsigned int i;
	unsigned int j;

	Data_In.word.wSysInfo.word.wPowerCapacity = (0x01030000 | (wPDU_Parameters[0] & 0xFFFF));
	Data_In.word.wSysInfo.word.wNominal_IPVolt = (0x01060000 | (wPDU_Parameters[18] & 0xFFFF));
	Data_In.word.wSysInfo.word.wNominal_OPVolt = (0x01070000 | (wPDU_Parameters[19] & 0xFFFF));


	Data_In.word.wSysInfo.word.PDC_ID = (0x01190000 | 1);
	//Data_In.word.wSysInfo.word.wPDCType = (0x012D0000 | INPUT_SINGLE);
	Data_In.word.wSysInfo.word.wConfiguration = (0x01040000 | 0x200);

	Data_In.word.wSysInfo.word.wFrequency = (0x01080000 | 500);
	Data_In.word.wSysThd.word.wAmbTempHighLimit	= 0x0032;
	Data_In.word.wSysThd.word.wGroundCurr = 50;

	CalculateThreshold();

	for (i =0; i<NUM_PANEL;i++)
	{	
	  for (j=0; j<84; j++)
	  {	
		Data_In.word.Max_Min_Limit[i][j] = 260;
	  }
	}	
	for (i = 0; i<NUM_PANEL; i++)
	{
	  for(j = 0; j<84; j++)
	  {

		Data_In.word.wUnderCurDemandLimit[i][j] = 0;
		Data_In.word.wOverCurDemandLimit[i][j] = 260;

		Data_In.word.wUnderKwDemandLimit[i][j] = 0;
		Data_In.word.wOverKwDemandLimit[i][j] = 384;	
	  }
	}
	for (i=0;i<NUM_PANEL;i++)
	{
	  for(j=0;j<4;j++)
	  {
		Data_In.word.wPanelAct[i].word.Active_Inactive[j] = 0xFFFFFFF;
	  }
	}
}

void CalculateThreshold()
{
	unsigned int i,j,k,pri,sec;

	wPri_Thd.word.wPhaseA_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*90)/100;	
	wPri_Thd.word.wPhaseB_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*90)/100;
	wPri_Thd.word.wPhaseC_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*90)/100;

	
	wPri_Thd.word.wPhaseA_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*110)/100;
	wPri_Thd.word.wPhaseB_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*110)/100;
	wPri_Thd.word.wPhaseC_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*110)/100;

	dwMathBuff = ((unsigned long)(Data_In.word.wSysInfo.word.wPowerCapacity & 0x0000FFFF))*1000;

	dwMathBuff = (dwMathBuff*10)/((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*3);
	i=dwMathBuff/3;
	

	wPri_Thd.word.wPhaseA_OverCurr = dwMathBuff*10;
	wPri_Thd.word.wPhaseB_OverCurr = dwMathBuff*10;
	wPri_Thd.word.wPhaseC_OverCurr = dwMathBuff*10;
	wPri_Thd.word.wNeutral_OverCurr = dwMathBuff*10;
	wPri_Thd.word.wPhaseA_UnderCurr = 0;
	wPri_Thd.word.wPhaseB_UnderCurr = 0;
	wPri_Thd.word.wPhaseC_UnderCurr = 0;
	wPri_Thd.word.wOverVoltTHD = 50;
	wPri_Thd.word.wOverCurrTHD = 150;
	wPri_Thd.word.wPowerfactor = 75;


	// Secondary Parameters Default
 
  if (no_xfmr)    //12345
    {
    	wSec_Thd.word.wPhaseA_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_OPVolt & 0x0000FFFF)*90)/100;	
    	wSec_Thd.word.wPhaseB_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_OPVolt & 0x0000FFFF)*90)/100;
    	wSec_Thd.word.wPhaseC_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_OPVolt & 0x0000FFFF)*90)/100;

    	wSec_Thd.word.wPhaseA_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_OPVolt & 0x0000FFFF)*110)/100;
    	wSec_Thd.word.wPhaseB_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_OPVolt & 0x0000FFFF)*110)/100;
    	wSec_Thd.word.wPhaseC_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_OPVolt & 0x0000FFFF)*110)/100;

    }
else
  {
    wSec_Thd.word.wPhaseA_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*90)/100; 
    wSec_Thd.word.wPhaseB_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*90)/100;
    wSec_Thd.word.wPhaseC_UnderVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*90)/100;

    wSec_Thd.word.wPhaseA_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*110)/100;
    wSec_Thd.word.wPhaseB_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*110)/100;
    wSec_Thd.word.wPhaseC_OverVolt = ((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*110)/100;
   }


	dwMathBuff = ((unsigned long)(Data_In.word.wSysInfo.word.wPowerCapacity & 0x0000FFFF))*1000;

	dwMathBuff = (dwMathBuff*10)/((Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)*3);
	i=dwMathBuff/3;

	
	wSec_Thd.word.wPhaseA_OverCurr = dwMathBuff*10;
	wSec_Thd.word.wPhaseB_OverCurr = dwMathBuff*10;
	wSec_Thd.word.wPhaseC_OverCurr = dwMathBuff*10;
	wSec_Thd.word.wNeutral_OverCurr = dwMathBuff*10;
	wSec_Thd.word.wPhaseA_UnderCurr = 0;
	wSec_Thd.word.wPhaseB_UnderCurr = 0;
	wSec_Thd.word.wPhaseC_UnderCurr = 0;
	//wSec_Thd.word.wOverVoltTHD = 50;
	//wSec_Thd.word.wOverCurrTHD = 150;
	//wSec_Thd.word.wPowerfactor = 75;
	
}


void KWH_Calc(unsigned int reg)
{

	volatile unsigned int 	i;
	unsigned long	*ptCTKwh_Error;
	unsigned int	*ptKWH_Error;
	static unsigned int 	dummy_cntr;
	unsigned long kw_dwMathBuff; 
	ULONG_2WORD kw_dwCalcBuff[6];

	dummy_cntr++;
	switch (reg)
	{
	  case PRIMARY:
		ptKWH_Error = &KWH_Error.dwPrimary;
		kw_dwMathBuff = 0;

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[0].all = 0;
		pthread_mutex_unlock(&kw_lock1);

		if((Data.word.Primary.KW & 0x0000FFFF) > 0xEA60) // 60000 = 600kw /10 
			Data.word.Primary.KW = (Data.word.Primary.KW & 0xFFFF0000);
		
		if(*(ptKWH_Error) > 3600)
			*(ptKWH_Error) = 0;	
		

		kw_dwMathBuff = (*(ptKWH_Error) + (Data.word.Primary.KW & 0x0000FFFF));

		if(kw_dwMathBuff > (0xEA60 + *(ptKWH_Error)))
			kw_dwMathBuff = 0;

		*(ptKWH_Error) = kw_dwMathBuff % 3600;
		*(map + KWH1_ERROR) = *(ptKWH_Error);
		kw_dwMathBuff = (kw_dwMathBuff / 3600);

		//lock
		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[0].word[0] = (unsigned short)(Data.word.Primary.KWH_Lo & 0x0000FFFF);
		kw_dwCalcBuff[0].word[1] = (unsigned short)(Data.word.Primary.KWH_Hi & 0x0000FFFF);
		kw_dwCalcBuff[0].all += (unsigned long)kw_dwMathBuff;
		pthread_mutex_unlock(&kw_lock1);
		//unlock

	    if (kw_dwCalcBuff[0].all > 999999999)
		  kw_dwCalcBuff[0].all -= 999999999;
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Primary.KWH_Lo = (0x080A0000 | kw_dwCalcBuff[0].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + 11) = (0x080A0000 | kw_dwCalcBuff[0].word[0]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Primary.KWH_Hi = (0x080B0000 | kw_dwCalcBuff[0].word[1]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + 10) = (0x080B0000 | kw_dwCalcBuff[0].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Reg[11].reg_d.reg_value = (kw_dwCalcBuff[0].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		Reg[10].reg_d.reg_value = (kw_dwCalcBuff[0].word[1]);
		pthread_mutex_unlock(&kw_lock1);

		break;
	   
	  case SEC:
		// PhaseA
		ptKWH_Error = &KWH_Error.dwSecondary_PhaseA;
		kw_dwMathBuff = 0;

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[1].all = 0;
		pthread_mutex_unlock(&kw_lock1);

		if((Data.word.Secondary.KW_Phase_A& 0x0000FFFF) > 0x4E20) //20000 = 200kw/10
			Data.word.Secondary.KW_Phase_A = (Data.word.Secondary.KW_Phase_A & 0xFFFF0000);
		
		if(*(ptKWH_Error) > 3600)
			*(ptKWH_Error) = 0;	
		
		kw_dwMathBuff = (*(ptKWH_Error) + (Data.word.Secondary.KW_Phase_A & 0x0000FFFF));

		if(kw_dwMathBuff > (0x4E20 + *(ptKWH_Error)))
			kw_dwMathBuff = 0;

		*(ptKWH_Error) = kw_dwMathBuff % 3600;
		*(map + KWH1_ERROR + 1) = *(ptKWH_Error);
		kw_dwMathBuff = (kw_dwMathBuff / 3600);

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[1].word[0] = (unsigned short)(Data.word.Secondary.KWH_Phase_A_Lo & 0x0000FFFF);
		kw_dwCalcBuff[1].word[1] = (unsigned short)(Data.word.Secondary.KWH_Phase_A_Hi & 0x0000FFFF);
		kw_dwCalcBuff[1].all += (unsigned long)kw_dwMathBuff;
		pthread_mutex_unlock(&kw_lock1);

		if (kw_dwCalcBuff[1].all > 999999999)
		  kw_dwCalcBuff[1].all -= 999999999;
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Secondary.KWH_Phase_A_Lo = (0x090A0000 | kw_dwCalcBuff[1].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + PRIMARY_OFFSET + 11) = (0x090A0000 | kw_dwCalcBuff[1].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		Data.word.Secondary.KWH_Phase_A_Hi = (0x090B0000 | kw_dwCalcBuff[1].word[1]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + PRIMARY_OFFSET + 10) = (0x090B0000 | kw_dwCalcBuff[1].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Reg[PRIMARY_OFFSET + 11].reg_d.reg_value = (kw_dwCalcBuff[1].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		Reg[PRIMARY_OFFSET + 10].reg_d.reg_value = (kw_dwCalcBuff[1].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		

		// PhaseB
		ptKWH_Error = &KWH_Error.dwSecondary_PhaseB;
		kw_dwMathBuff = 0;

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[2].all = 0;
		pthread_mutex_unlock(&kw_lock1);
		
		if((Data.word.Secondary.KW_Phase_B & 0x0000FFFF) > 0x4E20) ////20000 = 200kw/10
			Data.word.Secondary.KW_Phase_B = (Data.word.Secondary.KW_Phase_B & 0xFFFF0000);
		
		if(*(ptKWH_Error) > 3600)
			*(ptKWH_Error) = 0;
			
		kw_dwMathBuff = (*(ptKWH_Error) + (Data.word.Secondary.KW_Phase_B & 0x0000FFFF));

		if(kw_dwMathBuff > (0x4E20 + *(ptKWH_Error)))
			kw_dwMathBuff = 0;

		*(ptKWH_Error) = kw_dwMathBuff % 3600;
		*(map + KWH1_ERROR + 2) = *(ptKWH_Error);
		kw_dwMathBuff = (kw_dwMathBuff / 3600);

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[2].word[0] = (unsigned short)(Data.word.Secondary.KWH_Phase_B_Lo & 0x0000FFFF);
		kw_dwCalcBuff[2].word[1] = (unsigned short)(Data.word.Secondary.KWH_Phase_B_Hi & 0x0000FFFF);
		kw_dwCalcBuff[2].all += (unsigned long)kw_dwMathBuff;
		pthread_mutex_unlock(&kw_lock1);

		if (kw_dwCalcBuff[2].all > 999999999)
		  kw_dwCalcBuff[2].all -= 999999999;
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Secondary.KWH_Phase_B_Lo = (0x090C0000 | kw_dwCalcBuff[2].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + PRIMARY_OFFSET + 13) = (0x090C0000 | kw_dwCalcBuff[2].word[0]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Secondary.KWH_Phase_B_Hi = (0x090D0000 | kw_dwCalcBuff[2].word[1]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + PRIMARY_OFFSET + 12) = (0x090D0000 | kw_dwCalcBuff[2].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Reg[PRIMARY_OFFSET + 13].reg_d.reg_value = (kw_dwCalcBuff[2].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		Reg[PRIMARY_OFFSET + 12].reg_d.reg_value = (kw_dwCalcBuff[2].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		 

		//PhaseC
		ptKWH_Error = &KWH_Error.dwSecondary_PhaseC;
		kw_dwMathBuff = 0;

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[3].all = 0;
		pthread_mutex_unlock(&kw_lock1);

		if((Data.word.Secondary.KW_Phase_C & 0x0000FFFF) > 0x4E20) ////20000 = 200kw/10
			Data.word.Secondary.KW_Phase_C = (Data.word.Secondary.KW_Phase_C & 0xFFFF0000);
		
		if(*(ptKWH_Error) > 3600)
			*(ptKWH_Error) = 0;
			
		kw_dwMathBuff = (*(ptKWH_Error) + (Data.word.Secondary.KW_Phase_C & 0x0000FFFF));

		if(kw_dwMathBuff > (0x4E20 + *(ptKWH_Error)))
			kw_dwMathBuff = 0;

		*(ptKWH_Error) = kw_dwMathBuff % 3600;
		*(map + KWH1_ERROR + 3) = *(ptKWH_Error);
		kw_dwMathBuff = (kw_dwMathBuff / 3600);

		pthread_mutex_lock(&kw_lock1);
		kw_dwCalcBuff[3].word[0] = (unsigned short)(Data.word.Secondary.KWH_Phase_C_Lo & 0x0000FFFF);
		kw_dwCalcBuff[3].word[1] = (unsigned short)(Data.word.Secondary.KWH_Phase_C_Hi & 0x0000FFFF);
		kw_dwCalcBuff[3].all += (unsigned long)kw_dwMathBuff;
		pthread_mutex_unlock(&kw_lock1);

		if (kw_dwCalcBuff[3].all > 999999999)
		  kw_dwCalcBuff[3].all -= 999999999;
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Secondary.KWH_Phase_C_Lo = (0x090E0000 | kw_dwCalcBuff[3].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + PRIMARY_OFFSET + 15) = (0x090E0000 | kw_dwCalcBuff[3].word[0]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Data.word.Secondary.KWH_Phase_C_Hi = (0x090F0000 | kw_dwCalcBuff[3].word[1]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		*(map + PRIMARY_OFFSET + 14) = (0x090F0000 | kw_dwCalcBuff[3].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		
		pthread_mutex_lock(&kw_lock1);
		Reg[PRIMARY_OFFSET + 15].reg_d.reg_value = (kw_dwCalcBuff[3].word[0]);
		pthread_mutex_unlock(&kw_lock1);

		pthread_mutex_lock(&kw_lock1);
		Reg[PRIMARY_OFFSET + 14].reg_d.reg_value = (kw_dwCalcBuff[3].word[1]);
		pthread_mutex_unlock(&kw_lock1);
		

		break; 

	  case PANEL1:

		ptCTKwh_Error = (unsigned long *)&Data_nv.array[SYSTEM_GAINS];

		for (i=0; i<84; i++)
		{
			kw_dwMathBuff = 0;

			pthread_mutex_lock(&kw_lock1);
			kw_dwCalcBuff[4].all = 0;
			pthread_mutex_unlock(&kw_lock1);
		
			if(*(ptCTKwh_Error + i) > 3600)
				*(ptCTKwh_Error + i) = 0;

			if((Data.array[RMS_3_OFFSET + i] & 0x0000FFFF) > 0x67F)  // 63 * 264 / 10
				Data.array[RMS_3_OFFSET + i] = (Data.array[RMS_3_OFFSET + i] & 0xFFFF0000);
			
			kw_dwMathBuff = (unsigned long)(*(ptCTKwh_Error + i) + (Data.array[RMS_3_OFFSET + i] & 0x0000FFFF));

			if(kw_dwMathBuff > (0x67F + *(ptCTKwh_Error + i)))
				kw_dwMathBuff = 0;

			*(ptCTKwh_Error + i) = kw_dwMathBuff % 3600;
			*(map + SYSTEM_GAINS + i) = *(ptCTKwh_Error + i);
			kw_dwMathBuff = (kw_dwMathBuff / 3600);
			
			pthread_mutex_lock(&kw_lock1);
			kw_dwCalcBuff[4].word[0] = (unsigned short)(Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2)] & 0x0000FFFF);
			kw_dwCalcBuff[4].word[1] = (unsigned short)(Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2) + 1] & 0x0000FFFF);
			kw_dwCalcBuff[4].all += (unsigned long)kw_dwMathBuff;
			pthread_mutex_unlock(&kw_lock1);

			if (kw_dwCalcBuff[4].all > 999999999)
				kw_dwCalcBuff[4].all -= 999999999;
			
			pthread_mutex_lock(&kw_lock1);
			Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2)] = ((((0x2D00|(i*2))|(0<<14))<<16) | kw_dwCalcBuff[4].word[0]);
			pthread_mutex_unlock(&kw_lock1);
			pthread_mutex_lock(&kw_lock1);
			*(map + PANEL24_OVERCURR_FLAG + (i*2)) = (unsigned short)kw_dwCalcBuff[4].word[0];
			pthread_mutex_unlock(&kw_lock1);
			pthread_mutex_lock(&kw_lock1);
			Reg[PANEL24_OVERCURR_FLAG + (i*2)].reg_d.reg_value = (unsigned short)kw_dwCalcBuff[4].word[0];
			pthread_mutex_unlock(&kw_lock1);
			
			pthread_mutex_lock(&kw_lock1);
			Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2) + 1] = ((((0x2D00|(i*2 + 1))|(0<<14))<<16) | kw_dwCalcBuff[4].word[1]);
			pthread_mutex_unlock(&kw_lock1);
			pthread_mutex_lock(&kw_lock1);
			*(map + PANEL24_OVERCURR_FLAG + (i*2) + 1) = (unsigned short)kw_dwCalcBuff[4].word[1];
			pthread_mutex_unlock(&kw_lock1);
			pthread_mutex_lock(&kw_lock1);
			Reg[PANEL24_OVERCURR_FLAG + (i*2) + 1].reg_d.reg_value = (unsigned short)kw_dwCalcBuff[4].word[1];
			pthread_mutex_unlock(&kw_lock1);

		}

		break;
	  case PANEL2:

		ptCTKwh_Error = (unsigned long *)&Data_nv.array[KWH0_ERROR];
		for (i=0; i<84; i++)
		{
			kw_dwMathBuff = 0; 

			pthread_mutex_lock(&kw_lock1);
			kw_dwCalcBuff[5].all = 0;
			pthread_mutex_unlock(&kw_lock1);
		
			if(*(ptCTKwh_Error + i) > 3600)
				*(ptCTKwh_Error + i) = 0;
			if((Data.array[KW_0_OFFSET + i] & 0x0000FFFF) > 0x67F) // 63 * 264 / 10
				Data.array[KW_0_OFFSET + i] = (Data.array[KW_0_OFFSET + i] & 0xFFFF0000);
			
			kw_dwMathBuff = (unsigned long)(*(ptCTKwh_Error + i) + (Data.array[KW_0_OFFSET + i] & 0x0000FFFF));
			if(kw_dwMathBuff > (0x67F + *(ptCTKwh_Error + i)))
				kw_dwMathBuff = 0;
			*(ptCTKwh_Error + i) = kw_dwMathBuff % 3600;
			*(map + KWH0_ERROR + i) = *(ptCTKwh_Error + i);
			kw_dwMathBuff = (kw_dwMathBuff / 3600);
			
			pthread_mutex_lock(&kw_lock1);
			kw_dwCalcBuff[5].word[0] = (unsigned short)(Data.array[KWH_0_OFFSET + (i*2)] & 0x0000FFFF);
			kw_dwCalcBuff[5].word[1] = (unsigned short)(Data.array[KWH_0_OFFSET + (i*2) + 1] & 0x0000FFFF);
			kw_dwCalcBuff[5].all += (unsigned long)kw_dwMathBuff;
			pthread_mutex_unlock(&kw_lock1);

			if (kw_dwCalcBuff[5].all > 999999999)
				kw_dwCalcBuff[5].all -= 999999999;
			
			pthread_mutex_lock(&kw_lock1);
			Data.array[KWH_0_OFFSET + (i*2)] = ((((0x2D00|(i*2))|(1<<14))<<16) | kw_dwCalcBuff[5].word[0]);
			pthread_mutex_unlock(&kw_lock1);

			pthread_mutex_lock(&kw_lock1);
			*(map + KWH0_OFFSET + (i*2)) = (unsigned short)kw_dwCalcBuff[5].word[0];
			pthread_mutex_unlock(&kw_lock1);

			pthread_mutex_lock(&kw_lock1);
			Reg[KWH0_OFFSET + (i*2)].reg_d.reg_value = (unsigned short)kw_dwCalcBuff[5].word[0];
			pthread_mutex_unlock(&kw_lock1);
			
			pthread_mutex_lock(&kw_lock1);
			Data.array[KWH_0_OFFSET + (i*2) + 1] = ((((0x2D00|(i*2 + 1))|(1<<14))<<16) | kw_dwCalcBuff[5].word[1]);
			pthread_mutex_unlock(&kw_lock1);

			pthread_mutex_lock(&kw_lock1);
			*(map + KWH0_OFFSET + (i*2) + 1) = (unsigned short)kw_dwCalcBuff[5].word[1];
			pthread_mutex_unlock(&kw_lock1);

			pthread_mutex_lock(&kw_lock1);
			Reg[KWH0_OFFSET + (i*2) + 1].reg_d.reg_value = (unsigned short)kw_dwCalcBuff[5].word[1];
			pthread_mutex_unlock(&kw_lock1);

		}
		break;

	  default:
		break;
	}
  
}

void System_Status_Update(void)
{
	//unsigned short  system_stat;
	unsigned char alarm; 
	unsigned char dummy_char;
	static unsigned short panel_count = 0;
	static unsigned short panel_count1 = 0;
	static unsigned short epo_count = 0;
	static unsigned short repo_count = 0;	
	static unsigned short ip_breaker_status = 0;
	static unsigned short ip_breaker_trip_status = 0;
	
	static unsigned short op_breaker_trip_status = 0;
	static unsigned short tr_125_count = 0;
	static unsigned short tr_150_count = 0;
	unsigned char dummy1;
	unsigned int dry_position[6];
	// EPO and REPO
	dwSecDevPhaseAB = ((Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF) > (Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF))?((Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF) - (Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF)):((Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF) - (Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF));
	dwSecDevPhaseBC = ((Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF) > (Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF))?((Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF) - (Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF)):((Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF) - (Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF));
	dwSecDevPhaseCA = ((Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF) > (Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF))?((Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF) - (Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF)):((Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF) - (Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF));
	dwPriDevPhaseAB = ((Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF) > (Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF))?((Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF) - (Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF)):((Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF) - (Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF));
	dwPriDevPhaseBC = ((Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF) > (Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF))?((Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF) - (Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF)):((Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF) - (Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF));
	dwPriDevPhaseCA = ((Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF) > (Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF))?((Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF) - (Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF)):((Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF) - (Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF));
		
	nos_i2c_read(0, 0x43, P14I0EE5V6408_GPIO_EXPANDER_INPUT_STATUS_REG, &dummy_char, 1);
	Data.word.System_Status.EPO = (dummy_char >> 6);
	Data_nv.word.System_Status.EPO = (dummy_char >> 6);
	/*if (Data.word.System_Status.EPO == 0)
	{
	  epo_count++;
	  if (epo_count >= 5)
	  {
	   if(!event_flag[25])
	   {
		dummy1 = 25;
		event_flag[25] = 1;
		write(fd5[1],&dummy1,1);
	   }
	  }
	}
	else
	{
	  epo_count = 0;
	  if(event_flag[25])
	  {
		dummy1 = 26;
		write(fd5[1],&dummy1,1);
		event_flag[25] = 0;
	  }
	}*/
	if (dummy_char >> 7)
	  Data.word.System_Status.REPO = 0;
	else
	  Data.word.System_Status.REPO = 1;
	
	Data_nv.word.System_Status.REPO = (dummy_char >> 7);
	/*if (Data.word.System_Status.REPO == 1)
	{
	  repo_count++;
	  if (repo_count >= 5)
	  {
	   if(!event_flag[27])
	   {
		dummy1 = 27;
		write(fd5[1],&dummy1,1);
		event_flag[27] = 1;
	   }
	  }
	}
	else
	{
	  repo_count = 0;
	  if(event_flag[27])
	  {
		dummy1 = 28;
		write(fd5[1],&dummy1,1);
		event_flag[27] = 0;
	  }
	}*/

	// Circuit Breaker Status

	nos_i2c_read(1, 0x43, P14I0EE5V6408_GPIO_EXPANDER_INPUT_STATUS_REG, &dummy_char, 1);
	Data.word.System_Status.CB_Secondary_Trip = dummy_char;
	Data_nv.word.System_Status.CB_Secondary_Trip = dummy_char;
	if (no_xfmr)
	{
	if (Data.word.System_Status.CB_Secondary_Trip == 0)
	{
	  	status_count[0]++;
		if(status_count[0] > 3)
		{
			if(!event_flag[4])
			{
				dummy1 = 23;
				write(fd5[1],&dummy1,1);
				event_flag[4] = 1;
				alarm_status |= (1 << OP_BREAKER_TRIP);
				alarm_flag = 0;
			}
		}
	}
	else if ((Data.word.System_Status.CB_Secondary_Trip == 1))
	{
		status_count[0] = 0;
		if(event_flag[4])
		{
			dummy1 = 24;
			write(fd5[1],&dummy1,1);
			event_flag[4] = 0;
			//alarm_status &= (0 << OP_BREAKER);
			alarm_status &= ~(1 << OP_BREAKER_TRIP);
			alarm_flag = 0;
		}
	}
	}
	Data.word.System_Status.CB_Primary_Trip = (dummy_char >> 1);
	Data_nv.word.System_Status.CB_Primary_Trip = (dummy_char >> 1);
	if (Data.word.System_Status.CB_Primary_Trip == 0)
	{
		status_count[1]++;
		if(status_count[1] > 3)
		{
			if(!event_flag[1])
			{
				dummy1 = 8;
				write(fd5[1],&dummy1,1);
				event_flag[1] = 1;
				alarm_status |= (1 << IP_BREAKER_TRIP);
				alarm_flag = 0;
			}
		}
	}
	else if ((Data.word.System_Status.CB_Primary_Trip == 1))
	{
		status_count[1] = 0; 
		if(event_flag[1])
		{
			dummy1 = 9;
			write(fd5[1],&dummy1,1);
			event_flag[1] = 0;
			alarm_status &= ~(1 << IP_BREAKER_TRIP);
			alarm_flag = 0;
		}
	}

	
	nos_i2c_read(0, 0x44, P14I0EE5V6408_GPIO_EXPANDER_INPUT_STATUS_REG, &dummy_char, 1);
	
	if (dummy_char & 1)
	   Data.word.System_Status.Temp_125 = 1;
	else
	   Data.word.System_Status.Temp_125 = 0;
	Data.word.System_Status.Temp_125 = (dummy_char & 1);
	if (no_xfmr)
	{
	if (Data.word.System_Status.Temp_125 == 0)
	{
	   /*tr_125_count++;
	   if (tr_125_count >= 5)
	   {*/	  
	     if(!event_flag[10])
	     {
	      dummy1 = 10;
	      write(fd5[1],&dummy1,1);
	      event_flag[10] = 1;
	     }
	   //}
	}
    else
	{
	   //tr_125_count = 0;  
	   if(event_flag[10])
	   {
	     dummy1 = 11;
             write(fd5[1],&dummy1,1);
	     event_flag[10] = 0;
	   }
	}
	}

	if (dummy_char >> 1 & 1)
	  Data.word.System_Status.Temp_150 = 1;
	else
	  Data.word.System_Status.Temp_150 = 0;

	Data.word.System_Status.Temp_150 = ((dummy_char >> 1) & 1);
	if (no_xfmr)
	{
		if (Data.word.System_Status.Temp_150 == 1)
		{
			status_count[2]++;
			if(status_count[2] > 3)
			{
				if(!event_flag[12])
				{
					dummy1 = 12;
					write(fd5[1],&dummy1,1);
					event_flag[12] = 1;
					alarm_status |= (1 << XMR_OT);
					alarm_flag = 0;
				}
			}
		}
		else if (Data.word.System_Status.Temp_150 == 0)
		{
			status_count[2] = 0;
			if(event_flag[12])
			{
				dummy1 = 13;
				write(fd5[1],&dummy1,1);
				event_flag[12] = 0;
				alarm_status &= ~(1 << XMR_OT);
				alarm_flag = 0;
			}
		}
	}
	
	Data.word.System_Status.CB_Primary = (dummy_char >> 5);
	Data_nv.word.System_Status.CB_Primary = (dummy_char >> 5);
	if (Data.word.System_Status.CB_Primary == 1)
	{
	    status_count[3]++;
		if(status_count[3] > 3)
		{
			if(!event_flag[0])
			{
				dummy1 = 1;
				write(fd5[1],&dummy1,1);
				event_flag[0] = 1;
				alarm_status |= (1 << IP_BREAKER);
				alarm_flag = 0;

			}
		}
	}
	else if ((Data.word.System_Status.CB_Primary == 0))
	{
		status_count[3] = 0;
		if(event_flag[0])
		{
			dummy1 = 9;
			write(fd5[1],&dummy1,1);
			event_flag[0] =0;
			alarm_status &= ~(1 << IP_BREAKER);
			alarm_flag = 0;
		}
	}
	Data.word.System_Status.CB_Secondary = (dummy_char >> 6);
	Data_nv.word.System_Status.CB_Secondary = (dummy_char >> 6);
	if (no_xfmr)
	{
		if (Data.word.System_Status.CB_Secondary == 1)
		{
			status_count[4]++;
			if(status_count[4] > 3)
			{
				if(!event_flag[2])
				{
					dummy1 = 5;
					write(fd5[1],&dummy1,1);
					event_flag[2] = 1;
					alarm_status |= (1 << OP_BREAKER);
					alarm_flag = 0;
				}
			}
		}
		else if ((Data.word.System_Status.CB_Secondary == 0))
		{
			status_count[4] = 0;
			if(event_flag[2])
			{
				dummy1 = 24;
				write(fd5[1],&dummy1,1);
				event_flag[2] = 0;
				alarm_status &= ~(1 << OP_BREAKER);
				alarm_flag = 0;
			}
		}
	}
    system_stat = ((Data.word.System_Status.REPO << 10)|(Data.word.System_Status.EPO << 9)|(Data.word.System_Status.GroundFault << 8)|(Data.word.System_Status.FreqFail << 7)|(Data.word.System_Status.VoltageUnbalance << 6)|(Data.word.System_Status.CB_Secondary_Trip << 5)|(Data.word.System_Status.CB_Primary_Trip << 4)|(Data.word.System_Status.Temp_150 << 3)|(Data.word.System_Status.Temp_125 << 2)|(Data.word.System_Status.CB_Secondary << 1)|(Data.word.System_Status.CB_Primary));
	//printf("\nSystem Status : %x\n",system_stat);
	Reg[SECONDARY_OFFSET].reg_d.reg_value = (system_stat & 0x0000FFFF);
	*(map + SECONDARY_OFFSET) = (system_stat & 0x0000FFFF);

	dry_position[0] = bit_position(dry_contact_flag[0]);
	if (alarm_status & (1 << dry_position[0]))
	   icos_io_set_port(0,0x43,0,dry_on_off_flag[0]);
	else if (!(alarm_status & (1 << dry_position[0])))
	   icos_io_set_port(0,0x43,0,((!dry_on_off_flag[0]) & 0x000F));

	
	

	dry_position[1] = bit_position(dry_contact_flag[1]);
	if (alarm_status & (1 << dry_position[1]))
	   icos_io_set_port(0,0x43,1,dry_on_off_flag[1]);
	else if (!(alarm_status & (1 << dry_position[1])))
	   icos_io_set_port(0,0x43,1,((!dry_on_off_flag[1]) & 0x000F));
	

	dry_position[2] = bit_position(dry_contact_flag[2]);
	if (alarm_status & (1 << dry_position[2]))
	   icos_io_set_port(0,0x43,2,dry_on_off_flag[2]);
	else if (!(alarm_status & (1 << dry_position[2])))
	   icos_io_set_port(0,0x43,2,((!dry_on_off_flag[2]) & 0x000F));

	dry_position[3] = bit_position(dry_contact_flag[3]);
	if (alarm_status & (1 << dry_position[3]))
	   icos_io_set_port(0,0x43,3,dry_on_off_flag[3]);
	else if (!(alarm_status & (1 << dry_position[3])))
	   icos_io_set_port(0,0x43,3,((!dry_on_off_flag[3]) & 0x000F));

	dry_position[4] = bit_position(dry_contact_flag[4]);
	if (alarm_status & (1 << dry_position[4]))
	   icos_io_set_port(0,0x43,4,dry_on_off_flag[4]);
	else if (!(alarm_status & (1 << dry_position[4])))
	   icos_io_set_port(0,0x43,4,((!dry_on_off_flag[4]) & 0x000F));
	   
	   

	dry_position[5] = bit_position(dry_contact_flag[5]);
	if (alarm_status & (1 << dry_position[5]))
	   icos_io_set_port(0,0x43,5,dry_on_off_flag[5]);
	else if (!(alarm_status & (1 << dry_position[5])))
	   icos_io_set_port(0,0x43,5,((!dry_on_off_flag[4]) & 0x000F));
	
#if 0
	// DRY Contacts

	if (Data.word.System_Status.CB_Primary == 0)     // DRYA --> PRIMARY CB STATUS
	  icos_io_set_port(0,0x43,0,1);
	else
	  icos_io_set_port(0,0x43,0,0);

	if (Data.word.System_Status.CB_Secondary == 0)
	{     // DRYB --> SECONDARY CB STATUS
	  icos_io_set_port(0,0x43,1,1);
	}
	else
	{
	  icos_io_set_port(0,0x43,1,0);
	}

	if (Data.word.System_Status.CB_Primary_Trip == 0)     // DRYC --> Primary breaker trip
	  icos_io_set_port(0,0x43,2,1);
	else
	  icos_io_set_port(0,0x43,2,0);

	if (Data.word.System_Status.CB_Secondary_Trip == 0)     // DRYD --> Secondary breaker trip
	  icos_io_set_port(0,0x43,3,1);
	else
	  icos_io_set_port(0,0x43,3,0);

	if (Data.word.System_Status.Temp_150 == 0)     // DRYE --> Tr Temp 150
	  icos_io_set_port(0,0x43,5,1);
	else
	  icos_io_set_port(0,0x43,5,0);
#endif
	if ((Data.word.System_Parameter.Ambient_Temp & 0x0000FFFF) > Data_In.word.wSysThd.word.wAmbTempHighLimit) 
	{
	  if(AmbientTempcntr < 50)
		AmbientTempcntr++;
	  else
	  { 
	  	
		if (!event_flag[13])
		{
		//icos_io_set_port(0,0x43,3,1);
		  dummy1 = 13;
		  write(fd5[1],&dummy1,1);
		  event_flag[13] = 1;
		  alarm_status |= (1 << AMBIENT_TEMP);
		  alarm_flag = 0;
		}
	  }
	}
	else if ((Data.word.System_Parameter.Ambient_Temp & 0x0000FFFF) < (Data_In.word.wSysThd.word.wAmbTempHighLimit - 0x0005))
	{
	  if(AmbientTempcntr > 0)
		AmbientTempcntr--;
	  else
	  {
		if(event_flag[13])
		{
		//icos_io_set_port(0,0x43,3,0);
		  dummy1 = 14;
		  write(fd5[1],&dummy1,1);
		  event_flag[13] = 0;
		  alarm_status &= ~(1 << AMBIENT_TEMP);
		  alarm_flag = 0;
		}
	  }
	}

	        
	if ((((Data.word.Secondary.Frequency & 0x0000FFFF) + 30 ) < (Data_In.word.wSysInfo.word.wFrequency & 0x0000FFFF)) || 	
	((Data.word.Secondary.Frequency & 0x0000FFFF) > ((Data_In.word.wSysInfo.word.wFrequency & 0x0000FFFF) + 30)))
	{

	  if(!event_flag[19])
	  {
		dummy1 = 19;
		write(fd5[1],&dummy1,1);
		event_flag[19] = 1;
	  }
	}
	else if ((((Data.word.Secondary.Frequency & 0x0000FFFF) + 25 ) >= (Data_In.word.wSysInfo.word.wFrequency & 0x0000FFFF))
	|| 	
	((Data.word.Secondary.Frequency & 0x0000FFFF) <= ((Data_In.word.wSysInfo.word.wFrequency & 0x0000FFFF) + 25))) 
	{

	  if(event_flag[19])
	  {
		dummy1 = 20;
		write(fd5[1],&dummy1,1);
		event_flag[19] = 0;
	  }
	}
	
	if ((Data.word.System_Parameter.Ground_Curr & 0x0000FFFF) > Data_In.word.wSysThd.word.wGroundCurr)
	{
	  if (wGroundCurrentCntr < 10)
		wGroundCurrentCntr++;
	  else
	  {
		if(!event_flag[21])
		{
		  dummy1 = 21;
		  write(fd5[1],&dummy1,1);
		  event_flag[21] = 1;
		  alarm_status |= (1 << GROUND_CURRENT);
		  alarm_flag = 0;
		}
	  }
	}
	else
	{
	  if (wGroundCurrentCntr > 0)
		wGroundCurrentCntr--;
	  else
	  {

		if(event_flag[21])
		{
		  dummy1 = 22;
		  write(fd5[1],&dummy1,1);
		  event_flag[21] = 0;
		  alarm_status &= ~(1 << GROUND_CURRENT);
		  alarm_flag = 0;
		  
		}
	  }
	}

       if ((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) && (Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) && (Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) && ((Data.word.Secondary.PF_Phase_A & 0x0000FFFF) != 0xFFFF) && ((Data.word.Secondary.PF_Phase_B & 0x0000FFFF) != 0xFFFF) && ((Data.word.Secondary.PF_Phase_C & 0x0000FFFF) != 0xFFFF))
	{
	if ((Data.word.Secondary.PF_Phase_A & 0x0000FFFF) < OP_PF_LIMIT)
	{
	   Data.word.Sec_Status_Flag.PhaseA_PoorPF = 1;
	}
	else if ((Data.word.Secondary.PF_Phase_A & 0x0000FFFF) > OP_PF_LIMIT)
	{
	   Data.word.Sec_Status_Flag.PhaseA_PoorPF = 0;
	}

	if ((Data.word.Secondary.PF_Phase_B & 0x0000FFFF) < OP_PF_LIMIT)
	{
	   Data.word.Sec_Status_Flag.PhaseB_PoorPF = 1;
	}
	else if ((Data.word.Secondary.PF_Phase_B & 0x0000FFFF) > OP_PF_LIMIT)
	{
	   Data.word.Sec_Status_Flag.PhaseB_PoorPF = 0;
	}

	if ((Data.word.Secondary.PF_Phase_C & 0x0000FFFF) < OP_PF_LIMIT)
	{
	   Data.word.Sec_Status_Flag.PhaseC_PoorPF = 1;
	}
	else if ((Data.word.Secondary.PF_Phase_C & 0x0000FFFF) > OP_PF_LIMIT)
	{
	   Data.word.Sec_Status_Flag.PhaseC_PoorPF = 0;
	}
	}
      else
	{
	  Data.word.Sec_Status_Flag.PhaseA_PoorPF = 0;
	  Data.word.Sec_Status_Flag.PhaseB_PoorPF = 0;
	  Data.word.Sec_Status_Flag.PhaseC_PoorPF = 0;
	}

	if ((Data.word.Sec_Status_Flag.PhaseA_PoorPF)
	 ||(Data.word.Sec_Status_Flag.PhaseB_PoorPF)
	 ||(Data.word.Sec_Status_Flag.PhaseC_PoorPF))
	 {
	  if (wPFCntr < 10)
		wPFCntr++;
	  else
	  {
		if(!event_flag[46])
		{
		  dummy1 = 46;
		  write(fd5[1],&dummy1,1);
		  event_flag[46] = 1;
		  alarm_status |= (1 << POWER_FACTOR);
		  alarm_flag = 0;
		}
	  }
	 }
	else 
	{
	  if (wPFCntr > 0)
		wPFCntr--;
	  else
	  {

		if(event_flag[46])
		{
		  dummy1 = 47;
		  write(fd5[1],&dummy1,1);
		  event_flag[46] = 0;
	          alarm_status &= ~(1 << POWER_FACTOR);
		  alarm_flag = 0;
		}
	  }
	}
	
	
	if ((dwSecDevPhaseAB > LACKING_VOLT)
	||
	(dwSecDevPhaseBC > LACKING_VOLT)
	||
	(dwSecDevPhaseCA > LACKING_VOLT)
	||
	(dwPriDevPhaseAB > LACKING_VOLT)
	||
	(dwPriDevPhaseBC > LACKING_VOLT)
	||
	(dwSecDevPhaseCA > LACKING_VOLT))
	{
	  if(!event_flag[15])
	  {
		dummy1 = 15;
		write(fd5[1],&dummy1,1);
		event_flag[15] = 1;
	  }
	}
	else
	{
	  if((dwSecDevPhaseAB < (LACKING_VOLT - PHASE_LACK_HYSTER))
	  ||
	  (dwSecDevPhaseBC < (LACKING_VOLT - PHASE_LACK_HYSTER))
	  ||
	  (dwSecDevPhaseCA < (LACKING_VOLT - PHASE_LACK_HYSTER))
	  ||
	  (dwPriDevPhaseAB < (LACKING_VOLT - PHASE_LACK_HYSTER))
	  ||
	  (dwPriDevPhaseBC < (LACKING_VOLT - PHASE_LACK_HYSTER))
	  ||
	  (dwSecDevPhaseCA < (LACKING_VOLT - PHASE_LACK_HYSTER)))
	  {

		if (event_flag[15])
		{
		  dummy1 = 16;
		  write(fd5[1],&dummy1,1);
		  event_flag[15] = 0;
		}
	  }
	}
	
	
	dwSecVoltAvg = ((Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF)+(Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF)+(Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF))/3;
	dwSecDevPhaseAB = ((Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF) > dwSecVoltAvg)?((Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF) - dwSecVoltAvg):(dwSecVoltAvg - (Data.word.Secondary.L2L_Volt_Phase_AB & 0x0000FFFF));
	dwSecDevPhaseBC = ((Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF) > dwSecVoltAvg)?((Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF) - dwSecVoltAvg):(dwSecVoltAvg - (Data.word.Secondary.L2L_Volt_Phase_BC & 0x0000FFFF));
	dwSecDevPhaseCA = ((Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF) > dwSecVoltAvg)?((Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF) - dwSecVoltAvg):(dwSecVoltAvg - (Data.word.Secondary.L2L_Volt_Phase_CA & 0x0000FFFF));
	dwPriVoltAvg = ((Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF)+(Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF)+(Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF))/3;
	dwPriDevPhaseAB = ((Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF) > dwPriVoltAvg)?((Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF) - dwPriVoltAvg):(dwPriVoltAvg - (Data.word.Primary.L2L_Volt_Phase_AB & 0x0000FFFF));
	dwPriDevPhaseBC = ((Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF) > dwPriVoltAvg)?((Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF) - dwPriVoltAvg):(dwPriVoltAvg - (Data.word.Primary.L2L_Volt_Phase_BC & 0x0000FFFF));
	dwPriDevPhaseCA = ((Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF) > dwPriVoltAvg)?((Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF) - dwPriVoltAvg):(dwPriVoltAvg - (Data.word.Primary.L2L_Volt_Phase_CA & 0x0000FFFF));
	
	if (((dwPriDevPhaseAB * 100) > (dwPriVoltAvg * UNBALANCE_RATE))
	||
	((dwPriDevPhaseBC * 100) > (dwPriVoltAvg * UNBALANCE_RATE))
	||
	((dwSecDevPhaseCA * 100) > (dwSecVoltAvg * UNBALANCE_RATE)))
	{
	  if (((dwSecDevPhaseAB * 100) > (dwSecVoltAvg * UNBALANCE_RATE))
	  ||
	  ((dwSecDevPhaseBC * 100) > (dwSecVoltAvg * UNBALANCE_RATE))
	  ||
	  ((dwSecDevPhaseCA * 100) > (dwSecVoltAvg * UNBALANCE_RATE)))
	  {
		Data_nv.word.System_Status.VoltageUnbalance = 1;
		if (!event_flag[17])
		{
		  dummy1 = 17;
		  write(fd5[1],&dummy1,1);
		  event_flag[17] = 1;
		}
	  }
	  else
	  {
		Data_nv.word.System_Status.VoltageUnbalance = 0;
		if (event_flag[17])
		{
		  dummy1 = 18;
		  write(fd5[1],&dummy1,1);	
		  event_flag[17] = 0;
		}
	  }
	}
	else
	{
	  if (((dwPriDevPhaseAB * 100) < (dwPriVoltAvg * UNBALANCE_RATE))
	  ||
	  ((dwPriDevPhaseBC * 100) < (dwPriVoltAvg * UNBALANCE_RATE))
	  ||
	  ((dwSecDevPhaseCA * 100) < (dwSecVoltAvg * UNBALANCE_RATE))) 
	  {
		if (((dwSecDevPhaseAB * 100) > (dwSecVoltAvg * UNBALANCE_RATE))
		||
		((dwSecDevPhaseBC * 100) > (dwSecVoltAvg * UNBALANCE_RATE))
		||
		((dwSecDevPhaseCA * 100) > (dwSecVoltAvg * UNBALANCE_RATE)))
		{
		  Data_nv.word.System_Status.VoltageUnbalance = 1;
		  if (!event_flag[17])
		  {
			dummy1 = 17;
			write(fd5[1],&dummy1,1);
			event_flag[17] = 1;
		  }
	    }
	    else
	    {
	      Data_nv.word.System_Status.VoltageUnbalance = 0;
		  if (event_flag[17])
		  {
		    dummy1 = 18;
		    write(fd5[1],&dummy1,1);	
		    event_flag[17] = 0;
		  }
	    }

	  }
    }


//if ((Data.word.System_Status.CB_Secondary_Trip == 1) && (Data.word.System_Status.CB_Secondary == 0) && (Data.word.System_Status.CB_Primary_Trip == 1) && (Data.word.System_Status.CB_Primary == 0))
	//{
if(wPDU_Parameters[13] > 1)
{
	if(((wPhaseA_PanelCurrent[0]/wPDU_Parameters[4])||(wPhaseB_PanelCurrent[0]/wPDU_Parameters[5])||(wPhaseC_PanelCurrent[0]/wPDU_Parameters[6])) || ((wPhaseA_PanelCurrent[1]/wPDU_Parameters[4])||(wPhaseB_PanelCurrent[1]/wPDU_Parameters[5])||(wPhaseC_PanelCurrent[1]/wPDU_Parameters[6])) || ((wPhaseA_PanelCurrent[2]/wPDU_Parameters[4])||(wPhaseB_PanelCurrent[2]/wPDU_Parameters[5])||(wPhaseC_PanelCurrent[2]/wPDU_Parameters[6])))
	{
	panel_count++;
	if (panel_count == 60)
	{
	 if (!event_flag[43])     
	  {
	   dummy1 = 43;
	
	   write(fd5[1],&dummy1,1);
	   event_flag[43] = 1;
	   alarm_status |= (1 << PANEL_OVERLOAD);
	   alarm_flag = 0;
	   
	   //Reg[PRIMARY_OFFSET + 44].reg_d.reg_value = 1;
	
	  }
	}
	if(((wPhaseA_PanelCurrent[0]/wMax_Panel_Limit)||(wPhaseB_PanelCurrent[0]/wMax_Panel_Limit)||(wPhaseC_PanelCurrent[0]/wMax_Panel_Limit)) || ((wPhaseA_PanelCurrent[1]/wMax_Panel_Limit)||(wPhaseB_PanelCurrent[1]/wMax_Panel_Limit)||(wPhaseC_PanelCurrent[1]/wMax_Panel_Limit)) || ((wPhaseA_PanelCurrent[2]/wMax_Panel_Limit)||(wPhaseB_PanelCurrent[2]/wMax_Panel_Limit)||(wPhaseC_PanelCurrent[2]/wMax_Panel_Limit)))
	{
	
	 panel_count1++;
	 if (!event_flag[45])
	  {
	   dummy1 = 45;
	   write(fd5[1],&dummy1,1);
	   event_flag[45] = 1;
	   alarm_status |= (1 << PANEL_OVERLOAD_140);
	   alarm_flag = 0;
	   //Reg[PRIMARY_OFFSET + 44].reg_d.reg_value = 2;	
	  }
	 if (panel_count1 >= 5)
	  {
	   alarm_status |= (1 << PANEL_OL_TRIP);
	   //alarm_flag = 0;
	   //icos_io_set_port(0,0x43,4,1);
	   panel_count1 = 1;
	  }
	}
	
	 if (panel_count >= wPDU_Parameters[20])
	  {
	   alarm_status |= (1 << PANEL_OL_TRIP);
	   //alarm_flag = 0;
	   //icos_io_set_port(0,0x43,4,1);
	   panel_count--;
	  }
	}
	else 
	{
	 event_flag[45] = 0;
	 alarm_status &= ~(1 << PANEL_OL_TRIP);
	 //alarm_flag = 0;
	 //icos_io_set_port(0,0x43,4,0);
	 panel_count = 0;
	 panel_count1 = 0;
	 //Reg[PRIMARY_OFFSET + 44].reg_d.reg_value = 0;

	 /*if ((Data.word.System_Status.CB_Secondary_Trip == 1) && (Data.word.System_Status.CB_Secondary == 0) && (Data.word.System_Status.CB_Primary_Trip == 1) && (Data.word.System_Status.CB_Primary == 0))
	 {*/
	  if (event_flag[43])                              
	  {
	   dummy1 = 44;
	   write(fd5[1],&dummy1,1);  
	   event_flag[43] = 0;
	   alarm_status &= ~(1 << PANEL_OVERLOAD);
	   alarm_status &= ~(1 << PANEL_OVERLOAD_140);
	   alarm_flag = 0;

		/*if ((Data.array[PANEL24_UNDERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL21_OVERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL22_OVERCURR_STATUS_FLAG] & 0x001FFFFF) || (Data.array[PANEL23_OVERCURR_STATUS_FLAG] & 0x001FFFFF))
		{
		  event_flag[69] = 0;
		}
		else
		{
		  event_flag[69] = 1;
		}
		if (Data.word.System_Status.CB_Secondary_Trip == 0)
		{
		  event_flag[4] = 0;
		}
		else
		{
		  event_flag[4] = 1;
		}*/
      
    }
		}
		
  } 

	Data.word.Sys.AlarmStatus1 = ((0x101 << 16) | (alarm_status & 0xFFFF));
	Data.word.Sys.AlarmStatus2 = ((0x102 << 16) | ((alarm_status & 0xFFFF0000) >> 16));
	
	
  //Data.word.Sys.AlarmStatus3 = ((0x109 << 16) | (alarm_status1 & 0xFFFF));
  //Data.word.Sys.AlarmStatus4 = ((0x10A << 16) | ((alarm_status1 & 0xFFFF0000) >> 16));        
	
	//*****************Added for sending alarms through modbus and BACnet*************************//
	#if 0
	for (alm_offset=0; alm_offset <32; alm_offset++)
	{  
		Reg[ALARM_STATUS + alm_offset].reg_d.reg_value = ((alarm_status >> alm_offset) & 0x1);
		*(map + ALARM_STATUS + alm_offset) = ((alarm_status >> alm_offset) & 0x1);
	}
		
	for (alm_offset1=0; alm_offset1 <32; alm_offset1++)
	{  
		Reg[ALARM_STATUS + alm_offset + alm_offset1].reg_d.reg_value = ((alarm_status1 >> alm_offset1) & 0x1); 
		*(map + ALARM_STATUS + alm_offset + alm_offset1) = ((alarm_status1 >> alm_offset) & 0x1);
	}
	
	Reg[ALARM_STATUS + alm_offset + alm_offset1].reg_d.reg_value = (Data.word.Sys.AlarmStatus1 & 0xFFFF);
	Reg[ALARM_STATUS + alm_offset + alm_offset1 + 1].reg_d.reg_value = (Data.word.Sys.AlarmStatus2 & 0xFFFF);
	Reg[ALARM_STATUS + alm_offset + alm_offset1 + 2].reg_d.reg_value = (Data.word.Sys.AlarmStatus3 & 0xFFFF);
	Reg[ALARM_STATUS + alm_offset + alm_offset1 + 3].reg_d.reg_value = (Data.word.Sys.AlarmStatus4 & 0xFFFF);

	*(map + ALARM_STATUS + alm_offset + alm_offset1) = (Data.word.Sys.AlarmStatus1 & 0xFFFF);
	*(map + ALARM_STATUS + alm_offset + alm_offset1 + 1) = (Data.word.Sys.AlarmStatus2 & 0xFFFF);
	*(map + ALARM_STATUS + alm_offset + alm_offset1 + 2) = (Data.word.Sys.AlarmStatus3 & 0xFFFF);
	*(map + ALARM_STATUS + alm_offset + alm_offset1 + 3) = (Data.word.Sys.AlarmStatus4 & 0xFFFF);
	
	#endif


	//**********************************************End************************************//

	

    /*************************** Activating/Deactivating Buzzer here ************/
	
	if (!alarm_flag)
	{
		if (alarm_status == 0)
		{
			alarm = 1;
			if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
		}
		else if (alarm_status != 0)
		{
			alarm = 12;
			if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
		}
	}
	else if (alarm_flag == 1)
	{ 
		alarm = 4;
		nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
	}

}

void AmbientTempCalc(void)
{
	unsigned short AdcAmbientTemp;
	unsigned short AdcAmbientTemp1;
	AdcAmbientTemp = (unsigned short)((status_result >> 16) & 0xFFFF);


	AdcAmbientTemp1 = ((unsigned long long)(AdcAmbientTemp * 0xA5)/(0x32CD));

	if(AdcAmbientTemp1 < 0x20)
         {
	  Data.word.System_Parameter.Ambient_Temp = ((0x600 << 16) | (0));
	  Reg[FW_VERSION0].reg_d.reg_value = (Data.word.System_Parameter.Ambient_Temp & 0x0000FFFF);
         }
	else if(AdcAmbientTemp1 > 0x94)
	 {
	  Data.word.System_Parameter.Ambient_Temp = ((0x600 << 16) | (0x94));
	  Reg[FW_VERSION0].reg_d.reg_value = (Data.word.System_Parameter.Ambient_Temp & 0x0000FFFF);
	 }
	else
	{
	  Data.word.System_Parameter.Ambient_Temp = ((0x600 << 16) | (((AdcAmbientTemp1 - 0x20)*5)/9));
	  Reg[FW_VERSION0].reg_d.reg_value = (Data.word.System_Parameter.Ambient_Temp & 0x0000FFFF);
	}
        //printf("\nAmbient Temperature Offset : %d\n",FW_VERSION0);	

}
/***********************New Process*************************/
void ADC_process(void)
{
	int    channel = 0, bus = 0x4A;
	icos_test_adc(bus, channel);
}

#if 0
static void icos_test_adc(int addr, int channel)
{
	nos_uint16    data = 0;
	nos_uint16    data1 = 0;
	nos_uint16	  data2 = 0;
	nos_uint16    data3 = 0;
	nos_uint16    result = 0;
	nos_uint16    temperature = 0;
	nos_uint32    temp_and_err_status = 0;
	unsigned char chn_no,chn_addr,i;
	unsigned short Offset = 12950;
	unsigned short temp;
	static unsigned char count = 0;
	sleep(1);
	chn_no = 0;
	chn_addr = 0x48;
	while(1) 
	{
	  switch (chn_addr)
	  {
		case 0x48: 

		  icos_adc_read_channel(1,
		              chn_addr,
		              chn_no,
		              &data,100);
		  if (data > Offset)
			temp = (data - Offset);
		  else
		  {
			temp = (data - Offset);
			temp = (~temp) + 1;
		  }
	  
	  	  AdcFanCurrent.array[(chn_addr - 0x48)*4 + chn_no] += (unsigned long long)(temp*temp);
	  

	  	  chn_addr++;
	  
		  break;
		case 0x49:
	    
	      icos_adc_read_channel(1,
                              chn_addr,
                              chn_no,
                              &data1,100);
		  if (data1 > Offset)
			 temp = (data1 - Offset);
		  else
		  {
		    temp = (data1 - Offset);
			temp = (~temp) + 1;
		  }
          
	 
		  AdcFanCurrent.array[(chn_addr - 0x48)*4 + chn_no] += (unsigned long long)(temp*temp);


		  chn_addr++;
		
		  break;
	    case 0x4A:
	 
		  icos_adc_read_channel(1,
		  chn_addr,
		  chn_no,
		  &data2,100);
		  if (chn_no != 1)
		  {
		    if (data2 > Offset)
			  temp = (data2 - Offset);
			else
			{
			  temp = (data2 - Offset);
			  temp = (~temp) + 1;
			}
		  }
		  else
			temperature_status = data2;

		  chn_addr++;
		  break;
	    case 0x4B:
	 
		  icos_adc_read_channel(1,
		  chn_addr,
		  chn_no,
		  &data3,100);
		  if (data3 > Offset)
			temp = (data3 - Offset);
		  else
		  {
			temp = (data3 - Offset);
			temp = (~temp) + 1;
		  }

		  AdcFanCurrent.array[((chn_addr - 1) - 0x48)*4 + chn_no] += (unsigned long long)(temp*temp);

		  chn_addr = 0x48;
		  chn_no++;
		  if (chn_no > 3)
		  {
			chn_addr = 0x48;
			chn_no = 0;
			Fan_cntr++;
		  }
		  break;
		default :
		  break;
	  }
	  if (Fan_cntr == NUM_ADC_SAMPLES)
	  {
		strFanParameter.word.dwFan1_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan1_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan2_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan2_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan3_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan3_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan4_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan4_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan5_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan5_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan6_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan6_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan7_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan7_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan8_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan8_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan9_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan9_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan10_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan10_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan11_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan11_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan12_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan12_current_sum) / NUM_ADC_SAMPLES))*100;
		AdcFanCurrent.word.dwFan1_current_sum = 0;
		AdcFanCurrent.word.dwFan2_current_sum = 0;
		AdcFanCurrent.word.dwFan3_current_sum = 0;
		AdcFanCurrent.word.dwFan4_current_sum = 0;
		AdcFanCurrent.word.dwFan5_current_sum = 0;
		AdcFanCurrent.word.dwFan6_current_sum = 0;
		AdcFanCurrent.word.dwFan7_current_sum = 0;
		AdcFanCurrent.word.dwFan8_current_sum = 0;
		AdcFanCurrent.word.dwFan9_current_sum = 0;
		AdcFanCurrent.word.dwFan10_current_sum = 0;
		AdcFanCurrent.word.dwFan11_current_sum = 0;
		AdcFanCurrent.word.dwFan12_current_sum = 0;
		Fan_cntr = 0;
	  }
	  for (i=0; i<12; i++)
	  {
		fan_alarm = 0x00000001 << i;

		if((strFanParameter.array[i] < 0x2000))
		{
		  fan_err_status |= fan_alarm;
		}
	  	else
		{
		  fan_err_status &= (~fan_alarm);
		}
	  }
      temp_and_err_status = ((unsigned int)(temperature_status << 16)|fan_err_status);
      write(fd1[1], &temp_and_err_status, 4); 
      usleep(10);
    }
}


static void icos_test_adc(int addr, int channel)
{
	nos_uint16    data = 0;
	nos_uint16    data1 = 0;
	nos_uint16	  data2 = 0;
	nos_uint16    data3 = 0;
	nos_uint16    result = 0;
	nos_uint16    temperature = 0;
	nos_uint32    temp_and_err_status = 0;
	unsigned char chn_no,chn_addr,i;
	unsigned short Offset = 12950;
	unsigned short temp;
	static unsigned char count = 0;
	sleep(1);
	chn_no = 0;
	chn_addr = 0x48;
	while(1) 
	{
	  switch (chn_addr)
	  {
		case 0x48: 

		  icos_adc_read_channel(1,
		              chn_addr,
		              chn_no,
		              &data,100);
                  //printf("\nChn_addr: %x\nChn_no: %x\nData : %x\n",chn_addr,chn_no,data);
		  if (data > Offset)
			temp = (data - Offset);
		  else
		  {
			temp = (data - Offset);
			temp = (~temp) + 1;
		  }
	  
	  	  //AdcFanCurrent.array[(chn_addr - 0x48)*4 + chn_no] += (unsigned long long)(temp*temp);
	          AdcFanCurrent.array[(chn_addr - 0x48)*4 + chn_no] = temp;

	  	  chn_addr++;
	  
		  break;
		case 0x49:
	    
	      icos_adc_read_channel(1,
                              chn_addr,
                              chn_no,
                              &data1,100);
		  //printf("\nChn_addr: %x\nChn_no: %x\nData : %x\n",chn_addr,chn_no,data1);
		  if (data1 > Offset)
			 temp = (data1 - Offset);
		  else
		  {
		    temp = (data1 - Offset);
			temp = (~temp) + 1;
		  }
          
	 
		  //AdcFanCurrent.array[(chn_addr - 0x48)*4 + chn_no] += (unsigned long long)(temp*temp);
                  AdcFanCurrent.array[(chn_addr - 0x48)*4 + chn_no] = temp;

		  chn_addr++;
		
		  break;
	    case 0x4A:
	 
		  icos_adc_read_channel(1,
		  chn_addr,
		  chn_no,
		  &data2,100);
		  if (chn_no != 1)
		  {
		    //printf("\nChn_addr: %x\nChn_no: %x\nData : %x\n",chn_addr,chn_no,data2);
		    if (data2 > Offset)
			  temp = (data2 - Offset);
			else
			{
			  temp = (data2 - Offset);
			  temp = (~temp) + 1;
			}
		  }
		  else
			temperature_status = data2;

		  chn_addr++;
		  break;
	    case 0x4B:
	 
		  icos_adc_read_channel(1,
		  chn_addr,
		  chn_no,
		  &data3,100);
		  //printf("\nChn_addr: %x\nChn_no: %x\nData : %x\n",chn_addr,chn_no,data3);
		  if (data3 > Offset)
			temp = (data3 - Offset);
		  else
		  {
			temp = (data3 - Offset);
			temp = (~temp) + 1;
		  }

		  //AdcFanCurrent.array[((chn_addr - 1) - 0x48)*4 + chn_no] += (unsigned long long)(temp*temp);
                  AdcFanCurrent.array[((chn_addr - 1) - 0x48)*4 + chn_no] = temp;

		  chn_addr = 0x48;
		  chn_no++;
		  if (chn_no > 3)
		  {
			chn_addr = 0x48;
			chn_no = 0;
			Fan_cntr++;
		  }
		  break;
		default :
		  break;
	  }
	  /*if (Fan_cntr == NUM_ADC_SAMPLES)
	  {
		strFanParameter.word.dwFan1_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan1_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan2_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan2_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan3_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan3_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan4_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan4_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan5_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan5_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan6_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan6_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan7_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan7_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan8_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan8_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan9_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan9_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan10_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan10_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan11_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan11_current_sum) / NUM_ADC_SAMPLES))*100;
		strFanParameter.word.dwFan12_current_sum_out = (unsigned int)sqrt(((AdcFanCurrent.word.dwFan12_current_sum) / NUM_ADC_SAMPLES))*100;
		AdcFanCurrent.word.dwFan1_current_sum = 0;
		AdcFanCurrent.word.dwFan2_current_sum = 0;
		AdcFanCurrent.word.dwFan3_current_sum = 0;
		AdcFanCurrent.word.dwFan4_current_sum = 0;
		AdcFanCurrent.word.dwFan5_current_sum = 0;
		AdcFanCurrent.word.dwFan6_current_sum = 0;
		AdcFanCurrent.word.dwFan7_current_sum = 0;
		AdcFanCurrent.word.dwFan8_current_sum = 0;
		AdcFanCurrent.word.dwFan9_current_sum = 0;
		AdcFanCurrent.word.dwFan10_current_sum = 0;
		AdcFanCurrent.word.dwFan11_current_sum = 0;
		AdcFanCurrent.word.dwFan12_current_sum = 0;
		Fan_cntr = 0;
	  }*/
	  AdcFanCurrent.array[5] = AdcFanCurrent.array[9];  //3216212 added for fan issue

	  for (i=0; i<12; i++)
	  {
		fan_alarm = 0x00000001 << i;

                //printf("\nOffset for fan %d : %d\n",i,AdcFanCurrent.array[i]);

		if((AdcFanCurrent.array[i] < fan_cntr_Parameters[0]))
		{
			if(fan_no_err_cntr[i] > 0)
			{
			    fan_no_err_cntr[i] = 0;	
			}
			else
			{
				fan_err_cntr[i]++;
				if(fan_err_cntr[i] > fan_cntr_Parameters[1])
				{
					fan_err_status |= fan_alarm;	
				}
			}
		}
	  	else
		{
			if(fan_err_cntr[i] > 0)
			{
				fan_err_cntr[i] = 0;
			}
			else
			{
				fan_no_err_cntr[i]++;
				if(fan_no_err_cntr[i] > fan_cntr_Parameters[2])
				{
		    		fan_err_status &= (~fan_alarm);
				}
		    }
		}
	  }
      temp_and_err_status = ((unsigned int)(temperature_status << 16)|fan_err_status);
      write(fd1[1], &temp_and_err_status, 4); 
      system("sleep 0.1s");//DK changed form 0.01 to 0.1 need to fine tune it
    }
}
#endif


static void icos_test_adc(int addr, int channel)
{
  nos_uint16    temp_data = 0;
  nos_uint16    data1 = 0;
  nos_uint16    data2 = 0;
  nos_uint16    data3 = 0;
  nos_uint16    result = 0;
  nos_uint16 config_data = 0; 
  nos_uint16 conver_data = 0;
  nos_uint16    temperature = 0;
  nos_uint32    temp_and_err_status = 0;
  unsigned char chn_no,chn_addr,i;
  //unsigned short Offset = 8806;
  unsigned short temp;
  static unsigned char count = 0;
  sleep(1);
  unsigned long int read_t;
  //chn_no = 1;
  //chn_addr = 0x4A;
  int fan_err_status = 0, config, get_fan_status = 0, fan_alarm = 0;
  int os_bit1;
  nos_uint16 config_data1, conver_data1;
  int fail_count[8] = {0,0,0,0,0,0,0,0};
  int struck_count[8] = {0,0,0,0,0,0,0,0};
  int fan_rec_count[8] = {0,0,0,0,0,0,0,0};
  
	while(1) 
	{
		//Reads temperature data from ADC through i2c
		os_bit1 = 0;
		nos_i2c_write_word(1, 0x4A, 0x01, 0xf3c3);
		//system("sleep 0.002s");
		while(os_bit1 != 1)
		{
			nos_i2c_read_word(1, 0x4A,0x01, &config_data1);
			os_bit1 = (config_data1 >> 7) & 0x0001;
		}
		nos_i2c_read_word(1, 0x4A, 0x00, &conver_data1);
		temp_data = ((conver_data1 << 8) & 0xff00) | ((conver_data1 >> 8) & 0x00ff);
		temperature_status = temp_data;	


		//Checks for Fan failure for all the 8 fans considering 10 samples of each at a time
		for(i = 0;i < fan_num;i++)
		{
			adc_flag = 1;
			fan_alarm = 0x00000001 << i;
			struck_count[i] = 0;
			fail_count[i] = 0;
			get_fan_status = get_adc_avg(adc_flag, chn_address[i], chn_number[i], &z_Offset[i], &fail_count[i], &struck_count[i]);
			
			if((fail_count[i] > fan_cntr_Parameters[0]) || (struck_count[i] > fan_cntr_Parameters[1]))
			{
				fan_err_cntr[i]++;
			}
			else
			{
				fan_err_cntr[i] = 0;
			}
				
			if(fan_err_cntr[i] > fan_cntr_Parameters[2])
			{
				fan_err_status |= fan_alarm;
			}
			else
			{
				fan_err_status &= (~fan_alarm);
			}
		}

		temp_and_err_status = ((unsigned int)(temperature_status << 16)|fan_err_status);
		write(fd1[1], &temp_and_err_status, 4); 
		system("sleep 4s");
	}   
}

int get_fan_offset(void)
{
	int i, count, s_count;
	for(i = 0; i < fan_num;i++)
	{
		adc_flag = 0;
		get_adc_avg(adc_flag, chn_address[i],chn_number[i],&z_Offset[i], &count, &s_count);
	}

}

int get_adc_avg(int adc_flag, int chn_addr, int chn_no, int *avg_value, int *count, int *jam_count)
{
	int adc_avg, adc_sum = 0;
	int sample_count = 0;
	int Offset;
	unsigned int temp = 0;
	int f_data, os_bit, config_value, i;
	nos_uint16 conver_data, config_data, d_first;
	int off_count,j_count;

	Offset = *avg_value;
	
	//printf("Chn_addr : %x Chn_no : %d\n", chn_addr, chn_no);
	config_value = (((chn_no + 4) << 4 ) | 0xF383);
    //printf("Data written into config register : %x\n",config_value);

	off_count = 0;
	j_count = 0;

    for(i =0;i < 10;i++)
    {
		os_bit = 0;
        nos_i2c_write_word(1, chn_addr, 0x01, config_value);
	    //system("sleep 0.002s");
		
        //nos_i2c_read_word(1, chn_addr,0x01, &config_data);
		//os_bit = (config_data >> 7) & 0x0001;

		while(os_bit != 1)
		{
			nos_i2c_read_word(1, chn_addr,0x01, &config_data);
			os_bit = (config_data >> 7) & 0x0001;
		}
      
        if(os_bit)
        {
			nos_i2c_read_word(1, chn_addr, 0x00, &conver_data);
		
			f_data = ((conver_data << 8) & 0xff00) | ((conver_data >> 8) & 0x00ff);
			
			adc_sum += f_data;
			sample_count++;
			
			if (f_data > Offset)
				temp = (f_data - Offset);
			else
			{
				temp = (f_data - Offset);
				temp = (~temp) + 1;
			}

			if(temp > fan_cntr_Parameters[4]) //default limit set : 9100 
			{
				j_count++;
			}
			//temp > 10000 failure(Fan struck) //default limit set : 100
			if(temp < fan_cntr_Parameters[3])
				off_count++;

        }
		//system("sleep 0.0001s");
    }
	//printf("jam count : %d\n",j_count);
	
	if(sample_count != 0)
		adc_avg = adc_sum/sample_count;

	if(!adc_flag)
		*avg_value = adc_avg;

	*count = off_count; 
	*jam_count = j_count;   
}




void config_process(void)
{	
	unsigned int k_loop = 0;
	unsigned int writestatus[4*84];
	ssize_t ret = 0;
	int file_fd = 0;
	unsigned char buffer[5]= {0,0,0,0,0};
	unsigned char *fptr;
	unsigned char cnfg_mode=0;
	int config_value_fd =0 ;
	int kwh_value_fd =0 ;
	int kwh_err_value_fd =0 ;
	/***************For system KWH - start*******************/
	int sysIPkwh_value_fd =0 ;
	int sysIPkwh_err_value_fd =0 ;
	int sysOPkwh_value_fd =0 ;
	int sysOPkwh_err_value_fd =0 ; 
	/***************End**************************************/
	int temp_kwh_fd = 0;
	volatile unsigned int j = 0;
	unsigned char data_buffer[5376];
	unsigned char *cptr_dummy_value = NULL;

	unsigned int data_buf[sizeof(NV_DATA)/4];
	Nv_Data *ptr;
	int *kwhptr;
	int shmid1;
	int* shmpointer1 = NULL;
	volatile unsigned int loop =0;
	key_t key;
	int logfile_fd;

	/**********************memmap******************************/
	int mem_fd;
	int *map;
	int i;
	mem_fd =open(FILEPATH,O_RDONLY);
	if(mem_fd == -1)
	{
	  perror("error opening the file");
	  exit(EXIT_FAILURE);
	}
	map = (int*)mmap(0,FILESIZE,PROT_READ,MAP_SHARED,mem_fd,0);
	if(map == MAP_FAILED)
	{
	  close(mem_fd);
	  perror("Error mapping the file");
	  exit(EXIT_FAILURE);
	}

	key = ftok("shmfile",65);
	shmid1 = shmget(key,10,0666|IPC_CREAT);
	shmpointer1 = shmat(shmid1,(void*)0,0);
	ptr = (Nv_Data *)shmpointer1;
	
	/********************mmap for etc binary file 12-04-22*************************/
	
	int kwh_mem_fd;
	kwh_mem_fd =open("/etc/kwhvaldata",O_RDONLY);
	if(kwh_mem_fd == -1)
	{
	  perror("error opening the file");
	  exit(EXIT_FAILURE);
	}
	map_etc = (int*)mmap(0,FILESIZE_KWHDATA,PROT_READ,MAP_SHARED,kwh_mem_fd,0);
	if(map_etc == MAP_FAILED)
	{
	  close(kwh_mem_fd);
	  perror("Error mapping the file");
	  exit(EXIT_FAILURE);
	}
	
	/**************************************************************/
	
	config_value_fd = open("/etc/cnfgvalue",O_RDONLY,S_IRUSR);

	if(config_value_fd == -1)
	{
	  config_value_fd = open("/etc/cnfgvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if(config_value_fd == -1)
	  {
	    perror("Error in Config Value Open:");
	  }
	  for (j=0; j<(4*84); j++)
	  {
	    dprintf(config_value_fd,"%d\n",260);
	  } 
	}
	close(config_value_fd);

	kwh_value_fd =open("/etc/kwhvalue",O_RDONLY,S_IRUSR);

	if(kwh_value_fd == -1)
	{
	  kwh_value_fd =open("/etc/kwhvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if ( kwh_value_fd == -1 )
	  {
	    perror("Error in kwh value file open:");
	  } 
	  for (j=0; j<(NO_OF_PANLES*2*84); j++)
	  { 
		dprintf(kwh_value_fd,"%d\n",0);
	  }
	} 
	close(kwh_value_fd);
	kwh_err_value_fd =open("/etc/kwherrvalue",O_RDONLY,S_IRUSR);

	if(kwh_err_value_fd == -1)
	{
	  kwh_err_value_fd =open("/etc/kwherrvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if ( kwh_err_value_fd == -1 )
	  {
		perror("Error in kwherr value file open:");
	  } 
	  for (j=0; j<(NO_OF_PANLES*84); j++)
	  { 
		dprintf(kwh_err_value_fd,"%d\n",0);
	  }

	} 
	close(kwh_err_value_fd);

  /***************For system KWH - start*******************/
	sysIPkwh_value_fd =open("/etc/sysIPkwhvalue",O_RDONLY|S_IRUSR);

	if(sysIPkwh_value_fd == -1)
	{
	  sysIPkwh_value_fd =open("/etc/sysIPkwhvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if ( sysIPkwh_value_fd == -1 )
	  {
		perror("Error in kwh value file open:");
	  } 
	  for (j=0; j<1 ; j++)
	  { 
		dprintf(sysIPkwh_value_fd,"%d\n",0);
	  }
	} 
	close(sysIPkwh_value_fd);

	sysIPkwh_err_value_fd =open("/etc/sysIPkwherrvalue",O_RDONLY|S_IRUSR);

	if(sysIPkwh_err_value_fd == -1)
	{
	  sysIPkwh_err_value_fd =open("/etc/sysIPkwherrvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if ( sysIPkwh_err_value_fd == -1 )
	  {
		perror("Error in kwherr value file open:");
	  } 
	  for (j=0; j<2; j++)
	  { 
		dprintf(sysIPkwh_err_value_fd,"%d\n",0);
	  }
	} 
	close(sysIPkwh_err_value_fd);
	j = 0;

	sysOPkwh_value_fd =open("/etc/sysOPkwhvalue",O_RDONLY|S_IRUSR);

	if(sysOPkwh_value_fd == -1)
	{
	  sysOPkwh_value_fd =open("/etc/sysOPkwhvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if ( sysOPkwh_value_fd == -1 )
	  {
		perror("Error in kwh value file open:");
	  } 
	  for (j=0; j<6 ; j++)
	  { 
		dprintf(sysOPkwh_value_fd,"%d\n",0);
	  }
	} 
	close(sysOPkwh_value_fd);

	sysOPkwh_err_value_fd =open("/etc/sysOPkwherrvalue",O_RDONLY|S_IRUSR);

	if(sysOPkwh_err_value_fd == -1)
	{
	  sysOPkwh_err_value_fd =open("/etc/sysOPkwherrvalue",O_RDWR|O_CREAT,S_IRUSR);
	  if ( sysOPkwh_err_value_fd == -1 )
	  {
		perror("Error in kwherr value file open:");
	  } 
	  for (j=0; j<3; j++)
	  { 
		dprintf(sysOPkwh_err_value_fd,"%d\n",0);
	  }
	} 
	close(sysOPkwh_err_value_fd);
	j = 0;
  /***************End**************************************/
  
	file_fd = open("/tmp/cnfgmode",O_RDWR|O_CREAT,S_IRUSR);
	if ( file_fd == -1 )
	{
	  perror("Error file open:");
	}
	else
	{
	  dprintf(file_fd,"%d\n",1);
	  close(file_fd);
	}


#if 0
	kwh_value_fd =open("/etc/kwhvalue",O_RDWR|O_CREAT,S_IRUSR);
	if ( kwh_value_fd == -1 )
	{
	  perror("Error kwh value file open:");
	}

	//read(kwh_value_fd,data_buffer,(4*4*84*(sizeof(unsigned int))));//commented DK
        memset(data_buffer,0,(2*2*84*10));
	read(kwh_value_fd,data_buffer,(2*2*84*10));//2 bords, High and low(2 values),84 branches, maximum 10 digits
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}
	close(kwh_value_fd);
	j = 0;
	ret = write(fd4[1], writestatus, (2*2*84*(sizeof(unsigned int))));//2 bords, High and low(2 values),84 branches, maximum int size

	sleep(1);

	kwh_err_value_fd =open("/etc/kwherrvalue",O_RDWR,S_IRUSR);
	if ( kwh_err_value_fd == -1 )
	{
	  perror("Error in kwherr value file open:");
	}

	//read(kwh_err_value_fd,data_buffer,(4*84*(sizeof(unsigned int))));//commented DK
        memset(data_buffer,0,(2*1*84*10));
	read(kwh_err_value_fd,data_buffer,(2*1*84*10));//2 bords, 1 kwherr value,84 branches, maximum 10 digits
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}
	close(kwh_err_value_fd);
	j = 0;
	ret = write(fd3[1], writestatus, (2*1*84*(sizeof(unsigned int))));//2 bords, 1 kwherr,84 branches, maximum int size
	/***************For system KWH - start*******************/

	sysIPkwh_value_fd =open("/etc/sysIPkwhvalue",O_RDWR|O_CREAT,S_IRUSR);
	if ( sysIPkwh_value_fd == -1 )
	{
	  perror("Error sysIPkwh value file open:");
	}
        
	memset(data_buffer,0,(4*4*(sizeof(unsigned int))));
	read(sysIPkwh_value_fd,data_buffer,(4*4*(sizeof(unsigned int))));
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}
	close(sysIPkwh_value_fd);
	j = 0;
	ret = write(fd8[1], writestatus, (2*1*(sizeof(unsigned int))));


	sysIPkwh_err_value_fd =open("/etc/sysIPkwherrvalue",O_RDWR|O_CREAT,S_IRUSR);
	if ( sysIPkwh_err_value_fd == -1 )
	{
	  perror("Error in sysIPkwherr value file open:");
	}

	memset(data_buffer,0,(4*4*(sizeof(unsigned int))));
	read(sysIPkwh_err_value_fd,data_buffer,(4*4*(sizeof(unsigned int))));
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}
	close(sysIPkwh_err_value_fd);
	j = 0;
	ret = write(fd7[1], writestatus, (2*4*(sizeof(unsigned int))));

	sysOPkwh_value_fd =open("/etc/sysOPkwhvalue",O_RDWR|O_CREAT,S_IRUSR);
	if ( sysOPkwh_value_fd == -1 )
	{
	  perror("Error sysOPkwh value file open:");
	}

	memset(data_buffer,0,(4*4*(sizeof(unsigned int))));
	read(sysOPkwh_value_fd,data_buffer,(4*4*(sizeof(unsigned int))));
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}
	close(sysOPkwh_value_fd);
	j = 0;
	ret = write(fd10[1], writestatus, (3*2*(sizeof(unsigned int))));


	sysOPkwh_err_value_fd =open("/etc/sysOPkwherrvalue",O_RDWR|O_CREAT,S_IRUSR);
	if ( sysOPkwh_err_value_fd == -1 )
	{
	  perror("Error in sysOPkwherr value file open:");
	}

	memset(data_buffer,0,(4*4*(sizeof(unsigned int))));
	read(sysOPkwh_err_value_fd,data_buffer,(4*4*(sizeof(unsigned int))));
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}
	close(sysOPkwh_err_value_fd);
	j = 0;
	ret = write(fd9[1], writestatus, (2*4*(sizeof(unsigned int))));
#endif

/*************************kwh reading from etc binary*************************************/

	for(k_loop=0;k_loop < 2;k_loop++)
	{
		writestatus[k_loop] = *(map_etc + KWH_PRIMARY + k_loop);
	}
	ret = write(fd8[1], writestatus, (2*1*(sizeof(unsigned int))));

	writestatus[0] = *(map_etc + KWHERROR_PRIMARY);	
	ret = write(fd7[1], writestatus, (1*(sizeof(unsigned int))));
	
	for(k_loop=0;k_loop < 6;k_loop++)
	{
		writestatus[k_loop] = *(map_etc + KWH_SECONDARY + k_loop);
	}
	ret = write(fd10[1], writestatus, (3*2*(sizeof(unsigned int))));
	
	for(k_loop=0;k_loop < 3;k_loop++)
	{
		writestatus[k_loop] = *(map_etc + KWHERROR_SECONDARY + k_loop);
	}
	ret = write(fd9[1], writestatus, (3*1*(sizeof(unsigned int))));

	for(k_loop=0;k_loop < 336;k_loop++)
	{
		writestatus[k_loop] = *(map_etc + KWH_PANEL1 + k_loop);
	}
	ret = write(fd4[1], writestatus, (2*2*84*(sizeof(unsigned int))));
	
	for(k_loop=0;k_loop < 168;k_loop++)
	{
		writestatus[k_loop] = *(map_etc + KWHERROR_PANEL1 + k_loop);
	}
	ret = write(fd3[1], writestatus, (2*1*84*(sizeof(unsigned int))));

	
/*****************************************End***********************************************/

	config_value_fd =open("/etc/cnfgvalue",O_RDWR|O_CREAT,S_IRUSR);
	if ( config_value_fd == -1 )
	{
	  perror("Error config value file open:");
	}

	memset(data_buffer,0,(16*84*(sizeof(unsigned int))));
	read(config_value_fd,data_buffer,(16*84*(sizeof(unsigned int))));
	close(config_value_fd);
	sleep(1);
	cptr_dummy_value = strtok(data_buffer,"\n");
	while(cptr_dummy_value != NULL)
	{
	  writestatus[j] = atoi(cptr_dummy_value);
	  cptr_dummy_value = strtok(NULL,"\n");
	  j++;
	}

	j=0;
	ret = write(fd2[1], writestatus, (NO_OF_PANLES*84*(sizeof(unsigned int))));
	sleep(1);

	maketimer(&copyTimerID); 
	settimer(&copyTimerID, 10, 0);
	
	/*************************** Setting time to 00:58 ***************************/
	
	//if(ntpsync_flag)
	//	system("date -s "00:58:00"");
	
	/*

	time_1.tm_year = 2020 - 1900;
	time_1.tm_mon  = 10 - 1;
	time_1.tm_mday = 1;
	time_1.tm_hour = 0;
	time_1.tm_min  = 58;
	time_1.tm_sec  = 0;
	
	time_t t = mktime(&time_1);
	
	if (t != (time_t) -1)
	{
		stime(&t);
	}
	*/
	if(ntpsync_flag)
	{
		ntp_fd = open("/home/root/ntp_update",O_RDONLY,S_IRUSR);
		if(ntp_fd == -1)
		{
			printf("No ntp file present:");
		}
		else
		{
			system("./ntp_update");
			printf("ntpsync_flag\n");
		}
	}
	close(ntp_fd);
	
    /*************************** While loop starting ***************************/
	while(1)
	{  
	  j = 0;
	  file_fd = open("/tmp/cnfgmode",O_RDONLY,S_IRUSR);
	  if ( file_fd == -1 )
	  {
		perror("Error read file open:");
	  }
	  read(file_fd,buffer,5);
	  close(file_fd);
	  fptr = strtok(buffer,"\n");
	  while(fptr != NULL)
	  {
		cnfg_mode = atoi(fptr);
		fptr = strtok(NULL,"\n");
	  }
	  sleep(1);
	  if(cnfg_mode == 2)
	  {
		config_value_fd =open("/tmp/cnfgvalue",O_RDWR|O_CREAT,S_IRUSR);
		if ( config_value_fd == -1 )
		{
		  perror("Error config value file open:");
		}
		
		memset(data_buffer,0,(16*84*(sizeof(unsigned int))));		
		read(config_value_fd,data_buffer,(16*84*(sizeof(unsigned int))));
		close(config_value_fd);
		sleep(1);
		cptr_dummy_value = strtok(data_buffer,"\n");
		while(cptr_dummy_value != NULL)
		{
		  writestatus[j] = atoi(cptr_dummy_value);
		  cptr_dummy_value = strtok(NULL,"\n");
		  j++;
		}

		j=0;
		ret = write(fd2[1], writestatus, (NO_OF_PANLES*84*(sizeof(unsigned int))));

		file_fd = open("/tmp/cnfgmode",O_RDWR|O_CREAT,S_IRUSR);
		if ( file_fd == -1 )
		{
		  perror("Error file open:");
		}
		else
		{
		  dprintf(file_fd,"%d\n",1);
		  close(file_fd);
		}
	  }
      if(copy_flag)
	  {
		copy_flag = 0;

		kwh_value_fd =open("/tmp/kwhvalue",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR); // changed here
		if ( kwh_value_fd == -1 )
		{
		  perror("Error temp kwh value file open:");
		} 

        for(loop=0;loop<(168*2);loop++)
        {
          dprintf(kwh_value_fd,"%d\n",(int)*(map+PANEL24_OVERCURR_FLAG+loop));
        }
        //dprintf(kwh_value_fd,"%x",0xAA55);
        close(kwh_value_fd);

        kwh_err_value_fd =open("/tmp/kwherrvalue",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR);
        if ( kwh_err_value_fd == -1 )
        {
          perror("Error temp kwh value file open:");
        } 
		for(loop=0;loop<(84*2);loop++)
		{
		  dprintf(kwh_err_value_fd,"%d\n",(int)*(map+SYSTEM_GAINS+loop));
		}
		//dprintf(kwh_err_value_fd,"%x",0xAA55);
		close(kwh_err_value_fd);

      /***************For system KWH - start*******************/
		sysIPkwh_value_fd =open("/tmp/sysIPkwhvalue",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR);
		if ( sysIPkwh_value_fd == -1 )
		{
		  perror("Error temp sys IP kwh value file open:");
		} 
      
		for(loop=0;loop<2;loop++)
		{
		  dprintf(sysIPkwh_value_fd,"%d\n",*(map + 10 + loop));

		}
		close(sysIPkwh_value_fd);

		sysIPkwh_err_value_fd =open("/tmp/sysIPkwherrvalue",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR);
		if ( sysIPkwh_err_value_fd == -1 )
		{
		  perror("Error temp sys IP err kwh value file open:");
		} 

		for(loop=0;loop<1;loop++)
		{
		  dprintf(sysIPkwh_err_value_fd,"%d\n",*(map + KWH1_ERROR + loop));
		}
		close(sysIPkwh_err_value_fd);

		sysOPkwh_value_fd =open("/tmp/sysOPkwhvalue",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR);
		if ( sysOPkwh_value_fd == -1 )
		{
		  perror("Error temp sys OP kwh value file open:");
		} 

		for(loop=0;loop<6;loop++)
		{
		  dprintf(sysOPkwh_value_fd,"%d\n",*(map + PRIMARY_OFFSET + 10 + loop));

		}
		close(sysOPkwh_value_fd);

		sysOPkwh_err_value_fd =open("/tmp/sysOPkwherrvalue",O_RDWR|O_CREAT|O_TRUNC,S_IRUSR);
		if ( sysOPkwh_err_value_fd == -1 )
		{
		  perror("Error temp sys OP kwh err value file open:");
		} 
		for(loop=0;loop<3;loop++)
		{
		  dprintf(sysOPkwh_err_value_fd,"%d\n",*(map + KWH1_ERROR + 1 + loop));
		}
		close(sysOPkwh_err_value_fd);
		
		/***************End**************************************/
		settimer(&copyTimerID ,180,0);

	  }
	}
}



void event_process(void)
{
	int i;
	unsigned int fan_status;
	char alarm;	
	char status;
	time_t rawtime;
	struct tm *info;
	unsigned char flag = 0;
	unsigned char flag1 = 0;
	unsigned int result = 0;
	unsigned char *cptr;
	unsigned short *sptr;
	unsigned short dummy1;
	int event_fd;
	int buzz_silence_fd;
	int buzz_status = 0;
	unsigned char buffer[4]= {0,0,0,0};

	buzz_silence_fd =open("/tmp/buzz_silence",O_RDONLY,S_IRUSR);

	if(buzz_silence_fd == -1)
	{
	  buzz_silence_fd = open("/tmp/buzz_silence",O_RDWR|O_CREAT,S_IRUSR);
	  if ( buzz_silence_fd == -1 )
	  {
		perror("Error in buzz silence file open:");
	  } 
	  dprintf(buzz_silence_fd,"%d\n",1);

	} 
	close(buzz_silence_fd);
	time(&rawtime);
	while(1)
	{
	  read(fd5[0],&result,4);
	  cptr = (unsigned char*)&result;
	  sptr = (unsigned short*)&result;
	  nos_i2c_read(0,0x22,0x2,&alarm,1);
	  fan_status = (unsigned short)*(sptr+1); 
	  buzz_silence_fd = open("/tmp/buzz_silence",O_RDONLY,S_IRUSR);

	  if (buzz_silence_fd == -1)
	  {
		perror("Error in buzz silence file open:");
	  }
	  read(buzz_silence_fd,buffer,3);
	  buzz_status = atoi(buffer);
	  close(buzz_silence_fd);
#if 0

	  event_fd =open("/etc/alarm.log",O_CREAT|O_APPEND|O_RDWR|S_IRUSR);
	  if (event_fd == -1)
	  {
		perror("Error in event file opening");
	  }
	  switch(*cptr)
	  {
		case 1:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","I/P Breaker OPEN");
		  /*alarm = (alarm & 0x04);
		  alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;
		}
		case 2:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","I/P Breaker CLOSE");
		  /*alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;
		}
		case 3:
		{
		  if (no_xfmr)
		  {
		  	time(&rawtime);
		  	dprintf(event_fd,"%s",ctime(&rawtime));
		  	dprintf(event_fd,"%s\n","Fan Failure");
		  	/*alarm = 12;
		  	if(buzz_status)
                         {
				nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
                                //printf("\nBuzz enabled\n");
                         }*/
			}
		  break;
		}
		case 4:
		{
			if (no_xfmr)
			{
		  	time(&rawtime);
		  	dprintf(event_fd,"%s",ctime(&rawtime));
		  	dprintf(event_fd,"%s\n","Fan Alarm - Recover");
		  	/*alarm = (alarm & 0x04);
		  	alarm = 1;
		  	if(buzz_status)
			{
				nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
				//printf("\nBuzz disabled\n");
			}*/
			}
		  break;
		}
		case 5:                                                                      
		{
			if (no_xfmr)
			{
		  	time(&rawtime);
		 	 	dprintf(event_fd,"%s",ctime(&rawtime));
		  	dprintf(event_fd,"%s\n","O/P breaker OPEN");
		  	/*alarm = 12;
		  	if(buzz_status)
					nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
			}
		  break;
		}
		case 6:
		{
			if (no_xfmr)
			{
		  	time(&rawtime);
		  	dprintf(event_fd,"%s",ctime(&rawtime));
		  	dprintf(event_fd,"%s\n","O/P breaker CLOSE");
		  	/*alarm = (alarm & 0x04);
		  	alarm = 1;
		  	if(buzz_status)
					nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
			}
		  break;
		}
		case 7:                                                                       // Buzzer OFF Event
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Buzzer OFF");
		  alarm &= 0xF7;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);

		  break;
		}
		case 8:                                                                       
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","I/P breaker Trip");
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;
		}
		case 9:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","I/P breaker- Recover");
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;
		}
	
		case 12:
		{
		  if (no_xfmr)
		  {
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Transformer Temp above 150");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
		  }

		  break;    
		}
		
		case 13:
		{
		  if (no_xfmr)
		  {
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Transformer Temp above 150 - Recover");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
		  }

		  break;    
		}
	
		case 21:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Ground Fault");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 22:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Ground Fault OK");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 23:
		{
			if (no_xfmr)
			{
		  	time(&rawtime);
		  	dprintf(event_fd,"%s",ctime(&rawtime));
		  	dprintf(event_fd,"%s\n","O/P breaker Trip");    // Red light
		  	/*alarm = 12;
		  	if(buzz_status)
					nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
			}
		  break;    
		}
		case 24:
		{
			if (no_xfmr)
			{
		  	time(&rawtime);
		  	dprintf(event_fd,"%s",ctime(&rawtime));
		  	dprintf(event_fd,"%s\n","O/P breaker - Recover");    // Green light
		  	/*alarm = (alarm & 0x04);
		  	alarm = 1;
		  	if(buzz_status)
					nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
			}
		  break;    
		}
	       case 25:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Ouput Over VTHD");
		  break;
		}
	       case 26:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Ouput Over VTHD - Recover");
		  break;
		}

	       case 27:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Ouput Over Current THD");
		  break;
		}
	       case 28:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Ouput Over Current THD - Recover");
		  break;
		}	
	
		case 33:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C UnderVolt");    // Red light
		  /*alarm = 12;
		if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		} 
		case 34:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C UnderVolt - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/
		  break;    
		} 
		case 35:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C UnderCurrent");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 36:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C UnderCurrent - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
		    nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 37:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Over Neutral Current ");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		break;    
		}
		case 38:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Over Neutral Current - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 39:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C OverVolt");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 40:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C OverVolt - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
		    nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 41:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C OverCurrent");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 42:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C OverCurrent - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 43:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C Panel Overload");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 44:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C Panel Overload - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 45:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C Panel Overload 140%");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 46:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Poor PF");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 47:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Poor PF - Recover");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		/*case 48:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C OverITHD - Recover");    // Green light
		  alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);

			break;    
		}*/
		case 49:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C HighCurrent");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
		    nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 50:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Input A/B/C HighCurrent - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 51:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C UnderVolt");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 52:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C UnderVolt - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 53:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C OverVolt");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 54:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C OverVolt- Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 55:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C UnderCurrent");    // Red light
		  /*alarm = 12;\
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 56:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C UnderCurrent - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 64:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C HighCurrent");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 65:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C HighCurrent - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 66:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C OverCurrent");    // Red light
		  /*alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		  break;    
		}
		case 67:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Output A/B/C OverCurrent - Recover");    // Green light
		  /*alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);*/

		break;    
		}
		case 68:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Branch Over Current Alarm");    // Green light
		  alarm = 12;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);

		  break;    
		}
		case 69:
		{
		  time(&rawtime);
		  dprintf(event_fd,"%s",ctime(&rawtime));
		  dprintf(event_fd,"%s\n","Branch Over Current Alarm - Recover");    // Green light
		  alarm = (alarm & 0x04);
		  alarm = 1;
		  if(buzz_status)
			nos_i2c_write(0, 0x22, 0x02, &alarm, 1);

		  break;    
		}
		default:
		{
		  break;
		}
	
          }

	  /*if (alarm_status == 0)
	  {
	      alarm = 1;
              if(buzz_status)
		nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
	  }
	  else if (alarm_status != 0)
	  {
	     alarm = 12;
	     if(buzz_status)
		nos_i2c_write(0, 0x22, 0x02, &alarm, 1);
	  }*/ 
          close(event_fd);
#endif
	}
	
}

int BACnet_process(void)
{

	int mem_fd;
	int *map;
	int i;
	mem_fd =open(FILEPATH,O_RDONLY);
	if(mem_fd == -1)
	{
	  perror("error opening the file");
	  exit(EXIT_FAILURE);
	}
	map = (int*)mmap(0,FILESIZE,PROT_READ,MAP_SHARED,mem_fd,0);
	if(map == MAP_FAILED)
	{
	  close(mem_fd);
	  perror("Error mapping the file");
	  exit(EXIT_FAILURE);
	}

	if(pthread_create(&bacnet_thread,NULL,bacnet_handler/*(pBAC_id)*/,NULL))
	{
	  printf("BACnet thread create error\n");
	  return -1;
	}
	if(pthread_create(&update_thread,NULL,BAC_update(map),NULL))
	{
	  printf("BACnet update thread create error\n");
	  return -1;
	}

	pthread_join(bacnet_thread,NULL);
	pthread_join(update_thread,NULL);
	return 0;
}

void* BAC_update(void* arg)
{
	unsigned int offset =0;
	int* map = (int*)arg;
	while(1)
	{
	  Analog_Input_Present_Value_Set(offset,(int)*(map+offset));//BACnet 
	  offset++;
	  if(offset == FIELD_TOTALENTRY)
	  {
		offset = 0;
		//sleep(1);
                system("sleep 10s");
	  }
          
	}
}

int bit_position(unsigned int value)
{
  unsigned char pos = 0;
  unsigned int i = 1;

  while (!(i & value))
   {
        i = i << 1;
	//printf("\nPosition : %d\n",pos);
        ++pos;
   }


  return pos;
}

void Dynammic_sorting(void)
{
   static short position = 0;
   int i;
   int Active_poles_panel1 = wPDU_Parameters[8];
   int Active_poles_panel2 = wPDU_Parameters[10] - wPDU_Parameters[9] + 1;
   int Active_poles_panel3 = wPDU_Parameters[12] - wPDU_Parameters[11] + 1;


   /*for (i=0; i< (84*NO_OF_SBOARDS) ; i++)
   {
      if (i < wPDU_Parameters[8])
      {
         Reg[SYSTEMPARAMETER + position].reg_d.reg_value = Current_Buffer[i];
         Data.array[SEC_OFFSET + position] = (((0x0D00|position) << 16) | Current_Buffer[i]);
	 Reg[RMS1_OFFSET + position].reg_d.reg_value = KW_Buffer[i];
	 Data.array[RMS_3_OFFSET + position] = (((0x1700|position) << 16) | KW_Buffer[i]);
         position++;
         
      }
      else if ((i >= (21*wPDU_Parameters[14])) && (i < (21*wPDU_Parameters[14] + Active_poles_panel2)))
      {
	 Reg[SYSTEMPARAMETER + position].reg_d.reg_value = Current_Buffer[i];
	 Data.array[SEC_OFFSET + position] = (((0x0D00|position) << 16) | Current_Buffer[i]);
	 Reg[RMS1_OFFSET + position].reg_d.reg_value = KW_Buffer[i];
	 Data.array[RMS_3_OFFSET + position] = (((0x1700|position) << 16) | KW_Buffer[i]);
         position++;
	 
      }
      else if ((i >= (21*(wPDU_Parameters[14] + wPDU_Parameters[15]))) && (i < (21*(wPDU_Parameters[14] + wPDU_Parameters[15]) + Active_poles_panel3)))
      {
	 Reg[SYSTEMPARAMETER + position].reg_d.reg_value = Current_Buffer[i];
	 Data.array[SEC_OFFSET + position] = (((0x0D00|position) << 16) | Current_Buffer[i]);
	 Reg[RMS1_OFFSET + position].reg_d.reg_value = KW_Buffer[i];
	 Data.array[RMS_3_OFFSET + position] = (((0x1700|position) << 16) | KW_Buffer[i]);
         position++;
	 //printf("Offset3: %d\n",i);
      }
   }  
   position = 0;*/

   for (i=0; i< (84*NO_OF_SBOARDS) ; i++)
   {
       if (Dynamic_Buffer[i] != 0)
       {
	 
	 if (i < 168)
	 {	
		Reg[SYSTEMPARAMETER + position].reg_d.reg_value = Current_Buffer[Dynamic_Buffer[i] - 1];
		*(map + SYSTEMPARAMETER + position) = Current_Buffer[Dynamic_Buffer[i] - 1];
		Data.array[SEC_OFFSET + position] = (((0x0D00 | position) << 16) | Current_Buffer[Dynamic_Buffer[i] - 1]);

		Reg[RMS1_OFFSET + position].reg_d.reg_value = KW_Buffer[Dynamic_Buffer[i] - 1];
		*(map + RMS1_OFFSET + position) = KW_Buffer[Dynamic_Buffer[i] - 1];
		Data.array[RMS_3_OFFSET + position] = (((0x1700 | position) << 16) | KW_Buffer[Dynamic_Buffer[i] - 1]);
	 }
	 if (i >= 168)
	 {
		
	 	//Reg[SYSTEMPARAMETER + position].reg_d.reg_value = Current_Buffer[Dynamic_Buffer[i] - 1];
		Data.array[RMS_1_OFFSET + (position - 168)] = (((0x4D00 | (position - 168)) << 16) | Current_Buffer[Dynamic_Buffer[i] - 1]);

		//Reg[RMS3_OFFSET + position].reg_d.reg_value = KW_Buffer[Dynamic_Buffer[i] - 1];
		Data.array[KW_1_OFFSET + (position - 168)] = (((0x5700 | (position - 168)) << 16) | KW_Buffer[Dynamic_Buffer[i] - 1]);
	 }
       //printf("\nCurrent Data : %x\n",Data.array[SEC_OFFSET]);	
       position++;
       //printf("\nposition : %d\n",position);
       }
   }
   position = 0;
}






