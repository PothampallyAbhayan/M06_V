
#include"support.h"
#include"PDC.h"
#include "netinet/in.h"
#include "arpa/inet.h"

struct can_frame frame;
struct canfd_frame fd_frame;

sensor_PKT  sensor_Rx_buffers[MAX_POOL_BUFFERS];
sensor_PKT  sensor_Tx_buffers[MAX_POOL_BUFFERS]; 

extern sensor_PKT  *sensor_Rx_pkt_pool_hdr ;
extern sensor_PKT  *sensor_Tx_pkt_pool_hdr ;

extern sensor_PKT  *sensor_Rx_pkt_hdr ;
extern sensor_PKT  *sensor_Tx_pkt_hdr ;

extern Can_data	Data;

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
#define MAX_MIN_SECONDARY ((sizeof(MAX_MIN_PARAMTER)/4) + MAX_MIN_PRIMARY)

#define CT_MAX_CURR_DEMAND_0  ((sizeof(DSP_PANEL_PARAMETER)/4) + MAX_MIN_SECONDARY)
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

	 
sensor_PKT *pkt_ptr = NULL;
volatile unsigned char flag;
volatile unsigned char time_stamp_flag;
sem_t semaphore_1;

struct sockaddr_can addr;
struct ifreq ifr;
struct can_filter board_filter[3];

timer_t timerid;

timer_t firstTimerID;
timer_t secondTimerID;
timer_t thirdTimerID;
timer_t fourthTimerID;
//timer_t fifthTimerID;



sigset_t mask;

pthread_t p_thread,thread_w,tcp_thread;
pthread_t db_thread;						// Thread for DB filling

THREAD_CTX p_ctx;
THREAD_CTX t_ctx;
pthread_mutex_t lock,db_lock;
FILE *logfile;
int log_fd;
clock_t start;
unsigned char req[100];
unsigned int wCalc_Cntr,k,wDemandSumCntr[4],regs,wKwh_Calc_Cntr;
unsigned int wDemandChkCntr1s_10ms;
unsigned int wDemandChkCntr1hr_10ms;
unsigned int wDemandChkCntr24hr_10ms;
unsigned long dwMathBuff;
extern EE_DATA_UNION	Data_In;
extern EE_SYS_INFO_UNION	wSysInfo;
extern Can_data	Data;
extern SYSTEM_STATUS_UPDATE	System_Status;
modbus_t *mod_ctx,*mod_ctx1;
modbus_t *tcp_ctx;
modbus_mapping_t *mapping ;
sensor_PKT *copy_CAN_Rx_to_buffer_2(sensor_PKT *p_ptr);

volatile char pool_pos;
static unsigned int Tx_frame_count = 0;

/*****************Stack support variable************************************************/


#define MBUS_IF_INDEX_OFFSET               0x7000

/* Types of registers */
typedef enum _modbus_reg_type_ {

    INPUT_REGISTER,
    HOLDING_REGISTER,
    COIL,
    DISCRETE_INPUT,
    RAW_REQUEST,

} MODBUS_REG_TYPE;

volatile static icos_uint32 if_index_cur = MBUS_IF_INDEX_OFFSET;
volatile static icos_uint32 if_index_start = MBUS_IF_INDEX_OFFSET;

/*****************Stack support variable************************************************/
    NOS_DB_HANDLE         *p_db = NULL;
    icos_int16            modbus_bus_no = 0; //rs485a = 0, rs485b = 1
    icos_int16            slave_id = 1;
    icos_uint32           reg_add = 0x800;
    icos_uchar            *data = NULL;
    icos_uint32           size, if_index;
    icos_uchar            data_wr[4] = {1,2,3,4};

    typedef union _mod_data
    {
      icos_int16 data;
      icos_uchar cdata[2];
    }mod_data;
    mod_data reg_data;
//End
void create_db_field(NOS_DB_HANDLE  *);
void DB_entry_creation_for(unsigned int ,unsigned int );
/*********************End******************************/
void System_Status_Update(void);

/********************************REG Filling for DB******************************************/
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

Temp_Reg	Reg[sizeof(Data)/4];
/*************************************End****************************************************/


int main()
{
    int i=0;
    ssize_t ret_in;
    unsigned char buffer[4]= {0,0,0,0};
    unsigned char signature;
    unsigned char flag = 0;
    int read_fd;
//for ip Address reading
    FILE *ip_fp;
    char line [50];
    struct in_addr addrptr;
    int retval;
    char *ipptr;
    unsigned short *sip_ptr;
//End

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

    printf("\nI2C Open finished\n");

    icos_io_init(0,0x43);//Device soft reset and impedence setting
    usleep(100);
    icos_io_init(1,0x43);//Device soft reset and impedence setting
    usleep(100);

    //printf("\n*******Init finished*****\n");

    icos_io_set_direction(0,0x43,0x3F,0x00);       // 0011 1111   ('0' for input(DRYA,DRYB,DRYC,DRYD,DRYE,DRYF) and '1' for output(REPO and EPO))
    //printf("\n*******set1 finished*****\n");
    usleep(100);
    icos_io_set_direction(0,0x44,0x04,0x00);	   // 0000 0100   ('0' for input (TEMP_RT1,TEMP_RT2,Aux_Power_Fail,ZCD,CBSTAT_M,CBSTAT_P1) and '1' for output (Fan_PWM))
    //printf("\n*******set2 finished*****\n");
    usleep(100);
    /*icos_io_set_direction(1,0x43,0x2D,0x00);
    printf("\n*******set2 finished*****\n");*/
    usleep(100);

    //printf("\n*******Direction set finished*****\n");

/**********************End*******************************/
/**********************ADC Initialisation****************/
    nos_i2c_device_open(1,0x4A);
    nos_i2c_device_open(1,0x49);
    nos_i2c_device_open(1,0x48);
    nos_i2c_device_open(1,0x4B);

    icos_adc_init(1,0x4A);
    icos_adc_init(1,0x49);
    icos_adc_init(1,0x48);
    icos_adc_init(1,0x4B);
    usleep(200);
    printf("\nADC channels opened and Inistialised\n");
/***********************END*******************************/

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
/***********************END*******************************/

    Data.word.System_Status.paraID = 0x200;              /********** Parameter ID setting for System Status******************/
/***Start Signature reading*****/
    do
    {
      read_fd =open("/etc/signature",O_RDONLY|S_IRUSR);
	  
      if(read_fd == -1) 
      {
	if(!(flag & 1))
	{
	  perror("log file open\n");
	  printf("\n*******Signature byte not found*****\n");
	  flag |= 1;

	}
      }
      else
      {
	flag &= ~(1);

	ret_in = read(read_fd,buffer,3);
	if((ret_in == 0xffffffff)|(buffer[0] < 48)|(buffer[1] < 48)|(buffer[2] < 48))
	{
	  if(!(flag & 2))
	  {
	    printf("\n*******Invalid Signature byte*****\n");
	    flag |= 2; 
	  } 
	}
	else
	{
	  flag &= ~(2);
	} 
      }

      close(read_fd);
    }while(flag);
	   
    signature = ((buffer[0]-48)*100)+((buffer[1]-48)*10)+(buffer[2]-48);
    printf("signature is :%d\n",signature);
/***End Signature reading*****/
/***Start ipv4 Address reading*****/
    ip_fp = fopen("/etc/PDC_IP","r");
    if(ip_fp == NULL)
    {
      perror("Error opening file");
      return -1;
    }

    ipptr = line;
    fscanf(ip_fp,"%s",ipptr);
    printf("ip Address is:%s\n",ipptr);
    memset(&addrptr,'\0',sizeof(addrptr));
    retval = inet_aton(ipptr,&addrptr);

    sip_ptr = (unsigned short*)&addrptr.s_addr;

    Data.word.Sys.IPAddress[0] = ((0x136 << 16 ) | (*sip_ptr));
    *++sip_ptr;
    Data.word.Sys.IPAddress[1] = ((0x137 << 16 ) | (*sip_ptr));

    //printf("\nip1 value is:%d\n",((*sip_ptr & 0xff00)>>8));
    //printf("\nip2 value is:%d\n",((*sip_ptr & 0x00ff)>>0));
    printf("\nport1 value is:%d\n",*++sip_ptr);

    //printf("\nip3 value is:%d\n",((*sip_ptr & 0xff00)>>8));
    //printf("\nip4 value is:%d\n",((*sip_ptr & 0x00ff)>>0));
/***End ipv4 Address reading*****/
    create_pkt_pool(100,Rx_PKT);
    sem_init(&semaphore_1,0,0);
    log_init();

    /* Initialize db if not exist
     */
    p_db = icos_mbusdb_get_db_handle();
    if ( p_db == NULL )
    {
    	p_db = icos_mbusdb_init ();
    }

    /*
     * Insert database entry for each register
     * Increment if_index for for each register
     */
	
    create_db_field(p_db);

    maketimer(&secondTimerID); //10ms
    maketimer(&firstTimerID); //10ms
    maketimer(&thirdTimerID); //10ms
    maketimer(&fourthTimerID); // 520us
    //maketimer(&fifthTimerID);

  
  
  if(pthread_create(&thread_w,NULL,write_message,NULL))
  {
    printf("thread create error\n");
  	return -1;
  }
 
  if ( pthread_create(&p_thread, NULL, handler_func, (void*)&p_ctx) != 0 )
  {
    return (-1);
  }
/**********************************DB thread********************************************/ 
  #if 1 
  if (pthread_create(&db_thread, NULL, db_filling, NULL))
  {
     printf("DB thread failed creation\n");
     return -1;
  }
  #endif
/**********************************DB thread End********************************************/ 

  
/*****************Stack support variable************************************************/
#if 0
/*
     * Initialize db if not exist
     */
    p_db = icos_mbusdb_get_db_handle();
    if ( p_db == NULL )
    {
    	p_db = icos_mbusdb_init ();
    }

    /*
     * Insert database entry for each register
     * Increment if_index for for each register
     */
	
    //create_db_field(p_db);
#endif
    //icos_mbusdb_update_reg( p_db, if_index_cur, modbus_bus_no, slave_id, reg_add, HOLDING_REGISTER, nos_strlen(data_wr), data_wr);//db
//End

  
  pthread_join(thread_w,NULL);
  pthread_join(p_thread,NULL);
  //#if 0
  pthread_join(db_thread,NULL);
  //#endif
  
   
  settimer(&firstTimerID, 2, 0); 

  
  return 0;
}


void DB_entry_creation_for(unsigned int base,unsigned int length)
{
    unsigned int i;
    printf("\nEntry : %d\n",if_index_start);
    for (i=0;i<length;i++)
    {
      icos_mbusdb_create_reg_interface(p_db, if_index_start,  modbus_bus_no );
      if_index_start++;
    }       

}
void create_db_field(NOS_DB_HANDLE  *p_db)
{
    int i;
    if (!icos_mbusdb_if_index_exists(p_db, if_index_cur)) {
        printf("if index not existing - bus = %d\n", modbus_bus_no);

     
      #if 0
	     
      DB_entry_creation_for(0x0100,0x2D);
      DB_entry_creation_for(0x0200,0x02);
      DB_entry_creation_for(0x0300,0x0F);
      DB_entry_creation_for(0x0400,0x0F);
      DB_entry_creation_for(0x0600,0x0D);
      DB_entry_creation_for(0x0700,0x23);
      #endif
	
      DB_entry_creation_for(0x0800,0x17);
      DB_entry_creation_for(0x0900,0x28);
      //DB_entry_creation_for(0x0A00,0x15);
      DB_entry_creation_for(0x0D00,0x53);
      DB_entry_creation_for(0x4D00,0x53);
      DB_entry_creation_for(0x8D00,0x53);

      DB_entry_creation_for(0xCD00,0x53);
      DB_entry_creation_for(0x1700,0x53);
      DB_entry_creation_for(0x5700,0x53);
      DB_entry_creation_for(0x9700,0x53);
      DB_entry_creation_for(0xD700,0x53);
      DB_entry_creation_for(0x1900,0x53);

      DB_entry_creation_for(0x5900,0x53);
      DB_entry_creation_for(0x9900,0x53);
      DB_entry_creation_for(0xD900,0x53);
      #if 0
      DB_entry_creation_for(0x1800,0x53);
      DB_entry_creation_for(0x5800,0x53);
      DB_entry_creation_for(0x9800,0x53);

      DB_entry_creation_for(0xD800,0x53);
      DB_entry_creation_for(0x0E00,0x53);
      DB_entry_creation_for(0x4E00,0x53);
      DB_entry_creation_for(0xCE00,0x53);
      DB_entry_creation_for(0x2C00,0x01);
      DB_entry_creation_for(0x0F00,0x01);

      DB_entry_creation_for(0x1A00,0x14);
      DB_entry_creation_for(0x5A00,0x14);
      DB_entry_creation_for(0x9A00,0x14);
      DB_entry_creation_for(0xDA00,0x14);
      DB_entry_creation_for(0x1B00,0x14);
      DB_entry_creation_for(0x5B00,0x14);

      DB_entry_creation_for(0x9B00,0x14);
      DB_entry_creation_for(0xDB00,0x14);
      DB_entry_creation_for(0x1C00,0x14);
      DB_entry_creation_for(0x5C00,0x14);
      DB_entry_creation_for(0x9C00,0x14);
      DB_entry_creation_for(0xDC00,0x14);

      DB_entry_creation_for(0x1D00,0x14);
      DB_entry_creation_for(0x5D00,0x14);
      DB_entry_creation_for(0x9D00,0x14);
      DB_entry_creation_for(0xDD00,0x14);
      DB_entry_creation_for(0x2A00,0x14);
      DB_entry_creation_for(0x6A00,0x14);

      DB_entry_creation_for(0xAA00,0x14);
      DB_entry_creation_for(0xEA00,0x14);
      DB_entry_creation_for(0x2B00,0x14);
      DB_entry_creation_for(0x6B00,0x14);
      DB_entry_creation_for(0xAB00,0x14);
      DB_entry_creation_for(0xEB00,0x14);

      DB_entry_creation_for(0x2C00,0x14);
      DB_entry_creation_for(0x6C00,0x14);
      DB_entry_creation_for(0xAC00,0x14);
      DB_entry_creation_for(0xEC00,0x14);
      DB_entry_creation_for(0x2D00,0x14);
      DB_entry_creation_for(0x6D00,0x14);
      DB_entry_creation_for(0xAD00,0x14);
      DB_entry_creation_for(0xED00,0x14);

      DB_entry_creation_for(0x3D00,0x53);
      DB_entry_creation_for(0x7D00,0x53);
      DB_entry_creation_for(0xBD00,0x53);
      DB_entry_creation_for(0xFD00,0x53);

      #endif
      printf("\n DB Entry created\n");
    }
    else {
        printf("if index existing\n");
    }
    //return 1;	
     
}
    

/*
 * Thread handler function
 * Passed as an argument while creating the thread
 */


void *handler_func ( void *ctx )
{
  THREAD_CTX  thread_ctx;
  fd_set temp_RD_fd,temp_WR_fd;
  int ret = -1;
  int nbytes;
  sensor_PKT *p_ptr = NULL;
  int len,len1,len_dummy ;//DK for checking;
  int s =-1;
  int *dat;
  int value = -1;
  int i;
  
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

  /*settimer(&fourthTimerID,0,10);*/
  

  if(st_can_filter(0x7F7,CAN_SFF_MASK,3) == -1)
  {
    perror("Error in can filter setting:");
  }
  
  FD_ZERO(&(p_ctx.read_fds));

 
  FD_SET(sock_can,&(p_ctx.read_fds));
  
  
  p_ctx.max_fd = sock_can+1;
  
 
  p_ctx.sock_can = sock_can;
 
  if ( ctx == NULL )
  {
    return (NULL);
  }

  thread_ctx = *((THREAD_CTX *)ctx);
  
  SetDefault();
  
  settimer(&fourthTimerID,1,10);
  //settimer(&fifthTimerID,0,10);
 
  while(1)
  {
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

     
      if ( FD_ISSET(thread_ctx.sock_can,&temp_RD_fd) ) 
      {

        p_ptr = copy_CAN_Rx_to_buffer_2(p_ptr);
        if (p_ptr == NULL) {
            //printf("ERROR: No free descriptors \n");
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
        if (nbytes < sizeof(struct can_frame)) {
          fprintf(stderr, "read: incomplete CAN frame\n");
          continue;
        }
	
	

        p_ptr->frame_no += 1;
        if(p_ptr->frame_no >= FRAME_LIMIT)
        {
          cleartimer(&secondTimerID);
          //printf("semaphore released from data handler\n");
   
          pthread_mutex_lock(&lock);
          enqueue_pkt(p_ptr,Tx_PKT);

          pthread_mutex_unlock(&lock);
          p_ptr = NULL;   
          sem_post(&semaphore_1);
        
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

    pthread_mutex_lock(&lock);
    pkt_ptr = release_pkt_from_free_pool(Rx_PKT);
    pthread_mutex_unlock(&lock);
    
    
    settimer(&secondTimerID, 0, 10);
    flag =1;
    pkt_ptr->frame_no = 0;
  }
  return 0;
}

sensor_PKT *copy_CAN_Rx_to_buffer_2(sensor_PKT *p_ptr)
{
//printf("pool position is %d\n",pool_pos);
  if(/*!flag && */(p_ptr == NULL))
  {
    //printf("Get a block \n");
    pthread_mutex_lock(&lock);
    p_ptr = release_pkt_from_free_pool(Rx_PKT);


    pthread_mutex_unlock(&lock);
   flag = 1;
   p_ptr->pool_position = ++pool_pos; 
    settimer(&secondTimerID, 0, 10);
    if (p_ptr != NULL) {
      p_ptr->frame_no = 0;
    }
    
  }
  //else{printf("frame count received\n");}
  //printf("packet number is %d\n",p_ptr->frame_no);

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
  while(1)
  {
    sem_wait(&semaphore_1);
    
    //printf("In msg write\n");
    set_address();
    transmit_pkt();
pthread_mutex_lock(&lock);

    append_pkt_to_free_pool(dequeue_pkt(Tx_PKT),Rx_PKT);
--pool_pos;
    pthread_mutex_unlock(&lock);
flag = 0;



  }
}

void * db_filling(void* arg)
{
  //printf("\nInside DB thread\n");
   unsigned int dummy_cntr=0;
   unsigned int count_limit=0;
   usleep(1000);
    while(1)
    {
      if(dummy_cntr >= (1000))
        {
	 dummy_cntr = 0;
	 count_limit = 0;
        }
      //pthread_mutex_lock(&db_lock);
      #if 1	      
      //for (;dummy_cntr<(100+count_limit);dummy_cntr++)
      for (dummy_cntr =0;dummy_cntr<1000;dummy_cntr++)
      {

	if(Reg[dummy_cntr].reg_flag == 1)
	{
//pthread_mutex_lock(&db_lock);
	
	icos_mbusdb_update_reg( p_db, Reg[dummy_cntr].if_index , 
                                modbus_bus_no, slave_id,
                                (icos_uint32)Reg[dummy_cntr].reg_addr,
                                Reg[dummy_cntr].reg_type,
                                Reg[dummy_cntr].reg_d.reg_value);
//pthread_mutex_unlock(&db_lock);
//printf("\nRegister : %x\t data: %x\t counter : %x\t offset: %x\n",Reg[dummy_cntr].reg_addr,Reg[dummy_cntr].reg_d.reg_value,dummy_cntr,count_limit);
	
	//usleep(5000);  
       }
//usleep(100);
      }
      #endif
      //pthread_mutex_unlock(&db_lock);
      count_limit = dummy_cntr;	
      usleep(100000);        	
    }
}


void log_init(void)
{

  log_fd =open("/tmp/paramter_log.log",O_CREAT|O_APPEND|O_WRONLY|S_IRUSR);
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

unsigned short buf[20];
void timerHandler( int sig, siginfo_t *si, void *uc )
{
  timer_t *tidp;

  tidp = si->si_value.sival_ptr;

  if ( *tidp == firstTimerID )
  {
    //Cal_Fan_Current();
    time_stamp_flag = 1;
  }
  else if ( *tidp == secondTimerID )
  {
    //Cal_Fan_Current();
    if(flag)
    {
      cleartimer(&secondTimerID);
      sem_post(&semaphore_1);
    }
  }
  else if ( *tidp == thirdTimerID )
  {
    cleartimer(&thirdTimerID);
    //printf("\nTimer cleared\n");
    time_stamp_flag = 0;
  }  
  else if (*tidp == fourthTimerID)
  {
	cleartimer(&fourthTimerID);
	//printf("\nTimer cleared -- 10ms\n");
	Pri_Winding_Calculation();
	Sec_Winding_Calculation();
	Branch_CT_Calculation();
	System_Status_Update();
	wCalc_Cntr++;
	//printf("\nwCalc_Cntr : %d\n",wCalc_Cntr);
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
		sFlag.kwh_calc_in_process = 0;}
	   }
           KWH_Calc(wKwh_Calc_Cntr);
	 }
	if (wDemandChkCntr1s_10ms >= 200)
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
	if (wDemandChkCntr1hr_10ms >= 3600)
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
	    wDemandChkCntr24hr_10ms = 0;
	  }
 	
	//modbus_parameter_filling1(mapping);
	AmbientTempCalc();

	Cal_Fan_Current();
	RMS_Fan_Current();
	
	settimer(&fourthTimerID,0,10);


  }
  /*else if (*tidp == fifthTimerID)
  {
	printf("\nINFO : Inside fifth Timer \n");
	cleartimer(&fifthTimerID);
	AmbientTempCalc();

	Cal_Fan_Current();
	RMS_Fan_Current();

	settimer(&fifthTimerID,0,10);
	
  }*/
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
  its.it_value.tv_nsec = time_msec * 1000000;;
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
  unsigned char *cptr = NULL;//:(unsigned char *)(dptr);
  unsigned short *sptr = NULL;//:(unsigned short *)(dptr);
  unsigned char paraID,Offset;
  unsigned short parameter_offset = 0;
  unsigned short dummy_parameter_offset = 0;
  unsigned short parameter_data =0;
  unsigned int result = 0;
  
  iptr = Data.array;
  
  if(temp_ptr)
  {
    for(loop = 0;loop < /*21 */temp_ptr->frame_no;loop++)
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
		Reg[Offset].if_index = (if_index_cur | Offset);
		Reg[Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
		Reg[Offset].reg_d.reg_value = (result & 0x0000FFFF);
		Reg[Offset].reg_type  = INPUT_REGISTER;
		Reg[Offset].reg_flag  = 1;
		//printf("\naddress : %x\t Value : %x\t type : %x\t Flag : %x\n",Reg[Offset].reg_addr,Reg[Offset].reg_d.reg_value,Reg[Offset].reg_type,Reg[Offset].reg_flag);
                
		//reg_add = 0x0800;
                //data_wr = (icos_uchar)((icos_uchar)result);
                //reg_data.data = (unsigned short)(result & 0x0000FFFF);
		//icos_mbusdb_update_reg( p_db, if_index_cur, modbus_bus_no, slave_id, reg_add, HOLDING_REGISTER, 2, &data_wr[0]/*&reg_data.cdata[0]*/);
		
	  }
	  else if (paraID == DSP_SEC_HI_ADDR)
	  {
	    *(iptr + PRI_OFFSET + Offset) = result;
	    Reg[PRI_OFFSET + Offset].if_index = (if_index_cur | (PRI_OFFSET + Offset));
	    Reg[PRI_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    Reg[PRI_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    Reg[PRI_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    Reg[PRI_OFFSET + Offset].reg_flag  = 1;
	     
          }
	  else if (paraID == BOARDI_CT_RMS_HI_ADDR)
          {
	     if(temp_ptr->sensor_data[loop].can_id == 0x7f7)
             {
		*(iptr + SEC_OFFSET + Offset) = result;
		Reg[SEC_OFFSET + Offset].if_index = (if_index_cur | (SEC_OFFSET + Offset));
	    	Reg[SEC_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[SEC_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[SEC_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[SEC_OFFSET + Offset].reg_flag  = 1;
             }
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fb){
		*(iptr + RMS_0_OFFSET + Offset) = result;
		Reg[RMS_0_OFFSET + Offset].if_index = (if_index_cur | (RMS_0_OFFSET + Offset));
		Reg[RMS_0_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[RMS_0_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[RMS_0_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[RMS_0_OFFSET + Offset].reg_flag  = 1;}
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fd){
		*(iptr + RMS_1_OFFSET + Offset) = result;
		Reg[RMS_1_OFFSET + Offset].if_index = (if_index_cur | (RMS_1_OFFSET + Offset));
	  	Reg[RMS_1_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[RMS_1_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[RMS_1_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[RMS_1_OFFSET + Offset].reg_flag  = 1;}
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fe){
		*(iptr + RMS_2_OFFSET + Offset) = result;
		Reg[RMS_2_OFFSET + Offset].if_index = (if_index_cur | (RMS_2_OFFSET + Offset));
		Reg[RMS_2_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[RMS_2_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[RMS_2_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[RMS_2_OFFSET + Offset].reg_flag  = 1;}
          }
	  else if (paraID == BOARDI_KW_HI_ADDR)
          {
	     if(temp_ptr->sensor_data[loop].can_id == 0x7f7)
             {
		*(iptr + RMS_3_OFFSET + Offset) = result;
		Reg[RMS_3_OFFSET + Offset].if_index = (if_index_cur | (RMS_3_OFFSET + Offset));
		Reg[RMS_3_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[RMS_3_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[RMS_3_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[RMS_3_OFFSET + Offset].reg_flag  = 1;
             }
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fb){
		*(iptr + KW_0_OFFSET + Offset) = result;
		Reg[KW_0_OFFSET + Offset].if_index = (if_index_cur | (KW_0_OFFSET + Offset));
		Reg[KW_0_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[KW_0_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[KW_0_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[KW_0_OFFSET + Offset].reg_flag  = 1;}
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fd){
		*(iptr + KW_1_OFFSET + Offset) = result;
		Reg[KW_1_OFFSET + Offset].if_index = (if_index_cur | (KW_1_OFFSET + Offset));
		Reg[KW_1_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[KW_1_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[KW_1_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[KW_1_OFFSET + Offset].reg_flag  = 1;}
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fe){
		*(iptr + KW_2_OFFSET + Offset) = result;
		Reg[KW_2_OFFSET + Offset].if_index = (if_index_cur | (KW_2_OFFSET + Offset));
		Reg[KW_2_OFFSET + Offset].reg_addr = ((result & 0xFFFF0000) >> 16);
	    	Reg[KW_2_OFFSET + Offset].reg_d.reg_value = (result & 0x0000FFFF);
	    	Reg[KW_2_OFFSET + Offset].reg_type = INPUT_REGISTER;
	    	Reg[KW_2_OFFSET + Offset].reg_flag  = 1;}	
          }
	  else if (paraID == FIRMWARE_VERSION)
	  {
	     if(temp_ptr->sensor_data[loop].can_id == 0x7f7)
		{if((Offset >= 0xA0) || (Offset <= 0xA7))
		   *(iptr + SYS + (Offset - 0xA0)) = result;
		     }
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fb)
		{if((Offset >= 0xA0) || (Offset <= 0xA7))
		   *(iptr + FIRMWARE_0 + (Offset - 0xA0)) = result;}
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fd)
		{if((Offset >= 0xA0) || (Offset <= 0xA7))
		   *(iptr + FIRMWARE_1 + (Offset - 0xA0)) = result;}
	     else if(temp_ptr->sensor_data[loop].can_id == 0x7fe)
		{if((Offset >= 0xA0) || (Offset <= 0xA7))
		   *(iptr + FIRMWARE_2 + (Offset - 0xA0)) = result;}	
	  }
	 else if (paraID == DSP_SYS_PARAMETER_HI_ADDR)
	  {
	       Data.word.System_Parameter.Ground_Curr = result;
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
  unsigned int* pptr,*dptr;
  unsigned int dummy_Tx_frame_count;
  struct can_frame temp_frame;
  struct can_frame *temp_ptr = &temp_frame;

 
  if(Tx_frame_count >= (sizeof(Data)/4))
    Tx_frame_count = 0;
  dummy_Tx_frame_count = Tx_frame_count;
  for(loop= dummy_Tx_frame_count;loop < (dummy_Tx_frame_count+FRAME_LIMIT);loop++,Tx_frame_count++)
  {
    if(Data.array[loop])
    {
      temp_ptr->can_dlc = 4;		
      dptr = (unsigned int*)temp_ptr->data;
      *dptr = Data.array[loop];
      temp_ptr->can_id = 0x0200;
      /*if ((((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x06) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x08) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x09) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x19) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x0D) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x4D) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x17) || (((unsigned char)(Data.array[loop] >> 24) & 0xFF) == 0x57))
	 {
	 mapping->tab_input_registers[(Data.array[loop] & 0xFFFF0000) >> 16] = (Data.array[loop] & 0x0000FFFF);
	 //printf("\nData.arrray[loop] : %x \n",Data.array[loop]);
	 }
	else if ((((unsigned char)(Data.array[loop] >> 24 ) & 0xFF) == 0x01) || ((((unsigned char)(Data.array[loop] >> 24 ) & 0xFF) == 0x0E) || ((((unsigned char)(Data.array[loop] >> 24 ) & 0xFF) == 0x4E) || ((((unsigned char)(Data.array[loop] >> 24 ) & 0xFF) == 0x18) || ((((unsigned char)(Data.array[loop] >> 24 ) & 0xFF) == 0x58))
	 {
	 mapping->tab_registers[(Data.array[loop] & 0xFFFF0000) >> 16] = (Data.array[loop] & 0x0000FFFF); 
	 } */
      nbytes = write(sock_can, &temp_frame, sizeof(struct can_frame));
      usleep(250);
    }
  }
 
}

/*void analytics(unsigned int *dptr, unsigned int *iptr)
{
  unsigned short BoardID = *(unsigned short *)(iptr);
  unsigned char *cptr = (unsigned char *)(dptr);
  unsigned short *sptr = (unsigned short *)(dptr);
  unsigned short parameter_offset;
  unsigned short parameter_data;
  unsigned char ParaID;
  unsigned char Offset;	
  
  parameter_offset = ((*sptr >> 8 )|(*sptr << 8));
  ParaID = (parameter_offset >> 8) & 0xFF;
  Offset = parameter_offset & 0xFF;
  sptr++;
  parameter_data = ((*sptr >> 8 )|(*sptr << 8));
  
  switch (BoardID)
  {
	case BOARD_I:
	switch (ParaID)
	{
		case BOARDI_CT_RMS_HI_ADDR:
		{
			if (Offset < NUM_BRANCH)
			  {RMS[0].array[Offset] = (unsigned int)parameter_data;
			  //printf("\n Board 1 First 42 \n");	
			  //add_to_log( ParaID , RMS[0].array[Offset]);
				}
			else
			  {RMS[1].array[NUM_BRANCH - Offset] = (unsigned int)parameter_data;
			  //printf("\n Board 1 Next 42 \n");
			  //add_to_log( ParaID , RMS[1].array[NUM_BRANCH - (unsigned int)Offset]);
				}
		break;
		}
		case DSP_PRIMARY_HI_ADDR:
		{
			Primary.array[Offset] = (unsigned int)parameter_data;
		break;		
		}
		case DSP_SEC_HI_ADDR:
		{
			Secondary.array[Offset] = (unsigned int)parameter_data;
		break;		
		}
		case DSP_SEC_PANEL1_HI_ADDR:
		{
			Secondary1.array[Offset] = (unsigned int)parameter_data;
		break;		
		}
		case DSP_SEC_PANEL2_HI_ADDR:
		{
			Secondary2.array[Offset] = (unsigned int)parameter_data;
		break;		
		}
		case DSP_SEC_PANEL3_HI_ADDR:
		{
			Secondary3.array[Offset] = (unsigned int)parameter_data;
		break;
		}
		case BOARDI_KW_HI_ADDR:
		{
			if (Offset < PANEL_CT_DATA_LENGTH)
				strCTKw[0].array[Offset] = (unsigned int)parameter_data;
			else
				strCTKw[1].array[Offset - PANEL_CT_DATA_LENGTH] = (unsigned int)parameter_data;
		break;		
		}
		case DSP_SYS_PARAMETER_HI_ADDR:
		{
			SysParameter.array[Offset] = (unsigned int)parameter_data;	
		break;
		}
		default:
		break;
	}
	break;
  	case BOARD_II:
  	{
	switch(ParaID)
	{
	case BOARDI_CT_RMS_HI_ADDR:
	{	
		if ((unsigned int)Offset < NUM_BRANCH)
			  {RMS[2].array[Offset] = (unsigned int)parameter_data;
			  //printf("\n Board 2 First 42 \n");	
			  add_to_log( ParaID , RMS[2].array[Offset]);}
			else
			  {RMS[3].array[NUM_BRANCH - (unsigned int)Offset] = (unsigned int)parameter_data;
			  //printf("\n Board 2 Next 42 \n");
			  add_to_log( ParaID , RMS[3].array[NUM_BRANCH - (unsigned int)Offset]);}	
			  break;
	}
	case BOARDI_KW_HI_ADDR:
	{
		if (Offset < PANEL_CT_DATA_LENGTH)
			strCTKw[2].array[Offset] = (unsigned int)parameter_data;
		else
			strCTKw[3].array[Offset - PANEL_CT_DATA_LENGTH] = (unsigned int)parameter_data;
		
	break; 
	}
	default:
	break;
	}
	break;
	}	
case BOARD_III:
	{
	switch(ParaID)
	{
	case BOARDI_CT_RMS_HI_ADDR:
	{		
			if ((unsigned int)Offset < NUM_BRANCH)
			  {RMS[4].array[Offset] = (unsigned int)parameter_data;	
			  add_to_log( ParaID , RMS[2].array[Offset]);}
			else
			  {RMS[5].array[NUM_BRANCH - (unsigned int)Offset] = (unsigned int)parameter_data;
			  add_to_log( ParaID , RMS[3].array[NUM_BRANCH - (unsigned int)Offset]);}
	break;
	}
	case BOARDI_KW_HI_ADDR:
	{
		if (Offset < PANEL_CT_DATA_LENGTH)
			strCTKw[4].array[Offset] = (unsigned int)parameter_data;
		else
			strCTKw[5].array[Offset - PANEL_CT_DATA_LENGTH] = (unsigned int)parameter_data;
		
	break; 
        }
	default:
	break;
	}
	break;
	}
  }	
  
}*/

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

	//LoadPer_Calc(SEC1);icos_mbusdb_update_reg
	//Max_Min_Parameter(SEC1);
	//Status_Update(SEC1);

	//LoadPer_Calc(SEC2);
	//Max_Min_Parameter(SEC2);
	//Status_Update(SEC2);

	//LoadPer_Calc(SEC3);
	//Max_Min_Parameter(SEC3);
	//Status_Update(SEC3);
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

	//Demand_Calc(PANEL3);
	//LoadPer_Calc(PANEL3);
	//Max_Min_Parameter(PANEL3);

	//Demand_Calc(PANEL4);
	//LoadPer_Calc(PANEL4);
	//Max_Min_Parameter(PANEL4);
}	

void LoadPer_Calc( unsigned int reg)
{
	unsigned int i;
	switch (reg)
	{
		case PRIMARY:
		{
			i = (Data.word.Primary.RMS_Curr_Phase_A&0xFFFF);
			dwMathBuff = ((unsigned long)i * 1000L);
			 (Data.word.Primary.LoadPer_Phase_A) = ((unsigned long)dwMathBuff / wPri_Thd.word.wPhaseA_OverCurr);
			 Reg[20].if_index = (if_index_cur | 20);
			 Reg[20].reg_addr = ((Data.word.Primary.LoadPer_Phase_A & 0xFFFF0000) >> 16);
			 Reg[20].reg_d.reg_value = (Data.word.Primary.LoadPer_Phase_A & 0x0000FFFF);
			 Reg[20].reg_type  = INPUT_REGISTER;
			 Reg[20].reg_flag  = 1;

			i = (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF);
			dwMathBuff = ((unsigned long)i * 1000L);
			 (Data.word.Primary.LoadPer_Phase_B) = ((unsigned long)dwMathBuff / wPri_Thd.word.wPhaseB_OverCurr);
			 Reg[21].if_index = (if_index_cur | 21);
			 Reg[21].reg_addr = ((Data.word.Primary.LoadPer_Phase_B & 0xFFFF0000) >> 16);
			 Reg[21].reg_d.reg_value = (Data.word.Primary.LoadPer_Phase_B & 0x0000FFFF);
			 Reg[21].reg_type  = INPUT_REGISTER;
			 Reg[21].reg_flag  = 1;

	

			i = (Data.word.Primary.RMS_Curr_Phase_C&0xFFFF);
			dwMathBuff = ((unsigned long)i * 1000L);
			 (Data.word.Primary.LoadPer_Phase_C) = ((unsigned long)dwMathBuff / wPri_Thd.word.wPhaseC_OverCurr);
			 Reg[22].if_index = (if_index_cur | 22);
			 Reg[22].reg_addr = ((Data.word.Primary.LoadPer_Phase_C & 0xFFFF0000) >> 16);
			 Reg[22].reg_d.reg_value = (Data.word.Primary.LoadPer_Phase_C & 0x0000FFFF);
			 Reg[22].reg_type  = INPUT_REGISTER;
			 Reg[22].reg_flag  = 1;
			
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
			(Data.word.Secondary.LoadPer_Phase_A) = ((unsigned long)dwMathBuff / wSec_Thd.word.wPhaseA_OverCurr);
			 Reg[PRI_OFFSET + 34].if_index = (if_index_cur | (PRI_OFFSET + 34));
			 Reg[PRI_OFFSET + 34].reg_addr = ((Data.word.Secondary.LoadPer_Phase_A & 0xFFFF0000) >> 16);
			 Reg[PRI_OFFSET + 34].reg_d.reg_value = (Data.word.Secondary.LoadPer_Phase_A & 0x0000FFFF);
			 Reg[PRI_OFFSET + 34].reg_type  = INPUT_REGISTER;
			 Reg[PRI_OFFSET + 34].reg_flag  = 1;
			
			i = (Data.word.Secondary.RMS_Curr_Phase_B&0xFFFF);
			dwMathBuff = ((unsigned long)i * 1000L);
			(Data.word.Secondary.LoadPer_Phase_B) = ((unsigned long)dwMathBuff / wSec_Thd.word.wPhaseB_OverCurr);
			 Reg[PRI_OFFSET + 35].if_index = (if_index_cur | (PRI_OFFSET + 34));
			 Reg[PRI_OFFSET + 35].reg_addr = ((Data.word.Secondary.LoadPer_Phase_B & 0xFFFF0000) >> 16);
			 Reg[PRI_OFFSET + 35].reg_d.reg_value = (Data.word.Secondary.LoadPer_Phase_B & 0x0000FFFF);
			 Reg[PRI_OFFSET + 35].reg_type  = INPUT_REGISTER;
			 Reg[PRI_OFFSET + 35].reg_flag  = 1;

			i = (Data.word.Secondary.RMS_Curr_Phase_C&0xFFFF);
			dwMathBuff = ((unsigned long)i * 1000L);
			(Data.word.Secondary.LoadPer_Phase_C) = ((unsigned long)dwMathBuff / wSec_Thd.word.wPhaseC_OverCurr);
			 Reg[PRI_OFFSET + 36].if_index = (if_index_cur | (PRI_OFFSET + 36));
		         Reg[PRI_OFFSET + 36].reg_addr = ((Data.word.Secondary.LoadPer_Phase_C & 0xFFFF0000) >> 16);
			 Reg[PRI_OFFSET + 36].reg_d.reg_value = (Data.word.Secondary.LoadPer_Phase_C & 0x0000FFFF);
			 Reg[PRI_OFFSET + 36].reg_type  = INPUT_REGISTER;
			 Reg[PRI_OFFSET + 36].reg_flag  = 1;
		break;
		}
		case PANEL1:
		{
			for(i=0; i<84; i++)
			{
				dwMathBuff = (unsigned long)((unsigned long)((Data.array[SEC_OFFSET + i]&0xFFFF)) * 1000);
				if(Data_In.word.wOverCurrentLimit[0][i])
				{
					
				Data.array[KW_3_OFFSET + i] = ((unsigned long)((0x1900|i)|(0<<14))<<16)|((unsigned long)(dwMathBuff/Data_In.word.wOverCurrentLimit[0][i]));
				Reg[KW_3_OFFSET + i].if_index = (if_index_cur | (KW_3_OFFSET + i));
				Reg[KW_3_OFFSET + i].reg_addr = ((Data.array[KW_3_OFFSET + i] & 0xFFFF0000) >> 16);
			        Reg[KW_3_OFFSET + i].reg_d.reg_value = (Data.array[KW_3_OFFSET + i] & 0x0000FFFF);
			        Reg[KW_3_OFFSET + i].reg_type  = INPUT_REGISTER;
			        Reg[KW_3_OFFSET + i].reg_flag  = 1;
					
				}
			}
		break;
		}
		case PANEL2:
		{
			for(i=0; i<84; i++)
			{
				dwMathBuff = (unsigned long)((unsigned long)((Data.array[RMS_0_OFFSET + i]&0xFFFF)) * 1000);
				if(Data_In.word.wOverCurrentLimit[1][i])
				{
				Data.array[LOAD_0_OFFSET + i] = ((unsigned long)((0x1900|i)|(1<<14))<<16)|((unsigned long)(dwMathBuff / Data_In.word.wOverCurrentLimit[1][i]));
				Reg[LOAD_0_OFFSET + i].if_index = (if_index_cur | (LOAD_0_OFFSET + i));
				Reg[LOAD_0_OFFSET + i].reg_addr = ((Data.array[LOAD_0_OFFSET + i] & 0xFFFF0000) >> 16);
			        Reg[LOAD_0_OFFSET + i].reg_d.reg_value = (Data.array[LOAD_0_OFFSET + i] & 0x0000FFFF);
			        Reg[LOAD_0_OFFSET + i].reg_type  = INPUT_REGISTER;
			        Reg[LOAD_0_OFFSET + i].reg_flag  = 1;

				}
			}
		break;	
		}
		case PANEL3:
		{
			for(i=0; i<84; i++)
			{
				dwMathBuff = (unsigned long)((unsigned long)((Data.array[RMS_1_OFFSET + i]&0xFFFF)) * 1000);
				if(Data_In.word.wOverCurrentLimit[2][i])
				{
				Data.array[LOAD_1_OFFSET + i] = ((unsigned long)((0x1900|i)|(2<<14))<<16)|((unsigned long)(dwMathBuff / Data_In.word.wOverCurrentLimit[2][i]));
				Reg[LOAD_1_OFFSET + i].if_index = (if_index_cur | (LOAD_0_OFFSET + i));
				Reg[LOAD_1_OFFSET + i].reg_addr = ((Data.array[LOAD_1_OFFSET + i] & 0xFFFF0000) >> 16);
			        Reg[LOAD_1_OFFSET + i].reg_d.reg_value = (Data.array[LOAD_1_OFFSET + i] & 0x0000FFFF);
			        Reg[LOAD_1_OFFSET + i].reg_type  = INPUT_REGISTER;
			        Reg[LOAD_1_OFFSET + i].reg_flag  = 1;
				}
			}
		break;
		}
		case PANEL4:
		{
			for(i=0; i<84; i++)
			{
				dwMathBuff = (unsigned long)((unsigned long)((Data.array[RMS_2_OFFSET + i]&0xFFFF)) * 1000);
				if(Data_In.word.wOverCurrentLimit[2][i])
				{
				Data.array[LOAD_2_OFFSET + i] = ((unsigned long)((0x1900|i)|(3<<14))<<16)|((unsigned long)(dwMathBuff / Data_In.word.wOverCurrentLimit[2][i]));
				Reg[LOAD_2_OFFSET + i].if_index = (if_index_cur | (LOAD_2_OFFSET + i));
				Reg[LOAD_2_OFFSET + i].reg_addr = ((Data.array[LOAD_2_OFFSET + i] & 0xFFFF0000) >> 16);
			        Reg[LOAD_2_OFFSET + i].reg_d.reg_value = (Data.array[LOAD_2_OFFSET + i] & 0x0000FFFF);
			        Reg[LOAD_2_OFFSET + i].reg_type  = INPUT_REGISTER;
			        Reg[LOAD_2_OFFSET + i].reg_flag  = 1;
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
	unsigned int *ptCTCurDemand;
	unsigned int *ptCTKwDemand;

	unsigned int i=0;
	unsigned int j=0;
	unsigned int alarm;
	
	switch (reg)
	  {
	  case PRIMARY:
	    {
		//printf("\nInside Primary case\n");
		Data.word.Pri_Status_Flag.paraID = 0x2C;

	    	if ((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) < wPri_Thd.word.wPhaseA_UnderVolt)
			Data.word.Pri_Status_Flag.PhaseA_UnderVolt = 1;
		else{
		  if((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) > wPri_Thd.word.wPhaseA_UnderVolt + VOLT_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseA_UnderVolt = 0;
		}

		if ((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseB_UnderVolt)
			Data.word.Pri_Status_Flag.PhaseB_UnderVolt = 1;
		else{
		  if((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_UnderVolt + VOLT_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseB_UnderVolt = 0;
		}

		if ((Data.word.Primary.L2N_Volt_Phase_C & 0x0000FFFF) < wPri_Thd.word.wPhaseC_UnderVolt)
			Data.word.Pri_Status_Flag.PhaseC_UnderVolt = 1;
		else{
		   if((Data.word.Primary.L2N_Volt_Phase_C & 0x0000FFFF) > wPri_Thd.word.wPhaseC_UnderVolt + VOLT_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseC_UnderVolt = 0;
		}

		if ((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) > wPri_Thd.word.wPhaseA_OverVolt)
			Data.word.Pri_Status_Flag.PhaseA_OverVolt = 1;
		else{
		  if((Data.word.Primary.L2N_Volt_Phase_A & 0x0000FFFF) < wPri_Thd.word.wPhaseA_OverVolt - VOLT_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseA_OverVolt = 0;
		}

		if ((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_OverVolt)
			Data.word.Pri_Status_Flag.PhaseB_OverVolt = 1;
		else{
		  if((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseB_OverVolt - VOLT_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseB_OverVolt = 0;
		}

		if ((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseC_OverVolt)
			Data.word.Pri_Status_Flag.PhaseC_OverVolt = 1;
		else{
		  if((Data.word.Primary.L2N_Volt_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseC_OverVolt - VOLT_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseC_OverVolt = 0;
		}

		// Current Status
		if ((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) < wPri_Thd.word.wPhaseA_UnderCurr)
			Data.word.Pri_Status_Flag.PhaseA_UnderCurr = 1;
		else{
		  if((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) > wPri_Thd.word.wPhaseA_UnderCurr + IP_UC_CURR_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseA_UnderCurr = 0;
		}

		if ((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) < wPri_Thd.word.wPhaseB_UnderCurr)
			Data.word.Pri_Status_Flag.PhaseB_UnderCurr = 1;
		else{
		  if((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_UnderCurr + IP_UC_CURR_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseB_UnderCurr = 0;
		}

		if ((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) < wPri_Thd.word.wPhaseC_UnderCurr)
			Data.word.Pri_Status_Flag.PhaseC_UnderCurr = 1;
		else{
		  if((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) > wPri_Thd.word.wPhaseC_UnderCurr + IP_UC_CURR_HYSTER)
		  	Data.word.Pri_Status_Flag.PhaseC_UnderCurr = 0;
		}

		if (((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) > wPri_Thd.word.wPhaseA_OverCurr * 9)
		{
		  	Data.word.Pri_Status_Flag.PhaseA_Curr_High =1;
		  if ((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) > wPri_Thd.word.wPhaseA_OverCurr)
		    	Data.word.Pri_Status_Flag.PhaseA_OverCurr = 1;
		  else {
		       if ((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) < (wPri_Thd.word.wPhaseA_OverCurr - IP_OC_CURR_HYSTER))
		         	Data.word.Pri_Status_Flag.PhaseA_OverCurr = 0;
		  }
		}
                else {
		  if((Data.word.Primary.RMS_Curr_Phase_A & 0x0000FFFF) < (wPri_Thd.word.wPhaseA_OverCurr * 9) - IP_HIGH_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseA_OverCurr = 0;
			Data.word.Pri_Status_Flag.PhaseA_Curr_High = 0;
		  }
		}
		
		if (((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) > wPri_Thd.word.wPhaseB_OverCurr * 9)
		{
		  	Data.word.Pri_Status_Flag.PhaseB_Curr_High =1;
		  if ((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) > wPri_Thd.word.wPhaseB_OverCurr)
		    	Data.word.Pri_Status_Flag.PhaseB_OverCurr = 1;
		  else {
		      if ((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) < (wPri_Thd.word.wPhaseB_OverCurr - IP_OC_CURR_HYSTER))
		         	Data.word.Pri_Status_Flag.PhaseB_OverCurr = 0;
		  }
		}
                else {
		  if(((Data.word.Primary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) < (wPri_Thd.word.wPhaseB_OverCurr * 9) - IP_HIGH_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseB_OverCurr = 0;
			Data.word.Pri_Status_Flag.PhaseB_Curr_High = 0;
		  }
		}

		if (((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) > wPri_Thd.word.wPhaseC_OverCurr * 9)
		{
		  	Data.word.Pri_Status_Flag.PhaseC_Curr_High =1;
		  if ((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) > wPri_Thd.word.wPhaseC_OverCurr)
		    	Data.word.Pri_Status_Flag.PhaseC_OverCurr = 1;
		  else {
		       if ((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) < (wPri_Thd.word.wPhaseC_OverCurr - IP_OC_CURR_HYSTER))
		         	Data.word.Pri_Status_Flag.PhaseC_OverCurr = 0;
		  }
		}
                else {
		  if(((Data.word.Primary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) < (wPri_Thd.word.wPhaseC_OverCurr * 9) - IP_HIGH_CURR_HYSTER)
		  {
			Data.word.Pri_Status_Flag.PhaseC_OverCurr = 0;
			Data.word.Pri_Status_Flag.PhaseC_Curr_High = 0;
		  }
		}

		if(((Data_In.word.wSysInfo.word.wSysConfig & 0x0000FFFF) & 0x0040) == 0x0040)
		  {
		  	if((Data.word.Primary.RMS_Curr_Neutral & 0x0000FFFF) > wPri_Thd.word.wNeutral_OverCurr)
			  	Data.word.Pri_Status_Flag.Neutral_OverCurr = 1;
			else {
			     if((Data.word.Primary.RMS_Curr_Neutral & 0x0000FFFF) < (wPri_Thd.word.wNeutral_OverCurr - IP_NEUTRAL_CURR_HYSTER))
				Data.word.Pri_Status_Flag.Neutral_OverCurr = 0;
			}
		  }
		else
		  		Data.word.Pri_Status_Flag.Neutral_OverCurr = 0;
		
		//printf("\nPriamry Flag : %x\n",Data.array[CT_MAX_KW_DEMAND_24HR_3]);
		// Over THD Current and Voltage -- [To be filled later]
	    break;
	    }
	  /*case SEC1:
	    {
		if(Secondary1.word.RMS_Curr_Phase_A < Data_In.word.wPanelThd[0].word.wPanel1PhaseA_UnderCurr)
		  StatusSecondary1.bit.PhaseA_UnderCurr = 1;
		else {
		  if(Secondary1.word.RMS_Curr_Phase_A > (Data_In.word.wPanelThd[0].word.wPanel1PhaseA_UnderCurr + PANEL1_UC_HYSTER))
		    StatusSecondary1.bit.PhaseA_UnderCurr = 0;
		  }

		if(Secondary1.word.RMS_Curr_Phase_B < Data_In.word.wPanelThd[0].word.wPanel1PhaseB_UnderCurr)
		  StatusSecondary1.bit.PhaseB_UnderCurr = 1;
		else {
		  if(Secondary1.word.RMS_Curr_Phase_B > (Data_In.word.wPanelThd[0].word.wPanel1PhaseB_UnderCurr + PANEL1_UC_HYSTER))
		    StatusSecondary1.bit.PhaseB_UnderCurr = 0;
		  }

		if(Secondary1.word.RMS_Curr_Phase_C < Data_In.word.wPanelThd[0].word.wPanel1PhaseC_UnderCurr)
		  StatusSecondary1.bit.PhaseC_UnderCurr = 1;
		else {
		  if(Secondary1.word.RMS_Curr_Phase_C > (Data_In.word.wPanelThd[0].word.wPanel1PhaseC_UnderCurr + PANEL1_UC_HYSTER))
		    StatusSecondary1.bit.PhaseC_UnderCurr = 0;
		  }		 

		if((Secondary1.word.RMS_Curr_Phase_A * 10) > (Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr * 9))
		{
		  StatusSecondary1.bit.PhaseA_Curr_High = 1;
		  if(Secondary1.word.RMS_Curr_Phase_A > Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr)
		    StatusSecondary1.bit.PhaseA_OverCurr = 1;
		  else {
		    if(Secondary1.word.RMS_Curr_Phase_A < (Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr - PANEL1_OC_CURR_HYSTER))
		      StatusSecondary1.bit.PhaseA_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary1.word.RMS_Curr_Phase_A * 10) < (Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr * 9) - PANEL1_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary1.bit.PhaseA_Curr_High = 0;
			StatusSecondary1.bit.PhaseA_OverCurr = 0;
		    }
		}

		if((Secondary1.word.RMS_Curr_Phase_B * 10) > (Data_In.word.wPanelThd[0].word.wPanel1PhaseB_OverCurr * 9))
		{
		  StatusSecondary1.bit.PhaseB_Curr_High = 1;
		  if(Secondary1.word.RMS_Curr_Phase_B > Data_In.word.wPanelThd[0].word.wPanel1PhaseB_OverCurr)
		    StatusSecondary1.bit.PhaseB_OverCurr = 1;
		  else {
		    if(Secondary1.word.RMS_Curr_Phase_B < (Data_In.word.wPanelThd[0].word.wPanel1PhaseB_OverCurr - PANEL1_OC_CURR_HYSTER))
		      StatusSecondary1.bit.PhaseB_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary1.word.RMS_Curr_Phase_B * 10) < (Data_In.word.wPanelThd[0].word.wPanel1PhaseB_OverCurr * 9) - PANEL1_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary1.bit.PhaseB_Curr_High = 0;
			StatusSecondary1.bit.PhaseB_OverCurr = 0;
		    }
		}

		if((Secondary1.word.RMS_Curr_Phase_C * 10) > (Data_In.word.wPanelThd[0].word.wPanel1PhaseC_OverCurr * 9))
		{
		  StatusSecondary1.bit.PhaseC_Curr_High = 1;
		  if(Secondary1.word.RMS_Curr_Phase_C > Data_In.word.wPanelThd[0].word.wPanel1PhaseC_OverCurr)
		    StatusSecondary1.bit.PhaseC_OverCurr = 1;
		  else {
		    if(Secondary1.word.RMS_Curr_Phase_C < (Data_In.word.wPanelThd[0].word.wPanel1PhaseC_OverCurr - PANEL1_OC_CURR_HYSTER))
		      StatusSecondary1.bit.PhaseC_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary1.word.RMS_Curr_Phase_C * 10) < (Data_In.word.wPanelThd[0].word.wPanel1PhaseC_OverCurr * 9) - PANEL1_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary1.bit.PhaseC_Curr_High = 0;
			StatusSecondary1.bit.PhaseC_OverCurr = 0;
		    }
		}
		
		if (Secondary1.word.RMS_Curr_Neutral > Data_In.word.wPanelThd[0].word.wPanel1Neutral_OverCurr)
		   StatusSecondary1.bit.Neutral_OverCurr = 1;
		else {
		  if (Secondary1.word.RMS_Curr_Neutral < (Data_In.word.wPanelThd[0].word.wPanel1Neutral_OverCurr - PANEL1_NEUTRAL_CURR))
		    StatusSecondary1.bit.Neutral_OverCurr = 0;
		}

		// Over THD Current and Voltage -- [To be filled later]
	    break;
	    }*/
	  /*case SEC2:
	    {
		if(Secondary2.word.RMS_Curr_Phase_A < Data_In.word.wPanelThd[0].word.wPanel2PhaseA_UnderCurr)
		  StatusSecondary2.bit.PhaseA_UnderCurr = 1;
		else {
		  if(Secondary2.word.RMS_Curr_Phase_A > (Data_In.word.wPanelThd[0].word.wPanel2PhaseA_UnderCurr + PANEL2_UC_HYSTER))
		    StatusSecondary2.bit.PhaseA_UnderCurr = 0;
		  }

		if(Secondary2.word.RMS_Curr_Phase_B < Data_In.word.wPanelThd[0].word.wPanel2PhaseB_UnderCurr)
		  StatusSecondary2.bit.PhaseB_UnderCurr = 1;
		else {
		  if(Secondary2.word.RMS_Curr_Phase_B > (Data_In.word.wPanelThd[0].word.wPanel2PhaseB_UnderCurr + PANEL2_UC_HYSTER))
		    StatusSecondary2.bit.PhaseB_UnderCurr = 0;
		  }

		if(Secondary2.word.RMS_Curr_Phase_C < Data_In.word.wPanelThd[0].word.wPanel2PhaseC_UnderCurr)
		  StatusSecondary2.bit.PhaseC_UnderCurr = 1;
		else {
		  if(Secondary2.word.RMS_Curr_Phase_C > (Data_In.word.wPanelThd[0].word.wPanel2PhaseC_UnderCurr + PANEL2_UC_HYSTER))
		    StatusSecondary2.bit.PhaseC_UnderCurr = 0;
		  }		 

		if((Secondary2.word.RMS_Curr_Phase_A * 10) > (Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr * 9))
		{
		  StatusSecondary2.bit.PhaseA_Curr_High = 1;
		  if(Secondary2.word.RMS_Curr_Phase_A > Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr)
		    StatusSecondary2.bit.PhaseA_OverCurr = 1;
		  else {
		    if(Secondary2.word.RMS_Curr_Phase_A < (Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr - PANEL2_OC_CURR_HYSTER))
		      StatusSecondary2.bit.PhaseA_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary2.word.RMS_Curr_Phase_A * 10) < (Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr * 9) - PANEL2_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary2.bit.PhaseA_Curr_High = 0;
			StatusSecondary2.bit.PhaseA_OverCurr = 0;
		    }
		}

		if((Secondary2.word.RMS_Curr_Phase_B * 10) > (Data_In.word.wPanelThd[0].word.wPanel2PhaseB_OverCurr * 9))
		{
		  StatusSecondary2.bit.PhaseB_Curr_High = 1;
		  if(Secondary2.word.RMS_Curr_Phase_B > Data_In.word.wPanelThd[0].word.wPanel2PhaseB_OverCurr)
		    StatusSecondary2.bit.PhaseB_OverCurr = 1;
		  else {
		    if(Secondary2.word.RMS_Curr_Phase_B < (Data_In.word.wPanelThd[0].word.wPanel2PhaseB_OverCurr - PANEL2_OC_CURR_HYSTER))
		      StatusSecondary2.bit.PhaseB_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary2.word.RMS_Curr_Phase_B * 10) < (Data_In.word.wPanelThd[0].word.wPanel2PhaseB_OverCurr * 9) - PANEL2_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary2.bit.PhaseB_Curr_High = 0;
			StatusSecondary2.bit.PhaseB_OverCurr = 0;
		    }
		}

		if((Secondary2.word.RMS_Curr_Phase_C * 10) > (Data_In.word.wPanelThd[0].word.wPanel2PhaseC_OverCurr * 9))
		{
		  StatusSecondary2.bit.PhaseC_Curr_High = 1;
		  if(Secondary2.word.RMS_Curr_Phase_C > Data_In.word.wPanelThd[0].word.wPanel2PhaseC_OverCurr)
		    StatusSecondary2.bit.PhaseC_OverCurr = 1;
		  else {
		    if(Secondary2.word.RMS_Curr_Phase_C < (Data_In.word.wPanelThd[0].word.wPanel2PhaseC_OverCurr - PANEL2_OC_CURR_HYSTER))
		      StatusSecondary2.bit.PhaseC_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary2.word.RMS_Curr_Phase_C * 10) < (Data_In.word.wPanelThd[0].word.wPanel2PhaseC_OverCurr * 9) - PANEL2_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary2.bit.PhaseC_Curr_High = 0;
			StatusSecondary2.bit.PhaseC_OverCurr = 0;
		    }
		}
		
		if (Secondary2.word.RMS_Curr_Neutral > Data_In.word.wPanelThd[0].word.wPanel2Neutral_OverCurr)
		   StatusSecondary2.bit.Neutral_OverCurr = 1;
		else {
		  if (Secondary2.word.RMS_Curr_Neutral < (Data_In.word.wPanelThd[0].word.wPanel2Neutral_OverCurr - PANEL2_NEUTRAL_CURR))
		    StatusSecondary2.bit.Neutral_OverCurr = 0;
		}
	    break;
	    }*/
	  /*case SEC3:
	    {
	    	if(Secondary3.word.RMS_Curr_Phase_A < Data_In.word.wPanelThd[1].word.wPanel1PhaseA_UnderCurr)
		  StatusSecondary3.bit.PhaseA_UnderCurr = 1;
		else {
		  if(Secondary3.word.RMS_Curr_Phase_A > (Data_In.word.wPanelThd[1].word.wPanel1PhaseA_UnderCurr + PANEL3_UC_HYSTER))
		    StatusSecondary3.bit.PhaseA_UnderCurr = 0;
		  }

		if(Secondary3.word.RMS_Curr_Phase_B < Data_In.word.wPanelThd[1].word.wPanel1PhaseB_UnderCurr)
		  StatusSecondary3.bit.PhaseB_UnderCurr = 1;
		else {
		  if(Secondary3.word.RMS_Curr_Phase_B > (Data_In.word.wPanelThd[1].word.wPanel1PhaseB_UnderCurr + PANEL3_UC_HYSTER))
		    StatusSecondary3.bit.PhaseB_UnderCurr = 0;
		  }

		if(Secondary3.word.RMS_Curr_Phase_C < Data_In.word.wPanelThd[1].word.wPanel1PhaseC_UnderCurr)
		  StatusSecondary3.bit.PhaseC_UnderCurr = 1;
		else {
		  if(Secondary3.word.RMS_Curr_Phase_C > (Data_In.word.wPanelThd[1].word.wPanel1PhaseC_UnderCurr + PANEL3_UC_HYSTER))
		    StatusSecondary3.bit.PhaseC_UnderCurr = 0;
		  }		 

		if((Secondary3.word.RMS_Curr_Phase_A * 10) > (Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr * 9))
		{
		  StatusSecondary3.bit.PhaseA_Curr_High = 1;
		  if(Secondary3.word.RMS_Curr_Phase_A > Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr)
		    StatusSecondary3.bit.PhaseA_OverCurr = 1;
		  else {
		    if(Secondary3.word.RMS_Curr_Phase_A < (Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr - PANEL3_OC_CURR_HYSTER))
		      StatusSecondary3.bit.PhaseA_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary3.word.RMS_Curr_Phase_A * 10) < (Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr * 9) - PANEL3_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary3.bit.PhaseA_Curr_High = 0;
			StatusSecondary3.bit.PhaseA_OverCurr = 0;
		    }
		}

		if((Secondary3.word.RMS_Curr_Phase_B * 10) > (Data_In.word.wPanelThd[1].word.wPanel1PhaseB_OverCurr * 9))
		{
		  StatusSecondary3.bit.PhaseB_Curr_High = 1;
		  if(Secondary3.word.RMS_Curr_Phase_B > Data_In.word.wPanelThd[1].word.wPanel1PhaseB_OverCurr)
		    StatusSecondary3.bit.PhaseB_OverCurr = 1;
		  else {
		    if(Secondary3.word.RMS_Curr_Phase_B < (Data_In.word.wPanelThd[1].word.wPanel1PhaseB_OverCurr - PANEL3_OC_CURR_HYSTER))
		      StatusSecondary3.bit.PhaseB_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary3.word.RMS_Curr_Phase_B * 10) < (Data_In.word.wPanelThd[1].word.wPanel1PhaseB_OverCurr * 9) - PANEL3_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary3.bit.PhaseB_Curr_High = 0;
			StatusSecondary3.bit.PhaseB_OverCurr = 0;
		    }
		}

		if((Secondary3.word.RMS_Curr_Phase_C * 10) > (Data_In.word.wPanelThd[1].word.wPanel1PhaseC_OverCurr * 9))
		{
		  StatusSecondary3.bit.PhaseC_Curr_High = 1;
		  if(Secondary3.word.RMS_Curr_Phase_C > Data_In.word.wPanelThd[1].word.wPanel1PhaseC_OverCurr)
		    StatusSecondary3.bit.PhaseC_OverCurr = 1;
		  else {
		    if(Secondary3.word.RMS_Curr_Phase_C < (Data_In.word.wPanelThd[1].word.wPanel1PhaseC_OverCurr - PANEL3_OC_CURR_HYSTER))
		      StatusSecondary3.bit.PhaseC_OverCurr = 0;
		  }
		}
		else {
		  if((Secondary3.word.RMS_Curr_Phase_C * 10) < (Data_In.word.wPanelThd[1].word.wPanel1PhaseC_OverCurr * 9) - PANEL3_HIGH_CURR_HYSTER)
		    {
		    	StatusSecondary3.bit.PhaseC_Curr_High = 0;
			StatusSecondary3.bit.PhaseC_OverCurr = 0;
		    }
		}
		
		if (Secondary3.word.RMS_Curr_Neutral > Data_In.word.wPanelThd[1].word.wPanel1Neutral_OverCurr)
		   StatusSecondary3.bit.Neutral_OverCurr = 1;
		else {
		  if (Secondary3.word.RMS_Curr_Neutral < (Data_In.word.wPanelThd[1].word.wPanel1Neutral_OverCurr - PANEL3_NEUTRAL_CURR))
		    StatusSecondary3.bit.Neutral_OverCurr = 0;
		}		
	    break;	
	    }*/	
	  case SEC:
	    {
		//printf("\nInside Secondary case\n");
		Data.word.Sec_Status_Flag.paraID = 0x0F;
		if ((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) < wSec_Thd.word.wPhaseA_UnderVolt)
			Data.word.Sec_Status_Flag.PhaseA_UnderVolt = 1;
		else{
		  if((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_UnderVolt + VOLT_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseA_UnderVolt = 0;
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) < wSec_Thd.word.wPhaseB_UnderVolt)
			Data.word.Sec_Status_Flag.PhaseB_UnderVolt = 1;
		else{
		  if((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_UnderVolt + VOLT_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseB_UnderVolt = 0;
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) < wSec_Thd.word.wPhaseC_UnderVolt)
			Data.word.Sec_Status_Flag.PhaseC_UnderVolt = 1;
		else{
		  if((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_UnderVolt + VOLT_HYSTER)
		 	 Data.word.Sec_Status_Flag.PhaseC_UnderVolt = 0;
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_OverVolt)
			Data.word.Sec_Status_Flag.PhaseA_OverVolt = 1;
		else{
		  if((Data.word.Secondary.L2N_Volt_Phase_A & 0x0000FFFF) < wSec_Thd.word.wPhaseA_OverVolt - VOLT_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseA_OverVolt = 0;
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_OverVolt)
			Data.word.Sec_Status_Flag.PhaseB_OverVolt = 1;
		else{
		  if((Data.word.Secondary.L2N_Volt_Phase_B & 0x0000FFFF) < wSec_Thd.word.wPhaseB_OverVolt - VOLT_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseB_OverVolt = 0;
		}

		if ((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_OverVolt)
			Data.word.Sec_Status_Flag.PhaseC_OverVolt = 1;
		else{
		  if((Data.word.Secondary.L2N_Volt_Phase_C & 0x0000FFFF) < wSec_Thd.word.wPhaseC_OverVolt - VOLT_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseC_OverVolt = 0;
		}

		// Current Status
		if ((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) < wSec_Thd.word.wPhaseA_UnderCurr)
			Data.word.Sec_Status_Flag.PhaseA_UnderCurr = 1;
		else{
		  if((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_UnderCurr + OP_UC_CURR_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseA_UnderCurr = 0;
		}

		if ((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) < wSec_Thd.word.wPhaseB_UnderCurr)
			Data.word.Sec_Status_Flag.PhaseB_UnderCurr = 1;
		else{
		  if((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_UnderCurr + OP_UC_CURR_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseB_UnderCurr = 0;
		}

		if ((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) < wSec_Thd.word.wPhaseC_UnderCurr)
			Data.word.Sec_Status_Flag.PhaseC_UnderCurr = 1;
		else{
		  if((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_UnderCurr + OP_UC_CURR_HYSTER)
		  	Data.word.Sec_Status_Flag.PhaseC_UnderCurr = 0;
		}

		if (((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseA_OverCurr * 9)
		{
		  	Data.word.Sec_Status_Flag.PhaseA_Curr_High =1;
		  if ((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) > wSec_Thd.word.wPhaseA_OverCurr)
		    	Data.word.Sec_Status_Flag.PhaseA_OverCurr = 1;
		  else {
		       if ((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) < (wSec_Thd.word.wPhaseA_OverCurr - OP_OC_CURR_HYSTER))
		         Data.word.Sec_Status_Flag.PhaseA_OverCurr = 0;
		  }
		}
                else {
		  if(((Data.word.Secondary.RMS_Curr_Phase_A & 0x0000FFFF) * 10) < (wSec_Thd.word.wPhaseA_OverCurr * 9) - OP_OC_CURR_HYSTER)
		  {
			Data.word.Sec_Status_Flag.PhaseA_OverCurr = 0;
			Data.word.Sec_Status_Flag.PhaseA_Curr_High = 0;
		  }
		}
		
		if (((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseB_OverCurr * 9)
		{
		  	Data.word.Sec_Status_Flag.PhaseB_Curr_High =1;
		  if ((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) > wSec_Thd.word.wPhaseB_OverCurr)
		    	Data.word.Sec_Status_Flag.PhaseB_OverCurr = 1;
		  else {
		       if ((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) < (wSec_Thd.word.wPhaseB_OverCurr - OP_OC_CURR_HYSTER))
		         Data.word.Sec_Status_Flag.PhaseB_OverCurr = 0;
		  }
		}
                else {
		  if(((Data.word.Secondary.RMS_Curr_Phase_B & 0x0000FFFF) * 10) < (wSec_Thd.word.wPhaseB_OverCurr * 9) - OP_HIGH_CURR_HYSTER)
		  {
			Data.word.Sec_Status_Flag.PhaseB_OverCurr = 0;
			Data.word.Sec_Status_Flag.PhaseB_Curr_High = 0;
		  }
		}

		if (((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) > wSec_Thd.word.wPhaseC_OverCurr * 9)
		{
		  	Data.word.Sec_Status_Flag.PhaseC_Curr_High =1;
		  if ((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) > wSec_Thd.word.wPhaseC_OverCurr)
		    	Data.word.Sec_Status_Flag.PhaseC_OverCurr = 1;
		  else {
		       if ((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) < (wSec_Thd.word.wPhaseC_OverCurr - OP_OC_CURR_HYSTER))
		         Data.word.Sec_Status_Flag.PhaseC_OverCurr = 0;
		  }
		}
                else {
		  if(((Data.word.Secondary.RMS_Curr_Phase_C & 0x0000FFFF) * 10) < (wSec_Thd.word.wPhaseC_OverCurr * 9) - OP_HIGH_CURR_HYSTER)
		  {
			Data.word.Sec_Status_Flag.PhaseC_OverCurr = 0;
			Data.word.Sec_Status_Flag.PhaseC_Curr_High = 0;
		  }
		}

		if((Data.word.Secondary.RMS_Curr_Neutral & 0x0000FFFF) > wSec_Thd.word.wNeutral_OverCurr)
		  	Data.word.Sec_Status_Flag.Neutral_OverCurr = 1;
		else {
	 	  if((Data.word.Secondary.RMS_Curr_Neutral & 0x0000FFFF) < (wSec_Thd.word.wNeutral_OverCurr - OP_NEUTRAL_CURR_HYSTER))
		    	Data.word.Sec_Status_Flag.Neutral_OverCurr = 0;
		}
		//printf("\nPriamry Flag : %x\n",Data.array[PRI_STATUS_FLAG]);
	    break;
	    }	 	
	  case PANEL1:
	    {
		//printf("\nConfiguration : %x\n",((Data_In.word.wSysInfo.word.wConfiguration)&0xFF00)>>8);
		if ((((Data_In.word.wSysInfo.word.wConfiguration & 0x0000FFFF)&0xFF00)>>8) < 0x01)
		  {
			Data_In.word.wPanelAct[0].word.Active_Inactive[0] = 0;
			Data_In.word.wPanelAct[0].word.Active_Inactive[1] = 0;
			//Data_In.word.wPanelAct[0].word.Active_Inactive_S1[2] = 0;

			Data_In.word.wPanelAct[0].word.Active_Inactive[2] = 0;
			Data_In.word.wPanelAct[0].word.Active_Inactive[3] = 0;
			//Data_In.word.wPanelAct[1].word.Active_Inactive_S2[2] = 0;

			//Data_In.word.wPanelAct[0].word.Active_Inactive_S3[0] = 0;
			//Data_In.word.wPanelAct[0].word.Active_Inactive_S3[1] = 0;
			//Data_In.word.wPanelAct[0].word.Active_Inactive_S1[2] = 0;

			//Data_In.word.wPanelAct[0].word.Active_Inactive_S4[0] = 0;
			//Data_In.word.wPanelAct[0].word.Active_Inactive_S4[1] = 0;
			//Data_In.word.wPanelAct[1].word.Active_Inactive_S2[2] = 0;

			/*Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[0] = 0;
			Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[1] = 0;
			Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[2] = 0;

			Data_In.word.wPanelCurDemandAct[1].word.Active_Inactive_S2[0] = 0;
			Data_In.word.wPanelCurDemandAct[1].word.Active_Inactive_S2[1] = 0;
			Data_In.word.wPanelCurDemandAct[1].word.Active_Inactive_S2[2] = 0;
			
			Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[0] = 0;
			Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[1] = 0;
			Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[2] = 0;

			Data_In.word.wPanelKwDemandAct[1].word.Active_Inactive_S2[0] = 0;
			Data_In.word.wPanelKwDemandAct[1].word.Active_Inactive_S2[1] = 0;
			Data_In.word.wPanelKwDemandAct[1].word.Active_Inactive_S2[2] = 0;*/
	    break;
	    }
		//ptCTCurDemand = &strCTCurDemand[0].array[0];
		//ptCTKwDemand = &strCTKwDemand[0].array[0];
		Data.word.Panel1_Status_Undercurr[0].paraID = 0x1A;
		Data.word.Panel1_Status_Undercurr[1].paraID = 0x5A;
		Data.word.Panel1_Status_Undercurr[2].paraID = 0x9A;
		Data.word.Panel1_Status_Undercurr[3].paraID = 0xDA;

		Data.word.Panel1_Status_Overcurr[0].paraID = 0x2A;
		Data.word.Panel1_Status_Overcurr[1].paraID = 0x6A;
		Data.word.Panel1_Status_Overcurr[2].paraID = 0xAA;
		Data.word.Panel1_Status_Overcurr[3].paraID = 0xEA;

		//printf("\nData : %x\n",Data.array[PANEL14_UNDERCURR_STATUS_FLAG]);
		//printf("\nData : %x\n",Data.array[PANEL11_OVERCURR_STATUS_FLAG]);
		

		for (i=0; i<21; i++)
		{
				
		  alarm = 0x00000001 << i;
		  //****************************UNDER CURRENT Panel 1 Strip 1  (1~21)**************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[0] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][i])
			Data.array[SEC_STATUS_FLAG] |= alarm;
		    else {
			if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][i] + BRANCH_UC_HYSTER))
			 Data.array[SEC_STATUS_FLAG] &= (~alarm);
			}
		  }
		 else
		  Data.array[SEC_STATUS_FLAG] &= (~alarm);
		  //****************************UNDER CURRENT Panel 1 Strip 2  (21~42)**************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[1] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 21+ i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][21+i])
			Data.array[PANEL11_UNDERCURR_STATUS_FLAG] |= alarm;
		    else {
			if((Data.array[SEC_OFFSET + 21+ i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][21+i] + BRANCH_UC_HYSTER))
			 Data.array[PANEL11_UNDERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		 else
		  Data.array[PANEL11_UNDERCURR_STATUS_FLAG] &= (~alarm);
		 	
		 //****************************UNDER CURRENT DEMAND Panel 1 Strip 1  (1~16)**************************// 
		 /* if(Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[0] & alarm)
		  {
			if(*(ptCTCurDemand + i) < Data_In.word.wUnderCurDemandLimit[0][i])
			  sCurDemandStatusP1S1.array[0] |= alarm;
			else
			{
			  if(*(ptCTCurDemand + i) > (Data_In.word.wUnderCurDemandLimit[0][i] + BRANCH_UCD_HYSTER))
			   sCurDemandStatusP1S1.array[0] &= (~alarm);
			}
			
		  }
		  else
			sCurDemandStatusP1S1.array[0] &= (~alarm);
                  */
		  //****************************UNDER CURRENT DEMAND Panel 1 Strip 2  (1~16)**************************// 
		  /*if(Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S2[0] & alarm)
		  {
			if(*(ptCTCurDemand + 21+i) < Data_In.word.wUnderCurDemandLimit[0][21+i])
			  sCurDemandStatusP1S2.array[0] |= alarm;
			else
			{
			  if(*(ptCTCurDemand + 21+i) > (Data_In.word.wUnderCurDemandLimit[0][21+i] + BRANCH_UCD_HYSTER))
			   sCurDemandStatusP1S2.array[0] &= (~alarm);
			}
			
		  }
		  else
			sCurDemandStatusP1S2.array[0] &= (~alarm);
		  
		  */	
		  //****************************UNDER KW DEMAND Panel 1 Strip 1  (1~16)**************************// 
		  /*if(Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[0] & alarm)
		  {
			if(*(ptCTKwDemand + i) < Data_In.word.wUnderKwDemandLimit[0][i])
			  sKwDemandStatusP1S1.array[0] |= alarm;
			else
			{
			  if(*(ptCTKwDemand + i) > (Data_In.word.wUnderKwDemandLimit[0][i] + BRANCH_UKWD_HYSTER))
			   sKwDemandStatusP1S1.array[0] &= (~alarm);
			}
			
		  }
		  else
			sKwDemandStatusP1S1.array[0] &= (~alarm);
                  */
		  //****************************UNDER KW DEMAND Panel 1 Strip 2  (1~16)**************************// 
		  /*if(Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[0] & alarm)
		  {
			if(*(ptCTKwDemand + 21+i) < Data_In.word.wUnderKwDemandLimit[0][21+i])
			  sKwDemandStatusP1S2.array[0] |= alarm;
			else
			{
			  if(*(ptCTKwDemand + 21+i) > (Data_In.word.wUnderKwDemandLimit[0][21+i] + BRANCH_UKWD_HYSTER))
			   sKwDemandStatusP1S2.array[0] &= (~alarm);
			}
			
		  }
		  else
			sKwDemandStatusP1S2.array[0] &= (~alarm);*/
		}
                 
		for (i=0; i<21; i++)
		{
		  alarm = 0x00000001 << i;
		  //********************************UNDER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[2] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][42+i])
			Data.array[PANEL12_UNDERCURR_STATUS_FLAG] |= alarm;
		    else{
			if ((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][42+i] + BRANCH_UC_HYSTER))
			  Data.array[PANEL12_UNDERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		  else
		    Data.array[PANEL12_UNDERCURR_STATUS_FLAG] &= (~alarm);  
		  //********************************UNDER CURRENT Panel 1 Strip 4(63~84)******************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[3] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][63+i])
			Data.array[PANEL13_UNDERCURR_STATUS_FLAG] |= alarm;
		    else{
			if ((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][63+i] + BRANCH_UC_HYSTER))
			  Data.array[PANEL13_UNDERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		  else
		    Data.array[PANEL13_UNDERCURR_STATUS_FLAG] &= (~alarm);
		  
		  //********************************UNDER CURRENT DEMAND Panel 1 Strip 1(17~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[1] & alarm)
		  {
		    if(*(ptCTCurDemand +16 +i) < Data_In.word.wUnderCurDemandLimit[0][16+i])
			sCurDemandStatusP1S1.array[1] |= alarm;
		    else{
			if (*(ptCTCurDemand +16 +i) > (Data_In.word.wUnderCurDemandLimit[0][16+i] + BRANCH_UCD_HYSTER))
			  sCurDemandStatusP1S1.array[1] &= (~alarm);
			}
		  }
		  else
		    sCurDemandStatusP1S1.array[1] &= (~alarm);  
		*/
		  //********************************UNDER CURRENT DEMAND Panel 1 Strip 2(17~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S2[1] & alarm)
		  {
		    if(*(ptCTCurDemand+21 +16 +i) < Data_In.word.wUnderCurDemandLimit[0][21+16+i])
			sCurDemandStatusP1S2.array[1] |= alarm;
		    else{
			if (*(ptCTCurDemand+21 +16 +i) > (Data_In.word.wUnderCurDemandLimit[0][21+16+i] + BRANCH_UCD_HYSTER))
			  sCurDemandStatusP1S2.array[1] &= (~alarm);
			}
		  }
		  else
		    sCurDemandStatusP1S2.array[1] &= (~alarm); 
	           */
		  //********************************UNDER KW DEMAND Panel 1 Strip 1(17~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[1] & alarm)
		  {
		    if(*(ptCTKwDemand +16 +i) < Data_In.word.wUnderKwDemandLimit[0][16+i])
			sKwDemandStatusP1S1.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand +16 +i) > (Data_In.word.wUnderKwDemandLimit[0][16+i] + BRANCH_UKWD_HYSTER))
			  sKwDemandStatusP1S1.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S1.array[1] &= (~alarm);  
		*/
		  //********************************UNDER KW DEMAND Panel 1 Strip 2(17~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[1] & alarm)
		  {
		    if(*(ptCTKwDemand+21 +16 +i) < Data_In.word.wUnderKwDemandLimit[0][21+16+i])
			sKwDemandStatusP1S2.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand+21 +16 +i) > (Data_In.word.wUnderKwDemandLimit[0][21+16+i] + BRANCH_UKWD_HYSTER))
			  sKwDemandStatusP1S2.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S2.array[1] &= (~alarm);*/
		}
		
		
		
		for (i=0; i<21; i++)
		{
		  alarm = 1 << i;
	          //********************************OVER CURRENT Panel 1 Strip 1(1~21)*****************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[0] & alarm)
		  {
		    //printf("\nPanel11_Overcurrent_alarm\n");
		    if ((Data.array[SEC_OFFSET + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][i]))
			Data.array[PANEL14_UNDERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
		       if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][i] - BRANCH_OC_HYSTER))
		       Data.array[PANEL14_UNDERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		     Data.array[PANEL14_UNDERCURR_STATUS_FLAG] &= (~alarm);

		  //********************************OVER CURRENT Panel 1 Strip 2(21~42)*****************************//
		  if(Data_In.word.wPanelAct[0].word.Active_Inactive[1] & alarm)
		  {
		    //printf("\nPanel12_Overcurrent_alarm\n");
		    if ((Data.array[SEC_OFFSET + 21 + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][21+i]))
			Data.array[PANEL11_OVERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
		       if((Data.array[SEC_OFFSET + 21 + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][21+i] - BRANCH_OC_HYSTER))
		       Data.array[PANEL11_OVERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		    Data.array[PANEL11_OVERCURR_STATUS_FLAG] &= (~alarm);

		  
		  
		  //********************************OVER KW DEMAND Panel 1 Strip 1(1~11)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[1] & alarm)
		  {
		    if(*(ptCTKwDemand +i) > Data_In.word.wOverKwDemandLimit[0][i])
			sKwDemandStatusP1S1.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand +i) < (Data_In.word.wOverKwDemandLimit[0][i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S1.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S1.array[1] &= (~alarm);  
			*/
		  //********************************OVER KW DEMAND Panel 1 Strip 2(1~11)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[1] & alarm)
		  {
		    if(*(ptCTKwDemand+21+i) > Data_In.word.wOverKwDemandLimit[0][21+i])
			sKwDemandStatusP1S2.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand +21+i) < (Data_In.word.wOverKwDemandLimit[0][21+i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S2.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S2.array[1] &= (~alarm);*/
                }
     		
		for (i=0;i<21;i++)
		{
		  alarm = 0x00000001 << i;
		  //********************************OVER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[2] & alarm)
		  {
		    
		    if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][42+i]))
			Data.array[PANEL12_OVERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
			if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][42+i] - BRANCH_OC_HYSTER))
			  Data.array[PANEL12_OVERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		    Data.array[PANEL12_OVERCURR_STATUS_FLAG] &= (~alarm);
		
		  //********************************OVER CURRENT Panel 1 Strip 2(12~21)*****************************//
		  if (Data_In.word.wPanelAct[0].word.Active_Inactive[3] & alarm)
		  {
		    
		    if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][63+i]))
			Data.array[PANEL13_OVERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
			if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][63+i] - BRANCH_OC_HYSTER))
			  Data.array[PANEL13_OVERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		    Data.array[PANEL13_OVERCURR_STATUS_FLAG] &= (~alarm);
		  
		  
		  //********************************OVER CURRENT DEMAND Panel 1 Strip 1(12~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[2] & alarm)
		  {
		    if(*(ptCTCurDemand +11+i) > (Data_In.word.wOverCurDemandLimit[0][11+i]))
			sCurDemandStatusP1S1.array[2] |= alarm;
		    else
		    {
			if(*(ptCTCurDemand+11+i) < (Data_In.word.wOverCurDemandLimit[0][11+i] - BRANCH_OCD_HYSTER))
			  sCurDemandStatusP1S1.array[2] &= (~alarm);
		    }
		  }
		  else
		    sCurDemandStatusP1S1.array[2] &= (~alarm);
			*/		
		  //********************************OVER CURRENT DEMAND Panel 1 Strip 2(12~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S2[2] & alarm)
		  {
		    if(*(ptCTCurDemand +21+11+i) > (Data_In.word.wOverCurDemandLimit[0][21+11+i]))
			sCurDemandStatusP1S2.array[2] |= alarm;
		    else
		    {
			if(*(ptCTCurDemand+21+11+i) < (Data_In.word.wOverCurDemandLimit[0][21+11+i] - BRANCH_OCD_HYSTER))
			  sCurDemandStatusP1S2.array[2] &= (~alarm);
		    }
		  }
		  else
		    sCurDemandStatusP1S2.array[2] &= (~alarm);
	*/	  
		  //********************************OVER KW DEMAND Panel 1 Strip 1(12~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[2] & alarm)
		  {
		    if(*(ptCTKwDemand +11+i) > (Data_In.word.wOverKwDemandLimit[0][11+i]))
			sKwDemandStatusP1S1.array[2] |= alarm;
		    else
		    {
			if(*(ptCTKwDemand+11+i) < (Data_In.word.wOverKwDemandLimit[0][11+i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S1.array[2] &= (~alarm);
		    }
		  }
		  else
		    sKwDemandStatusP1S1.array[2] &= (~alarm);
		*/	
		  //********************************OVER KW DEMAND Panel 1 Strip 2(12~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[2] & alarm)
		  {
		    if(*(ptCTKwDemand +21+11+i) > (Data_In.word.wOverKwDemandLimit[0][21+11+i]))
			sKwDemandStatusP1S2.array[2] |= alarm;
		    else
		    {
			if(*(ptCTKwDemand+21+11+i) < (Data_In.word.wOverKwDemandLimit[0][21+11+i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S2.array[2] &= (~alarm);
		    }
		  }
		  else
		    sKwDemandStatusP1S2.array[2] &= (~alarm);*/
		  }
		  /*printf("\nPanel11_UnderCurr : %x\n",Data.array[SEC_STATUS_FLAG]);
		  printf("\nPanel12_UnderCurr : %x\n",Data.array[PANEL11_UNDERCURR_STATUS_FLAG]);
		  printf("\nPanel13_UnderCurr : %x\n",Data.array[PANEL12_UNDERCURR_STATUS_FLAG]);
		  printf("\nPanel14_UnderCurr : %x\n",Data.array[PANEL13_UNDERCURR_STATUS_FLAG]);*/
		  /*printf("\nPanel11_OverCurr : %x\n",Data.array[PANEL14_UNDERCURR_STATUS_FLAG]);
		  printf("\nPanel12_OverCurr : %x\n",Data.array[PANEL11_OVERCURR_STATUS_FLAG]);
		  printf("\nPanel13_OverCurr : %x\n",Data.array[PANEL12_OVERCURR_STATUS_FLAG]);
		  printf("\nPanel14_OverCurr : %x\n",Data.array[PANEL13_OVERCURR_STATUS_FLAG]);*/
		
	       
	       break;  
	    }
	  case PANEL2:
		
		//printf("\nConfiguration : %x\n",((Data_In.word.wSysInfo.word.wConfiguration)&0xFF00)>>8);	
		if ((((Data_In.word.wSysInfo.word.wConfiguration & 0x0000FFFF)&0xFF00)>>8) < 0x02)
		  {
			Data_In.word.wPanelAct[1].word.Active_Inactive[0] = 0;
			Data_In.word.wPanelAct[1].word.Active_Inactive[1] = 0;
			//Data_In.word.wPanelAct[0].word.Active_Inactive_S1[2] = 0;

			Data_In.word.wPanelAct[1].word.Active_Inactive[2] = 0;
			Data_In.word.wPanelAct[1].word.Active_Inactive[3] = 0;
			//Data_In.word.wPanelAct[1].word.Active_Inactive_S2[2] = 0;

			//Data_In.word.wPanelAct[1].word.Active_Inactive_S3[0] = 0;
			//Data_In.word.wPanelAct[1].word.Active_Inactive_S3[1] = 0;
			//Data_In.word.wPanelAct[0].word.Active_Inactive_S1[2] = 0;

			//Data_In.word.wPanelAct[1].word.Active_Inactive_S4[0] = 0;
			//Data_In.word.wPanelAct[1].word.Active_Inactive_S4[1] = 0;
			//Data_In.word.wPanelAct[1].word.Active_Inactive_S2[2] = 0;

			/*Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[0] = 0;
			Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[1] = 0;
			Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[2] = 0;

			Data_In.word.wPanelCurDemandAct[1].word.Active_Inactive_S2[0] = 0;
			Data_In.word.wPanelCurDemandAct[1].word.Active_Inactive_S2[1] = 0;
			Data_In.word.wPanelCurDemandAct[1].word.Active_Inactive_S2[2] = 0;
			
			Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[0] = 0;
			Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[1] = 0;
			Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[2] = 0;

			Data_In.word.wPanelKwDemandAct[1].word.Active_Inactive_S2[0] = 0;
			Data_In.word.wPanelKwDemandAct[1].word.Active_Inactive_S2[1] = 0;
			Data_In.word.wPanelKwDemandAct[1].word.Active_Inactive_S2[2] = 0;*/
	    break;
	    }
		//ptCTCurDemand = &strCTCurDemand[1].array[0];
		//ptCTKwDemand = &strCTKwDemand[1].array[0];

		Data.word.Panel2_Status_Undercurr[0].paraID = 0x1B;
		Data.word.Panel2_Status_Undercurr[1].paraID = 0x5B;
		Data.word.Panel2_Status_Undercurr[2].paraID = 0x9B;
		Data.word.Panel2_Status_Undercurr[3].paraID = 0xDB;

		Data.word.Panel2_Status_Overcurr[0].paraID = 0x2B;
		Data.word.Panel2_Status_Overcurr[1].paraID = 0x6B;
		Data.word.Panel2_Status_Overcurr[2].paraID = 0xAB;
		Data.word.Panel2_Status_Overcurr[3].paraID = 0xEB;

		//printf("\nData : %x\n",Data.array[PANEL24_UNDERCURR_STATUS_FLAG]);
		//printf("\nData : %x\n",Data.array[PANEL21_OVERCURR_STATUS_FLAG]);

		/*printf("\nAddress of Panel2_Status_Overcurr[1] : %x\n",&Data.word.Panel2_Status_Overcurr[0]);
		printf("\nAddress of Panel2_Status_Overcurr[1] : %x\n",&Data.word.Panel2_Status_Overcurr[1]);*/

	    for (i=0; i<21; i++)
		{
				
		  alarm = 0x00000001 << i;
		  //****************************UNDER CURRENT Panel 1 Strip 1  (1~21)**************************//
		  if(Data_In.word.wPanelAct[1].word.Active_Inactive[0] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][i])
			Data.array[PANEL14_OVERCURR_STATUS_FLAG] |= alarm;
		    else {
			if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][i] + BRANCH_UC_HYSTER))
			 Data.array[PANEL14_OVERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		 else
		  Data.array[PANEL14_OVERCURR_STATUS_FLAG] &= (~alarm);
		  //****************************UNDER CURRENT Panel 1 Strip 2  (21~42)**************************//
		  if(Data_In.word.wPanelAct[1].word.Active_Inactive[1] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 21+ i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][21+i])
			Data.array[PANEL21_UNDERCURR_STATUS_FLAG] |= alarm;
		    else {
			if((Data.array[SEC_OFFSET + 21+ i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][21+i] + BRANCH_UC_HYSTER))
			 Data.array[PANEL21_UNDERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		 else
		  Data.array[PANEL21_UNDERCURR_STATUS_FLAG] &= (~alarm);
		 	
		 //****************************UNDER CURRENT DEMAND Panel 1 Strip 1  (1~16)**************************// 
		 /* if(Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[0] & alarm)
		  {
			if(*(ptCTCurDemand + i) < Data_In.word.wUnderCurDemandLimit[0][i])
			  sCurDemandStatusP1S1.array[0] |= alarm;
			else
			{
			  if(*(ptCTCurDemand + i) > (Data_In.word.wUnderCurDemandLimit[0][i] + BRANCH_UCD_HYSTER))
			   sCurDemandStatusP1S1.array[0] &= (~alarm);
			}
			
		  }
		  else
			sCurDemandStatusP1S1.array[0] &= (~alarm);
                  */
		  //****************************UNDER CURRENT DEMAND Panel 1 Strip 2  (1~16)**************************// 
		  /*if(Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S2[0] & alarm)
		  {
			if(*(ptCTCurDemand + 21+i) < Data_In.word.wUnderCurDemandLimit[0][21+i])
			  sCurDemandStatusP1S2.array[0] |= alarm;
			else
			{
			  if(*(ptCTCurDemand + 21+i) > (Data_In.word.wUnderCurDemandLimit[0][21+i] + BRANCH_UCD_HYSTER))
			   sCurDemandStatusP1S2.array[0] &= (~alarm);
			}
			
		  }
		  else
			sCurDemandStatusP1S2.array[0] &= (~alarm);
		  
		  */	
		  //****************************UNDER KW DEMAND Panel 1 Strip 1  (1~16)**************************// 
		  /*if(Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[0] & alarm)
		  {
			if(*(ptCTKwDemand + i) < Data_In.word.wUnderKwDemandLimit[0][i])
			  sKwDemandStatusP1S1.array[0] |= alarm;
			else
			{
			  if(*(ptCTKwDemand + i) > (Data_In.word.wUnderKwDemandLimit[0][i] + BRANCH_UKWD_HYSTER))
			   sKwDemandStatusP1S1.array[0] &= (~alarm);
			}
			
		  }
		  else
			sKwDemandStatusP1S1.array[0] &= (~alarm);
                  */
		  //****************************UNDER KW DEMAND Panel 1 Strip 2  (1~16)**************************// 
		  /*if(Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[0] & alarm)
		  {
			if(*(ptCTKwDemand + 21+i) < Data_In.word.wUnderKwDemandLimit[0][21+i])
			  sKwDemandStatusP1S2.array[0] |= alarm;
			else
			{
			  if(*(ptCTKwDemand + 21+i) > (Data_In.word.wUnderKwDemandLimit[0][21+i] + BRANCH_UKWD_HYSTER))
			   sKwDemandStatusP1S2.array[0] &= (~alarm);
			}
			
		  }
		  else
			sKwDemandStatusP1S2.array[0] &= (~alarm);*/
		}
                 
		for (i=0; i<21; i++)
		{
		  alarm = 0x00000001 << i;
		  //********************************UNDER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[1].word.Active_Inactive[2] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][42+i])
			Data.array[PANEL22_UNDERCURR_STATUS_FLAG] |= alarm;
		    else{
			if ((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][42+i] + BRANCH_UC_HYSTER))
			  Data.array[PANEL22_UNDERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		  else
		    Data.array[PANEL22_UNDERCURR_STATUS_FLAG] &= (~alarm);  
		  //********************************UNDER CURRENT Panel 1 Strip 4(63~84)******************************//
		  if (Data_In.word.wPanelAct[1].word.Active_Inactive[3] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) < Data_In.word.wUnderCurrentLimit[0][63+i])
			Data.array[PANEL23_UNDERCURR_STATUS_FLAG] |= alarm;
		    else{
			if ((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) > (Data_In.word.wUnderCurrentLimit[0][63+i] + BRANCH_UC_HYSTER))
			  Data.array[PANEL23_UNDERCURR_STATUS_FLAG] &= (~alarm);
			}
		  }
		  else
		    Data.array[PANEL23_UNDERCURR_STATUS_FLAG] &= (~alarm);
		  
		  //********************************UNDER CURRENT DEMAND Panel 1 Strip 1(17~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[1] & alarm)
		  {
		    if(*(ptCTCurDemand +16 +i) < Data_In.word.wUnderCurDemandLimit[0][16+i])
			sCurDemandStatusP1S1.array[1] |= alarm;
		    else{
			if (*(ptCTCurDemand +16 +i) > (Data_In.word.wUnderCurDemandLimit[0][16+i] + BRANCH_UCD_HYSTER))
			  sCurDemandStatusP1S1.array[1] &= (~alarm);
			}
		  }
		  else
		    sCurDemandStatusP1S1.array[1] &= (~alarm);  
		*/
		  //********************************UNDER CURRENT DEMAND Panel 1 Strip 2(17~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S2[1] & alarm)
		  {
		    if(*(ptCTCurDemand+21 +16 +i) < Data_In.word.wUnderCurDemandLimit[0][21+16+i])
			sCurDemandStatusP1S2.array[1] |= alarm;
		    else{
			if (*(ptCTCurDemand+21 +16 +i) > (Data_In.word.wUnderCurDemandLimit[0][21+16+i] + BRANCH_UCD_HYSTER))
			  sCurDemandStatusP1S2.array[1] &= (~alarm);
			}
		  }
		  else
		    sCurDemandStatusP1S2.array[1] &= (~alarm); 
	           */
		  //********************************UNDER KW DEMAND Panel 1 Strip 1(17~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[1] & alarm)
		  {
		    if(*(ptCTKwDemand +16 +i) < Data_In.word.wUnderKwDemandLimit[0][16+i])
			sKwDemandStatusP1S1.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand +16 +i) > (Data_In.word.wUnderKwDemandLimit[0][16+i] + BRANCH_UKWD_HYSTER))
			  sKwDemandStatusP1S1.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S1.array[1] &= (~alarm);  
		*/
		  //********************************UNDER KW DEMAND Panel 1 Strip 2(17~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[1] & alarm)
		  {
		    if(*(ptCTKwDemand+21 +16 +i) < Data_In.word.wUnderKwDemandLimit[0][21+16+i])
			sKwDemandStatusP1S2.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand+21 +16 +i) > (Data_In.word.wUnderKwDemandLimit[0][21+16+i] + BRANCH_UKWD_HYSTER))
			  sKwDemandStatusP1S2.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S2.array[1] &= (~alarm);*/
		}
		
		
		
		for (i=0; i<21; i++)
		{
		  alarm = 0x00000001 << i;
	          //********************************OVER CURRENT Panel 1 Strip 1(1~21)*****************************//
		  if(Data_In.word.wPanelAct[1].word.Active_Inactive[0] & alarm)
		  {
		    //printf("\nPanel21_Overcurrent_alarm\n");
		    if ((Data.array[SEC_OFFSET + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][i]))
			Data.array[PANEL24_UNDERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
		       if((Data.array[SEC_OFFSET + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][i] - BRANCH_OC_HYSTER))
		       Data.array[PANEL24_UNDERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		     Data.array[PANEL24_UNDERCURR_STATUS_FLAG] &= (~alarm);

		  //********************************OVER CURRENT Panel 1 Strip 2(21~42)*****************************//
		  if(Data_In.word.wPanelAct[1].word.Active_Inactive[1] & alarm)
		  {
		    //printf("\nPanel24_Overcurrent_alarm\n");
		    if ((Data.array[SEC_OFFSET + 21 + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][21+i]))
			Data.array[PANEL21_OVERCURR_STATUS_FLAG] |= (~alarm);
		    else
		    {
		       if((Data.array[SEC_OFFSET + 21 + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][21+i] - BRANCH_OC_HYSTER))
		       Data.array[PANEL21_OVERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		    Data.array[PANEL21_OVERCURR_STATUS_FLAG] &= (~alarm);

		  
		  
		  //********************************OVER KW DEMAND Panel 1 Strip 1(1~11)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[1] & alarm)
		  {
		    if(*(ptCTKwDemand +i) > Data_In.word.wOverKwDemandLimit[0][i])
			sKwDemandStatusP1S1.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand +i) < (Data_In.word.wOverKwDemandLimit[0][i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S1.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S1.array[1] &= (~alarm);  
			*/
		  //********************************OVER KW DEMAND Panel 1 Strip 2(1~11)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[1] & alarm)
		  {
		    if(*(ptCTKwDemand+21+i) > Data_In.word.wOverKwDemandLimit[0][21+i])
			sKwDemandStatusP1S2.array[1] |= alarm;
		    else{
			if (*(ptCTKwDemand +21+i) < (Data_In.word.wOverKwDemandLimit[0][21+i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S2.array[1] &= (~alarm);
			}
		  }
		  else
		    sKwDemandStatusP1S2.array[1] &= (~alarm);*/
                }
     		
		for (i=0;i<21;i++)
		{
		  alarm = 0x00000001 << i;
		  //********************************OVER CURRENT Panel 1 Strip 3(42~63)*****************************//
		  if (Data_In.word.wPanelAct[1].word.Active_Inactive[2] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][42+i]))
			Data.array[PANEL22_OVERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
			if((Data.array[SEC_OFFSET + 42 + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][42+i] - BRANCH_OC_HYSTER))
			  Data.array[PANEL22_OVERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		    Data.array[PANEL22_OVERCURR_STATUS_FLAG] &= (~alarm);
		
		  //********************************OVER CURRENT Panel 1 Strip 2(12~21)*****************************//
		  if (Data_In.word.wPanelAct[1].word.Active_Inactive[3] & alarm)
		  {
		    if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) > (Data_In.word.wOverCurrentLimit[0][63+i]))
			Data.array[PANEL23_OVERCURR_STATUS_FLAG] |= alarm;
		    else
		    {
			if((Data.array[SEC_OFFSET + 63 + i] & 0x0000FFFF) < (Data_In.word.wOverCurrentLimit[0][63+i] - BRANCH_OC_HYSTER))
			  Data.array[PANEL23_OVERCURR_STATUS_FLAG] &= (~alarm);
		    }
		  }
		  else
		    Data.array[PANEL23_OVERCURR_STATUS_FLAG] &= (~alarm);
		  
		  
		  //********************************OVER CURRENT DEMAND Panel 1 Strip 1(12~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S1[2] & alarm)
		  {
		    if(*(ptCTCurDemand +11+i) > (Data_In.word.wOverCurDemandLimit[0][11+i]))
			sCurDemandStatusP1S1.array[2] |= alarm;
		    else
		    {
			if(*(ptCTCurDemand+11+i) < (Data_In.word.wOverCurDemandLimit[0][11+i] - BRANCH_OCD_HYSTER))
			  sCurDemandStatusP1S1.array[2] &= (~alarm);
		    }
		  }
		  else
		    sCurDemandStatusP1S1.array[2] &= (~alarm);
			*/		
		  //********************************OVER CURRENT DEMAND Panel 1 Strip 2(12~21)*****************************//
		  /*if (Data_In.word.wPanelCurDemandAct[0].word.Active_Inactive_S2[2] & alarm)
		  {
		    if(*(ptCTCurDemand +21+11+i) > (Data_In.word.wOverCurDemandLimit[0][21+11+i]))
			sCurDemandStatusP1S2.array[2] |= alarm;
		    else
		    {
			if(*(ptCTCurDemand+21+11+i) < (Data_In.word.wOverCurDemandLimit[0][21+11+i] - BRANCH_OCD_HYSTER))
			  sCurDemandStatusP1S2.array[2] &= (~alarm);
		    }
		  }
		  else
		    sCurDemandStatusP1S2.array[2] &= (~alarm);
	*/	  
		  //********************************OVER KW DEMAND Panel 1 Strip 1(12~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S1[2] & alarm)
		  {
		    if(*(ptCTKwDemand +11+i) > (Data_In.word.wOverKwDemandLimit[0][11+i]))
			sKwDemandStatusP1S1.array[2] |= alarm;
		    else
		    {
			if(*(ptCTKwDemand+11+i) < (Data_In.word.wOverKwDemandLimit[0][11+i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S1.array[2] &= (~alarm);
		    }
		  }
		  else
		    sKwDemandStatusP1S1.array[2] &= (~alarm);
		*/	
		  //********************************OVER KW DEMAND Panel 1 Strip 2(12~21)*****************************//
		  /*if (Data_In.word.wPanelKwDemandAct[0].word.Active_Inactive_S2[2] & alarm)
		  {
		    if(*(ptCTKwDemand +21+11+i) > (Data_In.word.wOverKwDemandLimit[0][21+11+i]))
			sKwDemandStatusP1S2.array[2] |= alarm;
		    else
		    {
			if(*(ptCTKwDemand+21+11+i) < (Data_In.word.wOverKwDemandLimit[0][21+11+i] - BRANCH_OKWD_HYSTER))
			  sKwDemandStatusP1S2.array[2] &= (~alarm);
		    }
		  }
		  else
		    sKwDemandStatusP1S2.array[2] &= (~alarm);*/
		  }
		  /*printf("\nPanel21_UnderCurr : %x\n",Data.array[PANEL14_OVERCURR_STATUS_FLAG]);
		  printf("\nPanel22_UnderCurr : %x\n",Data.array[PANEL21_UNDERCURR_STATUS_FLAG]);
		  printf("\nPanel23_UnderCurr : %x\n",Data.array[PANEL22_UNDERCURR_STATUS_FLAG]);
		  printf("\nPanel24_UnderCurr : %x\n",Data.array[PANEL23_UNDERCURR_STATUS_FLAG]);*/
		  /*printf("\nPanel21_OverCurr : %x\n",Data.array[PANEL24_UNDERCURR_STATUS_FLAG]);
		  printf("\nPanel22_OverCurr : %x\n",Data.array[PANEL21_OVERCURR_STATUS_FLAG]);	
		  printf("\nPanel23_OverCurr : %x\n",Data.array[PANEL22_OVERCURR_STATUS_FLAG]);
		  printf("\nPanel24_OverCurr : %x\n",Data.array[PANEL23_OVERCURR_STATUS_FLAG]);*/
	       
	       break;  
	   // }
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
		  Data.word.Max_Min_Para[0].Max_Volt_PhaseA = ((unsigned long)(0xA11 << 16))|((Data.word.Primary.L2N_Volt_Phase_A&0xFFFF));
		}
		if (((Data.word.Max_Min_Para[0].Max_Volt_PhaseB&0xFFFF) < (Data.word.Primary.L2N_Volt_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Volt_PhaseB&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Volt_PhaseB = ((unsigned long)(0xA00 << 16))|((Data.word.Primary.L2N_Volt_Phase_B&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Max_Volt_PhaseC&0xFFFF) < (Data.word.Primary.L2N_Volt_Phase_C&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Volt_PhaseC&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Volt_PhaseC = ((unsigned long)(0xA01 << 16))|((Data.word.Primary.L2N_Volt_Phase_C&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Min_Volt_PhaseA&0xFFFF) > (Data.word.Primary.L2N_Volt_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Volt_PhaseA&0xFFFF) == 0))
		  Data.word.Max_Min_Para[0].Min_Volt_PhaseA = ((unsigned long)(0xA02 << 16))|((Data.word.Primary.L2N_Volt_Phase_A&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Min_Volt_PhaseB&0xFFFF) > (Data.word.Primary.L2N_Volt_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Volt_PhaseB&0xFFFF) == 0))
		  Data.word.Max_Min_Para[0].Min_Volt_PhaseB = ((unsigned long)(0xA03 << 16))|((Data.word.Primary.L2N_Volt_Phase_B&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Min_Volt_PhaseC&0xFFFF) > (Data.word.Primary.L2N_Volt_Phase_C&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Volt_PhaseC&0xFFFF) == 0))
		 Data.word.Max_Min_Para[0].Min_Volt_PhaseC = ((unsigned long)(0xA04 << 16))|((Data.word.Primary.L2N_Volt_Phase_C&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Max_Curr_PhaseA&0xFFFF) < (Data.word.Primary.RMS_Curr_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_PhaseA&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Curr_PhaseA = ((unsigned long)(0xA05 << 16))|((Data.word.Primary.RMS_Curr_Phase_A&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Max_Curr_PhaseB&0xFFFF) < (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_PhaseB&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Curr_PhaseB = ((unsigned long)(0xA06 << 16))|((Data.word.Primary.RMS_Curr_Phase_B&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Max_Curr_PhaseC&0xFFFF) < (Data.word.Primary.RMS_Curr_Phase_C&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_PhaseC&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Curr_PhaseC = ((unsigned long)(0xA07 << 16))|((Data.word.Primary.RMS_Curr_Phase_C&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Min_Curr_PhaseA&0xFFFF) > (Data.word.Primary.RMS_Curr_Phase_A&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_PhaseA&0xFFFF) == 0))
		 Data.word.Max_Min_Para[0].Min_Curr_PhaseA = ((unsigned long)(0xA09 << 16))|((Data.word.Primary.RMS_Curr_Phase_A&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Min_Curr_PhaseB&0xFFFF) > (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_PhaseB&0xFFFF) == 0))
		  Data.word.Max_Min_Para[0].Min_Curr_PhaseB = ((unsigned long)(0xA0A << 16))|((Data.word.Primary.RMS_Curr_Phase_B&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Min_Curr_PhaseC&0xFFFF) > (Data.word.Primary.RMS_Curr_Phase_B&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_PhaseC&0xFFFF) == 0))
		 Data.word.Max_Min_Para[0].Min_Curr_PhaseC = ((unsigned long)(0xA0B << 16))|((Data.word.Primary.RMS_Curr_Phase_B&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Max_Curr_Neutral&0xFFFF) < (Data.word.Primary.RMS_Curr_Neutral&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Curr_Neutral&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Curr_Neutral = ((unsigned long)(0xA08 << 16))|((Data.word.Primary.RMS_Curr_Neutral&0xFFFF)); 

		if (((Data.word.Max_Min_Para[0].Min_Curr_Neutral&0xFFFF) > (Data.word.Primary.RMS_Curr_Neutral&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Curr_Neutral&0xFFFF) == 0))
		  Data.word.Max_Min_Para[0].Min_Curr_Neutral = ((unsigned long)(0xA0C << 16))|((Data.word.Primary.RMS_Curr_Neutral&0xFFFF));

		if (((Data.word.Max_Min_Para[0].Max_Freq_PhaseA&0xFFFF) < (Data.word.Secondary.Frequency&0xFFFF)) || ((Data.word.Max_Min_Para[0].Max_Freq_PhaseA&0xFFFF) == 0xFFFF))
		  Data.word.Max_Min_Para[0].Max_Freq_PhaseA = ((unsigned long)(0xA0D << 16))|((Data.word.Secondary.Frequency&0xFFFF)); 

		if (((Data.word.Max_Min_Para[0].Min_Freq_PhaseA&0xFFFF) > (Data.word.Secondary.Frequency&0xFFFF)) || ((Data.word.Max_Min_Para[0].Min_Freq_PhaseA&0xFFFF) == 0))
		  Data.word.Max_Min_Para[0].Min_Freq_PhaseA = ((unsigned long)(0xA10 << 16))|((Data.word.Secondary.Frequency&0xFFFF));		
	    break;
	    }
	    case SEC1:
		break;
	    case SEC2:
		break;
	    case SEC3:
		break;
	    /*case SEC:
	      {
		if ((Data.word.Max_Min_Para[1].Max_Volt_PhaseA < Data.word.Secondary.L2N_Volt_Phase_A) || (Data.word.Max_Min_Para[1].Max_Volt_PhaseA == 0xFFFF))
		  Data.word.Max_Min_Para[1].Max_Volt_PhaseA = Data.word.Secondary.L2N_Volt_Phase_A;

		if ((Data.word.Max_Min_Para[1].Max_Volt_PhaseB < Data.word.Secondary.L2N_Volt_Phase_B) || (Data.word.Max_Min_Para[1].Max_Volt_PhaseB == 0xFFFF))
		 Data.word.Max_Min_Para[1].Max_Volt_PhaseB = Data.word.Secondary.L2N_Volt_Phase_B;

		if ((Data.word.Max_Min_Para[1].Max_Volt_PhaseC < Data.word.Secondary.L2N_Volt_Phase_C) || (Data.word.Max_Min_Para[1].Max_Volt_PhaseC == 0xFFFF))
		 Data.word.Max_Min_Para[1].Max_Volt_PhaseC = Data.word.Secondary.L2N_Volt_Phase_C;

		if ((Data.word.Max_Min_Para[1].Min_Volt_PhaseA > Data.word.Secondary.L2N_Volt_Phase_A) || (Data.word.Max_Min_Para[1].Min_Volt_PhaseA == 0))
		  Data.word.Max_Min_Para[1].Min_Volt_PhaseA = Data.word.Secondary.L2N_Volt_Phase_A;

		if ((Data.word.Max_Min_Para[1].Min_Volt_PhaseB > Data.word.Secondary.L2N_Volt_Phase_B) || (Data.word.Max_Min_Para[1].Min_Volt_PhaseB == 0))
		  Data.word.Max_Min_Para[1].Min_Volt_PhaseB = Data.word.Secondary.L2N_Volt_Phase_B;

		if ((Data.word.Max_Min_Para[1].Min_Volt_PhaseC > Data.word.Secondary.L2N_Volt_Phase_C) || (Data.word.Max_Min_Para[1].Min_Volt_PhaseC == 0))
		  Data.word.Max_Min_Para[1].Min_Volt_PhaseC = Data.word.Secondary.L2N_Volt_Phase_C;

		if ((Data.word.Max_Min_Para[1].Max_Curr_PhaseA < Data.word.Secondary.RMS_Curr_Phase_A) || (Data.word.Max_Min_Para[1].Max_Curr_PhaseA == 0xFFFF))
		  Data.word.Max_Min_Para[1].Max_Curr_PhaseA = Data.word.Secondary.RMS_Curr_Phase_A;

		if ((Data.word.Max_Min_Para[1].Max_Curr_PhaseB < Data.word.Secondary.RMS_Curr_Phase_B) || (Data.word.Max_Min_Para[1].Max_Curr_PhaseB == 0xFFFF))
		  Data.word.Max_Min_Para[1].Max_Curr_PhaseB = Data.word.Secondary.RMS_Curr_Phase_B;

		if ((Data.word.Max_Min_Para[1].Max_Curr_PhaseC < Data.word.Secondary.RMS_Curr_Phase_C) || (Data.word.Max_Min_Para[1].Max_Curr_PhaseC == 0xFFFF))
		  Data.word.Max_Min_Para[1].Max_Curr_PhaseC = Data.word.Secondary.RMS_Curr_Phase_C;

		if ((Data.word.Max_Min_Para[1].Min_Curr_PhaseA > Data.word.Secondary.RMS_Curr_Phase_A) || (Data.word.Max_Min_Para[1].Min_Curr_PhaseA == 0))
		  Data.word.Max_Min_Para[1].Min_Curr_PhaseA = Data.word.Secondary.RMS_Curr_Phase_A;

		if ((Data.word.Max_Min_Para[1].Min_Curr_PhaseB > Data.word.Secondary.RMS_Curr_Phase_B) || (Data.word.Max_Min_Para[1].Min_Curr_PhaseB == 0))
		  Data.word.Max_Min_Para[1].Min_Curr_PhaseB = Data.word.Secondary.RMS_Curr_Phase_B;

		if ((Data.word.Max_Min_Para[1].Min_Curr_PhaseC > Data.word.Secondary.RMS_Curr_Phase_B) || (Data.word.Max_Min_Para[1].Min_Curr_PhaseC == 0))
		  Data.word.Max_Min_Para[1].Min_Curr_PhaseC = Data.word.Secondary.RMS_Curr_Phase_B;

		if ((Data.word.Max_Min_Para[1].Max_Curr_Neutral < Data.word.Secondary.RMS_Curr_Neutral) || (Data.word.Max_Min_Para[1].Max_Curr_Neutral == 0xFFFF))
		  Data.word.Max_Min_Para[1].Max_Curr_Neutral = Data.word.Secondary.RMS_Curr_Neutral; 

		if ((Data.word.Max_Min_Para[1].Min_Curr_Neutral > Data.word.Secondary.RMS_Curr_Neutral) || (Data.word.Max_Min_Para[1].Min_Curr_Neutral == 0))
		  Data.word.Max_Min_Para[1].Min_Curr_Neutral = Data.word.Secondary.RMS_Curr_Neutral;

		if ((Data.word.Max_Min_Para[1].Max_Freq_PhaseA < Data.word.Secondary.Frequency) || (Data.word.Max_Min_Para[1].Max_Freq_PhaseA == 0xFFFF))
		 Data.word.Max_Min_Para[1].Max_Freq_PhaseA = Data.word.Secondary.Frequency; 

		if ((Data.word.Max_Min_Para[1].Min_Freq_PhaseA > Data.word.Secondary.Frequency) || (Data.word.Max_Min_Para[1].Min_Freq_PhaseA == 0))
		  Data.word.Max_Min_Para[1].Min_Freq_PhaseA = Data.word.Secondary.Frequency;
	      break;
 	      } */
	    case PANEL1:
	      {
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[MAX_MIN_SECONDARY];//&strCTMaxCurDemand[0].array[0];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_3];//&strCTMaxKWDemand[0].array[0];
		  ptMinCurrent = (unsigned int *)&Data.array[MAXIMUM_3_OFFSET];//&MinCurrent[0].array[0];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_3];//&MaxCurrent[0].array[0];
		  ptCTCurDemand	= (unsigned int *)&Data.array[MINIMUM_3_OFFSET];//&strCTCurDemand[0].array[0];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_CURR_DEMAND_3];//&strCTKwDemand[0].array[0];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_KW_DEMAND_3];//&strCTMaxCurDemand24hr[0].array[0];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_3];//&strCTMaxKWDemand24hr[0].array[0];
		  for (i=0; i < 84; i++)
		  {
		             
		    if ((((*(ptMaxCurrent + i)&0x0000FFFF) < (Data.array[SEC_OFFSET + i]&0x0000FFFF)) || ((*(ptMaxCurrent + i)&0x0000FFFF) == 0xFFFF)) && ((Data.array[SEC_OFFSET + i]&0x0000FFFF) != 0xFFFF))
				{
				*(ptMaxCurrent + i) = ((unsigned long)((0x1800|i)|(0<<14))<<16)|(Data.array[SEC_OFFSET + i]&0x0000FFFF);
				}

			if ((((*(ptMinCurrent + i)&0x0000FFFF) > (Data.array[SEC_OFFSET + i]&0x0000FFFF)) || ((*(ptMinCurrent + i)&0x0000FFFF) == 0)) && ((Data.array[SEC_OFFSET + i]&0x0000FFFF) != 0x0000))
				{
				  *(ptMinCurrent + i) = ((unsigned long)((0x0E00|i)|(0<<14)<<16))|((Data.array[SEC_OFFSET + i]&0x0000FFFF));
				  }

			if ((((*(ptCTMaxCurDemand + i)&0x0000FFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxCurDemand + i) = ((unsigned long)((0x1600|i)|(0<<14)<<16))|((*(ptCTCurDemand + i)&0xFFFF));

			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxKWDemand + i) = ((unsigned long)((0x7000|i)|(0<<14)<<16))|((*(ptCTKwDemand + i)&0xFFFF));
		  }

		  if (P1_TEST_CAPTURE_MAX_DEMAND)
		  {
			for (i=0; i < 84; i++)
			 {
				(*(ptCTMaxCurDemand24hr +i)) = (*(ptCTMaxCurDemand + i)&0xFFFF);
				(*(ptCTMaxKWDemand24hr + i)) = (*(ptCTMaxKWDemand + i)&0xFFFF);

				(*(ptCTMaxKWDemand + i)) = 0;
				(*(ptCTMaxCurDemand + i)) = 0;
			 }
		  	P1_CAPTURE_MAX_DEMAND_DONE;
		  }	
	      break;
	      }
	    case PANEL2:
	      {
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_0];//&strCTMaxCurDemand[0].array[0];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_0];//&strCTMaxKWDemand[0].array[0];
		  ptMinCurrent = (unsigned int *)&Data.array[MAXIMUM_0_OFFSET];//&MinCurrent[0].array[0];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[MINIMUM_0_OFFSET];//&MaxCurrent[0].array[0];
		  ptCTCurDemand	= (unsigned int *)&Data.array[CT_CURR_DEMAND_0];//&strCTCurDemand[0].array[0];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_0];//&strCTKwDemand[0].array[0];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_0];//&strCTMaxCurDemand24hr[0].array[0];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_24HR_0];//&strCTMaxKWDemand24hr[0].array[0];
		
		  for (i=0; i < 84; i++)
		  {
		 	if ((((*(ptMaxCurrent + i)&0xFFFF) < (Data.array[RMS_0_OFFSET + i]&0xFFFF)) || ((*(ptMaxCurrent + i)&0xFFFF) == 0xFFFF)) && ((Data.array[RMS_0_OFFSET + i]&0xFFFF) != 0xFFFF))
				*(ptMaxCurrent + i) = ((unsigned long)((0x1800|i)|(1<<14)<<16))|((Data.array[RMS_0_OFFSET + i]&0xFFFF));

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[RMS_0_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[RMS_0_OFFSET + i]&0xFFFF) != 0x0000))
				*(ptMinCurrent + i) = ((unsigned long)((0x0E00|i)|(1<<14)<<16))|((Data.array[RMS_0_OFFSET + i]&0xFFFF));

			if ((((*(ptCTMaxCurDemand + i)&0xFFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxCurDemand + i) = ((unsigned long)((0x1600|i)|(1<<14)<<16))|((*(ptCTCurDemand + i)&0xFFFF));

			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxKWDemand + i) = ((unsigned long)((0x7000|i)|(1<<14)<<16))|((*(ptCTKwDemand + i)&0xFFFF));
		  }

		  if (P2_TEST_CAPTURE_MAX_DEMAND)
		  {
			for (i=0; i < 84; i++)
			 {
				(*(ptCTMaxCurDemand24hr +i)) = (*(ptCTMaxCurDemand + i)&0xFFFF);
				(*(ptCTMaxKWDemand24hr + i)) = (*(ptCTMaxKWDemand + i)&0xFFFF);

				(*(ptCTMaxKWDemand + i)) = 0;
				(*(ptCTMaxCurDemand + i)) = 0;
			 }
		  	P2_CAPTURE_MAX_DEMAND_DONE;
		  }	
	      break;
	      }
	    case PANEL3:
	      {
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_1];//&strCTMaxCurDemand[0].array[0];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_1];//&strCTMaxKWDemand[0].array[0];
		  ptMinCurrent = (unsigned int *)&Data.array[MAXIMUM_1_OFFSET];//&MinCurrent[0].array[0];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[MINIMUM_1_OFFSET];//&MaxCurrent[0].array[0];
		  ptCTCurDemand	= (unsigned int *)&Data.array[CT_CURR_DEMAND_1];//&strCTCurDemand[0].array[0];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_1];//&strCTKwDemand[0].array[0];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_1];//&strCTMaxCurDemand24hr[0].array[0];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_24HR_1];//&strCTMaxKWDemand24hr[0].array[0];
		
		  for (i=0; i < 84; i++)
		  {
		 	if ((((*(ptMaxCurrent + i)&0xFFFF) < (Data.array[RMS_1_OFFSET + i]&0xFFFF)) || ((*(ptMaxCurrent + i)&0xFFFF) == 0xFFFF)) && ((Data.array[RMS_1_OFFSET + i]&0xFFFF) != 0xFFFF))
				*(ptMaxCurrent + i) = ((unsigned long)((0x1800|i)|(2<<14)<<16))|((Data.array[RMS_1_OFFSET + i]&0xFFFF));

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[RMS_1_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[RMS_1_OFFSET + i]&0xFFFF) != 0x0000))
				*(ptMinCurrent + i) = ((unsigned long)((0x0E00|i)|(2<<14)<<16))|((Data.array[RMS_1_OFFSET + i]&0xFFFF));

			if ((((*(ptCTMaxCurDemand + i)&0xFFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxCurDemand + i) = ((unsigned long)((0x1600|i)|(2<<14)<<16))|((*(ptCTCurDemand + i)&0xFFFF));

			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxKWDemand + i) = ((unsigned long)((0x8000|i)|(2<<14)<<16))|((*(ptCTKwDemand + i)&0xFFFF));
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
		  ptCTMaxCurDemand = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_2];//&strCTMaxCurDemand[0].array[0];
		  ptCTMaxKWDemand  = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_2];//&strCTMaxKWDemand[0].array[0];
		  ptMinCurrent = (unsigned int *)&Data.array[MAXIMUM_2_OFFSET];//&MinCurrent[0].array[0];
	 	  ptMaxCurrent = (unsigned int *)&Data.array[MINIMUM_2_OFFSET];//&MaxCurrent[0].array[0];
		  ptCTCurDemand	=(unsigned int *)&Data.array[CT_CURR_DEMAND_2];//&strCTCurDemand[0].array[0];
		  ptCTKwDemand = (unsigned int *)&Data.array[CT_KW_DEMAND_2];//&strCTKwDemand[0].array[0];
		  ptCTMaxCurDemand24hr = (unsigned int *)&Data.array[CT_MAX_CURR_DEMAND_24HR_2];//&strCTMaxCurDemand24hr[0].array[0];
		  ptCTMaxKWDemand24hr = (unsigned int *)&Data.array[CT_MAX_KW_DEMAND_24HR_2];//&strCTMaxKWDemand24hr[0].array[0];
		
		  for (i=0; i < 84; i++)
		  {
		 	if ((((*(ptMaxCurrent + i)&0xFFFF) < (Data.array[RMS_2_OFFSET + i]&0xFFFF)) || ((*(ptMaxCurrent + i)&0xFFFF) == 0xFFFF)) && ((Data.array[RMS_2_OFFSET + i]&0xFFFF) != 0xFFFF))
				*(ptMaxCurrent + i) = ((unsigned long)((0x1800|i)|(3<<14)<<16))|((Data.array[RMS_2_OFFSET + i]&0xFFFF));

			if ((((*(ptMinCurrent + i)&0xFFFF) > (Data.array[RMS_2_OFFSET + i]&0xFFFF)) || ((*(ptMinCurrent + i)&0xFFFF) == 0)) && ((Data.array[RMS_2_OFFSET + i]&0xFFFF) != 0x0000))
				*(ptMinCurrent + i) = ((unsigned long)((0x0E00|i)|(3<<14)<<16))|((Data.array[RMS_2_OFFSET + i]&0xFFFF));

			if ((((*(ptCTMaxCurDemand + i)&0xFFFF) < (*(ptCTCurDemand + i)&0xFFFF)) || ((*(ptCTMaxCurDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTCurDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxCurDemand + i) = ((unsigned long)((0x1600|i)|(3<<14)<<16))|((*(ptCTCurDemand + i)&0xFFFF));

			if ((((*(ptCTMaxKWDemand + i)&0xFFFF) < (*(ptCTKwDemand + i)&0xFFFF)) || ((*(ptCTMaxKWDemand + i)&0xFFFF) == 0xFFFF)) && ((*(ptCTKwDemand + i)&0xFFFF) != 0xFFFF))
				*(ptCTMaxKWDemand + i) = ((unsigned long)((0x8000|i)|(3<<14)<<16))|((*(ptCTKwDemand + i)&0xFFFF));
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
			(*(ptCurrDemandSum + i))+=Data.array[SEC_OFFSET + i];
			(*(ptKwDemandSum + i))+= Data.array[RMS_3_OFFSET + i];
		}
		sFlag.pnl1_demand_chk_1sec = 0;
		wDemandSumCntr[0]++;
	      }

           if (sFlag.pnl1_demand_chk_1hr)
	     { 
		for (i=0; i<84; i++)
		{
			*(ptCTCurDemand + i) = (*(ptCurrDemandSum + i) / wDemandSumCntr[0]);
			*(ptCTKwDemand + i) = (*(ptKwDemandSum + i) / wDemandSumCntr[0]);

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
			(*(ptCurrDemandSum + i))+=Data.array[RMS_0_OFFSET + i];
			(*(ptKwDemandSum + i))+= Data.array[KW_0_OFFSET + i];
		}
		sFlag.pnl2_demand_chk_1sec = 0;
		wDemandSumCntr[1]++;
	      }

           if (sFlag.pnl2_demand_chk_1hr)
	     { 
		for (i=0; i<84; i++)
		{
			*(ptCTCurDemand + i) = (*(ptCurrDemandSum + i) / wDemandSumCntr[1]);
			*(ptCTKwDemand + i) = (*(ptKwDemandSum + i) / wDemandSumCntr[1]);

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
		for (i=0; i<42; i++)					// Every 1 sec
		{
			(*(ptCurrDemandSum + i))+=Data.array[RMS_1_OFFSET + i];
			(*(ptKwDemandSum + i))+= Data.array[KW_1_OFFSET + i];
		}
		sFlag.pnl3_demand_chk_1sec = 0;
		wDemandSumCntr[2]++;
	      }

           if (sFlag.pnl3_demand_chk_1hr)
	     { 
		for (i=0; i<42; i++)
		{
			*(ptCTCurDemand + i) = (*(ptCurrDemandSum + i) / wDemandSumCntr[2]);
			*(ptCTKwDemand + i) = (*(ptKwDemandSum + i) / wDemandSumCntr[2]);

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
		for (i=0; i<42; i++)					// Every 1 sec
		{
			(*(ptCurrDemandSum + i))+=Data.array[RMS_2_OFFSET + i];
			(*(ptKwDemandSum + i))+= Data.array[KW_2_OFFSET + i];
		}
		sFlag.pnl4_demand_chk_1sec = 0;
		wDemandSumCntr[3]++;
	      }

           if (sFlag.pnl4_demand_chk_1hr)
	     { 
		for (i=0; i<42; i++)
		{
			*(ptCTCurDemand + i) = (*(ptCurrDemandSum + i) / wDemandSumCntr[2]);
			*(ptCTKwDemand + i) = (*(ptKwDemandSum + i) / wDemandSumCntr[2]);

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
	
	Data_In.word.wSysInfo.word.wPowerCapacity = (0x01030000 | 0x4000);
	Data_In.word.wSysInfo.word.wNominal_IPVolt = (0x01060000 | 0x2300);
	Data_In.word.wSysInfo.word.wNominal_OPVolt = (0x01070000 | 0x2300);
	//printf("\nPower Capacity : %d\n",Data_In.word.wSysInfo.word.wPowerCapacity);

	Data_In.word.wSysInfo.word.PDC_ID = (0x01190000 | 1);
	Data_In.word.wSysInfo.word.wPDCType = (0x012D0000 | INPUT_SINGLE);
	Data_In.word.wSysInfo.word.wConfiguration = (0x01040000 | 0x200);

	Data_In.word.wSysInfo.word.wFrequency = (0x01080000 | 500);
	Data_In.word.wSysThd.word.wAmbTempHighLimit	= 400;
	Data_In.word.wSysThd.word.wGroundCurr = 50;
	
	CalculateThreshold();
	
	for (i = 0; i<NUM_PANEL; i++)
	{
		for(j = 0; j<84; j++)
		{
			Data_In.word.wUnderCurrentLimit[i][j] = 0;
			Data_In.word.wOverCurrentLimit[i][j] = 160;

			Data_In.word.wUnderCurDemandLimit[i][j] = 0;
			Data_In.word.wOverCurDemandLimit[i][j] = 160;

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
	//unsigned int dwMathBuff;


	// Primary Parameters Default	
	wPri_Thd.word.wPhaseA_UnderVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*87;	
	wPri_Thd.word.wPhaseB_UnderVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*87;
	wPri_Thd.word.wPhaseC_UnderVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*87;
	//printf("\nPhase A Under Volt : %x\n",wPri_Thd.word.wPhaseA_UnderVolt);
	
	
	wPri_Thd.word.wPhaseA_OverVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*110;
	wPri_Thd.word.wPhaseB_OverVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*110;
	wPri_Thd.word.wPhaseC_OverVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*110;

	dwMathBuff = ((unsigned long)(Data_In.word.wSysInfo.word.wPowerCapacity & 0x0000FFFF))*1000;
	dwMathBuff = dwMathBuff/(Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF);
	i=dwMathBuff/3;

	dwMathBuff = ((unsigned long)i * 75)/10;

	wPri_Thd.word.wPhaseA_OverCurr = dwMathBuff;
	wPri_Thd.word.wPhaseB_OverCurr = dwMathBuff;
	wPri_Thd.word.wPhaseC_OverCurr = dwMathBuff;
	wPri_Thd.word.wNeutral_OverCurr = i*10;
	wPri_Thd.word.wPhaseA_UnderCurr = 0;
	wPri_Thd.word.wPhaseB_UnderCurr = 0;
	wPri_Thd.word.wPhaseC_UnderCurr = 0;
	wPri_Thd.word.wOverVoltTHD = 50;
	wPri_Thd.word.wOverCurrTHD = 150;
	wPri_Thd.word.wPowerfactor = 75;


	// Secondary Parameters Default
	
	wSec_Thd.word.wPhaseA_UnderVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*87;	
	wSec_Thd.word.wPhaseB_UnderVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*87;
	wSec_Thd.word.wPhaseC_UnderVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*87;
	
	wSec_Thd.word.wPhaseA_OverVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*110;
	wSec_Thd.word.wPhaseB_OverVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*110;
	wSec_Thd.word.wPhaseC_OverVolt = (Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF)/100*110;

	dwMathBuff = ((unsigned long)(Data_In.word.wSysInfo.word.wPowerCapacity & 0x0000FFFF))*1000;
	dwMathBuff = dwMathBuff/(Data_In.word.wSysInfo.word.wNominal_IPVolt & 0x0000FFFF);
	i=dwMathBuff/3;

	dwMathBuff = ((unsigned long)i * 80)/10;

	wSec_Thd.word.wPhaseA_OverCurr = dwMathBuff;
	wSec_Thd.word.wPhaseB_OverCurr = dwMathBuff;
	wSec_Thd.word.wPhaseC_OverCurr = dwMathBuff;
	wSec_Thd.word.wNeutral_OverCurr = i*10;
	wSec_Thd.word.wPhaseA_UnderCurr = 0;
	wSec_Thd.word.wPhaseB_UnderCurr = 0;
	wSec_Thd.word.wPhaseC_UnderCurr = 0;
	wSec_Thd.word.wOverVoltTHD = 50;
	wSec_Thd.word.wOverCurrTHD = 150;
	wSec_Thd.word.wPowerfactor = 75;
	/*for (i=0;i < 3 ; i++)
		{
			//printf("\nInHere\n");
			Data_In.word.wPanelThd[i].word.wPanel1PhaseA_OverCurr = wSec_Thd.word.wPhaseA_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel2PhaseA_OverCurr = wSec_Thd.word.wPhaseA_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel1PhaseB_OverCurr = wSec_Thd.word.wPhaseB_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel2PhaseB_OverCurr = wSec_Thd.word.wPhaseB_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel1PhaseC_OverCurr = wSec_Thd.word.wPhaseC_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel2PhaseC_OverCurr = wSec_Thd.word.wPhaseC_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel1Neutral_OverCurr = wSec_Thd.word.wNeutral_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel2Neutral_OverCurr = wSec_Thd.word.wNeutral_OverCurr/j;
			Data_In.word.wPanelThd[i].word.wPanel1PhaseA_UnderCurr = 0;
			Data_In.word.wPanelThd[i].word.wPanel2PhaseA_UnderCurr = 0;
			Data_In.word.wPanelThd[i].word.wPanel1PhaseB_UnderCurr = 0;
			Data_In.word.wPanelThd[i].word.wPanel2PhaseB_UnderCurr = 0;
			Data_In.word.wPanelThd[i].word.wPanel1PhaseC_UnderCurr = 0;
			Data_In.word.wPanelThd[i].word.wPanel2PhaseC_UnderCurr = 0;
			Data_In.word.wPanelThd[i].word.wPanel1_OverVoltTHD = 50;
			Data_In.word.wPanelThd[i].word.wPanel1_OverCurrTHD = 150;
			Data_In.word.wPanelThd[i].word.wPanel1_Powerfactor = 75;
			Data_In.word.wPanelThd[i].word.wPanel2_OverVoltTHD = 50;
			Data_In.word.wPanelThd[i].word.wPanel2_OverCurrTHD = 150;
			Data_In.word.wPanelThd[i].word.wPanel2_Powerfactor = 75;
		}*/	
	
}

ULONG_2WORD dwCalcBuff[6];
void KWH_Calc(unsigned int reg)
{
	//static ULONG_2WORD dwCalcBuff[6];
	unsigned int 	i;
	unsigned long	*ptCTKwh_Error;
	unsigned int	*ptKWH_Error;
	static unsigned int 	dummy_cntr;
	dummy_cntr++;
	switch (reg)
	  {
	   case PRIMARY:
	   	ptKWH_Error = &KWH_Error.dwPrimary;
		dwMathBuff = (*(ptKWH_Error) + (Data.word.Primary.KW & 0x0000FFFF));
		*(ptKWH_Error) = dwMathBuff % 360000;
		dwMathBuff = (dwMathBuff / 360000);
		dwCalcBuff[0].word[0] = (Data.word.Primary.KWH_Lo & 0x0000FFFF);
		dwCalcBuff[0].word[1] = (Data.word.Primary.KWH_Hi & 0x0000FFFF);
		dwCalcBuff[0].all += dwMathBuff;

		if (dwCalcBuff[0].all > 999999999)
		   dwCalcBuff[0].all -= 999999999;
		else
		   {Data.word.Primary.KWH_Lo = (0x080A0000 | dwCalcBuff[0].word[0]);
		    Data.word.Primary.KWH_Hi = (0x080B0000 | dwCalcBuff[0].word[1]);
	            Reg[10].if_index = (if_index_cur | 0x0A);
		    Reg[10].reg_addr = (0x080A);
	            Reg[10].reg_d.reg_value = (dwCalcBuff[0].word[0]);
		    Reg[10].reg_type  = INPUT_REGISTER;
	            Reg[10].reg_flag  = 1;

		    Reg[11].if_index = (if_index_cur | 0x0B);
		    Reg[11].reg_addr = (0x080B);
	            Reg[11].reg_d.reg_value = (dwCalcBuff[0].word[1]);
		    Reg[11].reg_type  = INPUT_REGISTER;
	            Reg[11].reg_flag  = 1;
		   }

	   	break;
	   /*case SEC1:
		ptKWH_Error = &KWH_Error.dwPanel1_PhaseA;
		dwMathBuff = (*(ptKWH_Error) + Secondary1.word.KW_Phase_A);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary1.word.KWH_Phase_A_Hi;
		dwCalcBuff.word[1] = Secondary1.word.KWH_Phase_A_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary1.word.KWH_Phase_A_Hi = dwCalcBuff.word[0];
		    Secondary1.word.KWH_Phase_A_Lo = dwCalcBuff.word[1];} 

		// PhaseB
		ptKWH_Error = &KWH_Error.dwPanel1_PhaseB;
		dwMathBuff = (*(ptKWH_Error) + Secondary1.word.KW_Phase_B);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary1.word.KWH_Phase_B_Hi;
		dwCalcBuff.word[1] = Secondary1.word.KWH_Phase_B_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary1.word.KWH_Phase_B_Hi = dwCalcBuff.word[0];
		    Secondary1.word.KWH_Phase_B_Lo = dwCalcBuff.word[1];} 

		//PhaseC
		ptKWH_Error = &KWH_Error.dwPanel1_PhaseC;
		dwMathBuff = (*(ptKWH_Error) + Secondary1.word.KW_Phase_C);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary1.word.KWH_Phase_C_Hi;
		dwCalcBuff.word[1] = Secondary1.word.KWH_Phase_C_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary1.word.KWH_Phase_C_Hi = dwCalcBuff.word[0];
		    Secondary1.word.KWH_Phase_C_Lo = dwCalcBuff.word[1];} 
		break;*/
	   /*case SEC2:
		//printf("\nInside Sec2\n");
		// PhaseA
		ptKWH_Error = &KWH_Error.dwPanel2_PhaseA;
		dwMathBuff = (*(ptKWH_Error) + SedwCalcBuffcondary2.word.KW_Phase_A);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary2.word.KWH_Phase_A_Hi;
		dwCalcBuff.word[1] = Secondary2.word.KWH_Phase_A_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary2.word.KWH_Phase_A_Hi = dwCalcBuff.word[0];
		    Secondary2.word.KWH_Phase_A_Lo = dwCalcBuff.word[1];} 

		// PhaseB
		ptKWH_Error = &KWH_Error.dwPanel2_PhaseB;
		dwMathBuff = (*(ptKWH_Error) + Secondary2.word.KW_Phase_B);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary2.word.KWH_Phase_B_Hi;
		dwCalcBuff.word[1] = Secondary2.word.KWH_Phase_B_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary2.word.KWH_Phase_B_Hi = dwCalcBuff.word[0];
		    Secondary2.word.KWH_Phase_B_Lo = dwCalcBuff.word[1];} 

		//PhaseC
		ptKWH_Error = &KWH_Error.dwPanel2_PhaseC;
		dwMathBuff = (*(ptKWH_Error) + Secondary2.word.KW_Phase_C);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary2.word.KWH_Phase_C_Hi;
		dwCalcBuff.word[1] = Secondary2.word.KWH_Phase_C_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary2.word.KWH_Phase_C_Hi = dwCalcBuff.word[0];
		    Secondary2.word.KWH_Phase_C_Lo = dwCalcBuff.word[1];}
		break;*/
	   /*case SEC3:
		//printf("\nInside Sec3\n");
		// PhaseA
		ptKWH_Error = &KWH_Error.dwPanel3_PhaseA;
		dwMathBuff = (*(ptKWH_Error) + Secondary3.word.KW_Phase_A);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary3.word.KWH_Phase_A_Hi;
		dwCalcBuff.word[1] = Secondary3.word.KWH_Phase_A_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary3.word.KWH_Phase_A_Hi = dwCalcBuff.word[0];
		    Secondary3.word.KWH_Phase_A_Lo = dwCalcBuff.word[1];} 

		// PhaseB
		ptKWH_Error = &KWH_Error.dwPanel3_PhaseB;
		dwMathBuff = (*(ptKWH_Error) + Secondary3.word.KW_Phase_B);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary3.word.KWH_Phase_B_Hi;
		dwCalcBuff.word[1] = Secondary3.word.KWH_Phase_B_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary3.word.KWH_Phase_B_Hi = dwCalcBuff.word[0];
		    Secondary3.word.KWH_Phase_B_Lo = dwCalcBuff.word[1];} 

		//PhaseC
		ptKWH_Error = &KWH_Error.dwPanel3_PhaseC;
		dwMathBuff = (*(ptKWH_Error) + Secondary3.word.KW_Phase_C);
		*(ptKWH_Error) = dwMathBuff % 36000;
		dwMathBuff = (dwMathBuff / 36000);
		dwCalcBuff.word[0] = Secondary3.word.KWH_Phase_C_Hi;
		dwCalcBuff.word[1] = Secondary3.word.KWH_Phase_C_Lo;
		dwCalcBuff.all += dwMathBuff;

		if (dwCalcBuff.all > 999999999)
		   dwCalcBuff.all -= 999999999;
		else
		   {Secondary3.word.KWH_Phase_C_Hi = dwCalcBuff.word[0];
		    Secondary3.word.KWH_Phase_C_Lo = dwCalcBuff.word[1];}
		break;*/ 

	   case SEC:
		//printf("\nInside Sec\n");
		// PhaseA
		ptKWH_Error = &KWH_Error.dwSecondary_PhaseA;
		dwMathBuff = (*(ptKWH_Error) + (Data.word.Secondary.KW_Phase_A & 0x0000FFFF));
		*(ptKWH_Error) = dwMathBuff % 360000;
		dwMathBuff = (dwMathBuff / 360000);
		
		dwCalcBuff[1].word[0] = (Data.word.Secondary.KWH_Phase_A_Lo & 0x0000FFFF);
		dwCalcBuff[1].word[1] = (Data.word.Secondary.KWH_Phase_A_Hi & 0x0000FFFF);
		dwCalcBuff[1].all += dwMathBuff;
		
		if (dwCalcBuff[1].all > 999999999)
		   dwCalcBuff[1].all -= 999999999;
		else
		   {Data.word.Secondary.KWH_Phase_A_Lo = (0x090A0000 | dwCalcBuff[1].word[0]);
		    Data.word.Secondary.KWH_Phase_A_Hi = (0x090B0000 | dwCalcBuff[1].word[1]);
		    Reg[PRI_OFFSET + 10].if_index = (if_index_cur | (PRI_OFFSET + 10));
		    Reg[PRI_OFFSET + 10].reg_addr = (0x090A);
	            Reg[PRI_OFFSET + 10].reg_d.reg_value = (dwCalcBuff[1].word[0]);
		    Reg[PRI_OFFSET + 10].reg_type  = INPUT_REGISTER;
	            Reg[PRI_OFFSET + 10].reg_flag  = 1;

		    Reg[PRI_OFFSET + 11].if_index = (if_index_cur | (PRI_OFFSET + 11));
		    Reg[PRI_OFFSET + 11].reg_addr = (0x090B);
	            Reg[PRI_OFFSET + 11].reg_d.reg_value = (dwCalcBuff[1].word[1]);
		    Reg[PRI_OFFSET + 11].reg_type  = INPUT_REGISTER;
	            Reg[PRI_OFFSET + 11].reg_flag  = 1;
		   } 

		// PhaseB
		ptKWH_Error = &KWH_Error.dwSecondary_PhaseB;
		dwMathBuff = (*(ptKWH_Error) + (Data.word.Secondary.KW_Phase_B & 0x0000FFFF));
		*(ptKWH_Error) = dwMathBuff % 360000;
		dwMathBuff = (dwMathBuff / 360000);
		dwCalcBuff[2].word[0] = (Data.word.Secondary.KWH_Phase_B_Lo & 0x0000FFFF);
		dwCalcBuff[2].word[1] = (Data.word.Secondary.KWH_Phase_B_Hi & 0x0000FFFF);
		dwCalcBuff[2].all += dwMathBuff;
		if (dwCalcBuff[2].all > 999999999)
		   dwCalcBuff[2].all -= 999999999;
		else
		   {Data.word.Secondary.KWH_Phase_B_Lo = (0x090C0000 | dwCalcBuff[2].word[0]);
		    Data.word.Secondary.KWH_Phase_B_Hi = (0x090D0000 | dwCalcBuff[2].word[1]);
		    Reg[PRI_OFFSET + 12].if_index = (if_index_cur | (PRI_OFFSET + 12));
		    Reg[PRI_OFFSET + 12].reg_addr = (0x090C);
	            Reg[PRI_OFFSET + 12].reg_d.reg_value = (dwCalcBuff[2].word[0]);
		    Reg[PRI_OFFSET + 12].reg_type  = INPUT_REGISTER;
	            Reg[PRI_OFFSET + 12].reg_flag  = 1;

		    Reg[PRI_OFFSET + 13].if_index = (if_index_cur | (PRI_OFFSET + 13));
		    Reg[PRI_OFFSET + 13].reg_addr = (0x090D);
	            Reg[PRI_OFFSET + 13].reg_d.reg_value = (dwCalcBuff[2].word[1]);
		    Reg[PRI_OFFSET + 13].reg_type  = INPUT_REGISTER;
	            Reg[PRI_OFFSET + 13].reg_flag  = 1;
		   } 

		//PhaseC
		ptKWH_Error = &KWH_Error.dwSecondary_PhaseC;
		dwMathBuff = (*(ptKWH_Error) + (Data.word.Secondary.KW_Phase_C & 0x0000FFFF));
		*(ptKWH_Error) = dwMathBuff % 360000;
		dwMathBuff = (dwMathBuff / 360000);
		
		dwCalcBuff[3].word[0] = (Data.word.Secondary.KWH_Phase_C_Lo & 0x0000FFFF);
		dwCalcBuff[3].word[1] = (Data.word.Secondary.KWH_Phase_C_Hi & 0x0000FFFF);
		dwCalcBuff[3].all += dwMathBuff;

		if (dwCalcBuff[3].all > 999999999)
		   dwCalcBuff[3].all -= 999999999;
		else
		   {Data.word.Secondary.KWH_Phase_C_Lo = (0x090E0000 | dwCalcBuff[3].word[0]);
		    Data.word.Secondary.KWH_Phase_C_Hi = (0x090F0000 | dwCalcBuff[3].word[1]);
		    Reg[PRI_OFFSET + 14].if_index = (if_index_cur | (PRI_OFFSET + 14));
		    Reg[PRI_OFFSET + 14].reg_addr = (0x090E);
	            Reg[PRI_OFFSET + 14].reg_d.reg_value = (dwCalcBuff[3].word[0]);
		    Reg[PRI_OFFSET + 14].reg_type  = INPUT_REGISTER;
	            Reg[PRI_OFFSET + 14].reg_flag  = 1;

		    Reg[PRI_OFFSET + 15].if_index = (if_index_cur | (PRI_OFFSET + 15));
		    Reg[PRI_OFFSET + 15].reg_addr = (0x090F);
	            Reg[PRI_OFFSET + 15].reg_d.reg_value = (dwCalcBuff[3].word[1]);
		    Reg[PRI_OFFSET + 15].reg_type  = INPUT_REGISTER;
	            Reg[PRI_OFFSET + 15].reg_flag  = 1;
		   }

		break; 

	   case PANEL1:
		//printf("\nInside Panel1\n");
		ptCTKwh_Error = &strCTKwh_Error[0].array[0];
		for (i=0; i<84; i++){
		  dwMathBuff = (unsigned int)(*(ptCTKwh_Error + i) + (Data.array[RMS_3_OFFSET + i] & 0x0000FFFF));
		  /*if (i==2)
		  {
		  printf("\nValue1 : %x\n",(Data.array[RMS_3_OFFSET + i] & 0x0000FFFF));
printf("\nValue2 : %x\n",(Data.array[RMS_3_OFFSET + i] ));
		  printf("\nError for PANEL 1 : %x\n",dwMathBuff);		
		  }*/

		  *(ptCTKwh_Error + i) = dwMathBuff % 12000;
		  
		  dwMathBuff = (dwMathBuff / 12000);
		  //if (i==2)
		  //add_to_log(dwMathBuff);

		  dwCalcBuff[4].word[0] = (Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2)] & 0x0000FFFF);
		  dwCalcBuff[4].word[1] = (Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2) + 1] & 0x0000FFFF);
		  dwCalcBuff[4].all += dwMathBuff;

		  if (dwCalcBuff[4].all > 999999999)
		      dwCalcBuff[4].all -= 999999999;
		  else
		   {
			Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2)] = ((((0x2D00|(i*2))|(0<<14))<<16) | dwCalcBuff[4].word[0]);
			Data.array[PANEL24_OVERCURR_STATUS_FLAG + (i*2) + 1] = ((((0x2D00|(i*2 + 1))|(0<<14))<<16) | dwCalcBuff[4].word[1]);
		   }
		}
	
		break;
	   case PANEL2:
		//printf("\nInside Panel2\n");
		ptCTKwh_Error = &strCTKwh_Error[1].array[0];
		for (i=0; i<84; i++){
		  dwMathBuff = (*(ptCTKwh_Error + i) + (Data.array[KW_0_OFFSET + i] & 0x0000FFFF));
		  *(ptCTKwh_Error + i) = dwMathBuff % 12000;
		  dwMathBuff = (dwMathBuff / 12000);

		  dwCalcBuff[5].word[0] = (Data.array[KWH_0_OFFSET + (i*2)] & 0x0000FFFF);
		  dwCalcBuff[5].word[1] = (Data.array[KWH_0_OFFSET + (i*2) + 1] & 0x0000FFFF);
		  dwCalcBuff[5].all += dwMathBuff;

		  if (dwCalcBuff[5].all > 999999999)
		      dwCalcBuff[5].all -= 999999999;
		  else
		   {
			Data.array[KWH_0_OFFSET + (i*2)] = ((((0x2D00|(i*2))|(1<<14))<<16) | dwCalcBuff[5].word[0]);
			Data.array[KWH_0_OFFSET + (i*2) + 1] = ((((0x2D00|(i*2 + 1))|(1<<14))<<16) | dwCalcBuff[5].word[1]);
		   }
		}
		break;
	   /*case PANEL3:
		//printf("\nInside Panel3\n"); 
		ptCTKwh_Error = &strCTKwh_Error[2].array[0];
		for (i=0; i<42; i++){
		  dwMathBuff = (*(ptCTKwh_Error + i) + strCTKw[2].array[i]);
		  *(ptCTKwh_Error + i) = dwMathBuff % 36000;
		  dwMathBuff = (dwMathBuff / 36000);

		  dwCalcBuff.word[0] = strCTKwh[2].array[i*2];
		  dwCalcBuff.word[1] = strCTKwh[2].array[(i*2)+1];
		  dwCalcBuff.all += dwMathBuff;

		  if (dwCalcBuff.all > 999999999)
		      dwCalcBuff.all -= 999999999;
		  else
		   {
			strCTKwh[2].array[2*i] = dwCalcBuff.word[0];
			strCTKwh[2].array[(2*i)+1] = dwCalcBuff.word[1];
		   }
		}
		break;*/
	   default:
	   break;
	  }

	  /*if (dummy_cntr == 36000)
	  {
		printf("\nPrimary KWH Hi : %x\n",Data.word.Primary.KWH_Hi);
		printf("\nPrimary KWH Lo : %x\n",Data.word.Primary.KWH_Lo);	
		printf("\nSecondary A KWH Hi : %x\n",Data.word.Secondary.KWH_Phase_A_Hi);
		printf("\nSecondary A KWH Lo : %x\n",Data.word.Secondary.KWH_Phase_A_Lo);
		printf("\nSecondary B KWH Hi : %x\n",Data.word.Secondary.KWH_Phase_B_Hi);
		printf("\nSecondary B KWH Lo : %x\n",Data.word.Secondary.KWH_Phase_B_Lo);
		printf("\nSecondary C KWH Hi : %x\n",Data.word.Secondary.KWH_Phase_C_Hi);
		printf("\nSecondary C KWH Lo : %x\n",Data.word.Secondary.KWH_Phase_C_Lo);
		dummy_cntr -= 36000;
	  }*/
}

void System_Status_Update(void)
{
	unsigned char dummy_char;
	
	// EPO and REPO
	
	nos_i2c_read(0, 0x43, P14I0EE5V6408_GPIO_EXPANDER_INPUT_STATUS_REG, &dummy_char, 1);
	Data.word.System_Status.EPO = (dummy_char >> 6);
	Data.word.System_Status.REPO = (dummy_char >> 7);

	// Circuit Breaker Status

	nos_i2c_read(0, 0x44, P14I0EE5V6408_GPIO_EXPANDER_INPUT_STATUS_REG, &dummy_char, 1);
	Data.word.System_Status.Temp_125 = dummy_char;
	Data.word.System_Status.Temp_150 = (dummy_char >> 1);
	Data.word.System_Status.CB_Primary = (dummy_char >> 5);
	Data.word.System_Status.CB_Secondary = (dummy_char >> 6);
	

	// DRY Contacts

	if (Data.word.System_Status.CB_Primary == CLOSE)     // DRYA --> PRIMARY CB STATUS
	    icos_io_set_port(0,0x43,0,1);
	else
	    icos_io_set_port(0,0x43,0,0);

	if (Data.word.System_Status.CB_Secondary == CLOSE)     // DRYB --> SECONDARY CB STATUS
	    icos_io_set_port(0,0x43,1,1);
	else
	    icos_io_set_port(0,0x43,1,0);

	if (Data.word.System_Status.Temp_150 == CLOSE)     // DRYC --> TEMP_150 STATUS
	    icos_io_set_port(0,0x43,3,1);
	else
	    icos_io_set_port(0,0x43,3,0);

	if (Data.word.System_Status.AmbTempHigh)     // DRYD --> AMBIENT TEMP HIGH
	    icos_io_set_port(0,0x43,3,1);
	else
	    icos_io_set_port(0,0x43,3,0);
	
	if (Data.word.Sec_Status_Flag.PhaseA_OverCurr 
	    || Data.word.Sec_Status_Flag.PhaseB_OverCurr
	    || Data.word.Sec_Status_Flag.PhaseC_OverCurr)     // DRYE --> SEC_PH_OVERLOAD
	    icos_io_set_port(0,0x43,4,1);
	else
	    icos_io_set_port(0,0x43,4,0);
		
	// Frequency

	if ((((Data.word.Secondary.Frequency & 0x0000FFFF) + 30) < (Data.word.Sys.wFrequency))
	   || ((Data.word.Secondary.Frequency & 0x0000FFFF) > (Data.word.Sys.wFrequency + 30)))
	      {
		Data.word.System_Status.FreqFail = 1;
	      }
	else if ((((Data.word.Secondary.Frequency & 0x0000FFFF) + 25) > (Data.word.Sys.wFrequency))
	   || ((Data.word.Secondary.Frequency & 0x0000FFFF) < (Data.word.Sys.wFrequency + 25)))
	      {
		Data.word.System_Status.FreqFail = 1;
	      }			


}

void AmbientTempCalc(void)
{
    unsigned int AdcAmbientTemp;
    icos_adc_read_channel(1,0x4A, 0 , &AdcAmbientTemp);
   
    if(AdcAmbientTemp < 17)
      Data.word.System_Parameter.Ambient_Temp = ((0x600 << 16) | (0));
    else if(AdcAmbientTemp > 90)
      Data.word.System_Parameter.Ambient_Temp = ((0x600 << 16) | (800));
    else
      SysParameter.word.Ambient_Temp = ((0x600 << 16) | (((AdcAmbientTemp - 17) * 100)/9) - 15);	

}

void Cal_Fan_Current(void)
{
	unsigned short Adc_fan_temp[NO_OF_FANS];
	unsigned int i;
	/*for (i=0; i < NO_OF_FANS; i++)
	  {
	    icos_adc_read_channel(1,0x4A,i+1, &Adc_fan_temp[i]);
	    //printf("\nFan reading %d : %x\n",1,Adc_fan_temp[1]); 
	  }*/
	//usleep(500);
	icos_adc_read_channel(1,0x4A,1,&Adc_fan_temp[0]);
	//usleep(500);
	icos_adc_read_channel(1,0x4A,1,&Adc_fan_temp[0]);
	//usleep(500);
	icos_adc_read_channel(1,0x4A,2,&Adc_fan_temp[1]);
	//usleep(500);
	icos_adc_read_channel(1,0x4A,3,&Adc_fan_temp[2]);
	//usleep(500);
	icos_adc_read_channel(1,0x49,0,&Adc_fan_temp[3]);
	//usleep(500);
	icos_adc_read_channel(1,0x49,1,&Adc_fan_temp[4]);
	//usleep(500);
	icos_adc_read_channel(1,0x49,2,&Adc_fan_temp[5]);
	//usleep(500);
	icos_adc_read_channel(1,0x49,3,&Adc_fan_temp[6]);
	//usleep(500);
	icos_adc_read_channel(1,0x48,0,&Adc_fan_temp[7]);
	//usleep(500);
	icos_adc_read_channel(1,0x48,1,&Adc_fan_temp[8]);
	//usleep(500);
	#if 0
	icos_adc_read_channel(1,0x48,2,&Adc_fan_temp[9]);
	//usleep(500);
	icos_adc_read_channel(1,0x48,3,&Adc_fan_temp[10]);
	//usleep(500);
	icos_adc_read_channel(1,0x4B,0,&Adc_fan_temp[11]);
	//usleep(500);
	#endif

	/*for (i=0; i < NO_OF_FANS; i++)
	  {
	    //icos_adc_read_channel(1,0x4A,i+1, &Adc_fan_temp[i]);
	    printf("\nFan reading %d : %x\n",i+1,Adc_fan_temp[i+1]); 
	  }*/
	  
	AdcFanCurrent.dwFan1_current_sum += ((signed int)(Adc_fan_temp[0] - strFanParameter.Fan1Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan1Offset));
	AdcFanCurrent.dwFan2_current_sum += ((signed int)(Adc_fan_temp[1] - strFanParameter.Fan2Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan2Offset));
	AdcFanCurrent.dwFan3_current_sum += ((signed int)(Adc_fan_temp[2] - strFanParameter.Fan3Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan3Offset));
	AdcFanCurrent.dwFan4_current_sum += ((signed int)(Adc_fan_temp[3] - strFanParameter.Fan4Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan4Offset));
	AdcFanCurrent.dwFan5_current_sum += ((signed int)(Adc_fan_temp[4] - strFanParameter.Fan5Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan5Offset));
	AdcFanCurrent.dwFan6_current_sum += ((signed int)(Adc_fan_temp[5] - strFanParameter.Fan6Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan6Offset));
	AdcFanCurrent.dwFan7_current_sum += ((signed int)(Adc_fan_temp[6] - strFanParameter.Fan7Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan7Offset));
	AdcFanCurrent.dwFan8_current_sum += ((signed int)(Adc_fan_temp[7] - strFanParameter.Fan8Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan8Offset));
	#if 0
	AdcFanCurrent.dwFan9_current_sum += ((signed int)(Adc_fan_temp[8] - strFanParameter.Fan9Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan9Offset));
	AdcFanCurrent.dwFan10_current_sum += ((signed int)(Adc_fan_temp[9] - strFanParameter.Fan10Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan10Offset));
	AdcFanCurrent.dwFan11_current_sum += ((signed int)(Adc_fan_temp[10] - strFanParameter.Fan11Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan11Offset));
	AdcFanCurrent.dwFan12_current_sum += ((signed int)(Adc_fan_temp[11] - strFanParameter.Fan12Offset)*(signed int)(Adc_fan_temp[0] - strFanParameter.Fan12Offset));
	#endif
	AdcFanCurrent.cnt++;

	if (AdcFanCurrent.cnt >= (500000 / (((Data.word.Secondary.Frequency & 0x0000FFFF) != 0xFFFF)?(Data.word.Secondary.Frequency & 0x0000FFFF):(500))))
	  {
	        AdcFanCurrent.dwFan1_current_sum_out = 	AdcFanCurrent.dwFan1_current_sum;
		AdcFanCurrent.dwFan2_current_sum_out = 	AdcFanCurrent.dwFan2_current_sum;
		AdcFanCurrent.dwFan3_current_sum_out = 	AdcFanCurrent.dwFan3_current_sum;
		AdcFanCurrent.dwFan4_current_sum_out = 	AdcFanCurrent.dwFan4_current_sum;
		AdcFanCurrent.dwFan5_current_sum_out = 	AdcFanCurrent.dwFan5_current_sum;
		AdcFanCurrent.dwFan6_current_sum_out = 	AdcFanCurrent.dwFan6_current_sum;
		AdcFanCurrent.dwFan7_current_sum_out = 	AdcFanCurrent.dwFan7_current_sum;
		AdcFanCurrent.dwFan8_current_sum_out = 	AdcFanCurrent.dwFan8_current_sum;
		#if 0
		AdcFanCurrent.dwFan9_current_sum_out = 	AdcFanCurrent.dwFan9_current_sum;
		AdcFanCurrent.dwFan10_current_sum_out = 	AdcFanCurrent.dwFan10_current_sum;
		AdcFanCurrent.dwFan11_current_sum_out = 	AdcFanCurrent.dwFan11_current_sum;
		AdcFanCurrent.dwFan12_current_sum_out = 	AdcFanCurrent.dwFan12_current_sum;
	        #endif 
		AdcFanCurrent.cnt_out = AdcFanCurrent.cnt;
		AdcFanCurrent.flag_OK = 1;

		AdcFanCurrent.dwFan1_current_sum = 0;
		AdcFanCurrent.dwFan2_current_sum = 0;
		AdcFanCurrent.dwFan3_current_sum = 0;
		AdcFanCurrent.dwFan4_current_sum = 0;
		AdcFanCurrent.dwFan5_current_sum = 0;
		AdcFanCurrent.dwFan6_current_sum = 0;
		AdcFanCurrent.dwFan7_current_sum = 0;
		AdcFanCurrent.dwFan8_current_sum = 0;
	        #if 0
		AdcFanCurrent.dwFan9_current_sum = 0;
		AdcFanCurrent.dwFan10_current_sum = 0;
		AdcFanCurrent.dwFan11_current_sum = 0;
		AdcFanCurrent.dwFan12_current_sum = 0;
		#endif
		AdcFanCurrent.cnt = 0;
	  }	
	
	
}

void RMS_Fan_Current(void)
{
	if(AdcFanCurrent.flag_OK == 1)
	{
	  Data.word.System_Parameter.Fan1RmsCurrent = Square_root(AdcFanCurrent.dwFan1_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan1Gain;
	  Data.word.System_Parameter.Fan2RmsCurrent = Square_root(AdcFanCurrent.dwFan2_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan2Gain;
	  Data.word.System_Parameter.Fan3RmsCurrent = Square_root(AdcFanCurrent.dwFan3_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan3Gain;
	  Data.word.System_Parameter.Fan4RmsCurrent = Square_root(AdcFanCurrent.dwFan4_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan4Gain;
	  Data.word.System_Parameter.Fan5RmsCurrent = Square_root(AdcFanCurrent.dwFan5_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan5Gain;
	  Data.word.System_Parameter.Fan6RmsCurrent = Square_root(AdcFanCurrent.dwFan6_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan6Gain;
	  Data.word.System_Parameter.Fan7RmsCurrent = Square_root(AdcFanCurrent.dwFan1_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan7Gain;
	  Data.word.System_Parameter.Fan8RmsCurrent = Square_root(AdcFanCurrent.dwFan1_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan8Gain;

	  printf("\nRMS current :1 : %d \n",Data.word.System_Parameter.Fan1RmsCurrent);
	  #if 0
	  Data.word.System_Parameter.Fan9RmsCurrent = Square_root(AdcFanCurrent.dwFan1_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan9Gain;
	  Data.word.System_Parameter.Fan10RmsCurrent = Square_root(AdcFanCurrent.dwFan10_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan10Gain;
	  Data.word.System_Parameter.Fan11RmsCurrent = Square_root(AdcFanCurrent.dwFan11_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan11Gain;
	  Data.word.System_Parameter.Fan12RmsCurrent = Square_root(AdcFanCurrent.dwFan12_current_sum_out/AdcFanCurrent.cnt_out)*strFanParameter.Fan12Gain;
	  #endif	
	}
}

double Square_root(unsigned int value)
{
	unsigned int number = value;
	double sqrt,temp;

	sqrt = number /2;
	temp = 0;

	while(sqrt != temp)
	 {
	    temp = sqrt;

	    sqrt = (number/temp + temp)/2;

	 }

     return sqrt;	
}




