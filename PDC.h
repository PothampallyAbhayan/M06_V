#define DSP_SYS_INFO_HI_ADDR		0x01
#define DSP_SYS_THD_HI_ADDR		0x02
#define DSP_PRIMARY_LIMIT_HI_ADDR	0x03
#define DSP_SECONDARY_LIMIT_HI_ADDR	0x04
#define DSP_PRI_SEC_GAIN_HI_ADDR	0x05
#define DSP_SYS_PARAMETER_HI_ADDR	0x06
#define DSP_PRIMARY_HI_ADDR		0x08
#define DSP_SEC_HI_ADDR			0x09
#define DSP_MAX_MIN_PRI_HI_ADDR		0x0A
#define DSP_SEC_PANEL1_HI_ADDR		0x50
#define DSP_SEC_PANEL2_HI_ADDR		0x51
#define DSP_SEC_PANEL3_HI_ADDR		0x60
#define BRANCH_GAIN			0x5B

#define PANEL_GAIN			168
#define SYSTEM_GAIN			19

#define P14I0EE5V6408_GPIO_EXPANDER_INPUT_STATUS_REG	0x0F





#define BOARDI_CT_RMS_HI_ADDR			0x52
#define BOARDI_CT_MIN_CURRENT_HI_ADDR		0x53
#define BOARDI_CT_MAX_CURRENT_HI_ADDR		0x54
#define BOARDI_LOADPER_HI_ADDR			0x55
#define FIRMWARE_VERSION			0x58
//#define BOARDI_OVER_CURRENT_LIMIT_HI_ADDR	0x58
#define BOARDI_PANEL_CURRENT_LIMIT_HI_ADDR	0x59
#define BOARDI_ACTIVE_INACTIVE_HI_ADDR		0x5A
#define BOARDI_PANEL_GAIN_HI_ADDR		0x5C
#define BOARDI_KW_HI_ADDR			0x5F


#define INPUT_SINGLE	0
#define NUM_PANEL	4
#define NUM_BRANCH	84
#define NUM_BOARD	2

#define NUM_ADC_SAMPLES	16

#define EEDATA_LENGTH	1081+16
#define EEDATA_PANEL_LEN	20

#define BOARD_I		0x7F7
#define BOARD_II	0x7FB
#define BOARD_III	0x7FD

#define PANEL_CT_DATA_LENGTH		42
#define TOTAL_BRANCH_CT_DATA_LENGTH	84
#define EEDATA_ACTIVE_INACTIVE_LEN	8

#define PRIMARY				0
#define SEC1				1
#define SEC2				2
#define SEC3				3
#define SEC4				4
#define SEC				5
#define PANEL1				6
#define PANEL2				7

#define PANEL3				8
#define PANEL4				9

#define MAX_MIN_PARAMETER_PRIMARY	10
#define INPUT_PARAMETERS		11
#define OUTPUT_PARAMETERS		12
#define SYSTEM_PARAMETERS		13

#define NO_OF_FANS			12

#define VOLT_HYSTER			50
#define IP_OC_CURR_HYSTER		((wPri_Thd.word.wPhaseA_OverCurr*5)/1000)
#define IP_HIGH_CURR_HYSTER		((wPri_Thd.word.wPhaseA_OverCurr*9*5)/1000)
#define IP_UC_CURR_HYSTER		((wPri_Thd.word.wPhaseA_UnderCurr*5)/1000)
#define IP_NEUTRAL_CURR_HYSTER		((wPri_Thd.word.wNeutral_OverCurr*5)/1000)
#define IP_ITHD_HYSTER			((wPri_Thd.word.wOverCurrTHD*10)/100)
#define IP_VTHD_HYSTER			((wPri_Thd.word.wOverCurrTHD*10)/100)
#define IP_PF_HYSTER			((wPri_Thd.word.wPowerfactor*10)/100)
#define PHASE_LACK_HYSTER		50
#define VOLT_UMBAL_HYSTER		2
#define GND_CURR_HYSTER			((Data_In.word.wSysThd.word.wGroundCurr * 10)/100)

#define LACKING_VOLT			300
#define UNBALANCE_RATE			3

#define OP_OC_CURR_HYSTER		((wSec_Thd.word.wPhaseA_OverCurr * 5)/100)
#define OP_HIGH_CURR_HYSTER		((wSec_Thd.word.wPhaseA_OverCurr * 9 * 5)/100)
#define OP_UC_CURR_HYSTER		((wSec_Thd.word.wPhaseA_UnderCurr *5)/100)
#define OP_VA_HYSTER			((Data_In.word.wSysInfo.word.wPowerCapacity * 5)/100)
#define OP_NEUTRAL_CURR_HYSTER		((wSec_Thd.word.wNeutral_OverCurr *5)/100)
#define OP_ITHD_HYSTER			((wSec_Thd.word.wOverCurrTHD *10)/100)
#define OP_VTHD_HYSTER			((wSec_Thd.word.wOverVoltTHD *10)/100)
#define OP_PF_LIMIT			75


#define PANEL1_OC_CURR_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr *5)/100)
#define PANEL1_HIGH_CURR_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel1PhaseA_OverCurr*9*5)/100)
#define PANEL1_UC_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel1PhaseA_UnderCurr *5)/100)
#define PANEL1_NEUTRAL_CURR		((Data_In.word.wPanelThd[0].word.wPanel1Neutral_OverCurr * 5)/100)
#define PANEL1_ITHD_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel1_OverCurrTHD *10)/100)
#define PANEL1_VTHD_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel1_OverVoltTHD *10)/100)
#define PANEL1_PF_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel1_Powerfactor *10)/100)

#define PANEL2_OC_CURR_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr *5)/100)
#define PANEL2_HIGH_CURR_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel2PhaseA_OverCurr*9*5)/100)
#define PANEL2_UC_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel2PhaseA_UnderCurr *5)/100)
#define PANEL2_NEUTRAL_CURR		((Data_In.word.wPanelThd[0].word.wPanel2Neutral_OverCurr * 5)/100)
#define PANEL2_ITHD_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel2_OverCurrTHD *10)/100)
#define PANEL2_VTHD_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel2_OverVoltTHD *10)/100)
#define PANEL2_PF_HYSTER		((Data_In.word.wPanelThd[0].word.wPanel2_Powerfactor *10)/100)

#define PANEL3_OC_CURR_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr *5)/100)
#define PANEL3_HIGH_CURR_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel1PhaseA_OverCurr*9*5)/100)
#define PANEL3_UC_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel1PhaseA_UnderCurr *5)/100)
#define PANEL3_NEUTRAL_CURR		((Data_In.word.wPanelThd[1].word.wPanel1Neutral_OverCurr * 5)/100)
#define PANEL3_ITHD_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel1_OverCurrTHD *10)/100)
#define PANEL3_VTHD_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel1_OverVoltTHD *10)/100)
#define PANEL3_PF_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel1_Powerfactor *10)/100)

#define PANEL4_OC_CURR_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel2PhaseA_OverCurr *5)/100)
#define PANEL4_HIGH_CURR_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel2PhaseA_OverCurr*9*5)/100)
#define PANEL4_UC_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel2PhaseA_UnderCurr *5)/100)
#define PANEL4_NEUTRAL_CURR		((Data_In.word.wPanelThd[1].word.wPanel2Neutral_OverCurr * 5)/100)
#define PANEL4_ITHD_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel2_OverCurrTHD *10)/100)
#define PANEL4_VTHD_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel2_OverVoltTHD *10)/100)
#define PANEL4_PF_HYSTER		((Data_In.word.wPanelThd[1].word.wPanel2_Powerfactor *10)/100)

#define BRANCH_OC_HYSTER		15
#define BRANCH_UC_HYSTER		15

#define OVER_TEMP_HYSTER		20
#define BRANCH_UCD_HYSTER		15
#define BRANCH_OCD_HYSTER		15
#define BRANCH_UKWD_HYSTER		15
#define BRANCH_OKWD_HYSTER		15

#define GetHiByte(a)			(*(unsigned char*)&a)
#define GetWord(a)			(*(unsigned int*)&a)
#define GetLowByte(a)			((unsigned char)a)
#define MakeWord(a,b)			(((unsigned int)a << 8) + b)

#define CLOSE				0
#define OPEN				1

#define TEST_CB_PRIMARY_CLOSE		

extern unsigned int iwKWHCalc_1sec_10ms;
extern unsigned int KWHFlag;
extern unsigned int wKwh_Calc_Cntr;
extern unsigned long dwMathBuff;

typedef struct
{
  	unsigned int Active_Inactive[4];
}EE_ACTIVE_INACTIVE;

typedef union
{
  	unsigned int array[EEDATA_ACTIVE_INACTIVE_LEN];
  	EE_ACTIVE_INACTIVE	word;
}EE_ACTIVE_INACTIVE_UNION;

typedef struct
{
	unsigned long long dwFan1_current_sum;
	unsigned long long dwFan2_current_sum;
	unsigned long long dwFan3_current_sum;
	unsigned long long dwFan4_current_sum;
	unsigned long long dwFan5_current_sum;
	unsigned long long dwFan6_current_sum;
	unsigned long long dwFan7_current_sum;
	unsigned long long dwFan8_current_sum;
	unsigned long long dwFan9_current_sum;
	unsigned long long dwFan10_current_sum;
	unsigned long long dwFan11_current_sum;
	unsigned long long dwFan12_current_sum;
	/*unsigned long long reserved;
	unsigned long long dwFan1_current_sum;
	unsigned long long dwFan2_current_sum;
	unsigned long long dwFan3_current_sum;
	unsigned long long dwFan12_current_sum;*/
	/*unsigned int dwFan1_current_sum_out;
	unsigned int dwFan2_current_sum_out;
	unsigned int dwFan3_current_sum_out;
	unsigned int dwFan4_current_sum_out;
	unsigned int dwFan5_current_sum_out;
	unsigned int dwFan6_current_sum_out;
	unsigned int dwFan7_current_sum_out;
	unsigned int dwFan8_current_sum_out;
	unsigned int dwFan9_current_sum_out;
	unsigned int dwFan10_current_sum_out;
	unsigned int dwFan11_current_sum_out;
	unsigned int dwFan12_current_sum_out;
	unsigned int cnt;
	unsigned int cnt_out;
	unsigned char flag_OK;*/
}_ADC_FAN_CURRENT;

typedef union
{
        _ADC_FAN_CURRENT  word;
	unsigned long long array[12];
}ADC_FAN_CURRENT;

ADC_FAN_CURRENT  AdcFanCurrent;

typedef struct
{
	unsigned int dwFan1_current_sum_out;
	unsigned int dwFan2_current_sum_out;
	unsigned int dwFan3_current_sum_out;
	unsigned int dwFan4_current_sum_out;
	unsigned int dwFan5_current_sum_out;
	unsigned int dwFan6_current_sum_out;
	unsigned int dwFan7_current_sum_out;
	unsigned int dwFan8_current_sum_out;
	unsigned int dwFan9_current_sum_out;
	unsigned int dwFan10_current_sum_out;
	unsigned int dwFan11_current_sum_out;
	unsigned int dwFan12_current_sum_out;
}FAN_PARAMETER;
typedef union
{
 FAN_PARAMETER		word;
 unsigned int           array[12];
}_FAN_PARAMETER;         
_FAN_PARAMETER strFanParameter;
/*typedef struct
{
	signed char Fan1;
	signed char Fan2;
	signed char Fan3;
	signed char Fan4;
	signed char Fan5;
	signed char Fan6;
	signed char Fan7;
	signed char Fan8;
	signed char Fan9;
	signed char Fan10;
	signed char Fan11;
	signed char Fan12;
	unsigned int Fan1Offset;
	unsigned int Fan2Offset;
	unsigned int Fan3Offset;
	unsigned int Fan4Offset;
	unsigned int Fan5Offset;
	unsigned int Fan6Offset;
	unsigned int Fan7Offset;
	unsigned int Fan8Offset;
	unsigned int Fan9Offset;
	unsigned int Fan10Offset;
	unsigned int Fan11Offset;
	unsigned int Fan12Offset;
	unsigned int cnt;
}FAN_PARAMETER;

FAN_PARAMETER		FanParameter;*/
typedef struct
{
	unsigned int CB_Primary:    1;
	unsigned int CB_Secondary:  1;
	unsigned int Temp_125:	    1;
	unsigned int Temp_150:	    1;
	unsigned int CB_Primary_Trip:   1;
	unsigned int CB_Secondary_Trip: 1;
	unsigned int VoltageUnbalance:	1;
	unsigned int FreqFail:		1;
	unsigned int GroundFault:	1;
	unsigned int EPO	:	1;
	unsigned int REPO	:	1;
	/*unsigned int Fan_Failure1:	1;
	unsigned int Fan_Failure2:	1;
	unsigned int Fan_Failure3:	1;
	unsigned int Fan_Failure4:	1;
	unsigned int Fan_Failure5:	1;
	unsigned int Fan_Failure6:	1;
	unsigned int Fan_Failure7:	1;
	unsigned int Fan_Failure8:	1;
	unsigned int reserved:		5;*/
	unsigned int Fan_status  :      13;	
	unsigned int paraID	:	8;	
}SYSTEM_STATUS_UPDATE;

SYSTEM_STATUS_UPDATE	System_Status;


typedef union
{
	unsigned long  all;
	unsigned short word[2];
}ULONG_2WORD;

typedef struct
{
	unsigned int PhaseA_UnderVolt:	1;
	unsigned int PhaseB_UnderVolt:	1;
	unsigned int PhaseC_UnderVolt:	1;
	unsigned int PhaseA_OverVolt:	1;
	unsigned int PhaseB_OverVolt:	1;
	unsigned int PhaseC_OverVolt:	1;
	
	unsigned int PhaseA_OverCurr:	1;
	unsigned int PhaseB_OverCurr:	1;
	unsigned int PhaseC_OverCurr:	1;

	unsigned int Neutral_OverCurr:	1;
	unsigned int PhaseA_UnderCurr:	1;
	unsigned int PhaseB_UnderCurr:	1;
	unsigned int PhaseC_UnderCurr:	1;

	unsigned int PhaseA_OverTHDVolt:	1;
	unsigned int PhaseB_OverTHDVolt:	1;
	unsigned int PhaseC_OverTHDVolt:	1;
	unsigned int PhaseA_OverTHDCurr:	1;
	unsigned int PhaseB_OverTHDCurr:	1;
	unsigned int PhaseC_OverTHDCurr:	1;

	unsigned int PhaseA_Curr_High:	1;
	unsigned int PhaseB_Curr_High:	1;
	unsigned int PhaseC_Curr_High:	1;
	unsigned int rsrvd:		2;
	unsigned int paraID:            8; 
}PRI_WINDING_STATUS_FLAG;

typedef union
{
	unsigned int array[2];
	PRI_WINDING_STATUS_FLAG	bit;
}PRIMARY_STATUS_FLAG;

PRIMARY_STATUS_FLAG	StatusPrimary;

typedef struct
{
	unsigned int Max_Volt_PhaseA;
	unsigned int Max_Volt_PhaseB;
	unsigned int Max_Volt_PhaseC;
	unsigned int Min_Volt_PhaseA;
	unsigned int Min_Volt_PhaseB;
	unsigned int Min_Volt_PhaseC;
	
	unsigned int Max_Curr_PhaseA;
	unsigned int Max_Curr_PhaseB;
	unsigned int Max_Curr_PhaseC;
	unsigned int Max_Curr_Neutral;
	unsigned int Min_Curr_PhaseA;
	unsigned int Min_Curr_PhaseB;
	unsigned int Min_Curr_PhaseC;
	unsigned int Min_Curr_Neutral;

	unsigned int Max_Freq_PhaseA;
	unsigned int Max_Freq_PhaseB;
	unsigned int Max_Freq_PhaseC;
	unsigned int Min_Freq_PhaseA;
	unsigned int Min_Freq_PhaseB;
	unsigned int Min_Freq_PhaseC;
	
	unsigned int Max_KW;
	unsigned int Min_KW;
}MAX_MIN_PARAMTER;

typedef union
{
	unsigned int array[22];
	MAX_MIN_PARAMTER  word;
}MAX_MIN_PARAMTER_UNION;

MAX_MIN_PARAMTER_UNION  Mx_Min_Parameter[2];

typedef struct
{
	unsigned int wAddrCntr;
}MAX_MIN_STATUS;

typedef union
{
	unsigned int array[1];
}MAX_MIN_STATUS_UNION;

MAX_MIN_STATUS_UNION  Max_Min_Status;

typedef struct
{
	unsigned int PhaseA_UnderVolt:	1;
	unsigned int PhaseB_UnderVolt:	1;
	unsigned int PhaseC_UnderVolt:	1;
	unsigned int PhaseA_OverVolt:	1;
	unsigned int PhaseB_OverVolt:	1;
	unsigned int PhaseC_OverVolt:	1;
	
	unsigned int PhaseA_OverCurr:	1;
	unsigned int PhaseB_OverCurr:	1;
	unsigned int PhaseC_OverCurr:	1;

	unsigned int Neutral_OverCurr:	1;
	unsigned int PhaseA_UnderCurr:	1;
	unsigned int PhaseB_UnderCurr:	1;
	unsigned int PhaseC_UnderCurr:	1;

	unsigned int PhaseA_OverTHDVolt:	1;
	unsigned int PhaseB_OverTHDVolt:	1;
	unsigned int PhaseC_OverTHDVolt:	1;
	unsigned int dummy1:	1;
	unsigned int dummy2:	1;
	unsigned int PhaseA_PoorPF:	1;

	unsigned int PhaseB_PoorPF:	1;
	unsigned int PhaseC_PoorPF:	1;
	//unsigned int PhaseC_PoorPF:	1;

	unsigned int PhaseA_OverTHDCurr:	1;
	unsigned int PhaseB_OverTHDCurr:	1;
	unsigned int PhaseC_OverTHDCurr:	1;
	unsigned int paraID:		8; 
}SEC_WINDING_STATUS_FLAG;

typedef union
{
	unsigned int array[2];
	SEC_WINDING_STATUS_FLAG	bit;
}SECONDARY_STATUS_FLAG;

SECONDARY_STATUS_FLAG	StatusSecondary;

typedef struct
{
	unsigned int PhaseA_OverCurr:	1;
	unsigned int PhaseB_OverCurr:	1;
	unsigned int PhaseC_OverCurr:	1;
	unsigned int Neutral_OverCurr:	1;
	unsigned int PhaseA_UnderCurr:	1;
	unsigned int PhaseB_UnderCurr:	1;
	unsigned int PhaseC_UnderCurr:	1;
	unsigned int PhaseA_OverTHDCurr:	1;
	unsigned int PhaseB_OverTHDCurr:	1;
	unsigned int PhaseC_OverTHDCurr:	1;
	unsigned int PhaseA_PoorPF:		1;
	unsigned int PhaseB_PoorPF:		1;
	unsigned int PhaseC_PoorPF:		1;
	unsigned int PhaseA_Curr_High:		1;
	unsigned int PhaseB_Curr_High:		1;
	unsigned int PhaseC_Curr_High:		1;

	unsigned int reserve:			16;
	
}PANEL_WINDING_STATUS_FLAG;

typedef union
{
	unsigned int array[2];
	PANEL_WINDING_STATUS_FLAG	bit;
}PANEL_STATUS_FLAG;

PANEL_STATUS_FLAG	StatusSecondary1;
PANEL_STATUS_FLAG	StatusSecondary2;
PANEL_STATUS_FLAG	StatusSecondary3;

typedef struct
{
	unsigned int L2L_Volt_Phase_AB;
	unsigned int L2L_Volt_Phase_BC;
	unsigned int L2L_Volt_Phase_CA;
	unsigned int L2N_Volt_Phase_A;
	unsigned int L2N_Volt_Phase_B;
	unsigned int L2N_Volt_Phase_C;
	unsigned int RMS_Curr_Phase_A;
	unsigned int RMS_Curr_Phase_B;
	unsigned int RMS_Curr_Phase_C;
	unsigned int RMS_Curr_Neutral;

	unsigned int KWH_Hi;
	unsigned int KWH_Lo;

	unsigned int KVA;
	unsigned int KW;
	unsigned int Volt_THD_Phase_A;
	unsigned int Volt_THD_Phase_B;
	unsigned int Volt_THD_Phase_C;
	unsigned int Curr_THD_Phase_A;
	unsigned int Curr_THD_Phase_B;
	unsigned int Curr_THD_Phase_C;
	unsigned int LoadPer_Phase_A;
	unsigned int LoadPer_Phase_B;
	unsigned int LoadPer_Phase_C;
	unsigned int KVAR; 

}PRI_STRUCTURE_PARAMETER;

typedef union
{
	unsigned int array[28];
	PRI_STRUCTURE_PARAMETER word;
}PRI_PARAMETER_UNION;

PRI_PARAMETER_UNION Primary;

typedef struct
{
	unsigned int cbFirmwareVersion1;
	unsigned int cbFirmwareVersion2;
	unsigned int cbFirmwareVersion3;
	unsigned int cbFirmwareVersion4;
	unsigned int cbFirmwareVersion5;
	unsigned int cbFirmwareVersion6;
	unsigned int cbFirmwareVersion7;
	unsigned int cbFirmwareVersion8;

}FIRMWARE_VERSION_STRUCTURE;

typedef struct
{
	unsigned int L2L_Volt_Phase_AB;
	unsigned int L2L_Volt_Phase_BC;
	unsigned int L2L_Volt_Phase_CA;
	unsigned int L2N_Volt_Phase_A;
	unsigned int L2N_Volt_Phase_B;
	unsigned int L2N_Volt_Phase_C;
	unsigned int RMS_Curr_Phase_A;
	unsigned int RMS_Curr_Phase_B;
	unsigned int RMS_Curr_Phase_C;
	unsigned int RMS_Curr_Neutral;
	unsigned int KWH_Phase_A_Hi;
	unsigned int KWH_Phase_A_Lo;
	unsigned int KWH_Phase_B_Hi;
	unsigned int KWH_Phase_B_Lo;
	unsigned int KWH_Phase_C_Hi;
	unsigned int KWH_Phase_C_Lo;
	unsigned int KVA_Phase_A;
	unsigned int KVA_Phase_B;
	unsigned int KVA_Phase_C;
	unsigned int KW_Phase_A;
	unsigned int KW_Phase_B;
	unsigned int KW_Phase_C;
	unsigned int PF_Phase_A;
	unsigned int PF_Phase_B;
	unsigned int PF_Phase_C;
	unsigned int Volt_THD_Phase_A;
	unsigned int Volt_THD_Phase_B;
	unsigned int Volt_THD_Phase_C;
	unsigned int Curr_THD_Phase_A;
	unsigned int Curr_THD_Phase_B;
	unsigned int Curr_THD_Phase_C;
	unsigned int Kfactor_Phase_A;
	unsigned int Kfactor_Phase_B;
	unsigned int Kfactor_Phase_C;
	unsigned int LoadPer_Phase_A;
	unsigned int LoadPer_Phase_B;
	unsigned int LoadPer_Phase_C;
	unsigned int Frequency;
	unsigned int KVAR_Phase_A;
	unsigned int KVAR_Phase_B;
	unsigned int KVAR_Phase_C;
	unsigned int Panel1_Load;
	unsigned int Panel2_Load;
	unsigned int Panel3_Load;
	unsigned int Panel1_PhaseA_Load;
	unsigned int Panel1_PhaseB_Load;
	unsigned int Panel1_PhaseC_Load;
	unsigned int Panel2_PhaseA_Load;
	unsigned int Panel2_PhaseB_Load;
	unsigned int Panel2_PhaseC_Load;
	unsigned int Panel3_PhaseA_Load;
	unsigned int Panel3_PhaseB_Load;
	unsigned int Panel3_PhaseC_Load;
	
}SEC_STRUCTURE_PARAMETER;

typedef union
{
	unsigned int array[44];
	SEC_STRUCTURE_PARAMETER word;
}SEC_PARAMETER_UNION;

SEC_PARAMETER_UNION Secondary;

typedef struct
{
	unsigned int RMS_Curr_Phase_A;
	unsigned int RMS_Curr_Phase_B;
	unsigned int RMS_Curr_Phase_C;
	unsigned int RMS_Curr_Neutral;
	unsigned int KWH_Phase_A_Hi;
	unsigned int KWH_Phase_A_Lo;
	unsigned int KWH_Phase_B_Hi;
	unsigned int KWH_Phase_B_Lo;
	unsigned int KWH_Phase_C_Hi;
	unsigned int KWH_Phase_C_Lo;
	unsigned int KVA_Phase_A;
	unsigned int KVA_Phase_B;
	unsigned int KVA_Phase_C;
	unsigned int KW_Phase_A;
	unsigned int KW_Phase_B;
	unsigned int KW_Phase_C;
	unsigned int PF_Phase_A;
	unsigned int PF_Phase_B;
	unsigned int PF_Phase_C;
	unsigned int Curr_THD_Phase_A;
	unsigned int Curr_THD_Phase_B;
	unsigned int Curr_THD_Phase_C;
	unsigned int LoadPer_Phase_A;
	unsigned int LoadPer_Phase_B;
	unsigned int LoadPer_Phase_C;
}STRUCTURE_PARAMETER1;

typedef union
{
	unsigned int array[25];
	STRUCTURE_PARAMETER1	word;
}PARAMETER1_UNION;

PARAMETER1_UNION	Secondary1;
PARAMETER1_UNION	Secondary2;
PARAMETER1_UNION	Secondary3;	

typedef struct
{
	unsigned int CT1_UnderCurr:	1;
	unsigned int CT2_UnderCurr:	1;
	unsigned int CT3_UnderCurr:	1;
	unsigned int CT4_UnderCurr:	1;
	unsigned int CT5_UnderCurr:	1;
	unsigned int CT6_UnderCurr:	1;
	unsigned int CT7_UnderCurr:	1;
	unsigned int CT8_UnderCurr:	1;
	unsigned int CT9_UnderCurr:	1;
	unsigned int CT10_UnderCurr:	1;
	unsigned int CT11_UnderCurr:	1;
	unsigned int CT12_UnderCurr:	1;
	unsigned int CT13_UnderCurr:	1;
	unsigned int CT14_UnderCurr:	1;
	unsigned int CT15_UnderCurr:	1;
	unsigned int CT16_UnderCurr:	1;
	unsigned int CT17_UnderCurr:	1;
	unsigned int CT18_UnderCurr:	1;
	unsigned int CT19_UnderCurr:	1;
	unsigned int CT20_UnderCurr:	1;
	unsigned int CT21_UnderCurr:	1;
	unsigned int reserved:		3;
	unsigned int paraID:		8;
	
}CT_CURRENT_STATUS_FLAG;

typedef union
{
	unsigned int array[4];
	CT_CURRENT_STATUS_FLAG  bit;
}CT_UNDERCURR_STATUS_FLAG;



typedef struct
{
	unsigned int CT1_UnderLimit:	1;
	unsigned int CT2_UnderLimit:	1;
	unsigned int CT3_UnderLimit:	1;
	unsigned int CT4_UnderLimit:	1;
	unsigned int CT5_UnderLimit:	1;
	unsigned int CT6_UnderLimit:	1;
	unsigned int CT7_UnderLimit:	1;
	unsigned int CT8_UnderLimit:	1;
	unsigned int CT9_UnderLimit:	1;
	unsigned int CT10_UnderLimit:	1;
	unsigned int CT11_UnderLimit:	1;
	unsigned int CT12_UnderLimit:	1;
	unsigned int CT13_UnderLimit:	1;
	unsigned int CT14_UnderLimit:	1;
	unsigned int CT15_UnderLimit:	1;
	unsigned int CT16_UnderLimit:	1;
	unsigned int CT17_UnderLimit:	1;
	unsigned int CT18_UnderLimit:	1;
	unsigned int CT19_UnderLimit:	1;
	unsigned int CT20_UnderLimit:	1;
	unsigned int CT21_UnderLimit:	1;
	unsigned int CT1_OverLimit:	1;
	unsigned int CT2_OverLimit:	1;
	unsigned int CT3_OverLimit:	1;
	unsigned int CT4_OverLimit:	1;
	unsigned int CT5_OverLimit:	1;
	unsigned int CT6_OverLimit:	1;
	unsigned int CT7_OverLimit:	1;
	unsigned int CT8_OverLimit:	1;
	unsigned int CT9_OverLimit:	1;
	unsigned int CT10_OverLimit:	1;
	unsigned int CT11_OverLimit:	1;
	unsigned int CT12_OverLimit:	1;
	unsigned int CT13_OverLimit:	1;
	unsigned int CT14_OverLimit:	1;
	unsigned int CT15_OverLimit:	1;
	unsigned int CT16_OverLimit:	1;
	unsigned int CT17_OverLimit:	1;
	unsigned int CT18_OverLimit:	1;
	unsigned int CT19_OverLimit:	1;
	unsigned int CT20_OverLimit:	1;
	unsigned int CT21_OverLimit:	1;
	unsigned int reserved	  :	6;
}PANEL1_STATUS_FLAG;

typedef union
{
	unsigned int array[3];
	PANEL1_STATUS_FLAG	bit;
}BRANCH_STATUS_FLAG;

BRANCH_STATUS_FLAG		sKwDemandStatusP1S1;
BRANCH_STATUS_FLAG		sKwDemandStatusP1S2;
BRANCH_STATUS_FLAG		sKwDemandStatusP2S1;
BRANCH_STATUS_FLAG		sKwDemandStatusP2S2;
BRANCH_STATUS_FLAG		sKwDemandStatusP3S1;
BRANCH_STATUS_FLAG		sKwDemandStatusP3S2;

BRANCH_STATUS_FLAG		sCurDemandStatusP1S1;
BRANCH_STATUS_FLAG		sCurDemandStatusP1S2;
BRANCH_STATUS_FLAG		sCurDemandStatusP2S1;
BRANCH_STATUS_FLAG		sCurDemandStatusP2S2;
BRANCH_STATUS_FLAG		sCurDemandStatusP3S1;
BRANCH_STATUS_FLAG		sCurDemandStatusP3S2;
	

typedef struct
{
	unsigned int Channel_1;
	unsigned int Channel_2;
	unsigned int Channel_3;
	unsigned int Channel_4;
	unsigned int Channel_5;
	unsigned int Channel_6;
	unsigned int Channel_7;
	unsigned int Channel_8;
	unsigned int Channel_9;
	unsigned int Channel_10;
	unsigned int Channel_11;
	unsigned int Channel_12;
	unsigned int Channel_13;
	unsigned int Channel_14;
	unsigned int Channel_15;
	unsigned int Channel_16;
	unsigned int Channel_17;
	unsigned int Channel_18;
	unsigned int Channel_19;
	unsigned int Channel_20;
	unsigned int Channel_21;
	unsigned int Channel_22;
	unsigned int Channel_23;
	unsigned int Channel_24;
	unsigned int Channel_25;
	unsigned int Channel_26;
	unsigned int Channel_27;
	unsigned int Channel_28;
	unsigned int Channel_29;
	unsigned int Channel_30;
	unsigned int Channel_31;
	unsigned int Channel_32;
	unsigned int Channel_33;
	unsigned int Channel_34;
	unsigned int Channel_35;
	unsigned int Channel_36;
	unsigned int Channel_37;
	unsigned int Channel_38;
	unsigned int Channel_39;
	unsigned int Channel_40;
	unsigned int Channel_41;
	unsigned int Channel_42;
	unsigned int Channel_43;
	unsigned int Channel_44;
	unsigned int Channel_45;
	unsigned int Channel_46;
	unsigned int Channel_47;
	unsigned int Channel_48;
	unsigned int Channel_49;
	unsigned int Channel_50;
	unsigned int Channel_51;
	unsigned int Channel_52;
	unsigned int Channel_53;
	unsigned int Channel_54;
	unsigned int Channel_55;
	unsigned int Channel_56;
	unsigned int Channel_57;
	unsigned int Channel_58;
	unsigned int Channel_59;
	unsigned int Channel_60;
	unsigned int Channel_61;
	unsigned int Channel_62;
	unsigned int Channel_63;
	unsigned int Channel_64;
	unsigned int Channel_65;
	unsigned int Channel_66;
	unsigned int Channel_67;
	unsigned int Channel_68;
	unsigned int Channel_69;
	unsigned int Channel_70;
	unsigned int Channel_71;
	unsigned int Channel_72;
	unsigned int Channel_73;
	unsigned int Channel_74;
	unsigned int Channel_75;
	unsigned int Channel_76;
	unsigned int Channel_77;
	unsigned int Channel_78;
	unsigned int Channel_79;
	unsigned int Channel_80;
	unsigned int Channel_81;
	unsigned int Channel_82;
	unsigned int Channel_83;
	unsigned int Channel_84;	
}DSP_PANEL_PARAMETER;

typedef union
{
	unsigned int array[84];
	DSP_PANEL_PARAMETER	word;
}PANEL_PARA_UNION;

PANEL_PARA_UNION	strCTKw[4];
PANEL_PARA_UNION	RMS[4];
PANEL_PARA_UNION	LoadPer[4];
PANEL_PARA_UNION	strCTCurDemand[4];
PANEL_PARA_UNION	strCTMaxCurDemand[4];
PANEL_PARA_UNION	strCTKwDemand[4];
PANEL_PARA_UNION	strCTMaxKWDemand24hr[4];
PANEL_PARA_UNION	strCTMaxCurDemand24hr[4];
PANEL_PARA_UNION	strCTMaxCurDemand[4];
PANEL_PARA_UNION	strCTMaxKWDemand[4];
PANEL_PARA_UNION	MinCurrent[4];
PANEL_PARA_UNION	MaxCurrent[4];

typedef struct
{
	unsigned int Ambient_Temp;
	unsigned int Ground_Curr;
	unsigned int no_xfmr;							// Transfomer YES/NO Status
	unsigned int Panel1_CT_Start;
	unsigned int Panel1_CT_End;
	unsigned int Panel2_CT_Start;
	unsigned int Panel2_CT_End;
	unsigned int Panel3_CT_Start;
	unsigned int Panel3_CT_End;
	unsigned int Panel4_CT_Start;
	unsigned int Panel4_CT_End;
	unsigned int PanelNo;
	unsigned int CTPanel_No;
	unsigned int Fan12RmsCurrent;
    unsigned int SlaveID;
}SYS_STRUCTURE_PARAMETER;

typedef union
{
	unsigned int array[14];
	SYS_STRUCTURE_PARAMETER 	word;	
}SYS_PARAMETER_UNION;

SYS_PARAMETER_UNION	SysParameter;	

typedef struct
{
	unsigned eeprom_default_flag	:1;
	unsigned epo			:1;
	unsigned repo			:1;
	unsigned epo_key_pressed	:1;
	unsigned repo_key_pressed	:1;
	unsigned restart_after_reset	:1;
	unsigned kwh_calc_in_process	:1;
	unsigned threshold_modify	:1;
	unsigned sci_threshold_modify	:1;
	unsigned eeprom_update		:1;
	unsigned aux_power_fail		:1;
	unsigned transmit_2_lcm		:1;
	unsigned waitSiteUpgradeCmdTxComplete	:1;
	unsigned dsp_fw_upgrade_passthrough	:1;
	unsigned sci2_data_in_buffer		:1;
	unsigned sci2_data_out_buffer		:1;
	
	unsigned pnl1_demand_chk_1sec		:1;
	unsigned pnl1_demand_chk_1hr		:1;
	unsigned pnl1_demand_chk_24hr		:1;
	
	unsigned pnl2_demand_chk_1sec		:1;
	unsigned pnl2_demand_chk_1hr		:1;
	unsigned pnl2_demand_chk_24hr		:1;
	
	unsigned pnl3_demand_chk_1sec		:1;
	unsigned pnl3_demand_chk_1hr		:1;
	unsigned pnl3_demand_chk_24hr		:1;

	unsigned pnl4_demand_chk_1sec		:1;
	unsigned pnl4_demand_chk_1hr		:1;
	unsigned pnl4_demand_chk_24hr		:1;	
}StructFlag;

StructFlag	sFlag;		

#define P1_CAPTURE_MAX_DEMAND		(sFlag.pnl1_demand_chk_24hr = 1)
#define P1_TEST_CAPTURE_MAX_DEMAND	(sFlag.pnl1_demand_chk_24hr == 1)
#define P1_CAPTURE_MAX_DEMAND_DONE	(sFlag.pnl1_demand_chk_24hr = 0)

#define P2_CAPTURE_MAX_DEMAND		(sFlag.pnl2_demand_chk_24hr = 1)
#define P2_TEST_CAPTURE_MAX_DEMAND	(sFlag.pnl2_demand_chk_24hr == 1)
#define P2_CAPTURE_MAX_DEMAND_DONE	(sFlag.pnl2_demand_chk_24hr = 0)

#define P3_CAPTURE_MAX_DEMAND		(sFlag.pnl3_demand_chk_24hr = 1)
#define P3_TEST_CAPTURE_MAX_DEMAND	(sFlag.pnl3_demand_chk_24hr == 1)
#define P3_CAPTURE_MAX_DEMAND_DONE	(sFlag.pnl3_demand_chk_24hr = 0)

#define P4_CAPTURE_MAX_DEMAND		(sFlag.pnl4_demand_chk_24hr = 1)
#define P4_TEST_CAPTURE_MAX_DEMAND	(sFlag.pnl4_demand_chk_24hr == 1)
#define P4_CAPTURE_MAX_DEMAND_DONE	(sFlag.pnl4_demand_chk_24hr = 0)

typedef struct
{
	unsigned int dwPrimary;
	unsigned int dwSecondary_PhaseA;
	unsigned int dwSecondary_PhaseB;
	unsigned int dwSecondary_PhaseC;
	unsigned int dwPanel1_PhaseA;
	unsigned int dwPanel1_PhaseB;
	unsigned int dwPanel1_PhaseC;
	unsigned int dwPanel2_PhaseA;
	unsigned int dwPanel2_PhaseB;
	unsigned int dwPanel2_PhaseC;
	unsigned int dwPanel3_PhaseA;
	unsigned int dwPanel3_PhaseB;
	unsigned int dwPanel3_PhaseC;
	unsigned int dwPanel4_PhaseA;
	unsigned int dwPanel4_PhaseB;
	unsigned int dwPanel4_PhaseC;
}_KWH_ERROR;

_KWH_ERROR	KWH_Error;

typedef struct
{
	unsigned int wPhaseA_UnderVolt;
	unsigned int wPhaseB_UnderVolt;
	unsigned int wPhaseC_UnderVolt;
	unsigned int wPhaseA_OverVolt;
	unsigned int wPhaseB_OverVolt;
	unsigned int wPhaseC_OverVolt;
	unsigned int wPhaseA_OverCurr;
	unsigned int wPhaseB_OverCurr;
	unsigned int wPhaseC_OverCurr;
	unsigned int wNeutral_OverCurr;
	unsigned int wPhaseA_UnderCurr;
	unsigned int wPhaseB_UnderCurr;
	unsigned int wPhaseC_UnderCurr;
	unsigned int wPowerfactor;
	unsigned int wOverVoltTHD;
	unsigned int wOverCurrTHD;
}EE_PRI_SEC_THD;

typedef union
{
	unsigned int array[16];
	EE_PRI_SEC_THD	word;
}EE_PRI_SEC_THD_UNION;

typedef struct
{
	unsigned int wPanel1PhaseA_OverCurr;
	unsigned int wPanel2PhaseA_OverCurr;
	unsigned int wPanel1PhaseB_OverCurr;
	unsigned int wPanel2PhaseB_OverCurr;
	unsigned int wPanel1PhaseC_OverCurr;
	unsigned int wPanel2PhaseC_OverCurr;
	unsigned int wPanel1Neutral_OverCurr;
	unsigned int wPanel2Neutral_OverCurr;
	unsigned int wPanel1PhaseA_UnderCurr;
	unsigned int wPanel2PhaseA_UnderCurr;
	unsigned int wPanel1PhaseB_UnderCurr;
	unsigned int wPanel2PhaseB_UnderCurr;
	unsigned int wPanel1PhaseC_UnderCurr;
	unsigned int wPanel2PhaseC_UnderCurr;
	unsigned int wPanel1_Powerfactor;
	unsigned int wPanel2_Powerfactor;
	unsigned int wPanel1_OverVoltTHD;
	unsigned int wPanel2_OverVoltTHD;
	unsigned int wPanel1_OverCurrTHD;
	unsigned int wPanel2_OverCurrTHD;
}EE_PANEL_CURR_THD;

typedef union
{
	unsigned int	array[EEDATA_PANEL_LEN];
	EE_PANEL_CURR_THD	word;
}EE_PANEL_CURR_THD_UNION;

typedef struct
{
	unsigned int wPassword;
	unsigned int AlarmStatus1;
	unsigned int AlarmStatus2;

	unsigned int wPowerCapacity;
	unsigned int wConfiguration;
	unsigned int wSysConfig;
	unsigned int wNominal_IPVolt;
	unsigned int wNominal_OPVolt;
	unsigned int wFrequency;
	
	unsigned int AlarmStatus3;
	unsigned int AlarmStatus4;
	unsigned int fan_num;
	unsigned int wEpochtimeL;
	unsigned int wEpochtimeH;
	unsigned int wEpochtime_flag;
	unsigned int buzzer_stat;   //0x10F
	
	

	unsigned int wManufactureName[1]; //0x110
//	unsigned int wModel_Name[8];  //0x111 to 0x118
	unsigned int Max_Volt_PhaseA; //111
	unsigned int Max_Volt_PhaseB; //112
	unsigned int Max_Volt_PhaseC; // 113
	unsigned int Min_Volt_PhaseA; //114
	unsigned int Min_Volt_PhaseB; //115
	unsigned int Min_Volt_PhaseC; //116
		
	unsigned int Max_Curr_PhaseA; //117
	unsigned int Max_Curr_PhaseB; //118
	unsigned int Max_Curr_PhaseC; //119
	unsigned int Min_Curr_PhaseA; //11A
	unsigned int Min_Curr_PhaseB; //11B
	unsigned int Min_Curr_PhaseC; //11C

	unsigned int PDC_ID;  //0x11D
	unsigned int sec_total_parameters[5];	//11E 11f 120 121 122	// 0x11E to 0x123

	
	unsigned int wSerial_Num[1];//123
        
	//unsigned int wSys_Fw_Ver[8];//124 - 12B
	unsigned int wSys_Fw_Ver[4];     //124-128
    unsigned int wStack_Fw_Ver[4];   //129-12B
//	unsigned int sec_KWH_Lo;  //12C
//	unsigned int sec_KWH_Hi;
//	unsigned int sec_KW;
//	unsigned int sec_KVA;
//	unsigned int sec_KVAR;
	
//	unsigned int wModbusVersion;//12C
	unsigned int sec_KVAR;  // 135
	unsigned int IPAddress[2];// 136

}EE_SYS_INFO;

typedef union
{
	unsigned int array[56];
	EE_SYS_INFO	word;
}EE_SYS_INFO_UNION;


typedef struct
{
	unsigned int wAmbTempHighLimit;
	unsigned int wGroundCurr;
}EE_SYS_THD;

typedef union
{
	unsigned int array[3];
	EE_SYS_THD	word;
}EE_SYS_THD_UNION;


typedef struct
{
	EE_ACTIVE_INACTIVE_UNION	wPanelAct[NUM_PANEL];
	EE_ACTIVE_INACTIVE_UNION	wPanelCurDemandAct[NUM_PANEL];
	EE_ACTIVE_INACTIVE_UNION	wPanelKwDemandAct[NUM_PANEL];
	EE_SYS_THD_UNION	wSysThd;	// 54
	EE_SYS_INFO_UNION	wSysInfo;	// 3
	EE_PANEL_CURR_THD_UNION wPanelThd[NUM_BOARD];
	//unsigned int wUnderCurrentLimit[NUM_PANEL][NUM_BRANCH];	// [4]*[84] = 336
 	//unsigned int wOverCurrentLimit[NUM_PANEL][NUM_BRANCH];	// [4]*[84] = 336
	unsigned int Max_Min_Limit[NUM_PANEL][NUM_BRANCH];  // [4]*[84] = 336
	unsigned int wUnderCurDemandLimit[NUM_PANEL][NUM_BRANCH]; // [4]*[84] = 336
	unsigned int wOverCurDemandLimit[NUM_PANEL][NUM_BRANCH];  // [4]*[84] = 336
	unsigned int wUnderKwDemandLimit[NUM_PANEL][NUM_BRANCH];  // [4]*[84] = 336
	unsigned int wOverKwDemandLimit[NUM_PANEL][NUM_BRANCH];   // [4]*[84] = 336 
	EE_PRI_SEC_THD_UNION		wPrimary;	// 16
	EE_PRI_SEC_THD_UNION		wSecondary;	// 16
}EE_DATA_STRUCTURE;

typedef union
{
	unsigned int array[EEDATA_LENGTH];	// 54 + 3 + 168 + 168 + 168 + 168 + 168 + 168 + 16 + 16
	EE_DATA_STRUCTURE word;
}EE_DATA_UNION;

EE_DATA_UNION		Data_In;
EE_PRI_SEC_THD_UNION	wPri_Thd;
EE_PRI_SEC_THD_UNION	wSec_Thd;

typedef struct
{
	unsigned long dwChannel1;
	unsigned long dwChannel2;
	unsigned long dwChannel3;
	unsigned long dwChannel4;
	unsigned long dwChannel5;
	unsigned long dwChannel6;
	unsigned long dwChannel7;
	unsigned long dwChannel8;
	unsigned long dwChannel9;
	unsigned long dwChannel10;
	unsigned long dwChannel11;
	unsigned long dwChannel12;
	unsigned long dwChannel13;
	unsigned long dwChannel14;
	unsigned long dwChannel15;
	unsigned long dwChannel16;
	unsigned long dwChannel17;
	unsigned long dwChannel18;
	unsigned long dwChannel19;
	unsigned long dwChannel20;
	unsigned long dwChannel21;
	unsigned long dwChannel22;
	unsigned long dwChannel23;
	unsigned long dwChannel24;
	unsigned long dwChannel25;
	unsigned long dwChannel26;
	unsigned long dwChannel27;
	unsigned long dwChannel28;
	unsigned long dwChannel29;
	unsigned long dwChannel30;
	unsigned long dwChannel31;
	unsigned long dwChannel32;
	unsigned long dwChannel33;
	unsigned long dwChannel34;
	unsigned long dwChannel35;
	unsigned long dwChannel36;
	unsigned long dwChannel37;
	unsigned long dwChannel38;
	unsigned long dwChannel39;
	unsigned long dwChannel40;
	unsigned long dwChannel41;
	unsigned long dwChannel42;
	unsigned long dwChannel43;
	unsigned long dwChannel44;
	unsigned long dwChannel45;
	unsigned long dwChannel46;
	unsigned long dwChannel47;
	unsigned long dwChannel48;
	unsigned long dwChannel49;
	unsigned long dwChannel50;
	unsigned long dwChannel51;
	unsigned long dwChannel52;
	unsigned long dwChannel53;
	unsigned long dwChannel54;
	unsigned long dwChannel55;
	unsigned long dwChannel56;
	unsigned long dwChannel57;
	unsigned long dwChannel58;
	unsigned long dwChannel59;
	unsigned long dwChannel60;
	unsigned long dwChannel61;
	unsigned long dwChannel62;
	unsigned long dwChannel63;
	unsigned long dwChannel64;
	unsigned long dwChannel65;
	unsigned long dwChannel66;
	unsigned long dwChannel67;
	unsigned long dwChannel68;
	unsigned long dwChannel69;
	unsigned long dwChannel70;
	unsigned long dwChannel71;
	unsigned long dwChannel72;
	unsigned long dwChannel73;
	unsigned long dwChannel74;
	unsigned long dwChannel75;
	unsigned long dwChannel76;
	unsigned long dwChannel77;
	unsigned long dwChannel78;
	unsigned long dwChannel79;
	unsigned long dwChannel80;
	unsigned long dwChannel81;
	unsigned long dwChannel82;
	unsigned long dwChannel83;
	unsigned long dwChannel84;	
	/*unsigned long reserved;
	unsigned long reserved1;*/
}CT_LONG_BUFF;


typedef union
{
	unsigned long array[84];
	CT_LONG_BUFF	word;
}CT_LONG_BUFF_UNION;

CT_LONG_BUFF_UNION	strCTKwh_Error[4];
CT_LONG_BUFF_UNION	strCurrDemandSum[4];
CT_LONG_BUFF_UNION	strKwDemandSum[4];

typedef struct
{
	unsigned int wCT1_KWH_High;
	unsigned int wCT1_KWH_Low;

	unsigned int wCT2_KWH_High;
	unsigned int wCT2_KWH_Low;

	unsigned int wCT3_KWH_High;
	unsigned int wCT3_KWH_Low;

	unsigned int wCT4_KWH_High;
	unsigned int wCT4_KWH_Low;

	unsigned int wCT5_KWH_High;
	unsigned int wCT5_KWH_Low;

	unsigned int wCT6_KWH_High;
	unsigned int wCT6_KWH_Low;

	unsigned int wCT7_KWH_High;
	unsigned int wCT7_KWH_Low;

	unsigned int wCT8_KWH_High;
	unsigned int wCT8_KWH_Low;

	unsigned int wCT9_KWH_High;
	unsigned int wCT9_KWH_Low;

	unsigned int wCT10_KWH_High;
	unsigned int wCT10_KWH_Low;

	unsigned int wCT11_KWH_High;
	unsigned int wCT11_KWH_Low;

	unsigned int wCT12_KWH_High;
	unsigned int wCT12_KWH_Low;

	unsigned int wCT13_KWH_High;
	unsigned int wCT13_KWH_Low;

	unsigned int wCT14_KWH_High;
	unsigned int wCT14_KWH_Low;

	unsigned int wCT15_KWH_High;
	unsigned int wCT15_KWH_Low;

	unsigned int wCT16_KWH_High;
	unsigned int wCT16_KWH_Low;

	unsigned int wCT17_KWH_High;
	unsigned int wCT17_KWH_Low;

	unsigned int wCT18_KWH_High;
	unsigned int wCT18_KWH_Low;

	unsigned int wCT19_KWH_High;
	unsigned int wCT19_KWH_Low;

	unsigned int wCT20_KWH_High;
	unsigned int wCT20_KWH_Low;

	unsigned int wCT21_KWH_High;
	unsigned int wCT21_KWH_Low;

	unsigned int wCT22_KWH_High;
	unsigned int wCT22_KWH_Low;

	unsigned int wCT23_KWH_High;
	unsigned int wCT23_KWH_Low;

	unsigned int wCT24_KWH_High;
	unsigned int wCT24_KWH_Low;

	unsigned int wCT25_KWH_High;
	unsigned int wCT25_KWH_Low;

	unsigned int wCT26_KWH_High;
	unsigned int wCT26_KWH_Low;

	unsigned int wCT27_KWH_High;
	unsigned int wCT27_KWH_Low;

	unsigned int wCT28_KWH_High;
	unsigned int wCT28_KWH_Low;

	unsigned int wCT29_KWH_High;
	unsigned int wCT29_KWH_Low;

	unsigned int wCT30_KWH_High;
	unsigned int wCT30_KWH_Low;

	unsigned int wCT31_KWH_High;
	unsigned int wCT31_KWH_Low;

	unsigned int wCT32_KWH_High;
	unsigned int wCT32_KWH_Low;

	unsigned int wCT33_KWH_High;
	unsigned int wCT33_KWH_Low;

	unsigned int wCT34_KWH_High;
	unsigned int wCT34_KWH_Low;

	unsigned int wCT35_KWH_High;
	unsigned int wCT35_KWH_Low;

	unsigned int wCT36_KWH_High;
	unsigned int wCT36_KWH_Low;

	unsigned int wCT37_KWH_High;
	unsigned int wCT37_KWH_Low;

	unsigned int wCT38_KWH_High;
	unsigned int wCT38_KWH_Low;

	unsigned int wCT39_KWH_High;
	unsigned int wCT39_KWH_Low;

	unsigned int wCT40_KWH_High;
	unsigned int wCT40_KWH_Low;

	unsigned int wCT41_KWH_High;
	unsigned int wCT41_KWH_Low;

	unsigned int wCT42_KWH_High;
	unsigned int wCT42_KWH_Low;

	unsigned int wCT43_KWH_High;
	unsigned int wCT43_KWH_Low;

	unsigned int wCT44_KWH_High;
	unsigned int wCT44_KWH_Low;

	unsigned int wCT45_KWH_High;
	unsigned int wCT45_KWH_Low;

	unsigned int wCT46_KWH_High;
	unsigned int wCT46_KWH_Low;

	unsigned int wCT47_KWH_High;
	unsigned int wCT47_KWH_Low;

	unsigned int wCT48_KWH_High;
	unsigned int wCT48_KWH_Low;

	unsigned int wCT49_KWH_High;
	unsigned int wCT49_KWH_Low;

	unsigned int wCT50_KWH_High;
	unsigned int wCT50_KWH_Low;

	unsigned int wCT51_KWH_High;
	unsigned int wCT51_KWH_Low;

	unsigned int wCT52_KWH_High;
	unsigned int wCT52_KWH_Low;

	unsigned int wCT53_KWH_High;
	unsigned int wCT53_KWH_Low;

	unsigned int wCT54_KWH_High;
	unsigned int wCT54_KWH_Low;

	unsigned int wCT55_KWH_High;
	unsigned int wCT55_KWH_Low;

	unsigned int wCT56_KWH_High;
	unsigned int wCT56_KWH_Low;

	unsigned int wCT57_KWH_High;
	unsigned int wCT57_KWH_Low;

	unsigned int wCT58_KWH_High;
	unsigned int wCT58_KWH_Low;

	unsigned int wCT59_KWH_High;
	unsigned int wCT59_KWH_Low;

	unsigned int wCT60_KWH_High;
	unsigned int wCT60_KWH_Low;

	unsigned int wCT61_KWH_High;
	unsigned int wCT61_KWH_Low;

	unsigned int wCT62_KWH_High;
	unsigned int wCT62_KWH_Low;

	unsigned int wCT63_KWH_High;
	unsigned int wCT63_KWH_Low;

	unsigned int wCT64_KWH_High;
	unsigned int wCT64_KWH_Low;

	unsigned int wCT65_KWH_High;
	unsigned int wCT65_KWH_Low;

	unsigned int wCT66_KWH_High;
	unsigned int wCT66_KWH_Low;

	unsigned int wCT67_KWH_High;
	unsigned int wCT67_KWH_Low;

	unsigned int wCT68_KWH_High;
	unsigned int wCT68_KWH_Low;

	unsigned int wCT69_KWH_High;
	unsigned int wCT69_KWH_Low;

	unsigned int wCT70_KWH_High;
	unsigned int wCT70_KWH_Low;

	unsigned int wCT71_KWH_High;
	unsigned int wCT71_KWH_Low;

	unsigned int wCT72_KWH_High;
	unsigned int wCT72_KWH_Low;

	unsigned int wCT73_KWH_High;
	unsigned int wCT73_KWH_Low;

	unsigned int wCT74_KWH_High;
	unsigned int wCT74_KWH_Low;

	unsigned int wCT75_KWH_High;
	unsigned int wCT75_KWH_Low;

	unsigned int wCT76_KWH_High;
	unsigned int wCT76_KWH_Low;

	unsigned int wCT77_KWH_High;
	unsigned int wCT77_KWH_Low;

	unsigned int wCT78_KWH_High;
	unsigned int wCT78_KWH_Low;

	unsigned int wCT79_KWH_High;
	unsigned int wCT79_KWH_Low;

	unsigned int wCT80_KWH_High;
	unsigned int wCT80_KWH_Low;

	unsigned int wCT81_KWH_High;
	unsigned int wCT81_KWH_Low;

	unsigned int wCT82_KWH_High;
	unsigned int wCT82_KWH_Low;

	unsigned int wCT83_KWH_High;
	unsigned int wCT83_KWH_Low;

	unsigned int wCT84_KWH_High;
	unsigned int wCT84_KWH_Low;
	
}DSP_KWH_PANEL1_PARAMETER;

typedef union
{
	unsigned int array[168];
	DSP_KWH_PANEL1_PARAMETER	word;

}PANEL1_KWH_UNION;

PANEL1_KWH_UNION	strCTKwh[3];

typedef struct
{
	PRI_STRUCTURE_PARAMETER Primary; //27
	SEC_STRUCTURE_PARAMETER Secondary; //44
	DSP_PANEL_PARAMETER 	RMS[4]; // 86*4
	DSP_PANEL_PARAMETER 	ctKW[4]; // 86*4
	DSP_PANEL_PARAMETER 	LoadPer[4]; //86*4 
	MAX_MIN_PARAMTER   	Max_Min_Para[1]; // 2*22
	DSP_PANEL_PARAMETER 	strCTMaxCurDemand[4]; // 86*4
	DSP_PANEL_PARAMETER	strCTMaxKWDemand[4];  // 86*4
	DSP_PANEL_PARAMETER 	Maximum[4]; //86*4
	DSP_PANEL_PARAMETER 	Minimum[4];	//86*4
	DSP_PANEL_PARAMETER	strCTCurDemand[4]; //86*4
	DSP_PANEL_PARAMETER	strCTKwDemand[4]; //86*4
	DSP_PANEL_PARAMETER	strCTMaxCurDemand24hr[4]; //86*4
	DSP_PANEL_PARAMETER	strCTMaxKWDemand24hr[4]; //86*4
	PRI_WINDING_STATUS_FLAG			Pri_Status_Flag;	 //2
	SEC_WINDING_STATUS_FLAG			Sec_Status_Flag;	 //2
	CT_CURRENT_STATUS_FLAG			Panel1_Status_Undercurr[4];     // 4*4
	CT_CURRENT_STATUS_FLAG			Panel1_Status_Overcurr[4];      //4*4
	CT_CURRENT_STATUS_FLAG		        Panel2_Status_Undercurr[4];     //4*4
	CT_CURRENT_STATUS_FLAG			Panel2_Status_Overcurr[4];      // 4*4
	DSP_KWH_PANEL1_PARAMETER		KWH[4];                         //4*84*2
	EE_SYS_INFO				Sys;
	FIRMWARE_VERSION_STRUCTURE		Firmware[4];
	SYSTEM_STATUS_UPDATE	                System_Status;
	SYS_STRUCTURE_PARAMETER		        System_Parameter;	
	//SYS_STRUCTURE_PARAMETER Sys_Thd; //6
	//MAX_MIN_PARAMTER    Max_Min_Para; //22 
	//DSP_KWH_PANEL1_PARAMETER Kwh[8]; //84*8
	//EE_ACTIVE_INACTIVE  Active_Inactive;
	//PRI_WINDING_STATUS_FLAG	Pri_Status_Flag;
	//SEC_WINDING_STATUS_FLAG Sec_Status_Flag;
	//CT_CURRENT_STATUS_FLAG	CT_Current_Flag;
	//PANEL1_STATUS_FLAG	
}CAN_DATA;

typedef union 
{
	CAN_DATA word;
	unsigned int array[sizeof(CAN_DATA)/4]; 
}Can_data;

Can_data	Data;

typedef struct
{
	PRI_STRUCTURE_PARAMETER			Primary;
	SEC_STRUCTURE_PARAMETER                 Secondary;
	SYSTEM_STATUS_UPDATE		        System_Status;
	EE_SYS_INFO				Sys;
	FIRMWARE_VERSION_STRUCTURE		Firmware[1];
	SYS_STRUCTURE_PARAMETER		        System_Parameter;
	DSP_PANEL_PARAMETER 	                RMS[2];
	DSP_PANEL_PARAMETER 	                LoadPer[2];
	DSP_PANEL_PARAMETER 	                ctKW[2];
	CT_CURRENT_STATUS_FLAG			Panel1_Status_Undercurr[4];     // 4*4
	CT_CURRENT_STATUS_FLAG			Panel1_Status_Overcurr[4];      //4*4
	CT_CURRENT_STATUS_FLAG		        Panel2_Status_Undercurr[4];     //4*4
	CT_CURRENT_STATUS_FLAG			Panel2_Status_Overcurr[4];      //4*4
	DSP_KWH_PANEL1_PARAMETER		KWH[2];
	DSP_PANEL_PARAMETER 	                curr_gain;
	DSP_PANEL_PARAMETER 	                kw_gain;
	DSP_PANEL_PARAMETER 	                sys_gain; 
	CT_LONG_BUFF	                        strCTKwh_Error[2];
	_KWH_ERROR				KWH_Error;	
}NV_DATA;

typedef union
{
	NV_DATA		word;
	unsigned int array[sizeof(NV_DATA)/4];
}Nv_Data;

Nv_Data			Data_nv;



