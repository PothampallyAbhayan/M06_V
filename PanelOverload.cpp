	#include<stdio.h>
	
	#define PANEL1_PHASEA  0
	#define PANEL1_PHASEB  PANEL1_PHASEA + 1
	#define PANEL1_PHASEC  PANEL1_PHASEB + 1
	
	int main()
	{
		unsigned int eventflag[70]={0};
		int alarmflag = 1, i;
		int panel_count = 0;
		unsigned int alarmstatus = 0;
		int wPDU_Parameters[3] = {0};
		int wPhaseA_PanelCurrent[3] = {0};
		int eventflag[50] = {0};
		
		wPDU_Parameters[0] = 1666;
		wPDU_Parameters[1] = 1666;
		wPDU_Parameters[2] = 1666;
		int phase = 1;
		 
		for(i = 0;i < 3;i++)
		{
			printf("PANEL1_PHASE%d :",phase);
			scanf("%d",wPhaseA_PanelCurrent[0]);
			phase++;
		}
		
	    
	    
	    for(; ;)
	    {
	    	//checks for Overload
			if((wPhaseA_PanelCurrent[0]/wPDU_Parameters[0]))
			{
			panel_count++;
			if (panel_count == 60)
			{
			 if (!event_flag[43])     
			  {
			   //dummy1 = 43;
			
			   //write(fd5[1],&dummy1,1);
			   event_flag[43] = 1;
			   alarm_status |= (1 << PANEL1_PHASEA);
			   alarm_flag = 0;
			
			  }
			}
			if((wPhaseA_PanelCurrent[1]/wPDU_Parameters[1]))
			{
			panel_count++;
			if (panel_count == 60)
			{
			 if (!event_flag[44])     
			  {
			   //dummy1 = 43;
			
			   //write(fd5[1],&dummy1,1);
			   event_flag[44] = 1;
			   alarm_status |= (1 << PANEL1_PHASEB);
			   alarm_flag = 0;
			
			  }
			}
			if((wPhaseA_PanelCurrent[2]/wPDU_Parameters[2]))
			{
			panel_count++;
			if (panel_count == 60)
			{
			 if (!event_flag[45])     
			  {
			   //dummy1 = 43;
			
			   //write(fd5[1],&dummy1,1);
			   event_flag[45] = 1;
			   alarm_status |= (1 << PANEL1_PHASEC);
			   alarm_flag = 0;
			
			  }
			}
			
			//Alarmbeep and event status
			if(!alarm_flag)
			{
				printf("---------------------------------------Alarmbeeps---------------------------------------------");
				if(eventflag[43] == 1)
				{
					printf("Panel1_PhaseA_Overload\n");
				}
				if(eventflag[43] == 1)
				{
					printf("Panel1_PhaseB_Overload\n");
				}
				if(eventflag[43] == 1)
				{
					printf("Panel1_PhaseC_Overload\n");
				}
				
			}
			
			
			
		    
	    }
	} 
