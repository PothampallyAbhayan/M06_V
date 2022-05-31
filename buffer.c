
#include"buffer.h"



sensor_PKT  *sensor_Rx_pkt_pool_hdr = NULL;

sensor_PKT  *sensor_Tx_pkt_pool_hdr = NULL;

sensor_PKT  *sensor_Rx_pkt_hdr = NULL;
sensor_PKT  *sensor_Tx_pkt_hdr = NULL;


int create_pkt_pool(unsigned int count,unsigned char pkt_type)
{
    sensor_PKT    *p_pkt = NULL, *p_tmp = NULL, *p_tmp2= NULL;
    unsigned int    i;
    

    if(Rx_PKT == pkt_type)
    {
    	p_pkt = p_tmp = &sensor_Rx_buffers[0];
	}
	else if(Tx_PKT == pkt_type)
	{
		p_pkt = p_tmp = &sensor_Tx_buffers[0];
	}
    else{
    	return -1;
	}
    
    p_tmp->frame_no = 0;

    for (i=0;i<(count);i++) {
    	
        p_tmp2 = (p_pkt+i+1);
        p_tmp->next = p_tmp2;
        p_tmp = p_tmp2;
        p_tmp->frame_no = i+1;
    }
    p_tmp->next = NULL;
    
    if(Rx_PKT == pkt_type )
    {
    	sensor_Rx_pkt_pool_hdr = p_pkt;
	}
	else if(Tx_PKT == pkt_type)
	{
		sensor_Tx_pkt_pool_hdr = p_pkt;
	}
	else{
		return -1;
	}
    return(0);
}


sensor_PKT * release_pkt_from_free_pool(unsigned char pkt_type)
{
	sensor_PKT  *p_tmp;
	if(Rx_PKT == pkt_type)
	{
		if (sensor_Rx_pkt_pool_hdr == NULL)
		{
                  printf("Packet-A not released \n");
		  return (NULL);

		}
	    
	    p_tmp = sensor_Rx_pkt_pool_hdr;
	    sensor_Rx_pkt_pool_hdr = sensor_Rx_pkt_pool_hdr->next;
	}
	else if (Tx_PKT == pkt_type){
		if (sensor_Tx_pkt_pool_hdr == NULL)
                {
                  printf("Packet-B not released \n");
	          return (NULL);
                }
	    p_tmp = sensor_Rx_pkt_pool_hdr;
	    sensor_Tx_pkt_pool_hdr = sensor_Tx_pkt_pool_hdr->next;
	}
	else
	{
          printf("Packet0 not released \n");
          return (NULL);
	}
	p_tmp->next = NULL;
        //printf("Allocated Pkt Descriptor \n");
	return (p_tmp);
}

sensor_PKT * append_pkt_to_free_pool(sensor_PKT *p_pkt,unsigned char pkt_type)
{
  if(p_pkt != NULL)//DK added for NULL check
  {
    if (Rx_PKT == pkt_type){
	  if (sensor_Rx_pkt_pool_hdr == NULL) {
      sensor_Rx_pkt_pool_hdr = p_pkt;
      sensor_Rx_pkt_pool_hdr->next = NULL;
      return sensor_Tx_pkt_pool_hdr;
      }

	  p_pkt->next = sensor_Rx_pkt_pool_hdr;
	  sensor_Rx_pkt_pool_hdr = p_pkt;
    }
    else if(Tx_PKT == pkt_type)
    {
	  if (sensor_Tx_pkt_pool_hdr == NULL) {
        sensor_Tx_pkt_pool_hdr = p_pkt;
        sensor_Tx_pkt_pool_hdr->next = NULL;
        return sensor_Tx_pkt_pool_hdr;
      }

      p_pkt->next = sensor_Tx_pkt_pool_hdr;
      sensor_Tx_pkt_pool_hdr = p_pkt;
	
    }
  }
    
  return sensor_Tx_pkt_pool_hdr;
}


int enqueue_pkt(sensor_PKT *p_pkt, unsigned char pkt_type)
{
  sensor_PKT *temp_p_pkt;
  
  if(Rx_PKT == pkt_type)
  {
  	temp_p_pkt = sensor_Rx_pkt_hdr;
  }
  else if(Tx_PKT == pkt_type) 
  {
    temp_p_pkt = sensor_Tx_pkt_hdr;	
  }
  
  if (temp_p_pkt == NULL) 
  {
    temp_p_pkt = p_pkt;
      	
    if(Rx_PKT == pkt_type)
    {
  	  sensor_Rx_pkt_hdr = temp_p_pkt ;
    }
    else if(Tx_PKT == pkt_type) 
    {
      sensor_Tx_pkt_hdr = temp_p_pkt ;	
    }
   
  }
  else
  {

	while(temp_p_pkt->next != NULL)
	{
	  temp_p_pkt = temp_p_pkt->next;
	}
	
	temp_p_pkt->next = p_pkt;
        p_pkt->next = NULL;
  }
  return 0;
}



sensor_PKT * dequeue_pkt(unsigned char pkt_type)
{
  sensor_PKT  *temp_p_pkt;
    
  if(Rx_PKT == pkt_type)
  {
    temp_p_pkt = sensor_Rx_pkt_hdr  ;
  }
  else if(Tx_PKT == pkt_type) 
  {
    temp_p_pkt = sensor_Tx_pkt_hdr  ;	
  }

  if (temp_p_pkt == NULL)
  {

	return (NULL);
  }
  else{
    if(Rx_PKT == pkt_type)
	{
		sensor_Rx_pkt_hdr = sensor_Rx_pkt_hdr->next;
	}
	else if(Tx_PKT == pkt_type)
	{
		sensor_Tx_pkt_hdr = sensor_Tx_pkt_hdr->next;
	}
  }

  return (temp_p_pkt);
}


void clear (void)
{    
  while ( getchar() != '\n' );
}
sensor_PKT *pkt_ptr;

void Pool_count(void)
{
  int count = 0;
  sensor_PKT *temp_ptr = sensor_Rx_pkt_pool_hdr;
  if(temp_ptr)
  {
	while(temp_ptr->next != NULL) 
	{
	  count++;
	  temp_ptr = temp_ptr->next;
	}
	count++;
	//printf(" Rx pool count is %d\n",count);
	  
  }
  else
  {
	//printf("No Rx packet to count\n");
  }
  count = 0;
  temp_ptr = sensor_Tx_pkt_hdr;
  if(temp_ptr)
  {
	if(temp_ptr != NULL)
	{
	  count++;
	  while(temp_ptr->next != NULL)
	  {
		count++;
		temp_ptr = temp_ptr->next;
	  } 
	}
	
	  printf("Tx pool count is %d\n",count);
  }
  else
  {
	printf("No Tx packet to count\n");
  }
	
}
sensor_PKT * release_pkt_from_pool(void)
{
  sensor_PKT *temp_pkt_ptr;
  temp_pkt_ptr = release_pkt_from_free_pool(Rx_PKT);	
  if(temp_pkt_ptr ==  NULL)
  {
    //printf("No Rx packet to release\n");
  }
  return temp_pkt_ptr;

}

void process_pkt(void)
{

  if(pkt_ptr) 
  {
    enqueue_pkt(pkt_ptr,Tx_PKT);
    pkt_ptr = NULL;
//    transmit_pkt();
  }
  else{
	printf("No packet to Process\n");
  }
    
}
void release_pkt_to_pool(void)
{
  sensor_PKT *temp_ptr  = dequeue_pkt(Tx_PKT);
  if(temp_ptr != NULL)
  {
	append_pkt_to_free_pool(temp_ptr,Rx_PKT);
  }
  else
  {
	printf("No packet to Process\n");
  }
   
}

void display_pkt (void)
{
  sensor_PKT *temp_ptr  = sensor_Tx_pkt_hdr;
  if(temp_ptr)
  {
    while(temp_ptr->next != NULL)
    {
 	 
 	  temp_ptr = temp_ptr->next;
    }
   

  }
  
  temp_ptr  = sensor_Rx_pkt_pool_hdr;
  if(temp_ptr)
  {
	while(temp_ptr->next != NULL){
 	
 	temp_ptr = temp_ptr->next;
 	
    }
    
  }
 
}

int display_Rx_pkt_count (void)
{
  sensor_PKT *temp_ptr = NULL; 
  int count = 0; 
  /*temp_ptr = sensor_Tx_pkt_hdr;
  if(temp_ptr)
  {
    while(temp_ptr->next != NULL)
    {
 	 
 	  temp_ptr = temp_ptr->next;
    }
   

  }*/
  
  temp_ptr  = sensor_Rx_pkt_pool_hdr;
  if(temp_ptr)
  {
	while(temp_ptr->next != NULL){
 	
 	temp_ptr = temp_ptr->next;
        count++;
 	
    }
    
  }
 return count;
}




