
#ifndef _BUFFER_H_
#define _BUFFER_H_


#include <stdio.h>
#include <stdlib.h>
//if 0
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

//#endif
#define Rx_PKT 1
#define Tx_PKT 2
#define MAX_POOL_BUFFERS    500
#define position(x) (0<= (x-1) <5 ? x:0)
typedef struct __attribute__((__packed__)) _sensor_pkt
{
	struct can_frame sensor_data[21*4];
	unsigned char pool_position;
	unsigned char frame_no;
	struct _sensor_pkt	*next;
} sensor_PKT;

extern sensor_PKT  sensor_Rx_buffers[MAX_POOL_BUFFERS];
extern sensor_PKT  sensor_Tx_buffers[1]; 

extern int create_pkt_pool(unsigned int ,unsigned char  );
extern sensor_PKT * release_pkt_from_free_pool(unsigned char );
extern sensor_PKT* append_pkt_to_free_pool(sensor_PKT *,unsigned char );
extern int enqueue_pkt(sensor_PKT *, unsigned char );
extern sensor_PKT * dequeue_pkt(unsigned char );
extern void display_pkt (void);
extern void release_pkt_to_pool(void);
extern void process_pkt(void);
extern sensor_PKT * release_pkt_from_pool(void);//void release_pkt_from_pool(void);
extern void transmit_pkt(void );

#endif /* _BUFFER_H_ */



