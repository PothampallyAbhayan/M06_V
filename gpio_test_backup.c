/*
 * gpio_test.c
 *
 *
 * Author    : CommsBU
 * Ver       : 1.0
 * Date      : 11-Mar-2016
 *
 * Middleware utility to configure GPIO manager.
 *
 * Copyright SFO Technologies Ltd, 2015-16
 *
 */

#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <getopt.h>

#include "nos_types.h"
//#include "nos_config.h"
//#include "nos_gpiomgr_ipc.h"
//#include "nos_errno.h"
#include "nos_i2c.h"
#include "support.h"
#include "PDC.h"
#include "string.h"
#include "stdio.h"
#include "netinet/in.h"
#include "arpa/inet.h"

#define TRUE                          1
#define FALSE                         0
sensor_PKT  sensor_Rx_buffers[MAX_POOL_BUFFERS];
sensor_PKT  sensor_Tx_buffers[MAX_POOL_BUFFERS]; 
typedef unsigned char  boolean;

static void help(int status, const char *argv0);

static const char *escape_chars = "\"\\'";

/*
 * The following are the various options of this CLI
 *
 * -n                : Port number
 * -d                : Direction 
 * -m                : Mode 
 * -f                : Polling frequency 
 * -i                : Interrupt type 
 * -c                : Channel name 
 * -s                : Display port statistics 
 * -o                : Option (set/unset) 
 * -h                : Help string
 * -t                : Test Option
 */

static const char *shortopts = "n:d:m:f:i:c:s:o:h:t:";

/*
 * Print help message
 */

static void help(int status, const char *argv0)
{
    const char *name = argv0;

    printf("Usage: %s [options]\n", name);
    printf("\tUtility for Testing GPIO Ports \n");
}

static void icos_test_gpio(void)
{

    nos_i2c_bus_open(0);
    sleep(1);

    nos_i2c_device_open(0, 0x43);
    sleep(1);

    icos_io_init(0, 0x43);
    sleep(1);

    icos_io_set_direction(0, 0x43, 0x3F, 0x00);
    sleep(1);

    icos_io_set_port(0, 0x43, 0, 1);
    sleep(1);

}

/*
 * The parameters for the DIO configuration is the following.
 *
 */
int main(int argc, char **argv)
{
    icos_test_gpio();
    printf("\n write finished\n");
    return 0;
}
