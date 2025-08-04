###################################################################################
#                                                                                 #
# Makefile for Linux                                                              #
#                                                                                 #
###################################################################################

ROOT=.
#TARGET_OBJ_DIR = $(ROOT)/build/obj_$(NOS_KERNEL)
#TARGET_BIN_DIR = $(ROOT)/$(NOS_BIN_DIR)
#TARGET_LIB_DIR = $(ROOT)/$(NOS_LIB_DIR)

# C compiler used is gcc

#CC = arm-cortexa8-linux-gnueabi-gcc
#cc = gcc
CC = arm-poky-linux-gnueabi-gcc

INCLUDE_DIR = -I $(ROOT)/include	                                           \
              -I ./libmodbus/include/modbus/                                  \
              -I ./bacnet-stack-0.8.7/include/                                \
              -I ./bacnet-stack-0.8.7/demo/object/                            \
              -I ./bacnet-stack-0.8.7/ports/linux/
CFLAGS = -O0 -ggdb3

#LIBS        = -L$(ROOT)/libs                                                \
#              -Wl,-rpath-link=$(TARGET_LIB_DIR)                             \
#             -lrt -l pthread                                               \
#              -L ./libmodbus/lib/                                           \
#              -l modbus                                                     \
#              -L ./bacnet-stack-0.8.6/lib/                                  \
#              -lbacnet
LIBS        = -L$(ROOT)/libs                                                \
              -Wl,-rpath-link=$(TARGET_LIB_DIR)                             \
              -lm                                                           \
              -lrt -l pthread                                               \
              -L ./libmodbus/lib/                                           \
              -lmodbus                                                      \
              -L./path/to/libs                                              \
              -licosmbusdb                                                   \
              -lmosquitto                                                     \
              -lnetsnmp                                                     \
              -lnetsnmpagent                                                    \
              -lnosdb                                                      \
              -lnoshci                                                       \
              -lnosinfra                                               \
              -lnoslibc                                                     \
              -lnosmqtt                                                     \
              -lnossnmp                                                     \
              -lnosdrivers                                                  \
              -lsqlite3                                                     \
              -li2c                                                         \
              -L ./bacnet-stack-0.8.7/lib/                                  \
              -lbacnet
#%.o: %.c $(DEPS)
#	$(CC) $(CFLAGS)  -g -O -c $? $< $(CFLAGS)
 
 %.o: %.c $(DEPS)
	$(CC) $(CFLAGS) $(INCLUDE_DIR) -g -O -c $? $< $(CFLAGS)

#can_mgr: can_mgr.o buffer.o
#	$(CC) -o M_06B.linux buffer.o can_mgr.o $(LIBS)
can_mgr: can_mgr.o buffer.o ai.o bacfile.o bacmain.o device.o nc.o schedule.o trendlog.o
	$(CC) $(CFLAGS) -o M_06C_T1.linux buffer.o can_mgr.o ai.o bacfile.o bacmain.o device.o nc.o \
       schedule.o trendlog.o $(LIBS)

clean:
	rm -rf M_06C_T1.linux
	rm -rf *.o
