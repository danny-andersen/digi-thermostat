OS=LINUX
SOC=BCM2835
DRIVER=RPi
CPUFLAGS=-march=armv6zk -mtune=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard
CFLAGS=-march=armv6zk -mtune=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard -Ofast -Wall -pthread 
PREFIX=/usr/local
LIB=rf24
LIBS=-lrf24 -lrf24network
LIBDEPRECATE=librf24-bcm.so
CC=arm-linux-gnueabihf-gcc
CXX=arm-linux-gnueabihf-g++
LIB_DIR=/usr/local/lib
INC_HEADER_DIR=-I/usr/local/include/RF24 -I/usr/local/include/RF24Network 
DRIVER_DIR=utility/RPi
ARCH_DIR=utility
SHARED_LINKER_FLAGS= -pthread -shared -Wl,-soname,librf24.so.
LDCONFIG=ldconfig
REMOTE_LDCONFIG=/sbin/ldconfig
