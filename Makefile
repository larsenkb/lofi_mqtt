#############################################################################
#
# Makefile for lofi_mqtt nRF24L01+ receiver on Odroid or Raspberry Pi
#   Uses "wiringPi" see http://wiringpi.com
#   Uses "libmosquitto" see https://mosqujitto.org
#
# Description:
# ------------
# use make all and make install to install the examples
# You can change the install directory by editing the prefix line
#
# $@   The file name of the target
# $%   The target member name
# $<   The name of the first prerequisite
# $?   The names of all the prerequisites that are newer than the target
# $^   The names of all the prerequisites
#
prefix := ~/bin

# Detect the Raspberry Pi by the existence of the bcm_host.h file
BCMLOC=/opt/vc/include/bcm_host.h

ifneq ("$(wildcard $(BCMLOC))","")
# The recommended compiler flags for the Raspberry Pi
CCFLAGS=-Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s
endif

# define all programs
PROGRAMS = lofi_mqtt
#SOURCES = ${PROGRAMS:=.cpp}

all: ${PROGRAMS}

lofi_mqtt: lofi_mqtt.c
	gcc ${CCFLAGS} -Wall -lwiringPi -lmosquitto -lpthread -o $@ $@.c

tags:
	ctags lofi_mqtt.c 

clean:
	rm -rf $(PROGRAMS) tags

install: all
	test -d $(prefix) || mkdir $(prefix)
	for prog in $(PROGRAMS); do \
	  install -m 0755 $$prog $(prefix); \
	done

.PHONY: install tags
