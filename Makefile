#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#
OTA_HOST ?= 192.168.4.1

IDF_PATH=$(PWD)/ESP8266_RTOS_SDK

PROJECT_NAME := blackmagic

#CFLAGS = -Og -Wno-error=maybe-uninitialized
#CPPFLAGS = -Og

include $(IDF_PATH)/make/project.mk

tftpflash: build/$(PROJECT_NAME).bin
	tftp -v -m octet $(OTA_HOST) -c put build/$(PROJECT_NAME).bin firmware.bin
