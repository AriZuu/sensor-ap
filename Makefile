#
# Copyright (c) 2012-2013, Ari Suutari <ari@stonepile.fi>.
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote
#     products derived from this software without specific prior written
#     permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

RELROOT = ../picoos/

PORT = msp430
#
# This is for TI ez430-rf2500 kit
#
#MCU ?= msp430f2274
#
# This is for Olimex MSP430-CCRF card
#
MCU ?= cc430f5137 

ifeq '$(MCU)' 'msp430f2274'

RADIO=MRFI_CC2500 NUM_CONNECTIONS=1 SIZE_INFRAME_Q=1
TEXAS_CONF=../sensor-ap/texas

else

RADIO=MRFI_CC430 NUM_CONNECTIONS=3 SIZE_INFRAME_Q=2
TEXAS_CONF=../sensor-ap/texas_ccrf

endif

export MCU

#
# SimpliciTI settings. Address must be unique.
#
SIMPLICITI_DEFINES =  SIZE_OUTFRAME_Q=1 \
	THIS_DEVICE_ADDRESS="{{0x78,0x56,0x34,0x12}}" \
	ACCESS_POINT \
	AP_IS_DATA_HUB \
	NUM_STORE_AND_FWD_CLIENTS=2 \
	STARTUP_JOINCONTEXT_ON \
	MAX_HOPS=2 \
	MAX_HOPS_FROM_AP=1 \
	MAX_NWK_PAYLOAD=9 \
	MAX_APP_PAYLOAD=18 \
	DEFAULT_LINK_TOKEN=0x01020304 \
	DEFAULT_JOIN_TOKEN=0x05060708 \
	APP_AUTO_ACK \
	ISM_EU LINK_SPEED=1 \
	$(RADIO) 

export SIMPLICITI_DEFINES

BUILD ?= DEBUG

include $(RELROOT)make/common.mak

NANO = 1
TARGET = ap
TEXAS_SRC=../simpliciti
export TEXAS_CONF
#
SRC_TXT =	ap_main.c
SRC_HDR =
SRC_OBJ =
SRC_LIB =
CDEFINES += $(SIMPLICITI_DEFINES)

DIR_USRINC += $(TEXAS_CONF)  \
		$(TEXAS_SRC)/bsp/ \
		$(TEXAS_SRC)/mrfi/ \
		$(TEXAS_SRC)/simpliciti/nwk/ \
		$(TEXAS_SRC)/simpliciti/nwk_applications/ \
		../picoos-micro ../picoos-micro/ports/msp430/hal5xx6xx

DIR_CONFIG = $(CURRENTDIR)
DIR_OUTPUT = $(CURRENTDIR)/bin
MODULES += ../simpliciti ../picoos-micro
EXEC_MAKEFILES += $(TEXAS_CONF)/bsp_external/Makefile

POSTLINK1 = msp430-size $(TARGETOUT)

# ---------------------------------------------------------------------------
# BUILD THE EXECUTABLE
# ---------------------------------------------------------------------------

include $(MAKE_OUT)

#
# Extra stuff to build distribution zip.
#
dist:
	rm -f ../dist/sensor-ap-`date +%Y%m%d`.zip
	cd ..; zip -qr dist/sensor-ap-`date +%Y%m%d`.zip sensor-ap -x "*/.*" \
		"*/bin/*" "*.launch" "*.hex" \
		"*/texas/bsp_external/*.c" "*/texas/bsp_external/*.h" \
		"*/texas_ccrf/bsp_external/*.h"

#
# Create patch for bsp_external files.
#
ORIG_BSP =	../simpliciti/bsp/boards
bsp-diff:
	cd texas/bsp_external; $(MAKE) bsp-diff
	cd texas_ccrf/bsp_external; $(MAKE) bsp-diff
