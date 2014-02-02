/*
 * Copyright (c) 2006-2013, Ari Suutari <ari@stonepile.fi>.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <picoos.h>
#include <picoos-u.h>

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "mrfi_defs.h"

#include <HAL_TLV.h>

#include "sensor_msg.h"
#include "calib_data.h"

#define MAIN_STACK_SIZE 220
#define IDLE_STACK_SIZE 60

#ifdef MRFI_CC430

// Olimex MSP430-CCRF

#define     LED_RED               BIT0
#define     LED_GREEN             0 // not present
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT

#endif

#ifdef MRFI_CC2500

// TI ez430-RF2500

#define     LED_RED               BIT0
#define     LED_GREEN             BIT1
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT

#endif

extern int __infod[];
unsigned char rf_freqoffset;

static void mainTask(void *arg);
static uint8_t simplicitiCallback(linkID_t);
static void ledBlink(int led, int cnt);
void radioGpioInterrupt(void);

static uint8_t sNumCurrentPeers = 0;
static linkID_t sLID[NUM_CONNECTIONS] ={ 0 };
struct s_TLV_Die_Record* die;
 unsigned char dieSize;
static SensorMsg msgBuf;
static OldSensorMsg* oldMsgBuf;
static POSFLAG_t radioFlag;

#define JOIN_FLAG 0
#define FRAME_FLAG 1
#define IN_A_ROW  3

static uint8_t simplicitiCallback(linkID_t lid)
{
  posFlagSet(radioFlag, (lid) ? FRAME_FLAG : JOIN_FLAG);
  return 0;
}

#ifdef MRFI_CC2500

void PORT_NAKED __attribute__((interrupt(PORT2_VECTOR)))
radioGpioInterrupt()
{
  portSaveContext();
  c_pos_intEnter();

  MRFI_GpioIsr();

  c_pos_intExit();
  portRestoreContext();
}

#endif

static void ledBlink(int led, int cnt)
{
  int i;

  if (led == 0)
    led = LED_RED;

  for (i = 0; i < cnt; i++) {

    if (i > 0)
      posTaskSleep(MS(300));

    LED_OUT |= led;
    posTaskSleep(MS(100));
    LED_OUT &= ~led;
  }
}

static void printSensor(char* name, int16_t id, int16_t value, int16_t multiplier)
{
  nosPrintf(" %s-%d=%s%d", name, id, (value < 0 ? "-" : ""),
                               abs(value) / multiplier);

  if (multiplier >= 10)
    nosPrintf(".%d", abs(value) % multiplier);
}

static void mainTask(void *memstart)
{
  int pass;
  VAR_t flag;
  addr_t myAddr;

  LED_DIR |= LED_RED;
  LED_OUT &= ~LED_RED;

#ifdef MRFI_CC2500
  LED_DIR |= LED_GREEN;
  LED_OUT &= ~LED_GREEN;
#endif

#ifdef MRFI_CC430

// VCore must be set to at least 2 or 
// low voltage error occurrs when radio is enabled.

  uint16_t actlevel;

  actlevel = (PMMCTL0 & PMMCOREV_3);
  if (actlevel < PMMCOREV_2)
    portSetVCore(PMMCOREV_2);

#endif

  nosPrint("AP start.\n");
  uosBootDiag();

  if (__infod[0] == CALIB_DATA_FINGERPRINT) {
    rf_freqoffset = __infod[1];
    nosPrintf("Radio frequency calibration offset is %d\n", (int)rf_freqoffset);
  }

  radioFlag = posFlagCreate();

#ifdef __CC430F5137__

  Get_TLV_Info(TLV_DIERECORD, 0, &dieSize, (unsigned int**) &die);
  if (die != NULL) {

    myAddr.addr[0] = die->die_y_position & 0xff;
    myAddr.addr[1] = die->die_x_position & 0xff;
    myAddr.addr[2] = die->wafer_id & 0xff;
    myAddr.addr[3] = die->wafer_id >> 8;

    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &myAddr);
  }

#endif

  SMPL_Init(simplicitiCallback);
  nosPrintf("SMPL_Init done, pktSize %d.\n", sizeof(msgBuf));
  ledBlink(LED_GREEN, 5);

  pass = 0;
  while (1) {

    flag = posFlagGet(radioFlag, POSFLAG_MODE_GETSINGLE);

    if (flag == JOIN_FLAG && (sNumCurrentPeers < NUM_CONNECTIONS)) {

      nosPrint("Sensor joining.\n");

      // Listen for a new connection.

      while (true) {

        if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
          break;
      }

      sNumCurrentPeers++;
      nosPrint("Sensor joined.\n");
      ledBlink(LED_RED, 3);
    }

    if (flag == FRAME_FLAG) {

      uint8_t len, i, a;

      /* process all frames waiting */
      for (i = 0; i < sNumCurrentPeers; ++i) {

        memset(&msgBuf, '\0', sizeof(msgBuf));
        if (SMPL_SUCCESS == SMPL_Receive(sLID[i], (uint8_t*) &msgBuf, &len)) {

          connInfo_t *info = nwk_getConnInfo(sLID[i]);
          int rssi_int;

          if (!info)
            continue;

          rssi_int = info->sigInfo.rssi;

          nosPrintf("#%04d[", (int)i);
          for (a = 0; a < NET_ADDR_SIZE; a++) {

            if (a)
              nosPrint("-");

            nosPrintf("%02x", (unsigned int)info->peerAddr[a]);
          }

          nosPrintf("]: rssi=%d", rssi_int);
          oldMsgBuf = (OldSensorMsg*)&msgBuf;
          if (oldMsgBuf->pktVersion == 2) {

            // handle old stuff
            nosPrintf(" pktver=%u",   (int)oldMsgBuf->pktVersion);
            printSensor("batt", 0, oldMsgBuf->batteryVoltage, 10);
            printSensor("temp", 0, oldMsgBuf->temperature, 10);
            printSensor("pulse", 0, oldMsgBuf->pulses, 1);
            printSensor("power", 0, oldMsgBuf->power, 1);
          }
          else {

            nosPrintf(" pktver=%u seq=%u",   (int)msgBuf.pktVersion, (unsigned int)msgBuf.seq);

            SensorMsgData* data = msgBuf.data;
            for (a = 0; a < MSG_MAX_SENSOR_VALUES; a++) {

              if (data->type == SENSOR_NULL)
                break;

              switch (data->type) {
              case SENSOR_TEMPERATURE:
                printSensor("temp", data->id, data->value, 10);
                break;

              case SENSOR_BATTVOLTAGE:
                printSensor("batt", data->id, data->value, 10);
                break;

              case SENSOR_PULSES:
                printSensor("pulse", data->id, data->value, 1);
                break;

              case SENSOR_POWER:
                printSensor("power", data->id, data->value, 1);
                break;

              default:
                printSensor("?", data->id, data->value, 1);
                break;
              }

              data++;
            }
          }

          nosPrint("\n");
          ledBlink(LED_GREEN, 1);
        }
      }
    }

    ++pass;
  }

}

/*
 * Initialize board pins.
 */

static void initPins(void)
{

#if PORTCFG_XT1_HZ > 0

#if defined(__cc430x513x)

  P5OUT = 0x00;
  P5SEL |= BIT1 + BIT0;

  UCSCTL6 &= (~XTS);        // Select Low-frequency mode.
  UCSCTL6 |= XCAP_3;        // Internal load cap.

#endif

#endif

#if NOSCFG_FEATURE_CONOUT == 1 || NOSCFG_FEATURE_CONIN == 1

/*
 * Program * TX & RX pin for usart use.
 */

  UCA0CTL1 |= UCSWRST;

#if defined(__msp430x22x4)

  P3DIR |= BIT4;                            // Set P3.4 as TX output
  P3SEL |= 0x30;            // P3.4,5 = USCI_A0 TXD/RXD

#elif defined(__cc430x513x)

  P1DIR |= BIT6;                            // Set P1.6 as TX output
  P1SEL |= BIT5 + BIT6;                     // Select P1.5 & P1.6 to UART function

#endif
#endif
}

int main(int argc, char **argv)
{
  initPins();
  uosInit();
  nosInit(mainTask, NULL, 10, MAIN_STACK_SIZE, IDLE_STACK_SIZE);
  return 0;
}
