/*
 * Copyright (c) 2012-2013, Ari Suutari <ari@stonepile.fi>.
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

typedef struct __attribute__((packed))
{

#define SENSOR_NULL         0
#define	SENSOR_TEMPERATURE	1
#define SENSOR_BATTVOLTAGE	2
#define SENSOR_PULSES		3
#define SENSOR_POWER		4

  uint8_t id;
  uint8_t type;
  int16_t value;
} SensorMsgData;

#define MSG_MAX_SENSOR_VALUES 4

typedef struct __attribute__((packed))
{
  uint8_t        pktVersion; // 3 now
  uint8_t        seq;
  SensorMsgData  data[4];
} SensorMsg;

typedef struct __attribute__((packed))
{
  int16_t   temperature;
  uint16_t  rawTemperature;
  uint16_t  cal30;
  uint16_t  cal85;
  uint8_t   batteryVoltage;
  uint8_t   noAckCount;
  uint8_t   pktVersion; // 2 now
  uint16_t  pulses; // wh
  uint16_t  power; // watts
} OldSensorMsg;
