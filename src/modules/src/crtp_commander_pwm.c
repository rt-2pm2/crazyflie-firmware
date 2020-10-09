/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>
#include <stdbool.h>

#include "crtp_commander.h"

#include "commander.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "motors.h"
#include "num.h"
#include "log.h"

#include "debug.h"

#define MAX_PWM  65000

static uint32_t motorPower[4];

/**
 * CRTP commander pwm packet format
 */
struct CommanderCrtpPWMValues
{
  uint16_t pwm0;
  uint16_t pwm1;
  uint16_t pwm2;
  uint16_t pwm3;
} __attribute__((packed));


void crtpCommanderPWMDecodeSetpoint(CRTPPacket *pk)
{
  struct CommanderCrtpPWMValues *values = (struct CommanderCrtpPWMValues*)pk->data;

  motorPower[0] = limitUint16(values->pwm0);
  motorPower[1] = limitUint16(values->pwm1);
  motorPower[2] = limitUint16(values->pwm2);
  motorPower[3] = limitUint16(values->pwm3);

  //DEBUG_PRINT("%lu %lu %lu %lu \n",
//		  motorPower[0], motorPower[1], motorPower[2], motorPower[3]);

  motorsSetRatio(MOTOR_M1, motorPower[0]);
  motorsSetRatio(MOTOR_M2, motorPower[1]);
  motorsSetRatio(MOTOR_M3, motorPower[2]);
  motorsSetRatio(MOTOR_M4, motorPower[3]);
}

LOG_GROUP_START(external_pwm)
LOG_ADD(PARAM_UINT32, motorPower0, &motorPower[0])
LOG_ADD(PARAM_UINT32, motorPower1, &motorPower[1])
LOG_ADD(PARAM_UINT32, motorPower2, &motorPower[2])
LOG_ADD(PARAM_UINT32, motorPower3, &motorPower[3])
LOG_GROUP_STOP(external_pwm)
