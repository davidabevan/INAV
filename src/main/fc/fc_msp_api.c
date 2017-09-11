/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Konstantin Sharlaimov <konstantin.sharlaimov@gmail.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/bitarray.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/compass/compass.h"
#include "drivers/max7456.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_msp.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/hil.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/serial_4way.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

static const char * const flightControllerIdentifier = INAV_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

/* MSP command handler context */
typedef struct {
    uint16_t cmd;
    uint8_t flags;
    sbuf_t * src;
    sbuf_t * dst;
    mspPostProcessFnPtr * mspPostProcessFn;
} mspHandlerContext_t;

/* MSP command handlers implementation */
static mspCommandResult_t mspHandlerMSP_API_VERSION(mspHandlerContext_t * ctx)
{
    sbufWriteU8(ctx->dst, MSP_PROTOCOL_VERSION);
    sbufWriteU8(ctx->dst, API_VERSION_MAJOR);
    sbufWriteU8(ctx->dst, API_VERSION_MINOR);
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_FC_VARIANT(mspHandlerContext_t * ctx)
{
    sbufWriteData(ctx->dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_FC_VERSION(mspHandlerContext_t * ctx)
{
    sbufWriteU8(ctx->dst, FC_VERSION_MAJOR);
    sbufWriteU8(ctx->dst, FC_VERSION_MINOR);
    sbufWriteU8(ctx->dst, FC_VERSION_PATCH_LEVEL);
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_BOARD_INFO(mspHandlerContext_t * ctx)
{
    sbufWriteData(ctx->dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
    sbufWriteU16(ctx->dst, hardwareRevision);
#else
    sbufWriteU16(ctx->dst, 0);
#endif
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_BUILD_INFO(mspHandlerContext_t * ctx)
{
    sbufWriteData(ctx->dst, buildDate, BUILD_DATE_LENGTH);
    sbufWriteData(ctx->dst, buildTime, BUILD_TIME_LENGTH);
    sbufWriteData(ctx->dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
    return MSP_RES_SUCCESS;
}

#ifdef HIL
static mspCommandResult_t mspHandlerMSP_HIL_STATE(mspHandlerContext_t * ctx)
{
    sbufWriteU16(ctx->dst, hilToSIM.pidCommand[ROLL]);
    sbufWriteU16(ctx->dst, hilToSIM.pidCommand[PITCH]);
    sbufWriteU16(ctx->dst, hilToSIM.pidCommand[YAW]);
    sbufWriteU16(ctx->dst, hilToSIM.pidCommand[THROTTLE]);
    return MSP_RES_SUCCESS;
}
#endif

static mspCommandResult_t mspHandlerMSP_SENSOR_STATUS(mspHandlerContext_t * ctx)
{
    sbufWriteU8(ctx->dst, isHardwareHealthy() ? 1 : 0);
    sbufWriteU8(ctx->dst, getHwGyroStatus());
    sbufWriteU8(ctx->dst, getHwAccelerometerStatus());
    sbufWriteU8(ctx->dst, getHwCompassStatus());
    sbufWriteU8(ctx->dst, getHwBarometerStatus());
    sbufWriteU8(ctx->dst, getHwGPSStatus());
    sbufWriteU8(ctx->dst, getHwRangefinderStatus());
    sbufWriteU8(ctx->dst, getHwPitotmeterStatus());
    sbufWriteU8(ctx->dst, HW_SENSOR_NONE);                   // Optical flow
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_MOTOR(mspHandlerContext_t * ctx)
{
    for (unsigned i = 0; i < 8; i++) {
        sbufWriteU16(ctx->dst, i < MAX_SUPPORTED_MOTORS ? motor[i] : 0);
    }
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_RC(mspHandlerContext_t * ctx)
{
    for (int i = 0; i < rxRuntimeConfig.channelCount; i++) {
        sbufWriteU16(ctx->dst, rcData[i]);
    }
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_RTC(mspHandlerContext_t * ctx)
{
    int32_t seconds = 0;
    uint16_t millis = 0;
    rtcTime_t rtcTime;
    if (rtcGet(&rtcTime)) {
        seconds = rtcTimeGetSeconds(&rtcTime);
        millis = rtcTimeGetMillis(&rtcTime);
    }
    sbufWriteU32(ctx->dst, (uint32_t)seconds);
    sbufWriteU16(ctx->dst, millis);
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP2_COMMON_TZ(mspHandlerContext_t * ctx)
{
    sbufWriteU16(ctx->dst, (uint16_t)timeConfig()->tz_offset);
    return MSP_RES_SUCCESS;
}



static mspCommandResult_t mspHandlerMSP_SET_LOOP_TIME(mspHandlerContext_t * ctx)
{
    gyroConfigMutable()->looptime = sbufReadU16(ctx->src);
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP_SET_RTC(mspHandlerContext_t * ctx)
{
    // Use seconds and milliseconds to make senders
    // easier to implement. Generating a 64 bit value
    // might not be trivial in some platforms.
    int32_t secs = (int32_t)sbufReadU32(ctx->src);
    uint16_t millis = sbufReadU16(ctx->src);
    rtcTime_t rtcTime = rtcTimeMake(secs, millis);
    rtcSet(&rtcTime);
    return MSP_RES_SUCCESS;
}

static mspCommandResult_t mspHandlerMSP2_COMMON_SET_TZ(mspHandlerContext_t * ctx)
{
    timeConfigMutable()->tz_offset = (int16_t)sbufReadU16(ctx->src);
    return MSP_RES_SUCCESS;
}

/* MSP handler table */
typedef mspCommandResult_t (*mspCommandHandlerPtr_t)(mspHandlerContext_t * ctx);

typedef struct {
    uint16_t cmd;                       // MSP API command code
    uint16_t len;                       // MSP incoming command length
    mspCommandHandlerPtr_t handler;     // Pointer to handler function
} mspCommandTableEntry_t;

const mspCommandTableEntry_t mspApiTable[] = {
    { MSP_API_VERSION,              0,  mspHandlerMSP_API_VERSION },
    { MSP_FC_VARIANT,               0,  mspHandlerMSP_FC_VARIANT },
    { MSP_FC_VERSION,               0,  mspHandlerMSP_FC_VERSION },
    { MSP_BOARD_INFO,               0,  mspHandlerMSP_BOARD_INFO },
    { MSP_BUILD_INFO,               0,  mspHandlerMSP_BUILD_INFO },
#ifdef HIL
    { MSP_HIL_STATE,                0,  mspHandlerMSP_HIL_STATE },
#endif
    { MSP_SENSOR_STATUS,            0,  mspHandlerMSP_SENSOR_STATUS },
    { MSP_MOTOR,                    0,  mspHandlerMSP_MOTOR },
    { MSP_RC,                       0,  mspHandlerMSP_RC },
    { MSP_RTC,                      0,  mspHandlerMSP_RTC },
    { MSP2_COMMON_TZ,               0,  mspHandlerMSP2_COMMON_TZ },

    { MSP_SET_LOOP_TIME,            2,  mspHandlerMSP_SET_LOOP_TIME },
    { MSP_SET_RTC,                  6,  mspHandlerMSP_SET_RTC },
    { MSP2_COMMON_SET_TZ,           2,  mspHandlerMSP2_COMMON_SET_TZ },
};

mspCommandResult_t mspFcProcessAPICommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    // Set up reply CMD and flags
    reply->cmd = cmd->cmd;
    reply->flags = 0;

    // Find a command 
    const uint16_t inDataSize = sbufBytesRemaining(&cmd->buf);
    for (unsigned i = 0; i < ARRAYLEN(mspApiTable); i++) {
        if (mspApiTable[i].cmd == cmd->cmd && mspApiTable[i].len == inDataSize) {
            mspHandlerContext_t cmdCtx = {
                .cmd = cmd->cmd,
                .flags = cmd->flags,
                .src = &cmd->buf,
                .dst = &reply->buf,
                .mspPostProcessFn = mspPostProcessFn
            };

            mspCommandResult_t res = mspApiTable[i].handler(&cmdCtx);

            if (res == MSP_RES_SUCCESS || res == MSP_RES_SUCCESS_NO_REPLY) {
                reply->flags = cmdCtx.flags;
            }

            return res;
        }
    }

    return MSP_RES_NOT_IMPLEMENTED;
}
