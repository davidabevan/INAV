/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_SERIALRX_CRSF) && defined(USE_TELEMETRY_CRSF)

#include "build/build_config.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/crc.h"
#include "common/streambuf.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/crsf.h"
#include "rx/rx.h"

#include "sensors/battery.h"

#include "telemetry/crsf.h"
#include "telemetry/telemetry.h"


#define CRSF_CYCLETIME_US                   100000  // 100ms, 10 Hz

// According to TBS: "CRSF over serial should always use a sync byte at the beginning of each frame.
// To get better performance it's recommended to use the sync byte 0xC8 to get better performance"
//
// Digitalentity: Using frame address byte as a sync field looks somewhat hacky to me, but seems it's needed to get CRSF working properly
#define CRSF_TELEMETRY_SYNC_BYTE            0xC8

static bool crsfTelemetryEnabled;
static uint8_t crsfCrc;
static uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX];

static void crsfInitializeFrame(sbuf_t *dst)
{
    crsfCrc = 0;
    dst->ptr = crsfFrame;
    dst->end = ARRAYEND(crsfFrame);

    sbufWriteU8(dst, CRSF_TELEMETRY_SYNC_BYTE);
}

static void crsfSerialize8(sbuf_t *dst, uint8_t v)
{
    sbufWriteU8(dst, v);
    crsfCrc = crc8_dvb_s2(crsfCrc, v);
}

static void crsfSerialize16(sbuf_t *dst, uint16_t v)
{
    // Use BigEndian format
    crsfSerialize8(dst,  (v >> 8));
    crsfSerialize8(dst, (uint8_t)v);
}

static void crsfSerialize32(sbuf_t *dst, uint32_t v)
{
    // Use BigEndian format
    crsfSerialize8(dst, (v >> 24));
    crsfSerialize8(dst, (v >> 16));
    crsfSerialize8(dst, (v >> 8));
    crsfSerialize8(dst, (uint8_t)v);
}

static void crsfSerializeData(sbuf_t *dst, const uint8_t *data, int len)
{
    for (int ii = 0; ii< len; ++ii) {
        crsfSerialize8(dst, data[ii]);
    }
}

static void crsfFinalize(sbuf_t *dst)
{
    sbufWriteU8(dst, crsfCrc);
    sbufSwitchToReader(dst, crsfFrame);
    // write the telemetry frame to the receiver.
    crsfRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
}

static int crsfFinalizeBuf(sbuf_t *dst, uint8_t *frame)
{
    sbufWriteU8(dst, crsfCrc);
    sbufSwitchToReader(dst, crsfFrame);
    const int frameSize = sbufBytesRemaining(dst);
    for (int ii = 0; sbufBytesRemaining(dst); ++ii) {
        frame[ii] = sbufReadU8(dst);
    }
    return frameSize;
}

/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t)
*/

/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­ 1000m offset )
uint8_t     Satellites in use ( counter )
*/
void crsfFrameGps(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsfSerialize8(dst, CRSF_FRAMETYPE_GPS);
    crsfSerialize32(dst, gpsSol.llh.lat); // CRSF and betaflight use same units for degrees
    crsfSerialize32(dst, gpsSol.llh.lon);
    crsfSerialize16(dst, (gpsSol.groundSpeed * 36 + 50) / 100); // gpsSol.groundSpeed is in cm/s
    crsfSerialize16(dst, DECIDEGREES_TO_CENTIDEGREES(gpsSol.groundCourse)); // gpsSol.groundCourse is 0.1 degrees, need 0.01 deg
    const uint16_t altitude = (getEstimatedActualPosition(Z) / 100) + 1000;
    crsfSerialize16(dst, altitude);
    crsfSerialize8(dst, gpsSol.numSat);
}

/*
0x08 Battery sensor
Payload:
uint16_t    Voltage ( mV * 100 )
uint16_t    Current ( mA * 100 )
uint24_t    Capacity ( mAh )
uint8_t     Battery remaining ( percent )
*/
void crsfFrameBatterySensor(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsfSerialize8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR);
    crsfSerialize16(dst, getBatteryVoltage() / 10); // vbat is in units of 0.01V
    crsfSerialize16(dst, getAmperage() / 10);
    const uint8_t batteryRemainingPercentage = calculateBatteryPercentage();
    crsfSerialize8(dst, (getMAhDrawn() >> 16));
    crsfSerialize8(dst, (getMAhDrawn() >> 8));
    crsfSerialize8(dst, (uint8_t)getMAhDrawn());
    crsfSerialize8(dst, batteryRemainingPercentage);
}

typedef enum {
    CRSF_ACTIVE_ANTENNA1 = 0,
    CRSF_ACTIVE_ANTENNA2 = 1
} crsfActiveAntenna_e;

typedef enum {
    CRSF_RF_MODE_4_HZ = 0,
    CRSF_RF_MODE_50_HZ = 1,
    CRSF_RF_MODE_150_HZ = 2
} crsrRfMode_e;

typedef enum {
    CRSF_RF_POWER_0_mW = 0,
    CRSF_RF_POWER_10_mW = 1,
    CRSF_RF_POWER_25_mW = 2,
    CRSF_RF_POWER_100_mW = 3,
    CRSF_RF_POWER_500_mW = 4,
    CRSF_RF_POWER_1000_mW = 5,
    CRSF_RF_POWER_2000_mW = 6
} crsrRfPower_e;

/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/

#define DECIDEGREES_TO_RADIANS10000(angle) ((int16_t)(1000.0f * (angle) * RAD))

void crsfFrameAttitude(sbuf_t *dst)
{
     sbufWriteU8(dst, CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
     crsfSerialize8(dst, CRSF_FRAMETYPE_ATTITUDE);
     crsfSerialize16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.pitch));
     crsfSerialize16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.roll));
     crsfSerialize16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.yaw));
}

/*
0x21 Flight mode text based
Payload:
char[]      Flight mode ( Null­terminated string )
*/
void crsfFrameFlightMode(sbuf_t *dst)
{
    // just do "OK" for the moment as a placeholder
    // write zero for frame length, since we don't know it yet
    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    crsfSerialize8(dst, CRSF_FRAMETYPE_FLIGHT_MODE);

    // use same logic as OSD, so telemetry displays same flight text as OSD when armed
    const char *flightMode = "OK";
    if (ARMING_FLAG(ARMED)) {
        if (isAirmodeActive()) {
            flightMode = "AIR";
        } else {
            flightMode = "ACRO";
        }
        if (FLIGHT_MODE(FAILSAFE_MODE)) {
            flightMode = "!FS!";
        } else if (FLIGHT_MODE(MANUAL_MODE)) {
            flightMode = "MANU";
        } else if (FLIGHT_MODE(NAV_RTH_MODE)) {
            flightMode = "RTH";
        } else if (FLIGHT_MODE(NAV_POSHOLD_MODE)) {
            flightMode = "HOLD";
        } else if (FLIGHT_MODE(NAV_CRUISE_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE)) {
            flightMode = "3CRS";
        } else if (FLIGHT_MODE(NAV_CRUISE_MODE)) {
            flightMode = "CRS";
        } else if (FLIGHT_MODE(NAV_ALTHOLD_MODE) && navigationRequiresAngleMode()) {
            flightMode = "AH";
        } else if (FLIGHT_MODE(NAV_WP_MODE)) {
            flightMode = "WP";
        } else if (FLIGHT_MODE(ANGLE_MODE)) {
            flightMode = "ANGL";
        } else if (FLIGHT_MODE(HORIZON_MODE)) {
            flightMode = "HOR";
        }
#ifdef USE_GPS
    } else if (feature(FEATURE_GPS) && (!STATE(GPS_FIX) || !STATE(GPS_FIX_HOME))) {
        flightMode = "WAIT"; // Waiting for GPS lock
#endif
    } else if (isArmingDisabled()) {
        flightMode = "!ERR";
    }

    crsfSerializeData(dst, (const uint8_t*)flightMode, strlen(flightMode));
    crsfSerialize8(dst, 0); // zero terminator for string
    // write in the length
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

#define BV(x)  (1 << (x)) // bit value

// schedule array to decide how often each type of frame is sent
#define CRSF_SCHEDULE_COUNT_MAX     5
static uint8_t crsfScheduleCount;
static uint8_t crsfSchedule[CRSF_SCHEDULE_COUNT_MAX];

static void processCrsf(void)
{
    static uint8_t crsfScheduleIndex = 0;
    const uint8_t currentSchedule = crsfSchedule[crsfScheduleIndex];

    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

    if (currentSchedule & BV(CRSF_FRAME_ATTITUDE)) {
        crsfInitializeFrame(dst);
        crsfFrameAttitude(dst);
        crsfFinalize(dst);
    }
    if (currentSchedule & BV(CRSF_FRAME_BATTERY_SENSOR)) {
        crsfInitializeFrame(dst);
        crsfFrameBatterySensor(dst);
        crsfFinalize(dst);
    }
    if (currentSchedule & BV(CRSF_FRAME_FLIGHT_MODE)) {
        crsfInitializeFrame(dst);
        crsfFrameFlightMode(dst);
        crsfFinalize(dst);
    }
#ifdef USE_GPS
    if (currentSchedule & BV(CRSF_FRAME_GPS)) {
        crsfInitializeFrame(dst);
        crsfFrameGps(dst);
        crsfFinalize(dst);
    }
#endif
    crsfScheduleIndex = (crsfScheduleIndex + 1) % crsfScheduleCount;
}

void initCrsfTelemetry(void)
{
    // check if there is a serial port open for CRSF telemetry (ie opened by the CRSF RX)
    // and feature is enabled, if so, set CRSF telemetry enabled
    crsfTelemetryEnabled = crsfRxIsActive();
    int index = 0;
    crsfSchedule[index++] = BV(CRSF_FRAME_ATTITUDE);
    crsfSchedule[index++] = BV(CRSF_FRAME_BATTERY_SENSOR);
    crsfSchedule[index++] = BV(CRSF_FRAME_FLIGHT_MODE);
    if (feature(FEATURE_GPS)) {
        crsfSchedule[index++] = BV(CRSF_FRAME_GPS);
    }
    crsfScheduleCount = (uint8_t)index;

 }

bool checkCrsfTelemetryState(void)
{
    return crsfTelemetryEnabled;
}

/*
 * Called periodically by the scheduler
 */
void handleCrsfTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t crsfLastCycleTime;

    if (!crsfTelemetryEnabled) {
        return;
    }
    // Give the receiver a chance to send any outstanding telemetry data.
    // This needs to be done at high frequency, to enable the RX to send the telemetry frame
    // in between the RX frames.
    crsfRxSendTelemetryData();

    // Actual telemetry data only needs to be sent at a low frequency, ie 10Hz
    if (currentTimeUs >= crsfLastCycleTime + CRSF_CYCLETIME_US) {
        crsfLastCycleTime = currentTimeUs;
        processCrsf();
    }
}

int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType)
{
    sbuf_t crsfFrameBuf;
    sbuf_t *sbuf = &crsfFrameBuf;

    crsfInitializeFrame(sbuf);
    switch (frameType) {
    default:
    case CRSF_FRAME_ATTITUDE:
        crsfFrameAttitude(sbuf);
        break;
    case CRSF_FRAME_BATTERY_SENSOR:
        crsfFrameBatterySensor(sbuf);
        break;
    case CRSF_FRAME_FLIGHT_MODE:
        crsfFrameFlightMode(sbuf);
        break;
#if defined(USE_GPS)
    case CRSF_FRAME_GPS:
        crsfFrameGps(sbuf);
        break;
#endif
    }
    const int frameSize = crsfFinalizeBuf(sbuf, frame);
    return frameSize;
}
#endif
