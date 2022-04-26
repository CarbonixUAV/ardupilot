/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* ????? Should above License agreement be modified with Carbonix License ????? */

#pragma once


#include "AP_EFI.h"
#include "AP_EFI_Backend.h"


#define EFI_PACKET_SYNC0 0x57            // First synchronization byte for a EFI packet
#define EFI_PACKET_SYNC0EX 0x75          // First synchronization byte for a special EFI packet (sdcard record)
#define EFI_PACKET_SYNC1 0xAC            // Second synchronization byte for a EFI packet
#define EFI_PACKET_SYNC1EX 0xC8          // Second synchronization byte for a special EFI packet (uplink recording)
#define EFI_PACKET_SIZE 1023             // An EFI packet can transport 1023 bytes
#define EFI_PACKET_HEADER 4              // An EFI packet has 4 bytes for the header
#define EFI_PACKET_OVERHEAD 8            // An EFI packet has 8 bytes of overhead
#define EFI_PACKET_TYPEBITS 6            // The number of bits for the type field of an EFI packet
#define EFI_PKT_CRC_SIZE 4               // CRC packet size
#define EFI_PKT_TELEMETRYFASTSUM 35      // Fast telemetry summary
#define EFI_PKT_TELEMETRYSLOWSUM 36      // Slow telemetry summary
#define FAST_TELEMETRY_PKT_SIZE 16       // Always
#define SLOW_TELEMETRY_PKT_SIZE 128      // Always
#define FAST_TELEMETRY_REFRESH_PERIOD 50 // 20Hz
#define SLOW_TELEMETRY_REFRESH_PERIOD 50 // 20Hz
#define RPM_SCALING_FACTOR 4.0           // Scaled by 4 from 0.0 to 16383.75.
#define THROTTLE_SCALING_FACTOR 2.0      // Scaled by 2 from 0.0 to 127.5.
#define RPM_THRESHOLD 100

#define BITMASK_BYTE 0xFF
#define BITMASK_NIBBLE 0xF

#define POSMAT 24
#define POSMAP1 32
#define POSMAP2 36
#define POSBARO1 44
#define POSBARO2 48
#define POSOAT 56
#define POSFUELCONSUMED1 16
#define POSFUELCONSUMED2 24
#define POSFUELP1 48
#define POSFUELP2 56

#define NIBBLE 4
#define BYTE 8

#define AIRPRESSURE_SCALE_FACTOR 31.25


/*!
 * Structure for Fast Telemetry Data
 */
typedef struct
{
    uint32_t time;         // Time information for the fast telemetry
    uint32_t reserved;     // Reserved bits for future expansion.
    uint8_t rpm1;          // Engine speed in revolutions per minute. Scaled by 4 from 0.0 to 16383.75.
    uint8_t rpm2;          // contd...
    uint8_t throttleSrc;   // Source of the throttle command information
    uint8_t throttleCmd;   // Throttle command (0 to 100%) going in.
    uint8_t throttlePos;   // Throttle position (0 to 100%)
    uint8_t injector3Duty; // The third injector duty cycle in percent.
    uint8_t injector1Duty; // The first injector duty cycle in percent.
    uint8_t injector2Duty; // The second injector duty cycle in percent.
} fastTelemetryPkt_t;


/*!
 * Structure for Slow Telemetry Data
 */
typedef struct
{
    uint64_t time;
    uint64_t sensors;
    uint64_t sensors2;
    uint64_t fuel;
    uint64_t injector;
    uint64_t slow_telemetry;
    uint64_t cpu;
    uint64_t sensor3;
    uint64_t dynamic_error;
    uint64_t sticky_error;
    uint64_t wear;
    uint64_t wear_extend;
    uint64_t comms;
    uint64_t sensor4;
    uint64_t sdcard;
    uint64_t extended_outputs;
} slowTelemetryPkt_t;


/*!
 * Data mapping between rawBytes and Telemetry packets
 */
typedef union
{
    uint8_t rawBytes[SLOW_TELEMETRY_PKT_SIZE]; // Raw bytes - max size
    fastTelemetryPkt_t fastInfo;               // Fast Telemetry Data mapping
    slowTelemetryPkt_t slowInfo;               // Slow Telemetry Data mapping
} rawData_t;


/*!
 * General structure of an IntelliJect EFI packet
 */
typedef struct
{
    uint8_t sync0;    // First synchronization byte to indicate a packet may be forthcoming
    uint8_t sync1;    // Second synchronization byte, and upper 2 bits of size
    uint16_t size;    // Lower 8 bits of packet size
    uint8_t type;     // 6 bit packet type, upper 2 bits reserved
    rawData_t data;
    uint32_t CRC1;
    uint32_t CRC2;
    uint32_t CRC3;
    uint32_t CRC4;
    uint16_t rxstate; // Receive state for processing a receive packet byte by byte, not part of the transmission
} efiPacket_t;

typedef unsigned float_bits;


/*!
 * Power4Flight IntelliJect EFI class definition
 */
class AP_EFI_Serial_IJ: public AP_EFI_Backend
{
public:
    /* Constructor with initialization */
    AP_EFI_Serial_IJ(AP_EFI &_frontend);

    /* To update the state structure */
    void update() override;

private:
    AP_HAL::UARTDriver *port;

    efiPacket_t efiPacket;

    // efi packet size
    //int efiMaxPktSize = sizeof(efiPacket) + FAST_TELEMETRY_PKT_SIZE + EFI_PKT_CRC_SIZE;
    int efiMaxPktSize = sizeof(efiPacket) + FAST_TELEMETRY_PKT_SIZE + EFI_PKT_CRC_SIZE;

    // periodic refresh 
    uint32_t last_response_ms; 

    // primary function to read IntelliJect EFI Packets
    bool read_intelliJectEfiPackets();
    bool read_intelliJectEfiPackets(int bytes_read);

    // for retrieving RPM and Throttle values
    bool decode_intelliJectTelemetryFastPacket();

    // for retrieving Fuel, Pressure and Temperature values
    bool decode_intelliJectTelemetrySlowPacket();


    float get_16bit_Float(uint16_t);
};