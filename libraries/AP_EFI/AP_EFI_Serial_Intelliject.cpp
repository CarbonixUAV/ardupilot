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

#include <AP_HAL/AP_HAL.h>
#include "AP_EFI_Serial_Intelliject.h"
//#include "stdio.h"

#include "GCS_MAVLink/GCS.h"

#if HAL_EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>


// extern const AP_HAL::HAL &hal;

uint8_t raw_buffer[2048] = {0};


AP_EFI_Serial_Intelliject::AP_EFI_Serial_Intelliject(AP_EFI &_frontend) : AP_EFI_Backend(_frontend){
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}


void AP_EFI_Serial_Intelliject::update()
{
    if (port != nullptr) {
        if (port->available() >= efi_max_pkt_size) {
            int bytes_read = port->read(raw_buffer, port->available());
            if (read_intelliJect_efi_packets(bytes_read)) {
                uint32_t now = AP_HAL::millis();
                internal_state.last_updated_ms = last_response_ms = now;
                copy_to_frontend();
            }
        }
    }
}


bool AP_EFI_Serial_Intelliject::read_intelliJect_efi_packets(int bytes_read)
{
    bool status = false;

    uint32_t received_CRC;

    for (int idx = 0; idx < bytes_read; idx++) {
        computed_checksum = 0xFFFFFFFFul;

        efi_packet.sync0 = get_byte_CRC32(raw_buffer[idx++]);
        efi_packet.sync1 = get_byte_CRC32(raw_buffer[idx++]) & 0xFC;

        if (efi_packet.sync0 == EFI_PACKET_SYNC0 && efi_packet.sync1 == EFI_PACKET_SYNC1) {
            uint8_t sizeLo = get_byte_CRC32(raw_buffer[idx++]);
            efi_packet.size = ((efi_packet.sync1 & 0x03) << 8) + sizeLo;
            efi_packet.type = get_byte_CRC32(raw_buffer[idx++]);

            for (int id_data = 0; id_data < efi_packet.size; id_data++) {
                efi_packet.data.raw_bytes[id_data] = get_byte_CRC32(raw_buffer[idx++]);
            }

            received_CRC = raw_buffer[idx++] << 24;
            received_CRC += raw_buffer[idx++] << 16;
            received_CRC += raw_buffer[idx++] << 8;
            received_CRC += raw_buffer[idx++];

            idx--;

            status = (received_CRC != computed_checksum) ? false : true;
        }

        if (status && efi_packet.type == EFI_PKT_TELEMETRY_SLOW_SUM) {
            status = decode_intelliJect_telemetry_slow_packet();
        } else if (status && efi_packet.type == EFI_PKT_TELEMETRY_FAST_SUM) {
            status = decode_intelliJect_telemetry_fast_packet();
        }
    }
    return status;
}


/*!
 * \return status of decode algorithm [Currently always returns TRUE].
 */
bool AP_EFI_Serial_Intelliject::decode_intelliJect_telemetry_fast_packet()
{
    bool status = true;

    internal_state.engine_speed_rpm = ((efi_packet.data.fast_info.rpm1 << BYTE) | (efi_packet.data.fast_info.rpm2 & BITMASK_BYTE)) / RPM_SCALING_FACTOR;
    internal_state.throttle_position_percent = efi_packet.data.fast_info.throttle_pos / THROTTLE_SCALING_FACTOR;

    return status;
}


/*!
 * \return status of decode algorithm [Currently always returns TRUE].
 */
bool AP_EFI_Serial_Intelliject::decode_intelliJect_telemetry_slow_packet()
{
    bool status = true;

    signed char temp_cht = (signed char)(efi_packet.data.slow_info.sensors & BITMASK_BYTE);
    signed char temp_mat = (signed char)((efi_packet.data.slow_info.sensors >> POS_MAT) & BITMASK_BYTE);

    /* C_TO_KELVIN is added since Mavlink subtracts this value. To retain consistency this code is modified */
    internal_state.cylinder_status[0].cylinder_head_temperature = C_TO_KELVIN((float)temp_cht);
    internal_state.intake_manifold_temperature = C_TO_KELVIN((float)temp_mat);

    internal_state.intake_manifold_pressure_kpa = ((((efi_packet.data.slow_info.sensors >> POS_MAP1) & BITMASK_BYTE) << NIBBLE) | ((efi_packet.data.slow_info.sensors >> POS_MAP2) & BITMASK_NIBBLE)) / AIRPRESSURE_SCALE_FACTOR;
    internal_state.atmospheric_pressure_kpa = ((((efi_packet.data.slow_info.sensors >> POS_BARO1) & BITMASK_NIBBLE) << BYTE) | ((efi_packet.data.slow_info.sensors >> POS_BARO2) & BITMASK_BYTE)) / AIRPRESSURE_SCALE_FACTOR;
    internal_state.cylinder_status[0].exhaust_gas_temperature = (efi_packet.data.slow_info.sensors >> POS_OAT) & BITMASK_BYTE;
    internal_state.estimated_consumed_fuel_volume_cm3 = get_16bit_Float((((efi_packet.data.slow_info.fuel >> POS_FUEL_CONSUMED1) & BITMASK_BYTE) << BYTE) | ((efi_packet.data.slow_info.fuel >> POS_FUEL_CONSUMED2) & BITMASK_BYTE));
    internal_state.fuel_consumption_rate_cm3pm = get_16bit_Float(((efi_packet.data.slow_info.fuel & BITMASK_BYTE) << BYTE) | ((efi_packet.data.slow_info.fuel >> BYTE) & BITMASK_BYTE));
    internal_state.fuel_pressure = get_16bit_Float((((efi_packet.data.slow_info.sensor4 >> POS_FUEL_P1) & BITMASK_BYTE) << BYTE) | ((efi_packet.data.slow_info.sensor4 >> POS_FUEL_P2) & BITMASK_BYTE));

    return status;
}


uint8_t AP_EFI_Serial_Intelliject::get_byte_CRC32(uint8_t data)
{
    computed_checksum = CRC32_compute_byte(data);

    return data;
}


// CRC32 matching
uint32_t AP_EFI_Serial_Intelliject::CRC32_compute_byte(uint8_t byte)
{
    uint32_t reg = computed_checksum;
    
    return (reg<<8) ^ crc_table[(reg >> 24) ^ byte];
}


/*!
 * \return status of decode algorithm [Currently always returns TRUE].
 */
float AP_EFI_Serial_Intelliject::get_16bit_Float(uint16_t value)
{
    union FP32 {
        uint32_t u;
        float f;
    };

    const union FP32 magic = { (254UL - 15UL) << 23U };
    const union FP32 was_inf_nan = { (127UL + 16UL) << 23U };
    union FP32 out;

    out.u = (value & 0x7FFFU) << 13U;
    out.f *= magic.f;
    if (out.f >= was_inf_nan.f) {
        out.u |= 255UL << 23U;
    }
    out.u |= (value & 0x8000UL) << 16U;

    return out.f;
}


#endif // HAL_EFI_ENABLED
