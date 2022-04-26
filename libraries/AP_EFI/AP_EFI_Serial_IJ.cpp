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
#include "AP_EFI_Serial_IJ.h"
#include "stdio.h"

#include "GCS_MAVLink/GCS.h"

#if HAL_EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>


extern const AP_HAL::HAL &hal;

uint8_t rawBuffer[2048] = {0};


AP_EFI_Serial_IJ::AP_EFI_Serial_IJ(AP_EFI &_frontend) : AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}


void AP_EFI_Serial_IJ::update()
{
    if (port != nullptr)
    {
        if (port->available() >= efiMaxPktSize)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "IJ bytes:%d", (int)port->available());

            int bytes_read = port->read(rawBuffer, port->available());
            if (read_intelliJectEfiPackets(bytes_read))
            {
                uint32_t now = AP_HAL::millis();
                internal_state.last_updated_ms = last_response_ms = now;
                copy_to_frontend();
            }
        }
    }
}


bool AP_EFI_Serial_IJ::read_intelliJectEfiPackets(int bytes_read)
{
	bool status = false;
	
	for ( int idx = 0; idx < bytes_read; idx++)
	{
		efiPacket.sync0 = rawBuffer[idx++];
		efiPacket.sync1 = rawBuffer[idx++] & 0xFC;
	
		if (efiPacket.sync0 == EFI_PACKET_SYNC0 && efiPacket.sync1 == EFI_PACKET_SYNC1)
		{
			uint8_t sizeLo = rawBuffer[idx++];
			efiPacket.size = ((efiPacket.sync1 & 0x03) << 8) + sizeLo;
			efiPacket.type = rawBuffer[idx++];
			
			for (int id_data = 0; id_data < efiPacket.size; id_data++)
			{
				efiPacket.data.rawBytes[id_data] = rawBuffer[idx++];
			}
			
            idx += 16;
			status = true;
		}
			
		if (status && efiPacket.type == EFI_PKT_TELEMETRYSLOWSUM)
		{
			status = decode_intelliJectTelemetrySlowPacket();
		}
		else if (status && efiPacket.type == EFI_PKT_TELEMETRYFASTSUM)
		{
			status = decode_intelliJectTelemetryFastPacket();
		}
		idx--;
	}
	return status;
}


/*!
 * \return status of decode algorithm [Currently always returns TRUE].
 */
bool AP_EFI_Serial_IJ::decode_intelliJectTelemetryFastPacket()
{
    bool status = true;

    internal_state.engine_speed_rpm = ((efiPacket.data.fastInfo.rpm1 << BYTE) | (efiPacket.data.fastInfo.rpm2 & BITMASK_BYTE)) / RPM_SCALING_FACTOR;
    internal_state.throttle_position_percent = efiPacket.data.fastInfo.throttlePos / THROTTLE_SCALING_FACTOR;

    // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "IJ Fast r:%d p:%d", (int)internal_state.engine_speed_rpm, (int)internal_state.throttle_position_percent);

    return status;
}



/*!
 * \return status of decode algorithm [Currently always returns TRUE].
 */
bool AP_EFI_Serial_IJ::decode_intelliJectTelemetrySlowPacket()
{
    bool status = true;

    signed char temp_cht = (signed char)(efiPacket.data.slowInfo.sensors & BITMASK_BYTE);
    signed char temp_mat = (signed char)((efiPacket.data.slowInfo.sensors >> POSMAT) & BITMASK_BYTE);
    
    /* C_TO_KELVIN is added since Mavlink subtracts this value. To retain consistency this code is modified */
    internal_state.cylinder_status[0].cylinder_head_temperature = C_TO_KELVIN((float)temp_cht);
    internal_state.intake_manifold_temperature = C_TO_KELVIN((float)temp_mat);
    
    internal_state.intake_manifold_pressure_kpa = ((((efiPacket.data.slowInfo.sensors >> POSMAP1) & BITMASK_BYTE) << NIBBLE) | ((efiPacket.data.slowInfo.sensors >> POSMAP2) & BITMASK_NIBBLE)) / AIRPRESSURE_SCALE_FACTOR;
    internal_state.atmospheric_pressure_kpa = ((((efiPacket.data.slowInfo.sensors >> POSBARO1) & BITMASK_NIBBLE) << BYTE) | ((efiPacket.data.slowInfo.sensors >> POSBARO2) & BITMASK_BYTE)) / AIRPRESSURE_SCALE_FACTOR;
    internal_state.cylinder_status[0].exhaust_gas_temperature = (efiPacket.data.slowInfo.sensors >> POSOAT) & BITMASK_BYTE;
    internal_state.estimated_consumed_fuel_volume_cm3 = get_16bit_Float((((efiPacket.data.slowInfo.fuel >> POSFUELCONSUMED1) & BITMASK_BYTE) << BYTE) | ((efiPacket.data.slowInfo.fuel >> POSFUELCONSUMED2) & BITMASK_BYTE));
    internal_state.fuel_consumption_rate_cm3pm = get_16bit_Float(((efiPacket.data.slowInfo.fuel & BITMASK_BYTE) << BYTE) | ((efiPacket.data.slowInfo.fuel >> BYTE) & BITMASK_BYTE));
    internal_state.fuel_pressure = get_16bit_Float((((efiPacket.data.slowInfo.sensor4 >> POSFUELP1) & BITMASK_BYTE) << BYTE) | ((efiPacket.data.slowInfo.sensor4 >> POSFUELP2) & BITMASK_BYTE));
    
    // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "IJ Slow T1:%f T2:%f P1:%f P2:%f", internal_state.cylinder_status[0].cylinder_head_temperature, internal_state.intake_manifold_temperature, internal_state.intake_manifold_pressure_kpa, internal_state.atmospheric_pressure_kpa);
    // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "IJ Slow T3:%f F1:%f F2:%f P3:%f", internal_state.cylinder_status[0].exhaust_gas_temperature, internal_state.estimated_consumed_fuel_volume_cm3, internal_state.fuel_consumption_rate_cm3pm, internal_state.fuel_pressure);

    return status;
}


/*!
 * \return status of decode algorithm [Currently always returns TRUE].
 */
float AP_EFI_Serial_IJ::get_16bit_Float(uint16_t value)
{
    union FP32
    {
        uint32_t u;
        float f;
    };

    const union FP32 magic = { (254UL - 15UL) << 23U };
    const union FP32 was_inf_nan = { (127UL + 16UL) << 23U };
    union FP32 out;

    out.u = (value & 0x7FFFU) << 13U;
    out.f *= magic.f;
    if (out.f >= was_inf_nan.f)
    {
        out.u |= 255UL << 23U;
    }
    out.u |= (value & 0x8000UL) << 16U;

    return out.f;
}


#endif // HAL_EFI_ENABLED
