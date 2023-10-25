#include "SIM_BattMonitor_AD7091R5.h"

#include "SITL.h"
#include <SITL/SIM_Aircraft.h>

// Registers Values for Config 
#define AD7091R5_RESET           0x02
#define AD7091R5_CH_ID(x)        ((x >> 5) & 0x03)
#define AD7091R5_RES_MASK        0x0F
#define AD7091R5_REF             3.3f
#define AD7091R5_RESOLUTION      (float)4096
#define AD7091R5_PERIOD_USEC     100000
#define AD7091R5_BASE_PIN        50
#define AD7091R5_CONF_CMD        0x04
#define AD7091R5_CHAN_ALL        0x0F
#define AD7091R5_CONF_PDOWN0     0x00

// Setup ADC output and configurations. Format to send to IC is (Register, MSB, LSB)
SITL::SIM_BattMonitor_AD7091R5::init()
{

    // set up registers for read/write    
    add_register("RESULT", AD7091R5DevReg::RESULT, 2, I2CRegisters::RegMode::RDONLY); //16 bit register
    add_register("CHANNEL", AD7091R5DevReg::CHANNEL, 1, I2CRegisters::RegMode::RDWR); //8 bit register
    add_register("CONFIG", AD7091R5DevReg::CONFIG, 2, I2CRegisters::RegMode::RDWR); //16 bit register

    // initial reset after power on required for correct operation
    reset();    
}

int SITL::SIM_BattMonitor_AD7091R5::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{   
    // TODO
    return I2CRegisters_ConfigurableLength::rdwr(data);    
}

// Update the SITL aircraft battery voltage and current states
// Voltage Pin is 50
// Current Pin is 51 and 52 (two channel option) set in parameters
// Get ADC conversion from all three channels by reading the CONVERSION RESULT Register

// Look at configuration register to obtain the 'fake' ADC readings to process
void SITL::SIM_BattMonitor_AD7091R5::update(const class Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_update_ms < 100) { // 10Hz
        return;
    }

    last_update_ms = now_ms;

    //apply calibration to obtain actual voltage and current from the converted results from IC
    if ((get_register(AD7091R5DevReg::CONFIG) != ((AD7091R5_CONF_CMD << 8) | AD7091R5_CONF_PDOWN0))) {

        return;
    }

    if (!(get_register(AD7091R5DevReg::CHANNEL) == AD7091R5_CHAN_ALL))
    {
        return;
    }

        //voltage conversion (calculated using battery monitor parameters)
        sitl->state.batt_voltage = (_data_to_volt(_analog_data[volt_pin.get() - AD7091R5_BASE_PIN ].data) - _volt_offset.get()) * _volt_multiplier.get();

        //current amps conversion (calculated using battery monitor parameters)
        sitl->state.battery_current = (_data_to_volt(_analog_data[_curr_pin.get() - AD7091R5_BASE_PIN].data) - _curr_amp_offset.get()) * _curr_amp_per_volt.get();

    

}

// Convert ADC to Voltage
float SITL::AP_BattMonitor_AD7091R5::_data_to_volt(uint32_t data)
{
    return (AD7091R5_REF/AD7091R5_RESOLUTION)*data;
}

// Register initialisation 
void SITL::AP_BattMonitor_AD7091R5::reset()
{

    // Set all registers to default as per Page 19 of datasheet
    set_register(RESULT, (uint8_t)0);
    set_register(CHANNEL, (uint8_t)0);
    set_register(CONF, (uint8_t)0xC0);
    set_register(ALERT_INDICATION, (uint8_t)0);
    set_register(CHANNEL_0_LOW_LIMIT, (uint8_t)0);



}