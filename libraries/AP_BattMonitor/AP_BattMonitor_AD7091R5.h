#pragma once

#include "AP_BattMonitor_Backend.h"

#ifndef AP_BATTMON_AD7091R5_ENABLED
#define AP_BATTMON_AD7091R5_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if AP_BATTMON_AD7091R5_ENABLED

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>

#define AD7091R5_I2C_ADDR        0x2F // A0 and A1 tied to GND
#define AD7091R5_I2C_BUS         0
#define AD7091R5_RESET           0x02
#define AD7091R5_RESULT_ADDR     0x00
#define AD7091R5_CHAN_ADDR       0x01
#define AD7091R5_CONF_ADDR       0x02
#define AD7091R5_CONF_PDOWN0     0x00
#define AD7091R5_CONF_PDOWN2     0x02
#define AD7091R5_CONF_PDOWN3     0x03
#define AD7091R5_CONF_PDOWN_MASK 0x03
#define AD7091R5_CONF_CMD        0x04
#define AD7091R5_CHAN_ALL        0x0F
#define AD7091R5_CH_ID(x)        ((x >> 5) & 0x03)
#define AD7091R5_RES_MASK        0x0F
#define AD7091R5_REF             3.3
#define AD7091R5_NO_OF_CHANNELS  4
#define AD7091R5_RESOLUTION      (float)4096
#define AD7091R5_PERIOD_USEC     100000

class AP_BattMonitor_AD7091R5 : public AP_BattMonitor_Backend
{
public:
    // Constructor
    AP_BattMonitor_AD7091R5(AP_BattMonitor &mon,
                            AP_BattMonitor::BattMonitor_State &mon_state,
                            AP_BattMonitor_Params &params);

    // Read the battery voltage and current.  Should be called at 10hz
    void read() override;
    void init(void) override;

    // returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    // returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void _timer();
    bool _config();
    bool _reset();
    bool _powerdown();
    bool _read_adc();
    float _data_to_volt(uint32_t data);

    static struct AnalogData {
        uint32_t data;
    } _analog_data[AD7091R5_NO_OF_CHANNELS];
    static bool _first;

    HAL_Semaphore sem; // semaphore for access to shared frontend data
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t volt_buff_pt;
    uint8_t curr_buff_pt;
    bool _i2c_polling;

    //register
    uint8_t reg_conf_l = AD7091R5_CONF_PDOWN0;  // using external reference
    uint8_t reg_conf_h = AD7091R5_CONF_CMD;     // command mode
    uint8_t reg_chan = AD7091R5_CHAN_ALL;

    protected:

    // Parameters
    AP_Float _volt_multiplier;          // voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _curr_amp_per_volt;        // voltage on current pin multiplied by this to calculate current in amps
    AP_Float _curr_amp_offset;          // offset voltage that is subtracted from current pin before conversion to amps
    AP_Float _volt_offset;              // offset voltage that is subtracted from voltage pin before conversion
    AP_Int8  _volt_pin;                 // board pin used to measure battery voltage
    AP_Int8  _curr_pin;                 // board pin used to measure battery current
};

#endif // AP_BATTMON_AD7091R5_ENABLED
