#include "AP_BattMonitor_AD7091R5.h"

/**
 * @brief You can use it to Read Current and voltage of 1-3 batteries.
 * Examples of Pin combination: 
 *  1)Pin 1 = Voltage 2,3,4 =  Current. For 3 battery combination Voltage will be same accross.
 *  2)Pin 1,2 = Voltage and Current Battery 1 -   Pin 3,4 = Voltage and Current Battery 2
 * Only the First instance of Battery Monitor will be reading the values from IC over I2C.
 * Make sure you understand the method of calculation usd in this driver before using it.
 * 
 * Pin number represents 50 = pin 1, 51 = pin 2 and so on 52, 53
 * 
 */
// 

#if AP_BATTMON_AD7091R5_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#define AD7091R5_I2C_ADDR        0x2F // A0 and A1 tied to GND
#define AD7091R5_I2C_BUS         0
#define AD7091R5_RESET           0x02
#define AD7091R5_RESULT_ADDR     0x00
#define AD7091R5_CHAN_ADDR       0x01
#define AD7091R5_CONF_ADDR       0x02
#define AD7091R5_CH_ID(x)        ((x >> 5) & 0x03)
#define AD7091R5_RES_MASK        0x0F
#define AD7091R5_REF             3.3f
#define AD7091R5_RESOLUTION      (float)4096
#define AD7091R5_PERIOD_USEC     100000
#define AD7091R5_BASE_PIN        50



extern const AP_HAL::HAL& hal;
const AP_Param::GroupInfo AP_BattMonitor_AD7091R5::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin on the AD7091R5 Ic
    // @Description: Sets the analog input pin that should be used for voltage monitoring on AD7091R5.
    // @Values: -1:Disabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 30, AP_BattMonitor_AD7091R5, _volt_pin, 0),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for Current monitoring on AD7091R5.
    // @Values: -1:Disabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 31, AP_BattMonitor_AD7091R5, _curr_pin, 0),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). 
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 32, AP_BattMonitor_AD7091R5, _volt_multiplier, 0),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. 
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 33, AP_BattMonitor_AD7091R5, _curr_amp_per_volt, 0),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 34, AP_BattMonitor_AD7091R5, _curr_amp_offset, 0),

    // @Param: VLT_OFFSET
    // @DisplayName: Volage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 35, AP_BattMonitor_AD7091R5, _volt_offset, 0),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

AP_BattMonitor_AD7091R5::AnalogData AP_BattMonitor_AD7091R5::_analog_data[AD7091R5_NO_OF_CHANNELS] = {0};
bool AP_BattMonitor_AD7091R5::_first = true;

/**
 * @brief Construct a new ap battmonitor ad7091r5::ap battmonitor ad7091r5 object
 * 
 * @param mon 
 * @param mon_state 
 * @param params 
 */
AP_BattMonitor_AD7091R5::AP_BattMonitor_AD7091R5(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

/**
 * @brief probe and initialize the sensor and register call back
 * 
 */
void AP_BattMonitor_AD7091R5::init(){
    // voltage and current pins from params and check if there are in range
    if (_volt_pin >= AD7091R5_BASE_PIN && _volt_pin <= AD7091R5_BASE_PIN + AD7091R5_NO_OF_CHANNELS) {
        volt_buff_pt = AD7091R5_BASE_PIN - _volt_pin.get();
    }
    if (_curr_pin >= AD7091R5_BASE_PIN && _curr_pin <= AD7091R5_BASE_PIN + AD7091R5_NO_OF_CHANNELS) {
        curr_buff_pt = AD7091R5_BASE_PIN - _curr_pin.get();
    }

    if (_first) { // only the first instance read the i2c device
        _first = false;
        _state.healthy = false;
        // probe i2c device
        _dev = hal.i2c_mgr->get_device(AD7091R5_I2C_BUS, AD7091R5_I2C_ADDR);
        if (_dev) {
            WITH_SEMAPHORE(_dev->get_semaphore());
            _dev->set_retries(10); // lots of retries during probe
            //config
            if (_reset() && _config()) {
                _state.healthy = true;
                _dev->set_retries(2); // drop to 2 retries for runtime
                _dev->register_periodic_callback(AD7091R5_PERIOD_USEC, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_AD7091R5::_read_adc, void));
            }
        }
    }
}

/**
 * config the adc
 * - command mode
 * - use external 3.3 reference
 * - all channels enabled
 * - set address pointer register to read the adc results
 */
bool AP_BattMonitor_AD7091R5::_config(){
    uint8_t data[6] = {AD7091R5_CONF_ADDR, AD7091R5_CONF_CMD, AD7091R5_CONF_PDOWN0, AD7091R5_CHAN_ADDR, AD7091R5_CHAN_ALL, AD7091R5_RESULT_ADDR};
    return _dev->transfer(data, sizeof(data), nullptr, 0);
}

/**
 * @brief read - read the voltage and curren
 * 
 */
void AP_BattMonitor_AD7091R5::read(){
    
    WITH_SEMAPHORE(sem);
    //voltage
    _state.voltage = (_data_to_volt(_analog_data[volt_buff_pt].data) - _volt_offset) * _volt_multiplier;

    //current amps
    _state.current_amps = (_data_to_volt(_analog_data[curr_buff_pt].data) - _curr_amp_offset) * _curr_amp_per_volt;
    

    // calculate time since last current read
    uint32_t tnow = AP_HAL::micros();
    uint32_t dt_us = tnow - _state.last_time_micros;

    // update total current drawn since startup
    update_consumed(_state, dt_us);

    // record time
    _state.last_time_micros = tnow;
}

/**
 * @brief read all four channels and store the results
 * 
 */
void AP_BattMonitor_AD7091R5::_read_adc() {
    uint8_t data[AD7091R5_NO_OF_CHANNELS*2] = {0};
    bool ret = _dev->transfer(nullptr, 0, data, sizeof(data));
    if (ret) {
        WITH_SEMAPHORE(sem);
        for (int i=0; i<AD7091R5_NO_OF_CHANNELS; i++) {
            uint8_t chan = AD7091R5_CH_ID(data[2*i]);
            _analog_data[chan].data = ((uint16_t)(data[2*i]&AD7091R5_RES_MASK)<<8) | data[2*i+1];
        }
    }
}

/**
 * @brief soft reset adc
 * 
 * @return true 
 * @return false 
 */
bool AP_BattMonitor_AD7091R5::_reset(){
    uint8_t reg = AD7091R5_CONF_CMD | AD7091R5_RESET;
    uint8_t data[3] = {AD7091R5_CONF_ADDR, reg, AD7091R5_CONF_PDOWN0};
    return _dev->transfer(data, sizeof(data), nullptr, 0);
}

/**
 * @brief power down adc
 * 
 * @return true 
 * @return false 
 */
bool AP_BattMonitor_AD7091R5::_powerdown(){
    uint8_t reg = (AD7091R5_CONF_PDOWN0 & ~AD7091R5_CONF_PDOWN_MASK) | AD7091R5_CONF_PDOWN2;
    uint8_t data[3] = {AD7091R5_CONF_ADDR, AD7091R5_CONF_CMD, reg};
    return _dev->transfer(data, sizeof(data), nullptr, 0);
}

/**
 * @brief convert binary reading to volts
 * 
 * @param data 
 * @return float 
 */
float AP_BattMonitor_AD7091R5::_data_to_volt(uint32_t data){
    return (AD7091R5_REF/AD7091R5_RESOLUTION)*data;
}

#endif // AP_BATTMON_AD7091R5_ENABLED
