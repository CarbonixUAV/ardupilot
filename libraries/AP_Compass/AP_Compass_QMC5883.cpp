/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Driver by RadioLink LjWang, Jun 2017
 * GPS compass module See<http://www.radiolink.com>
 */
#include "AP_Compass_QMC5883.h"

#include <stdio.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

//device ID's
#define QMC5883_NO_OF_DEV   2
#define QMC5883L_ID_VAL         0xFF
#define QMC5883P_ID_VAL         0x80

//common register
// data output rates for 5883L & 5883P
#define QMC5883_ODR_10HZ  (0x00 << 2)
#define QMC5883_ODR_50HZ  (0x01 << 2)
#define QMC5883_ODR_100HZ (0x02 << 2)
#define QMC5883_ODR_200HZ (0x03 << 2)

//Reset
#define QMC5883_RST 0x80


/****** QMC5883P defines ******/
#define QMC5883P_REG_CONF1              0x0A
#define QMC5883P_REG_CONF2              0x0B

//Register Address
#define QMC5883P_REG_ID                 0x00
#define QMC5883P_REG_DATA_OUTPUT_X      0x01
#define QMC5883P_REG_DATA_OUTPUT_Z_MSB  0x06
#define QMC5883P_REG_STATUS             0x09

// Sensor operation modes
#define QMC5883P_MODE_SUSPEND    0x00
#define QMC5883P_MODE_NORMAL     0x01
#define QMC5883P_MODE_SINGLE     0x02
#define QMC5883P_MODE_CONTINUOUS 0x03

// Over sampling Ratio OSR1
#define QMC5883P_OSR1_8 (0x00 << 4)
#define QMC5883P_OSR1_4 (0x01 << 4)
#define QMC5883P_OSR1_2	(0x02 << 4)
#define QMC5883P_OSR1_1	(0x03 << 4)

//RNG
#define QMC5883P_RNG_30G (0x00 << 2)
#define QMC5883P_RNG_12G (0x01 << 2)
#define QMC5883P_RNG_8G  (0x10 << 2)
#define QMC5883P_RNG_2G  (0x11 << 2)

#define QMC5883P_SET_XYZ_SIGN 0x29

//Status Val
#define QMC5883P_STATUS_DATA_READY 0x01

// Down sampling Rate OSR2
#define QMC5883P_OSR2_8	0x08



/****** QMC5883L defines ******/
#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A

// Sensor operation modes
#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

//RNG
#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

// Over sampling Ratio OSR1
#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128 (0x10 << 6)
#define QMC5883L_OSR_64	 (0x11	<< 6)

#define QMC5883L_REG_DATA_OUTPUT_X 0x00
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_ID 0x0D



#ifndef DEBUG
#define DEBUG 0
#endif

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_QMC5883::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                               bool force_external,
                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_QMC5883 *sensor = new AP_Compass_QMC5883(std::move(dev),force_external,rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_QMC5883::AP_Compass_QMC5883(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                         bool force_external,
                                         enum Rotation rotation)
    : _dev(std::move(dev))
    , _rotation(rotation)
	, _force_external(force_external)
{
}

bool AP_Compass_QMC5883::init()
{
    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(10);

#if 0
    _dump_registers();
#endif

    if(!_check_whoami()){
    	goto fail;
    }
    if(_dev_selected == QMC5883L_ID_VAL){
        if (!_dev->write_register(0x0B, 0x01)||
            !_dev->write_register(0x20, 0x40)||
            !_dev->write_register(0x21, 0x01)||
            !_dev->write_register(QMC5883L_REG_CONF1,
                            QMC5883L_MODE_CONTINUOUS|
                            QMC5883_ODR_100HZ|
                            QMC5883L_OSR_512|
                            QMC5883L_RNG_8G)) {
                    goto fail;
        }
    }
    if(_dev_selected == QMC5883P_ID_VAL){
        if (!_dev->write_register(QMC5883P_REG_DATA_OUTPUT_Z_MSB, QMC5883P_SET_XYZ_SIGN)||
            !_dev->write_register(QMC5883P_REG_CONF1,
                                QMC5883P_MODE_CONTINUOUS|
                                QMC5883_ODR_100HZ|
                                QMC5883P_OSR1_8|
                                QMC5883P_OSR2_8)||
            !_dev->write_register(QMC5883P_REG_CONF2,QMC5883P_OSR2_8)) {
            goto fail;
        }
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    //register compass instance
    _dev->set_device_type(DEVTYPE_QMC5883);
    if (!register_compass(_dev->get_bus_id(), _instance)) {
        return false;
    }
    set_dev_id(_instance, _dev->get_bus_id());

    printf("%s found on bus %u id %u address 0x%02x\n", name,
           _dev->bus_num(), _dev->get_bus_id(), _dev->get_bus_address());

    set_rotation(_instance, _rotation);

    if (_force_external) {
        set_external(_instance, true);
    }

    //Enable 100HZ
    _dev->register_periodic_callback(10000,
        FUNCTOR_BIND_MEMBER(&AP_Compass_QMC5883::timer, void));

    return true;

 fail:
    _dev->get_semaphore()->give();
    return false;
}

bool AP_Compass_QMC5883::_check_whoami()
{
    uint8_t whoami;
    if (_dev->read_registers(QMC5883L_REG_ID, &whoami,1)||
    		whoami == QMC5883L_ID_VAL){
        _dev_selected = whoami;
    	return true;
    }
    if (_dev->read_registers(QMC5883P_REG_ID, &whoami,1)||
            whoami == QMC5883P_ID_VAL) {
        _dev_selected = whoami;
        return true;
    }
    return false;
}

void AP_Compass_QMC5883::timer()
{
    struct PACKED {
    	le16_t rx;
    	le16_t ry;
    	le16_t rz;
    } buffer;

    const float range_scale = 1000.0f / 3000.0f;

    uint8_t status;
    if(_dev_selected == QMC5883L_ID_VAL){
        if(!_dev->read_registers(QMC5883L_REG_STATUS,&status,1)){
            return;
        }
        //new data is ready
        if (!(status & 0x04)) {
            return;
        }

        if(!_dev->read_registers(QMC5883L_REG_DATA_OUTPUT_X, (uint8_t *) &buffer, sizeof(buffer))){
            return ;
        }
    }
    else if(_dev_selected == QMC5883P_ID_VAL){
        if (!_dev->read_registers(QMC5883P_REG_STATUS,&status,1)) {
            return;
        }
        //new data is ready
        if (!(status & QMC5883P_STATUS_DATA_READY)) {
            return;
        }

        if (!_dev->read_registers(QMC5883P_REG_DATA_OUTPUT_X, (uint8_t *) &buffer, sizeof(buffer))) {
            return ;
        }
    }
    auto x = buffer.rx;
    auto y = buffer.ry;
    auto z = buffer.rz;

#if 0
    printf("mag.x:%d\n",x);
    printf("mag.y:%d\n",y);
    printf("mag.z:%d\n",z);
#endif

    Vector3f field = Vector3f{x * range_scale , y * range_scale, z * range_scale };

    // rotate to the desired orientation
    if (is_external(_instance)) {
        field.rotate(ROTATION_YAW_90);
    }

    accumulate_sample(field, _instance, 20);
}

void AP_Compass_QMC5883::read()
{
    drain_accumulated_samples(_instance);
}

void AP_Compass_QMC5883::_dump_registers()
{
    if(_dev_selected == QMC5883L_ID_VAL){
	  printf("QMC5883L registers dump\n");
	    for (uint8_t reg = QMC5883L_REG_DATA_OUTPUT_X; reg <= 0x30; reg++) {
	        uint8_t v;
	        _dev->read_registers(reg,&v,1);
	        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
	        if ((reg - ( QMC5883L_REG_DATA_OUTPUT_X-1)) % 16 == 0) {
	            printf("\n");
	        }
	    }
    }
    else if(_dev_selected == QMC5883P_ID_VAL){
         printf("QMC5883P registers dump\n");
        for (uint8_t reg = QMC5883P_REG_DATA_OUTPUT_X; reg <= 0x30; reg++) {
            uint8_t v;
            _dev->read_registers(reg,&v,1);
            printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
            if ((reg - ( QMC5883P_REG_DATA_OUTPUT_X-1)) % 16 == 0) {
                printf("\n");
            }
        }
    }
}

