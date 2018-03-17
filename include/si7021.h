/* ************************************************************************ *
 * esp-si7021/si7021.h
 * 
 * Author:      Jesse Braham <jesse@beta7.io>
 * License:     MIT (see LICENSE.txt)
 * 
 * Created:     March, 2018
 * Modified:    March, 2018
 * 
 * 
 * Datasheet:
 * https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
 * ************************************************************************ */

#ifndef __SI7021_H
#define __SI7021_H

// ---------------------------------------------------------------------------
// INCLUDES

#include <stdint.h>


// ---------------------------------------------------------------------------
// DEFINES

#define ACK_CHECK_EN                0x1
#define ACK_CHECK_DIS               0x0
#define ACK_VAL                     0x0
#define NACK_VAL                    0x1

#define SI7021_I2C_ADDR             0x40

#define SI7021_READ_RH_HOLD         0xE5
#define SI7021_READ_RH_NOHOLD       0xF5

#define SI7021_READ_TEMP_HOLD       0xE3
#define SI7021_READ_TEMP_NOHOLD     0xF3
#define SI7021_READ_TEMP_PREV_RH    0xE0

#define SI7021_RESET                0xFE

#define SI7021_WRITE_USER_REG       0xE6
#define SI7021_READ_USER_REG        0xE7


// ---------------------------------------------------------------------------
// MACROS


// ---------------------------------------------------------------------------
// STRUCTURES

struct si7021_reading
{
    int32_t humidity;
    int32_t temperature;
};


// ---------------------------------------------------------------------------
// CONSTANTS

static const uint8_t READ_ID_FIRST_BYTE[]  = { 0xFA, 0x0F };
static const uint8_t READ_ID_SECOND_BYTE[] = { 0xFC, 0xC9 };

static const uint8_t READ_FW_REVISION[]    = { 0x84, 0xB8 };


// ---------------------------------------------------------------------------
// GLOBALS


// ---------------------------------------------------------------------------
// FUNCTION PROTOTYPES

// public

esp_err_t readHumidity(const i2c_port_t i2c_num, int32_t *humidity);

esp_err_t readTemperature(const i2c_port_t i2c_num, int32_t *temperature);

esp_err_t readTemperatureAfterHumidity(const i2c_port_t i2c_num,
                                       int32_t *temperature);

esp_err_t readSensors(const i2c_port_t i2c_num,
                      struct si7021_reading *sensor_data);


// private

esp_err_t _writeCommand(const i2c_port_t i2c_num, const uint8_t i2c_addr,
                        const uint8_t i2c_command);

esp_err_t _readResults(const i2c_port_t i2c_num, const uint8_t i2c_addr,
                       uint16_t *bytes);

int32_t   _rh_code_to_pct(const uint16_t rh_code);

int32_t   _temp_code_to_celsius(const uint16_t temp_code);


#endif
