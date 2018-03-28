/* ************************************************************************ *
 * include/si7021.h
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

#include "sdkconfig.h"


// ---------------------------------------------------------------------------
// DEFINES

//
// i2c related values

#define I2C_TIMEOUT_MS              1000

#define ACK_CHECK_EN                0x1
#define ACK_CHECK_DIS               0x0

#define ACK_VAL                     0x0
#define NACK_VAL                    0x1

#define SI7021_I2C_ADDR             0x40


//
// sensor commands

#if CONFIG_USE_CLOCK_STRETCHING
    #define SI7021_READ_RH          0xE5
#else
    #define SI7021_READ_RH          0xF5
#endif

#if CONFIG_USE_CLOCK_STRETCHING
    #define SI7021_READ_TEMP        0xE3
#else
    #define SI7021_READ_TEMP        0xF3
#endif

#define SI7021_READ_TEMP_PREV_RH    0xE0


//
// register commands

#define SI7021_USER_REG_READ        0xE7
#define SI7021_USER_REG_WRITE       0xE6

#define SI7021_HTRE_REG_READ        0x11
#define SI7021_HTRE_REG_WRITE       0x51


//
// miscellaneous commands

#define SI7021_HEATER_ON            0x3E
#define SI7021_HEATER_OFF           0x3A

#define SI7021_RESET                0xFE


//
// user register settings

//
// D7    D6    D5    D4    D3    D2    D1    D0
// RES1  VDDS  RSVD  RSVD  RSVD  HTRE  RSVD  RES0
//


//
// heater control register settings

//
// D7    D6    D5    D4    D3    D2    D1    D0
// RSVD  RSVD  RSVD  RSVD  HTR3  HTR2  HTR1  HTR0
//


// ---------------------------------------------------------------------------
// STRUCTURES

struct si7021_reading
{
    float humidity;
    float temperature;
};


// ---------------------------------------------------------------------------
// CONSTANTS

static const uint8_t READ_ID_FIRST_ACCESS[]  = { 0xFA, 0x0F };
static const uint8_t READ_ID_SECOND_ACCESS[] = { 0xFC, 0xC9 };

static const uint8_t READ_FW_REVISION[]      = { 0x84, 0xB8 };


// ---------------------------------------------------------------------------
// FUNCTION PROTOTYPES

//
// external

esp_err_t readHumidity(const i2c_port_t i2c_num, float *humidity);

esp_err_t readTemperature(const i2c_port_t i2c_num, float *temperature);

esp_err_t readTemperatureAfterHumidity(const i2c_port_t i2c_num,
                                       float *temperature);

esp_err_t readSensors(const i2c_port_t i2c_num,
                      struct si7021_reading *sensor_data);


esp_err_t readSerialNumber(const i2c_port_t i2c_num, uint8_t *serial);

esp_err_t readFirmwareRevision(const i2c_port_t i2c_num, uint8_t *revision);


esp_err_t readRegister(const i2c_port_t i2c_num, const uint8_t command,
                       uint8_t *settings);

esp_err_t writeRegister(const i2c_port_t i2c_num, const uint8_t command,
                        const uint8_t settings);


esp_err_t softwareReset(const i2c_port_t i2c_num);


//
// internal

/**
 * perform a sensor reading, and write the value in its correct units to the
 * specified location in memory.
 * 
 * @param   i2c_num         the I2C port to read from
 * @param   i2c_command     specify which reading to perform
 * @param   output          the location in memory in which to write the
 *                          resulting value
 * @param   fn              a pointer to the conversion function to use on the
 *                          raw value read from the sensor
 * 
 * @returns esp_err_t       the success status of the read
 */
esp_err_t _getSensorReading(const i2c_port_t num,
                            const uint8_t i2c_command, float *output,
                            float (*fn)(const uint16_t));


/**
 * read the specified number of bytes from the I2C port specified to the
 * provided location in memory.
 * 
 * @param   i2c_num         the I2C port to read from
 * @param   i2c_command     a pointer to the location in memory in which to
 *                          write the data to
 * @param   nbytes          the number of bytes to read from the queue
 * 
 * @returns esp_err_t       the success status of the read
 */
esp_err_t _readResponseBytes(const i2c_port_t i2c_num,
                             uint8_t *output, const size_t nbytes);

/**
 * write the specified number of bytes from the provided location in memory to
 * the I2C port specified.
 * 
 * @param   i2c_num         the I2C port to write to
 * @param   i2c_command     a pointer to the location in memory containing the
 *                          data byte(s) to write
 * @param   nbytes          the number of bytes to write to the queue
 * 
 * @returns esp_err_t       the success status of the write
 */
esp_err_t _writeCommandBytes(const i2c_port_t i2c_num,
                             const uint8_t *i2c_command, const size_t nbytes);


/**
 * Given a 'humidity code' read from the sensor, convert the code to a decimal
 * value representing the relative humidity in percent, and return it.
 * 
 * @param   rh_code     the encoded relative humidity reading from the sensor
 * 
 * @returns float       the current relative humidity as read by the sensor,
 *                      in percent
 */
float _rh_code_to_pct(const uint16_t rh_code);

/**
 * Given a 'temperature code' read from the sensor, convert the code to a
 * decimal value representing the temperature in degrees celsius, and return
 * it.
 * 
 * @param   temp_code   the encoded temperature reading from the sensor
 * 
 * @returns float       the current temperature as read by the sensor, in
 *                      degrees celsius
 */
float _temp_code_to_celsius(const uint16_t temp_code);


#endif
