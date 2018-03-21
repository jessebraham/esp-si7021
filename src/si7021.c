/* ************************************************************************ *
 * esp-si7021/si7021.c
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

#include "esp_system.h"
#include "driver/i2c.h"

#include "si7021.h"


// ---------------------------------------------------------------------------
// EXTERNAL FUNCTIONS

//
// sensor readings

esp_err_t
readHumidity(const i2c_port_t i2c_num, float *humidity)
{
    // read the relative humidity from the sensor. write the value returned
    // out to the humidity pointer after using the _rh_code_to_pct function to
    // convert the humidity code to a percentage value.
    esp_err_t ret = _getSensorReading(i2c_num, SI7021_READ_RH, humidity,
                                      &_rh_code_to_pct);

    return ret;
}

esp_err_t
readTemperature(const i2c_port_t i2c_num, float *temperature)
{
    // read the temperature from the sensor. write the value returned out to
    // the temperature pointer after using the _temp_code_to_celsius function
    // to convert the temperature code to a value in degrees celsius.
    esp_err_t ret = _getSensorReading(i2c_num, SI7021_READ_TEMP, temperature,
                                      &_temp_code_to_celsius);

    return ret;
}

esp_err_t
readTemperatureAfterHumidity(const i2c_port_t i2c_num, float *temperature)
{
    // read the temperature from the sensor. write the value returned out to
    // the temperature pointer after using the _temp_code_to_celsius function
    // to convert the temperature code to a value in degrees celsius.
    esp_err_t ret = _getSensorReading(i2c_num, SI7021_READ_TEMP_PREV_RH,
                                      temperature, &_temp_code_to_celsius);

    return ret;
}

esp_err_t
readSensors(const i2c_port_t i2c_num, struct si7021_reading *sensor_data)
{
    // read the sensor's value for relative humidity, converting the resulting
    // value to a percentage. write the value out to the sensor data struct's
    // humidity member.
    esp_err_t ret = readHumidity(i2c_num, &sensor_data->humidity);

    if (ret != ESP_OK)
        return ret;

    // read the sensor's value for temperature, using the value read in the
    // previous call for humidity. convert the resulting value to degrees
    // celsius. write the value out to the sensor data struct's temperature
    // member.
    ret = readTemperatureAfterHumidity(i2c_num, &sensor_data->temperature);

    return ret;
}

//
// device identification and information

esp_err_t
readSerialNumber(const i2c_port_t i2c_num, uint8_t *serial)
{
    // to retrieve the serial number of the sensor, we need to write out the
    // pair of command bytes. this isn't technically correct, as we're not
    // performing the first access prior to the second access, but it seems to
    // work just fine anyways. Potential future improvment.
    esp_err_t ret = _writeCommandBytes(i2c_num, READ_ID_SECOND_ACCESS, 2);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, and the sensor can take
    // up to [VALUE NEEDED]ms to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // the serial number command returns a 32-bit value, so read the four
    // bytes.
    uint8_t buf[4];
    ret = _readResponseBytes(i2c_num, buf, 4);

    if (ret != ESP_OK)
        return ret;

    // the first byte of the response (SNB_3) is the serial number. write the
    // value to the serial pointer.
    *serial = buf[0];

    return ESP_OK;
}

esp_err_t
readFirmwareRevision(const i2c_port_t i2c_num, uint8_t *revision)
{
    // to retrieve the firmware revision of the sensor, we need to write out
    // the pair of command bytes.
    esp_err_t ret = _writeCommandBytes(i2c_num, READ_FW_REVISION, 2);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, and the sensor can take
    // up to [VALUE NEEDED]ms to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // the firmware revision command returns a 16-bit value, so read the pair
    // of bytes.
    uint8_t buf[2];
    ret = _readResponseBytes(i2c_num, buf, 2);

    if (ret != ESP_OK)
        return ret;

    // we're only concerned with the first of the two bytes. store the value
    // of the first byte in the revision pointer.
    *revision = buf[0];

    return ESP_OK;
}

//
// register settings

esp_err_t
readRegister(const i2c_port_t i2c_num, const uint8_t command,
             uint8_t *settings)
{
    // write the single read register command byte to the i2c bus.
    esp_err_t ret = _writeCommandBytes(i2c_num, &command, 1);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, and the sensor can take
    // up to [VALUE NEEDED]ms to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // the register read commands return an 8-bit value, so read the single
    // byte.
    uint8_t reg;
    ret = _readResponseBytes(i2c_num, &reg, 1);

    if (ret != ESP_OK)
        return ret;

    // write the byte read from the i2c bus out to the settings pointer.
    *settings = reg;

    return ESP_OK;
}

esp_err_t
writeRegister(const i2c_port_t i2c_num, const uint8_t command,
              const uint8_t settings)
{
    // construct the full command by appending the settings byte to the
    // command byte.
    const uint8_t full_command[] = { command, settings };

    // write the full command to the i2c bus to apply the settings.
    esp_err_t ret = _writeCommandBytes(i2c_num, full_command, 2);

    return ret;
}

//
// other miscellaneous features

esp_err_t
softwareReset(const i2c_port_t i2c_num)
{
    const uint8_t command = SI7021_RESET;

    // in order to perform a software reset on the sensor, we simply need to
    // write out the single command byte.
    esp_err_t ret = _writeCommandBytes(i2c_num, &command, 1);

    return ret;
}


// ---------------------------------------------------------------------------
// INTERNAL FUNCTIONS

//
// generic tasks

esp_err_t
_getSensorReading(const i2c_port_t i2c_num, const uint8_t command,
                  float *output, float (*fn)(const uint16_t))
{
    // write the specified number of provided command bytes to the i2c bus,
    // targetting the address of the Si7021.
    esp_err_t ret = _writeCommandBytes(i2c_num, &command, 1);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, and the sensor can take
    // up to [VALUE NEEDED]ms to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // all sensor readings return a 16-bit value for this sensor, so read the
    // pair of bytes.
    uint8_t buf[2];
    ret = _readResponseBytes(i2c_num, buf, 2);

    if (ret != ESP_OK)
        return ret;

    // re-assemble the bytes, and call the specified code-conversion function
    // to finally retrieve our final sensor reading value, writing it out to
    // the output pointer.
    uint16_t bytes = buf[0] << 8 | buf[1];
    *output = fn(bytes);

    return ESP_OK;
}

//
// hardware interaction

esp_err_t
_readResponseBytes(const i2c_port_t i2c_num, uint8_t *output,
                   const size_t nbytes)
{
    // create and initialize a command link prior to commanding the i2c master
    // to generate a start signal.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write the 7-bit address of the sensor to the queue, using the last bit
    // to indicate we are performing a read. read nbytes number of bytes from
    // the response into the buffer.
    i2c_master_write_byte(cmd, SI7021_I2C_ADDR << 1 | I2C_MASTER_READ,
                          ACK_CHECK_EN);

    for (size_t i = 0; i < nbytes; i++)
    {
        i2c_master_read_byte(cmd, &output[i], i == nbytes - 1
                                              ? NACK_VAL
                                              : ACK_VAL);
    }

    // command the i2c master to generate a stop signal. send all queued
    // commands, blocking until all commands have been sent. note that this is
    // *not* thread-safe. finally, free the i2c command link.
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd,
                                         I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t
_writeCommandBytes(const i2c_port_t i2c_num, const uint8_t *i2c_command,
                   const size_t nbytes)
{
    // create and initialize a command link prior to commanding the i2c master
    // to generate a start signal.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write the 7-bit address of the sensor to the bus, using the last bit to
    // indicate we are performing a write. write each of the the provided
    // command bytes to the queue, the number of which is specified by nbytes.
    i2c_master_write_byte(cmd, SI7021_I2C_ADDR << 1 | I2C_MASTER_WRITE,
                          ACK_CHECK_EN);

    for (size_t i = 0; i < nbytes; i++)
        i2c_master_write_byte(cmd, i2c_command[i], ACK_CHECK_EN);

    // command the i2c master to generate a stop signal. send all queued
    // commands, blocking until all commands have been sent. note that this is
    // *not* thread-safe. finally, free the i2c command link.
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd,
                                         I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

//
// conversion functions

float
_rh_code_to_pct(const uint16_t rh_code)
{
    // refer to page 21 of the datasheet for more information.
    return (( (125.0 * rh_code) / 65536.0 ) - 6.0);
}

float
_temp_code_to_celsius(const uint16_t temp_code)
{
    // refer to page 22 of the datasheet for more information.
    return (( (175.72 * temp_code) / 65536.0 ) - 46.85);
}
