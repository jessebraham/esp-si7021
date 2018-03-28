/* ************************************************************************ *
 * src/si7021.c
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

#include "driver/i2c.h"

#include "si7021.h"


// ---------------------------------------------------------------------------
// EXTERNAL FUNCTIONS

//
// sensor readings

esp_err_t
readHumidity(const i2c_port_t i2c_num, float *humidity)
{
    esp_err_t ret = _getSensorReading(i2c_num, SI7021_READ_RH, humidity,
                                      &_rh_code_to_pct);

    return ret;
}

esp_err_t
readTemperature(const i2c_port_t i2c_num, float *temperature)
{
    esp_err_t ret = _getSensorReading(i2c_num, SI7021_READ_TEMP, temperature,
                                      &_temp_code_to_celsius);

    return ret;
}

esp_err_t
readTemperatureAfterHumidity(const i2c_port_t i2c_num, float *temperature)
{
    esp_err_t ret = _getSensorReading(i2c_num, SI7021_READ_TEMP_PREV_RH,
                                      temperature, &_temp_code_to_celsius);

    return ret;
}

esp_err_t
readSensors(const i2c_port_t i2c_num, struct si7021_reading *sensor_data)
{
    esp_err_t ret = readHumidity(i2c_num, &sensor_data->humidity);

    if (ret != ESP_OK)
        return ret;

    ret = readTemperatureAfterHumidity(i2c_num, &sensor_data->temperature);

    return ret;
}

//
// device identification and information

esp_err_t
readSerialNumber(const i2c_port_t i2c_num, uint8_t *serial)
{
    // this isn't technically correct, as we're not performing the first
    // access prior to the second access, but it seems to work just fine
    // anyways. Potential future improvment.
    esp_err_t ret = _writeCommandBytes(i2c_num, READ_ID_SECOND_ACCESS, 2);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, as the sensor can take
    // some time to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // the serial number command returns a 32-bit value.
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
    esp_err_t ret = _writeCommandBytes(i2c_num, READ_FW_REVISION, 2);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, as the sensor can take
    // some time to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // the firmware revision command returns a 16-bit value.
    uint8_t buf[2];
    ret = _readResponseBytes(i2c_num, buf, 2);

    if (ret != ESP_OK)
        return ret;

    *revision = buf[0];

    return ESP_OK;
}

//
// register settings

esp_err_t
readRegister(const i2c_port_t i2c_num, const uint8_t command,
             uint8_t *settings)
{
    esp_err_t ret = _writeCommandBytes(i2c_num, &command, 1);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, as the sensor can take
    // some time to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // the register read commands return an 8-bit value.
    uint8_t reg;
    ret = _readResponseBytes(i2c_num, &reg, 1);

    if (ret != ESP_OK)
        return ret;

    *settings = reg;

    return ESP_OK;
}

esp_err_t
writeRegister(const i2c_port_t i2c_num, const uint8_t command,
              const uint8_t settings)
{
    const uint8_t full_command[] = { command, settings };
    esp_err_t ret = _writeCommandBytes(i2c_num, full_command, 2);

    return ret;
}

//
// other miscellaneous features

esp_err_t
softwareReset(const i2c_port_t i2c_num)
{
    const uint8_t command = SI7021_RESET;
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
    esp_err_t ret = _writeCommandBytes(i2c_num, &command, 1);

    if (ret != ESP_OK)
        return ret;

    // delay for 100ms between write and read calls, as the sensor can take
    // some time to respond.
    vTaskDelay(100 / portTICK_RATE_MS);

    // all sensor readings return a 16-bit value for this sensor.
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
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write the 7-bit address of the sensor to the queue, using the last bit
    // to indicate we are performing a read.
    i2c_master_write_byte(cmd, SI7021_I2C_ADDR << 1 | I2C_MASTER_READ,
                          ACK_CHECK_EN);

    // read nbytes number of bytes from the response into the buffer. make
    // sure we send a NACK with the final byte rather than an ACK.
    for (size_t i = 0; i < nbytes; i++)
    {
        i2c_master_read_byte(cmd, &output[i], i == nbytes - 1
                                              ? NACK_VAL
                                              : ACK_VAL);
    }

    // send all queued commands, blocking until all commands have been sent.
    // note that this is *not* thread-safe.
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
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write the 7-bit address of the sensor to the bus, using the last bit to
    // indicate we are performing a write.
    i2c_master_write_byte(cmd, SI7021_I2C_ADDR << 1 | I2C_MASTER_WRITE,
                          ACK_CHECK_EN);

    for (size_t i = 0; i < nbytes; i++)
        i2c_master_write_byte(cmd, i2c_command[i], ACK_CHECK_EN);

    // send all queued commands, blocking until all commands have been sent.
    // note that this is *not* thread-safe.
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
