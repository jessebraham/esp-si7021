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
// PUBLIC FUNCTIONS

esp_err_t
readHumidity(const i2c_port_t i2c_num, int32_t *humidity)
{
    esp_err_t ret = _writeCommand(i2c_num, SI7021_I2C_ADDR,
                                  SI7021_READ_RH_HOLD);

    if (ret != ESP_OK)
        return ret;

    vTaskDelay(100 / portTICK_RATE_MS);

    uint16_t bytes;
    ret = _readResults(i2c_num, SI7021_I2C_ADDR, &bytes);

    if (ret != ESP_OK)
        return ret;

    *humidity = _rh_code_to_pct(bytes);

    return ESP_OK;
}

esp_err_t
readTemperature(const i2c_port_t i2c_num, int32_t *temperature)
{
    esp_err_t ret = _writeCommand(i2c_num, SI7021_I2C_ADDR,
                                  SI7021_READ_TEMP_HOLD);

    if (ret != ESP_OK)
        return ret;

    vTaskDelay(100 / portTICK_RATE_MS);

    uint16_t bytes;
    ret = _readResults(i2c_num, SI7021_I2C_ADDR, &bytes);

    if (ret != ESP_OK)
        return ret;

    *temperature = _temp_code_to_celsius(bytes);

    return ESP_OK;
}

esp_err_t
readTemperatureAfterHumidity(const i2c_port_t i2c_num, int32_t *temperature)
{
    esp_err_t ret = _writeCommand(i2c_num, SI7021_I2C_ADDR,
                                  SI7021_READ_TEMP_PREV_RH);

    if (ret != ESP_OK)
        return ret;

    vTaskDelay(100 / portTICK_RATE_MS);

    uint16_t bytes;
    ret = _readResults(i2c_num, SI7021_I2C_ADDR, &bytes);

    if (ret != ESP_OK)
        return ret;

    *temperature = _temp_code_to_celsius(bytes);

    return ESP_OK;
}

esp_err_t
readSensors(const i2c_port_t i2c_num, struct si7021_reading *sensor_data)
{
    esp_err_t ret = readHumidity(i2c_num, &sensor_data->humidity);

    if (ret != ESP_OK)
        return ret;

    ret = readTemperatureAfterHumidity(i2c_num, &sensor_data->temperature);

    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}


// ---------------------------------------------------------------------------
// PRIVATE FUNCTIONS

esp_err_t
_writeCommand(const i2c_port_t i2c_num, const uint8_t i2c_addr,
              const uint8_t i2c_command)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_command, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd,
                                         1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t
_readResults(const i2c_port_t i2c_num, const uint8_t i2c_addr,
             uint16_t *bytes)
{
    uint8_t msb, lsb;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &msb, ACK_VAL);
    i2c_master_read_byte(cmd, &lsb, NACK_VAL);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd,
                                         1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    *bytes = msb << 8 | lsb;

    return ret;
}

int32_t
_rh_code_to_pct(const uint16_t rh_code)
{
    // returned value is in hundredths of a percent, in order to maintain two-
    // decimal precision while avoiding floating point numbers. in order to
    // get the proper decimal value in percent, divide the result by 100.
    return (( (125 * rh_code) >> 16 ) - 6) * 100;
}

int32_t
_temp_code_to_celsius(const uint16_t temp_code)
{
    // returned value is in hundredths of degrees, in order to maintain two-
    // decimal precision while avoiding floating point numbers. in order to
    // get the proper decimal value in celsius, divide the result by 100.
    return (( (17572 * temp_code) >> 16 ) - 4685);
}