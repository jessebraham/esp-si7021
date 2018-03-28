# ESP-Si7021

An [ESP-IDF](https://github.com/espressif/esp-idf/) component for the Silicon Labs Si7021-A20 Relative Humidity and Temperature Sensor with I2C Interface.


### Links
- [Product Page](https://www.silabs.com/products/sensors/humidity/si7006-13-20-21-34)  
- [Datasheet](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf)  
- [Adafruit Breakout Board](https://www.adafruit.com/product/3251)  
- [SparkFun Breakout Board](https://www.sparkfun.com/products/13763)  


### Features

The following features have been implemented:  

- Measure Relative Humidity  
- Measure Temperature  
- Read and Write Registers  
- Read the Electronic Serial Number  
- Read the Firmware Revision  
- Perform a software reset of the sensor  


### Requirements

This component requires the [ESP-IDF](https://github.com/espressif/esp-idf/) IoT Development Environment as well as the C99 Standard Library, and has no other external dependencies.


### Example

TODO: provide an example


### API

#### Data Structures

```c
struct si7021_reading
{
    float humidity;
    float temperature;
};
```

#### Functions

```c
esp_err_t readHumidity(const i2c_port_t i2c_num, float *humidity);
```

```c
esp_err_t readTemperature(const i2c_port_t i2c_num, float *temperature);
```

```c
esp_err_t readTemperatureAfterHumidity(const i2c_port_t i2c_num,
                                       float *temperature);
```

```c
esp_err_t readSensors(const i2c_port_t i2c_num,
                      struct si7021_reading *sensor_data);
```


```c
esp_err_t readSerialNumber(const i2c_port_t i2c_num, uint8_t *serial);
```

```c
esp_err_t readFirmwareRevision(const i2c_port_t i2c_num, uint8_t *revision);
```


```c
esp_err_t readRegister(const i2c_port_t i2c_num, const uint8_t command,
                       uint8_t *settings);
```

```c
esp_err_t writeRegister(const i2c_port_t i2c_num, const uint8_t command,
                        const uint8_t settings);
```


```c
esp_err_t softwareReset(const i2c_port_t i2c_num);
```


### To Do

- [ ] implement **all** features documented in the datasheet
  - [ ] acknowledge the fact that checksums are, in fact, a thing  
  - [ ] implement an easier method of manipulating the user and heater control registers  
    - [ ] user register - measurement resolution  
    - [ ] user register - vdd status  
    - [ ] user register - heater status  
    - [ ] heater control register - heater current  
- [ ] properly comment header and source files  
- [ ] provide example application  
- [ ] write documentation  
