# Arduino library for BG96 Eseye Telemetry Module

Simple Arduino library to enable use for the BG96-hosted Eseye Telemetry Module

## What is ETM?

Eseye Telemetry Module is an application that runs on a modem module enabling secure connection to cloud-based IoT services. This works in conjunction with Eseye Anynet Secure SIMs to provide secure end-to-end communication without the need to pre-configure security credentials on the device. All security credentials are sent to the SIM and used to establish a cloud connection without any user intervention. Applications using ETM make use of modem AT commands to publish and subscribe to topics on the cloud service.  

## Purpose of this library

To provide a simple API to the Arduino UART and AT commands

## Prerequisites

### You will need
Quectel BG96 modem installed with ETM software from Eseye  
Provisioned Eseye Anynet Secure SIM  

### For the examples
Anynet Secure SIM provisioned to a thing on AWSIoT  
Arduino DUE  
Arduino MEGA click shield  
MikroElektronika LTE IoT 2 click (BG96 modem)  
#### optional:  
MikroElektronika WEATHER click set up for i2c (see image)  
MikroElectronika OLED B click set up for i2c (see image)  

Examples with BME280 are built with https://github.com/finitespace/BME280.git

## Images
Kit of all parts for ETM_due_bme280_oledb example
![Kit of parts](/images/kitofparts.jpg)

Assembled ETM_due example
![ETM DUE example](/images/etm_due.jpg)

Assembled ETM_due_bme280 example
![ETM DUE BME280 example](/images/etm_due_bme280.jpg)

Assembled ETM_due_bme280_oledb example
![ETM DUE BME280 oledb example](/images/etm_due_bme280_oledb.jpg)

Running ETM DUE BME280 oledb board
![ETM running example](/images/etm_due_bme280_oledb_running.jpg)
