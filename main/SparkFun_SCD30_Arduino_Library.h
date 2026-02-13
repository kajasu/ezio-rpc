/*
  SCD30 CO2 Sensor Library (ESP-IDF Port)
  Based on SparkFun SCD30 Arduino Library
  
  The SCD30 measures CO2 with accuracy of +/- 30ppm.
*/

#ifndef __SparkFun_SCD30_ARDUINO_LIBRARY_H__
#define __SparkFun_SCD30_ARDUINO_LIBRARY_H__

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

// The default I2C address for the SCD30 is 0x61.
#define SCD30_ADDRESS 0x61

// Available commands
#define COMMAND_CONTINUOUS_MEASUREMENT 0x0010
#define COMMAND_SET_MEASUREMENT_INTERVAL 0x4600
#define COMMAND_GET_DATA_READY 0x0202
#define COMMAND_READ_MEASUREMENT 0x0300
#define COMMAND_AUTOMATIC_SELF_CALIBRATION 0x5306
#define COMMAND_SET_FORCED_RECALIBRATION_FACTOR 0x5204
#define COMMAND_SET_TEMPERATURE_OFFSET 0x5403
#define COMMAND_SET_ALTITUDE_COMPENSATION 0x5102
#define COMMAND_RESET 0xD304 // Soft reset
#define COMMAND_STOP_MEAS 0x0104
#define COMMAND_READ_FW_VER 0xD100

typedef union
{
    uint8_t array[4];
    float value;
} ByteToFl;

class SCD30
{
public:
    SCD30(void);

    esp_err_t begin(i2c_port_t wirePort, bool autoCalibrate = false, bool measBegin = true);

    bool isConnected();

    bool beginMeasuring(uint16_t pressureOffset);
    bool beginMeasuring(void);
    bool StopMeasurement(void);

    bool setAmbientPressure(uint16_t pressure_mbar);

    bool getSettingValue(uint16_t registerAddress, uint16_t *val);
    bool getFirmwareVersion(uint16_t *val) { return (getSettingValue(COMMAND_READ_FW_VER, val)); }
    uint16_t getCO2(void);
    float getHumidity(void);
    float getTemperature(void);

    uint16_t getMeasurementInterval(void);
    bool getMeasurementInterval(uint16_t *val) { return (getSettingValue(COMMAND_SET_MEASUREMENT_INTERVAL, val)); }
    bool setMeasurementInterval(uint16_t interval);

    uint16_t getAltitudeCompensation(void);
    bool getAltitudeCompensation(uint16_t *val) { return (getSettingValue(COMMAND_SET_ALTITUDE_COMPENSATION, val)); }
    bool setAltitudeCompensation(uint16_t altitude);

    bool getAutoSelfCalibration(void);
    bool setAutoSelfCalibration(bool enable);

    bool getForcedRecalibration(uint16_t *val) { return (getSettingValue(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, val)); }
    bool setForcedRecalibrationFactor(uint16_t concentration);

    float getTemperatureOffset(void);
    bool getTemperatureOffset(uint16_t *val) { return (getSettingValue(COMMAND_SET_TEMPERATURE_OFFSET, val)); }
    bool setTemperatureOffset(float tempOffset);

    bool dataAvailable();
    bool readMeasurement();

    void reset();

    bool sendCommand(uint16_t command, uint16_t arguments);
    bool sendCommand(uint16_t command);

    uint16_t readRegister(uint16_t registerAddress);

    uint8_t computeCRC8(uint8_t data[], uint8_t len);

    void useStaleData(bool enable) { _useStaleData = enable; }

private:
    i2c_port_t _i2cPort;

    // Global main datums
    float co2 = 0;
    float temperature = 0;
    float humidity = 0;
    bool _useStaleData = false;

    // These track the staleness of the current data
    bool co2HasBeenReported = true;
    bool humidityHasBeenReported = true;
    bool temperatureHasBeenReported = true;
};

#endif
