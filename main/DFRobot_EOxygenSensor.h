/*!
 * @file DFRobot_EOxygenSensor.h
 * @brief Define the basic structure of class DFRobot_EOxygenSensor (ESP-IDF port)
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @version V1.0
 * @date 2021-12-28
 * @url https://github.com/DFRobot/DFRobot_EOxygenSensor
 */
#ifndef __DFRobot_EOxygenSensor__
#define __DFRobot_EOxygenSensor__

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#define E_OXYGEN_ADDRESS_0       0x70
#define E_OXYGEN_ADDRESS_1       0x71
#define E_OXYGEN_ADDRESS_2       0x72
#define E_OXYGEN_ADDRESS_3       0x73

#define OXYGEN_DATA              0x10
#define CALIBRATION_STATE        0x13
#define CALIBRATION_SENSOR       0x18

#define CALIBRATION_20_9         0x01
#define CALIBRATION_99_5         0x02
#define CALIBRATION_CLEAR        0x03

class DFRobot_EOxygenSensor_I2C {
public:
    DFRobot_EOxygenSensor_I2C(i2c_port_t port = I2C_NUM_0, uint8_t addr = E_OXYGEN_ADDRESS_0);
    ~DFRobot_EOxygenSensor_I2C();

    /**
     * @brief Initialize and check I2C connection
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t begin(void);

    /**
     * @brief Get oxygen concentration in % VOL
     * @return concentration percentage (0.0-100.0)
     */
    float readOxygenConcentration(void);

    /**
     * @brief Check calibration status
     * @return state byte
     */
    uint8_t readCalibrationState(void);

    /**
     * @brief Calibrate in air with O2 concentration of 20.9% Vol
     * @return true if successful
     */
    bool calibration_20_9(void);

    /**
     * @brief Calibrate in air with O2 concentration of 99.5% Vol
     * @return true if successful
     */
    bool calibration_99_5(void);

    /**
     * @brief Clear calibration data
     * @return true if successful
     */
    bool clearCalibration(void);

private:
    esp_err_t writeData(uint8_t reg, const uint8_t *data, uint8_t len);
    esp_err_t readData(uint8_t reg, uint8_t *data, uint8_t len);

    i2c_port_t _port;
    uint8_t _addr;
    float _oldVol;
};

#endif
