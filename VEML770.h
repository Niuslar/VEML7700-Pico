/**
 * @file VEML770.h
 * @brief Adafruit VEML770 ambient light sensor (ALS) driver for Raspberry Pi Pico
 * @author niuslar
 * @date 20 Jun 2023
 */

#ifndef VEML770_H
#define VEML770_H

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdint.h>

class VEML7700
{
public:
    VEML7700();
    ~VEML7700();

    /**
     * @brief Initialise i2c peripherals and define GPIOs to use
     * @param sdaPin GPIO Pin to be configured as SDA (Please refer to Pico's Pinout to check pins available)
     * @param sclPin GPIO Pin to be configures as SCL
     * @param i2cInstance i2c0 or i2c1
     * @param baudRate 100000 (STANDARD) or 400000 (FAST)
     */
    void Init(uint8_t sdaPin = PICO_DEFAULT_I2C_SDA_PIN, uint8_t sclPin = PICO_DEFAULT_I2C_SCL_PIN, i2c_inst_t *i2cInstance = i2c0, uint32_t baudRate = 100 * 1000);

    /**
     * @brief Read Lux levels
     * @return Lux levels from 0 to 120k
     * */
    float ReadLux();

    /**
     * @brief Enable readings
     * @param enable True to enable readings, false to shutdown ALS
     */
    void Enable(bool enable);

    enum class gainValues_t
    {
        ALS_GAIN_X1 = 0,   // Gain x1
        ALS_GAIN_X2 = 1,   // Gain x2
        ALS_GAIN_X1_8 = 2, // Gain x1/8
        ALS_GAIN_X1_4 = 3  // Gain x1/4
    };

    /**
     * @brief Set ALS gain
     * @param newGain Gain as defined in the gainValues_t enum class
     */
    void SetGain(gainValues_t newGain);

    enum class integrationTime_t
    {
        ALS_IT_25MS = 12,
        ALS_IT_50MS = 8,
        ALS_IT_100MS = 0,
        ALS_IT_200MS = 1,
        ALS_IT_400MS = 2,
        ALS_IT_800MS = 3
    };

    /**
     * @brief Set ALS integration timing (how long the sensor will capture photons)
     * @param integrationTiming as defined in the integrationTime_t enum class
     */
    void SetIntegrationTiming(integrationTime_t integrationTiming);

    enum class persProt_t
    {
        ALS_PERS_1 = 0,
        ALS_PERS_2 = 1,
        ALS_PERS_4 = 2,
        ALS_PERS_8 = 3
    };

    /**
     * @brief Set the persistence protect number setting as defined
     * @param protNumber as defined in the persProt_t enum class
     */
    void SetPersistenceProtect(persProt_t protNumber);

    /**
     * @brief Enable Interrupt that will set the ALS_IF_L or ALS_IF_H bits when the programmed number of measurements (ALS_PERS) stay above/below the set threshold.
     * @param enable True to enable interrupt. False to disable it
     */
    void EnableInterrupt(bool enable);

    /**
     * @brief Set a high threshold that will be used in case the interrupt setting is enabled
     * @param highThreshold Max threshold in lux
     */
    void SetHighLimit(uint16_t highThreshold);

    /**
     * @brief Set a low threshold that will be used in case the interrupt setting is enabled
     * @param lowThreshold Min threshold in lux
     */
    void SetLowLimit(uint16_t lowThreshold);

    /**
     * @brief CheckInterruptStatus Read register status
     * @return 0 if no interrupt was triggered, 1 if the low threshold was exceeded and 2 if the high threshold was exceeded
     */
    uint8_t CheckInterruptStatus();

private:
    uint16_t ReadRegister(const uint8_t *regAddr);

    /* Send a 16-bit value as 2 8-bit values */
    void WriteRegister(const uint8_t *regAddr, uint16_t value);

    uint16_t LuxToDigital(uint16_t luxValue);

    uint16_t GetMultiplicationFactor();

    uint16_t ALS_CONF_0_REG = 0; // Hold the current config for the als

    static constexpr uint8_t ALS_CONF_0_COMMAND =   0x00;
    static constexpr uint8_t ALS_WH_COMMAND =       0x01;
    static constexpr uint8_t ALS_WL_COMMAND =       0x02;
    static constexpr uint8_t POWER_SAVING_COMMAND = 0x03;
    static constexpr uint8_t ALS_COMMAND =          0x04;
    static constexpr uint8_t WHITE_COMMAND =        0x05;
    static constexpr uint8_t ALS_INT_COMMAND =      0x06;

    static constexpr uint8_t ALS_CONF_GAIN_BIT =    11;
    static constexpr uint8_t ALS_CONF_IT_BIT =      6;
    static constexpr uint8_t ALS_CONF_PERS_BIT =    4;
    static constexpr uint8_t ALS_CONF_INT_EN_BIT =  1;
    static constexpr uint8_t ALS_CONF_SD_BIT =      0;

    gainValues_t m_gainValue = gainValues_t::ALS_GAIN_X1_8;                // Gain X1 is default
    integrationTime_t m_integrationTime = integrationTime_t::ALS_IT_100MS; // 100 ms is default

    i2c_inst_t *m_i2cX = nullptr;
    uint8_t m_deviceAddress = 0x10;
    uint8_t m_rxBuffer[2] = {0};

    static constexpr float MAX_RESOLUTION = 0.0036; // Resolution with a gain X2 and integration time of 800ms

    uint32_t m_defaultTimeout = 100000; // 100 ms is the default timeout for I2C messages
};

#endif /* VEML770_H */