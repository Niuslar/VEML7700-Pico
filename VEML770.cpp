
#include "VEML770.h"

VEML7700::VEML7700()
{
}

VEML7700::~VEML7700() {}

void VEML7700::Init(uint8_t sdaPin, uint8_t sclPin, i2c_inst_t *i2cInstance, uint32_t baudRate)
{
    if (i2cInstance == nullptr)
        return;

    /* Setup I2C and GPIOs */
    m_i2cX = i2cInstance;
    i2c_init(i2cInstance, baudRate);
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    /* Setup VEML770 sensor */
    /* Default values are
     *  ALS GAIN = x1
     *  ALS IT   = 100 ms
     *  ALS PERS = 1
     *  ALS INT EN = 0
     *  ALS SD (shutdown) = 0 (Turn on when Init is called)
     */

    ALS_CONF_0_REG = 0;
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);

    /* Wait 2.5ms before the first reading */
    sleep_us(2500);
}

float VEML7700::ReadLux()
{
    uint16_t wholeWhite = ReadRegister(&ALS_COMMAND);

    return (float)wholeWhite;
}

void VEML7700::Enable(bool enable)
{
    /* Clear bits */
    ALS_CONF_0_REG &= ~(1 << ALS_CONF_SD_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= (enable << ALS_CONF_SD_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::SetGain(gainValues_t newGain)
{
    /* Clear bits */
    ALS_CONF_0_REG &= ~(3 << ALS_CONF_GAIN_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= ((uint8_t)newGain << ALS_CONF_GAIN_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::SetIntegrationTiming(integrationTime_t integrationTiming)
{
    /* Clear bits */
    ALS_CONF_0_REG &= ~(15 << ALS_CONF_IT_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= ((uint8_t)integrationTiming << ALS_CONF_IT_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::SetPersistenceProtect(persProt_t protNumber)
{
    /* Clear bits */
    ALS_CONF_0_REG &= ~(3 << ALS_CONF_PERS_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= ((uint8_t)protNumber << ALS_CONF_PERS_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::EnableInterrupt(bool enable)
{
    /* Clear bits */
    ALS_CONF_0_REG &= ~(1 << ALS_CONF_INT_EN_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= ((uint8_t)enable << ALS_CONF_INT_EN_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::SetHighLimit(uint16_t highThreshold)
{
    WriteRegister(&ALS_WH_COMMAND, highThreshold);
}

void VEML7700::SetLowLimit(uint16_t lowThreshold)
{
    /* Write config */
    WriteRegister(&ALS_WL_COMMAND, lowThreshold);
}

/* Private Methods */

uint16_t VEML7700::ReadRegister(const uint8_t *regAddr)
{
    i2c_write_timeout_us(m_i2cX, m_deviceAddress, regAddr, 1, true, m_defaultTimeout);
    i2c_read_timeout_us(m_i2cX, m_deviceAddress, m_rxBuffer, 2, false, m_defaultTimeout);

    uint16_t data = (m_rxBuffer[1] << 8) | (m_rxBuffer[0] & 0xFF);
    return data;
}

void VEML7700::WriteRegister(const uint8_t *regAddr, uint16_t value)
{
    const int messageLen = 3;
    uint8_t message[messageLen]; // Include command and data to send
    message[0] = *regAddr;
    message[1] = value >> 8;    // MSB
    message[2] = value && 0xFF; // LSB
    i2c_write_timeout_us(m_i2cX, m_deviceAddress, message, messageLen, true, m_defaultTimeout);
}