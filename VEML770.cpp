
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
     *  ALS GAIN = x1/8  -> To avoid oversaturation as explained in application notes
     *  ALS IT   = 100 ms
     *  ALS PERS = 1
     *  ALS INT EN = 0
     *  ALS SD (shutdown) = 0 (Turn on when Init is called)
     */

    ALS_CONF_0_REG = 0 | ((uint8_t)m_gainValue << ALS_CONF_GAIN_BIT);
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);

    /* Wait 2.5ms before the first reading */
    sleep_us(2500);
}

float VEML7700::ReadLux()
{
    uint16_t alsDigitalValue = ReadRegister(&ALS_COMMAND);

    uint16_t multiplicationFactor = GetMultiplicationFactor();

    float actualResolution = MAX_RESOLUTION * multiplicationFactor;

    float luxReading = alsDigitalValue * actualResolution;

    return luxReading;
}

void VEML7700::Enable(bool enable)
{
    /* Clear bits */
    ALS_CONF_0_REG &= ~(1 << ALS_CONF_SD_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= (!enable << ALS_CONF_SD_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::SetGain(gainValues_t newGain)
{
    m_gainValue = newGain;
    /* Clear bits */
    ALS_CONF_0_REG &= ~(3 << ALS_CONF_GAIN_BIT);

    /* Set bits */
    ALS_CONF_0_REG |= ((uint8_t)newGain << ALS_CONF_GAIN_BIT);

    /* Write config */
    WriteRegister(&ALS_CONF_0_COMMAND, ALS_CONF_0_REG);
}

void VEML7700::SetIntegrationTiming(integrationTime_t integrationTiming)
{
    m_integrationTime = integrationTiming;

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
    uint16_t digitalValue = LuxToDigital(highThreshold);
    WriteRegister(&ALS_WH_COMMAND, digitalValue);
}

void VEML7700::SetLowLimit(uint16_t lowThreshold)
{
    uint16_t digitalValue = LuxToDigital(lowThreshold);
    WriteRegister(&ALS_WL_COMMAND, digitalValue);
}

uint8_t VEML7700::CheckInterruptStatus()
{
    uint16_t interruptRegister = ReadRegister(&ALS_INT_COMMAND);

    if (interruptRegister & (1 << 15))
    {
        // Low threshold
        return 1;
    }
    else if (interruptRegister & (1 << 14))
    {
        // High threshol
        return 2;
    }

    return 0;
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
    message[1] = value & 0xFF; // LSB
    message[2] = value >> 8;   // MSB
    i2c_write_timeout_us(m_i2cX, m_deviceAddress, message, messageLen, false, m_defaultTimeout);
}

uint16_t VEML7700::LuxToDigital(uint16_t luxValue)
{
    uint16_t multiplicationFactor = GetMultiplicationFactor();
    float actualResolution = MAX_RESOLUTION * multiplicationFactor;

    float digitalValue = luxValue / actualResolution;

    return (uint16_t)digitalValue;
}

uint16_t VEML7700::GetMultiplicationFactor()
{
    uint16_t multiplicationFactor = 1;

    switch (m_integrationTime)
    {
    case integrationTime_t::ALS_IT_400MS:
        multiplicationFactor *= 2;
        break;
    case integrationTime_t::ALS_IT_200MS:
        multiplicationFactor *= 4;
        break;
    case integrationTime_t::ALS_IT_100MS:
        multiplicationFactor *= 8;
        break;
    case integrationTime_t::ALS_IT_50MS:
        multiplicationFactor *= 16;
        break;
    case integrationTime_t::ALS_IT_25MS:
        multiplicationFactor *= 32;
        break;
    default:
        break;
    }

    switch (m_gainValue)
    {
    case gainValues_t::ALS_GAIN_X1:
        multiplicationFactor *= 2;
        break;
    case gainValues_t::ALS_GAIN_X1_4:
        multiplicationFactor *= 8;
        break;
    case gainValues_t::ALS_GAIN_X1_8:
        multiplicationFactor *= 16;
        break;
    default:
        break;
    }

    return multiplicationFactor;
}