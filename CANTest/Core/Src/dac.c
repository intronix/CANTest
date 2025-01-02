#include "dac.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

void MCP4725_Write(uint16_t value)
{
    // Ensure value is within 12-bit range
    value &= 0x0FFF;

    uint8_t packet[3];
    packet[0] = MCP4725_CMD_WRITEDAC;
    packet[1] = value / 16;        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
    packet[2] = (value % 16) << 4; // Lower data bits (D3.D2.D1.D0.x.x.x.x)

    // Send the data over I2C
    HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, packet, 3, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MCP4725_SetVoltage(uint16_t voltage_mv)
{
    // Convert voltage in millivolts to raw 12-bit value (0-4095)
    // Using 3.0V (3000mV) reference
    uint32_t raw_value = (voltage_mv * 4095UL) / MCP4725_VREF_MV;
    
    // Ensure we don't exceed 12 bits
    if(raw_value > 4095) raw_value = 4095;
    
    MCP4725_Write((uint16_t)raw_value);
    return HAL_OK;
}

HAL_StatusTypeDef MCP4725_SetVoltageFloat(float voltage)
{
    // Ensure voltage is within valid range (0-3.0V)
    if(voltage < 0.0f) voltage = 0.0f;
    if(voltage > MCP4725_VREF) voltage = MCP4725_VREF;
    
    // Convert voltage to raw 12-bit value
    uint16_t raw_value = (uint16_t)((voltage * 4095.0f) / MCP4725_VREF);
    
    MCP4725_Write(raw_value);
    return HAL_OK;
}
