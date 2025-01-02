#ifndef DAC_H
#define DAC_H

#include "main.h"

// MCP4725 I2C Address
#define MCP4725_ADDR 0x61 << 1  // Left-shift for STM32 HAL

// MCP4725 Commands
#define MCP4725_CMD_WRITEDAC 0x40      // Write to DAC register
#define MCP4725_CMD_WRITEDACEEPROM 0x60 // Write to DAC register and EEPROM

// Reference voltage
#define MCP4725_VREF 3.0f              // Reference voltage in volts
#define MCP4725_VREF_MV 3000           // Reference voltage in millivolts

// Function prototypes
void MCP4725_Write(uint16_t value);
HAL_StatusTypeDef MCP4725_SetVoltage(uint16_t voltage_mv);
HAL_StatusTypeDef MCP4725_SetVoltageFloat(float voltage);  // New function for float voltage

#endif /* DAC_H */
