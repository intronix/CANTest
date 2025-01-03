#ifndef EEPROM_H
#define EEPROM_H

#include "stm32f1xx_hal.h"

#define EEPROM_ADDR 0xA0  // Device address for 24C EEPROM (Adjust if needed)

// EEPROM Address definitions for DAC calibration
#define EEPROM_DAC_LOW_ADDR  0x00    // Address for storing DAC low value
#define EEPROM_DAC_HIGH_ADDR 0x02    // Address for storing DAC high value

// Function prototypes
void EEPROM_Init(void);
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t address, uint8_t data);
uint8_t EEPROM_ReadByte(uint16_t address);
HAL_StatusTypeDef EEPROM_WriteWord(uint16_t address, uint16_t data);
uint16_t EEPROM_ReadWord(uint16_t address);
HAL_StatusTypeDef EEPROM_WriteWordArray(uint16_t address, uint16_t *data, uint16_t size);
uint16_t *EEPROM_ReadWordArray(uint16_t address, uint16_t size);

#endif /* EEPROM_H */
