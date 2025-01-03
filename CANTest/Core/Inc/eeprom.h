#ifndef EEPROM_H
#define EEPROM_H

#include "stm32f1xx_hal.h"

#define EEPROM_ADDR 0xA0  // Device address for 24C EEPROM (Adjust if needed)

// Function prototypes
void EEPROM_Init(void);
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t address, uint8_t data);
uint8_t EEPROM_ReadByte(uint16_t address);

#endif /* EEPROM_H */
