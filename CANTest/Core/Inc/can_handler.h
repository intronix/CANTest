#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "main.h"

// External declarations for variables needed in main
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[1];
extern uint32_t TxMailbox;
extern uint32_t errorCounter;
extern uint8_t messageReceived;

// Function prototypes
void CAN_Handler_Init(void);
void CAN_Config_Filter(void);
HAL_StatusTypeDef CAN_Start(void);
void CAN_Handler_Process(void);
uint32_t CAN_Get_Error_Status(void);

// Callback functions
void CAN_RX_Callback(CAN_HandleTypeDef *hcan);
void CAN_Error_Callback(CAN_HandleTypeDef *hcan);

#endif /* CAN_HANDLER_H */
