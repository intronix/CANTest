#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "main.h"

// CAN Message IDs
#define CAN_TX_ID 0x7E1

// Timing definitions
#define CAN_TX_INTERVAL 100 // 100ms interval

// External declarations for variables needed in main
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];  // Changed to 8 bytes to match implementation
extern uint32_t TxMailbox;
extern uint32_t txErrorCounter;
extern uint32_t rxErrorCounter;
extern uint32_t current_baud_rate;
extern uint32_t rxMessageCounter;
extern uint32_t txMessageCounter;
extern uint8_t messageReceived;

// Function prototypes
void CAN_Handler_Init(void);
void CAN_Config_Filter(void);
HAL_StatusTypeDef CAN_Start(void);
void CAN_Handler_Process(void);
uint32_t CAN_Get_Error_Status(void);
void CAN_Send_Periodic_Message(void);
uint32_t CAN_Calculate_Baud_Rate(void);
uint32_t CAN_Get_TxErrorCounter(void);
uint32_t CAN_Get_RxErrorCounter(void);

// Callback functions
void CAN_RX_Callback(CAN_HandleTypeDef *hcan);
void CAN_Error_Callback(CAN_HandleTypeDef *hcan);

#endif /* CAN_HANDLER_H */
