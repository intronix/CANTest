#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "main.h"

// CAN Message IDs
#define CAN_TX_ID 0x7E2

// Timing definitions
#define CAN_TX_INTERVAL 100 // 100ms interval

// External declarations for variables needed in main
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[1];
extern uint32_t TxMailbox;
extern uint32_t errorCounter;
extern uint8_t messageReceived;
extern uint32_t can_tx_counter;
extern uint32_t current_baud_rate;

// Error counter variables
extern uint8_t can_tx_error_counter;
extern uint8_t can_rx_error_counter;
extern uint32_t can_last_error_code;

// Function prototypes
void MX_CAN_Init(void);
void CAN_Handler_Init(void);
void CAN_Config_Filter(void);
HAL_StatusTypeDef CAN_Start(void);
void CAN_Handler_Process(void);
uint32_t CAN_Get_Error_Status(void);
void CAN_Send_Periodic_Message(void);
uint32_t CAN_Calculate_Baud_Rate(void);
void CAN_Update_Error_Counters(void);

#endif /* CAN_HANDLER_H */
