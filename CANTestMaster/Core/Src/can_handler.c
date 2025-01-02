#include "can_handler.h"

// Global variables
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0xDE, 0xAD, 0xBE, 0xEF}; // Initialize with the required data
uint8_t RxData[1];
uint32_t TxMailbox;
uint32_t errorCounter = 0;
uint8_t messageReceived = 0;
uint32_t can_tx_counter = 0;
uint32_t current_baud_rate = 0;
static uint32_t lastTxTime = 0;

// Error counter variables
uint8_t can_tx_error_counter = 0;
uint8_t can_rx_error_counter = 0;
uint32_t can_last_error_code = 0;

void CAN_Update_Error_Counters(void)
{
    // Read the CAN Error Status Register (ESR)
    uint32_t esr = READ_REG(hcan.Instance->ESR);
    
    // Extract TX and RX error counters
    can_tx_error_counter = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    can_rx_error_counter = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    
    // Store the last error code
    can_last_error_code = (esr & CAN_ESR_LEC);
    
    // Check for error conditions
    if (can_tx_error_counter > 127 || can_rx_error_counter > 127) {
        errorCounter++;  // Increment global error counter if either counter exceeds warning level
    }
}

uint32_t CAN_Calculate_Baud_Rate(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = hcan.Init.Prescaler;
    uint32_t bs1 = ((hcan.Init.TimeSeg1 >> 16) & 0x0F) + 1;  // Convert BS1 value to actual TQ
    uint32_t bs2 = ((hcan.Init.TimeSeg2 >> 20) & 0x07) + 1;  // Convert BS2 value to actual TQ
    uint32_t total_tq = 1 + bs1 + bs2;  // 1 is for sync segment
    
    // Calculate baud rate: PCLK1 / (prescaler * total_tq)
    current_baud_rate = pclk1 / (prescaler * total_tq);
    
    return current_baud_rate;
}

void MX_CAN_Init(void)
{
    /* USER CODE BEGIN CAN_Init 0 */

    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 9;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* USER CODE BEGIN CAN_Init 2 */
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    canfilterconfig.FilterIdHigh = 0x103<<5;
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0x103<<5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN1 (master can)

    if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE END CAN_Init 2 */
}

void CAN_Handler_Init(void)
{
    // Initialize CAN peripheral using MX_CAN_Init
    MX_CAN_Init();

    // Calculate and store the actual baud rate
    current_baud_rate = CAN_Calculate_Baud_Rate();

    // Initialize CAN parameters
    TxHeader.DLC = 4;  // 4 bytes of data
    TxHeader.IDE = CAN_ID_STD; // Standard ID
    TxHeader.RTR = CAN_RTR_DATA; // Data frame
    TxHeader.StdId = CAN_TX_ID; // ID 0x7E2
    
    errorCounter = 0;
    messageReceived = 0;
    can_tx_counter = 0;
    lastTxTime = 0;
}

void CAN_Config_Filter(void)
{
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    canfilterconfig.FilterIdHigh = 0x103<<5;
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0x103<<5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN1 (master can)

    if(HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef CAN_Start(void)
{
    HAL_StatusTypeDef status;
    
    // Start the CAN peripheral
    status = HAL_CAN_Start(&hcan);
    if (status != HAL_OK)
    {
        return status;
    }
    
    // Activate notifications for error states
    status = HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR_WARNING |
                                               CAN_IT_ERROR_PASSIVE |
                                               CAN_IT_BUSOFF |
                                               CAN_IT_LAST_ERROR_CODE |
                                               CAN_IT_ERROR);
    return status;
}

void CAN_Handler_Process(void)
{
    // Update error counters
    CAN_Update_Error_Counters();
    
    // Check for received messages
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
        {
            messageReceived = 1;
        }
        else
        {
            errorCounter++;
        }
    }
    
    // Send periodic message
    CAN_Send_Periodic_Message();
}

void CAN_Send_Periodic_Message(void)
{
    uint32_t currentTime = HAL_GetTick();
    
    // Check if it's time to send the message
    if(currentTime - lastTxTime >= CAN_TX_INTERVAL)
    {
        // Send the message
        if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK)
        {
            lastTxTime = currentTime;
            can_tx_counter++;
        }
    }
}

uint32_t CAN_Get_Error_Status(void)
{
    return HAL_CAN_GetError(&hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        errorCounter = 0;
        messageReceived = 1;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    errorCounter++;
    messageReceived = 0;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}
