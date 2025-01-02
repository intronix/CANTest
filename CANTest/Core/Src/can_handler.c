#include "can_handler.h"

// Global variables
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[1];
uint32_t TxMailbox;
uint32_t errorCounter = 0;
uint8_t messageReceived = 0;

void CAN_Handler_Init(void)
{
    // Enable interrupts
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

    // Initialize CAN parameters
    TxHeader.DLC = 1;  // data length
    errorCounter = 0;
    messageReceived = 0;
}

void CAN_Config_Filter(void)
{
    CAN_FilterTypeDef canfilterconfig;
    
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0;
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0;
    canfilterconfig.FilterMaskIdLow = 0;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    
    if(HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef CAN_Start(void)
{
    HAL_StatusTypeDef status;
    
    // Start CAN
    status = HAL_CAN_Start(&hcan);
    if(status != HAL_OK)
    {
        return status;
    }
    
    // Activate notifications
    status = HAL_CAN_ActivateNotification(&hcan, 
        CAN_IT_RX_FIFO0_MSG_PENDING | 
        CAN_IT_ERROR | 
        CAN_IT_BUSOFF | 
        CAN_IT_LAST_ERROR_CODE);
    
    return status;
}

void CAN_Handler_Process(void)
{
    uint32_t errorStatus = HAL_CAN_GetError(&hcan);
    if(errorStatus != HAL_CAN_ERROR_NONE || errorCounter > 0)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);  // Toggle LED
        HAL_Delay(100);  // 100ms delay for error indication
    }
    else if(messageReceived)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);  // Toggle LED
        HAL_Delay(500);  // 500ms delay for message received indication
        messageReceived = 0; // Clear message received flag
    }
}

uint32_t CAN_Get_Error_Status(void)
{
    return HAL_CAN_GetError(&hcan);
}

// HAL CAN Callbacks
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        errorCounter = 0; // Reset error counter on successful reception
        messageReceived = 1; // Set flag to indicate message received
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    errorCounter++; // Increment error counter
    messageReceived = 0; // Clear message received flag on error
}

// Interrupt Handlers
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}
