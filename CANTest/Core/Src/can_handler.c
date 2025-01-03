#include "can_handler.h"

// Global variables
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0xDE, 0xAD, 0xBE, 0xEF}; // Initialize with the required data
uint8_t RxData[8];  // Changed to 8 bytes to receive full CAN frame
uint32_t TxMailbox;
uint32_t txErrorCounter = 0;
uint32_t rxErrorCounter = 0;
uint32_t current_baud_rate = 0;
uint32_t rxMessageCounter = 0;
uint32_t txMessageCounter = 0;
uint8_t messageReceived = 0;
static uint32_t lastTxTime = 0;
static uint32_t lastBlinkTime = 0;  // For LED blinking
static uint8_t ledState = 0;        // Track LED state

void CAN_Handler_Init(void)
{
    // Enable interrupts
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

    // Initialize CAN parameters
    TxHeader.DLC = 4;  // 4 bytes of data
    TxHeader.IDE = CAN_ID_STD; // Standard ID
    TxHeader.RTR = CAN_RTR_DATA; // Data frame
    TxHeader.StdId = CAN_TX_ID; // ID 0x7E1
    
    txErrorCounter = 0;
    rxErrorCounter = 0;
    rxMessageCounter = 0;
    txMessageCounter = 0;
    messageReceived = 0;
    lastTxTime = 0;
    lastBlinkTime = 0;
    ledState = 0;
    
    // Calculate and store initial baud rate
    CAN_Calculate_Baud_Rate();
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
    uint32_t currentTime = HAL_GetTick();
    uint32_t errorStatus = HAL_CAN_GetError(&hcan);
    
    if(messageReceived)
    {
        // Non-blocking LED blink
        if (currentTime - lastBlinkTime >= 500)  // 500ms interval
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);  // Toggle LED
            lastBlinkTime = currentTime;
            ledState = !ledState;
            
            // If LED has completed one blink cycle (ON->OFF)
            if (ledState == 0)
            {
                messageReceived = 0; // Clear message received flag
                txErrorCounter = 0;
                rxErrorCounter = 0;
            }
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
            txMessageCounter++;  // Increment transmit counter on successful transmission
            lastTxTime = currentTime;
        }
        else
        {
            txErrorCounter++; // Increment tx error counter
        }
    }
}

uint32_t CAN_Get_TxErrorCounter(void)
{
    return ((hcan.Instance->ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
}

uint32_t CAN_Get_RxErrorCounter(void)
{
    return ((hcan.Instance->ESR & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
}

uint32_t CAN_Get_Error_Status(void)
{
    return ((CAN_Get_TxErrorCounter() << 16) | CAN_Get_RxErrorCounter());
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

// HAL CAN Callbacks
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        rxMessageCounter++;  // Increment receive counter on successful reception
        rxErrorCounter = 0; // Reset rx error counter on successful reception
        messageReceived = 1; // Set flag to indicate message received
    }
    else
    {
        rxErrorCounter++; // Increment rx error counter
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    // Read hardware error counters
    txErrorCounter = CAN_Get_TxErrorCounter();
    rxErrorCounter = CAN_Get_RxErrorCounter();
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
