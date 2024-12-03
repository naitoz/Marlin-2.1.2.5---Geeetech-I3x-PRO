/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(CAN_TOOLHEAD)

#include "../platforms.h"
#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../gcode/queue.h"
#include "../../module/temperature.h"
#include "../../libs/numtostr.h"
#include "../../inc/MarlinConfig.h"
#include "../../feature/controllerfan.h"
#include "../../core/serial.h"

#include "../SHARED/FDCAN.h"

#define CAN_DEBUG // Enable to show CAN debug messages

#define FDCAN_RX_FIFO0_MASK           (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N) // Fifo 0: Message lost | Fifo full | New message
#define FDCAN_RX_FIFO1_MASK           (FDCAN_IR_RF1L | FDCAN_IR_RF1F | FDCAN_IR_RF1N) // Fifo 1: Message lost | Fifo full | New message
#define FDCAN_RX_FIFO_MASK            (FDCAN_RX_FIFO0_MASK | FDCAN_RX_FIFO1_MASK)     // Fifo  : Message lost | Fifo full | New message
#define HAL_TIM_FLAG_ALL              (TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4 | TIM_FLAG_UPDATE | TIM_FLAG_BREAK | TIM_FLAG_BREAK2 | TIM_FLAG_TRIGGER | TIM_FLAG_COM)

#define CAN_TIMESTAMP_NO              7777

// IMPORTANT NOTE
// ===============
// 1. In \Users\<username>\.platformio\packages\framework-arduinoststm32@4.20600.231001\libraries\SrcWrapper\src\HardwareTimer.cpp
//    Add function "__weak" in front of "void TIM16_IRQHandler(void)"

FDCAN_HandleTypeDef hfdcan2;

bool Head_Not_configured = true;               // Check if the configuration was send to the head
char gcode_type[4]  = { 'D', 'G', 'M', 'D' }; // This way an extended ID is always > 2048
//char * codeP; // code can walk along with progress in can_gcode
//char can_gcode[MAX_CMD_SIZE + 20];
char can_gcode_buffer[MAX_CMD_SIZE + 20];   // Gcode that is being received in several CAN messages
volatile uint32_t CAN_Error_Code        = 0; // Signals an CAN error occured, report to host
volatile uint32_t Old_Error_Code        = 0; // Remember last error code so messages don't repeat
volatile uint32_t gcode_counter         = 0; // Count received gcode lines (debugging)
volatile uint32_t CAN_message_counter   = 0; // Count sent CAN message (debugging)
volatile bool send_gcode_count      = false;
volatile uint32_t t[5]  = { 0, 0, 0, 0, 0 }; // Timestamps for time sync
volatile bool CAN_request_time_sync = false; // Request a timestamp
volatile uint32_t time_offset           = 0; // Time offset in micro seconds in relation to host time

extern "C" void TIM16_IRQHandler(void);
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

void CAN_Add_Parameters(uint32_t parameter, float value, char *codeP) { // Add received CAN parameters to can_gcode string, used in ISR!
  *codeP++ = char(parameter + 64); // Add parameter 'X'

  if (!isnan(value)) { // No value behind parameter if value is "Not A Number"
    if (value < 0.0) {
      *codeP++ = '-'; // Add minus sign to gcode string
      value = -value; // Make value positive
    }

    // Convert float to string
    uint32_t i1 = int(value); // Get integer part of float
    itoa(i1, codeP, 10);      // Assume 1-4 digits
    codeP += strlen(codeP);   // Adjust pointer
    value = (value - i1);     // Get fractional part

    uint32_t j = 0;
    uint32_t flag = 0;
    for (int i = 0; i < 5; i++) { // 6 digits, 1 extra for rounding
      value *= 10.0;
      i1 = int(value);
      j = (10 * j)  + i1;
      value = value - i1;
    }

    if (value >= 0.5)
      j++; // Round up

    if (j) {
      *codeP++ = '.';
      if (j < 10000) {
        *codeP++ = '0';
        if (j < 1000) {
          *codeP++ = '0';
          if (j < 100) {
            *codeP++ = '0';
              if (j < 10)
                *codeP++ = '0';
          }
        }
      }

      while (!(j % 10)) // Remove trailing zeros, 120 --> 12
        j = j / 10;

      itoa(j, codeP, 10);
    }
  }
}

HAL_StatusTypeDef CAN_Receive(uint32_t Fifo) { // ISR! Process received CAN message in interrupt handler!
  HAL_StatusTypeDef status = HAL_OK;
  static volatile uint32_t parameter_counter = 0;
  char * codeP = can_gcode_buffer;  // Put gcode pointer to start of can_gcode_buffer;
  uint8_t can_rxbuf[8];

  FDCAN_RxHeaderTypeDef CanRxHeader;  // Receive CAN message buffer

  status = HAL_FDCAN_GetRxMessage(&hfdcan2, Fifo, &CanRxHeader, can_rxbuf);
  if (status != HAL_OK)
    return status;

  uint32_t identifier = CanRxHeader.Identifier;
  if (CanRxHeader.IdType == FDCAN_EXTENDED_ID) { // Receiving new gcode

    if (((identifier >> IDENTIFIER_GCODE_NUMBER_OFFSET) & IDENTIFIER_GCODE_MASK) == ((GCODE_TYPE_M << (IDENTIFIER_GCODE_TYPE_OFFSET - IDENTIFIER_GCODE_NUMBER_OFFSET)) + CAN_TIMESTAMP_NO)) { // TIME SYNC RESPONSE MESSAGE
      uint32_t *uint32p = (uint32_t *)can_rxbuf;
      t[3] = micros(); // Record time sync response message receive time
      t[1] = *uint32p++;
      t[2] = *uint32p;
      return status;
    }

    // Check for M997 reset command
    if (((identifier >> IDENTIFIER_GCODE_NUMBER_OFFSET) & IDENTIFIER_GCODE_MASK) == ((GCODE_TYPE_M << (IDENTIFIER_GCODE_TYPE_OFFSET - IDENTIFIER_GCODE_NUMBER_OFFSET)) + 997)) { // CODE IS M997, RESTART HEAD UNCONDITIONALLY
      flashFirmware(0);
      while (1);
    }

    if (((identifier >> IDENTIFIER_GCODE_NUMBER_OFFSET) & IDENTIFIER_GCODE_MASK) == ((GCODE_TYPE_M << (IDENTIFIER_GCODE_TYPE_OFFSET - IDENTIFIER_GCODE_NUMBER_OFFSET)) + 115))
      CAN_request_time_sync = true; // M115 --> Request a time sync from the host

    memset(can_gcode_buffer, 0, MAX_CMD_SIZE);   // Clear buffer

    if (parameter_counter) // Previous gcode must be complete, no more pending parameters
      CAN_Error_Code |= CAN_ERROR_INCOMPLETE_GCODE_RECEIVED;

    parameter_counter = (identifier >> IDENTIFIER_PARAMETER_COUNT_OFFSET) & IDENTIFIER_PARAMETER_COUNT_MASK; // Get parameter_counter, bits 25, 26, 27 of 29 bit extended identifier

    *codeP++ = gcode_type[(identifier >> IDENTIFIER_GCODE_TYPE_OFFSET) & IDENTIFIER_GCODE_TYPE_MASK]; // "G"
    itoa((identifier >> IDENTIFIER_GCODE_NUMBER_OFFSET) & IDENTIFIER_GCODE_NUMBER_MASK, codeP, 10); // G"92"

    // Check if head configuration was received, marked by M710 command
    if (((identifier >> IDENTIFIER_GCODE_NUMBER_OFFSET) & IDENTIFIER_GCODE_MASK) == ((GCODE_TYPE_M << (IDENTIFIER_GCODE_TYPE_OFFSET - IDENTIFIER_GCODE_NUMBER_OFFSET)) + 710)) { // CODE IS M710, M=2, INDICATES CONFIGURATION WAS RECEIVED
      Head_Not_configured = false; // HEAD SETUP COMPLETE
      send_gcode_count    = true;  // Report the current received gcode count to the host (debugging)
    }
  }

  // If present, add parameters to the received gcode
  if (parameter_counter && CanRxHeader.DataLength && ((identifier >> IDENTIFIER_PARAMTER1_OFFSET) & IDENTIFIER_PARAMETER_MASK)) { // Get 1st parameter, make sure it's not empty.
    codeP += strlen(codeP); // There is already data in the buffer, adjust the pointer
    CAN_Add_Parameters(identifier & IDENTIFIER_PARAMETER_MASK, *(float*)(can_rxbuf), codeP);
    parameter_counter--;

    if (parameter_counter && (CanRxHeader.DataLength == FDCAN_DLC_BYTES_8) && ((identifier >> IDENTIFIER_PARAMTER2_OFFSET) & IDENTIFIER_PARAMETER_MASK)) { // Get 2nd parameter, make sure it's not empty.
      codeP += strlen(codeP);
      CAN_Add_Parameters((identifier >> IDENTIFIER_PARAMTER2_OFFSET) & IDENTIFIER_PARAMETER_MASK, *(float*)(can_rxbuf + 4), codeP);
      parameter_counter--;
    }
  }

  if (parameter_counter == 0) { // Gcode is complete, process the gcode
    gcode_counter++;

    uint32_t ms = millis();
    bool queue_status;
    do {  // BLOCKING! Wait for free Marlin command buffer for max 50ms
      queue_status = queue.enqueue_one(can_gcode_buffer);
    }
    while (!queue_status && (millis() - ms < 50));

    if (!queue_status) // Could not store the received CAN command in Marlin command buffer
      CAN_Error_Code |= CAN_ERROR_MARLIN_CMD_BUFFER_OVERFLOW;
  }

  return HAL_OK;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) { // ISR! Override "weak" function
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) { // Check the error status first, it might be cleared by reading a message
    CAN_Error_Code |= CAN_ERROR_FIFO_OVERFLOW;
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST); // Clear interrupt flag
  }

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    CAN_Receive(FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) { // ISR! Override "weak" function
  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) { // Check the error status first, it might be cleared by reading a message
    CAN_Error_Code |= CAN_ERROR_FIFO_OVERFLOW;
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_IT_RX_FIFO1_MESSAGE_LOST); // Clear interrupt flag
  }

if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
  CAN_Receive(FDCAN_RX_FIFO1);
}

void TIM16_IRQHandler(void) { // FDCAN INTERRUPT HANDLER, OVERRIDE WEAK FUNCTION
  if ((HardwareTimer_Handle[TIMER16_INDEX]) && (HardwareTimer_Handle[TIMER16_INDEX]->handle.Instance->SR & HAL_TIM_FLAG_ALL)) // Interrupt was caused by timer
    HAL_TIM_IRQHandler(&HardwareTimer_Handle[TIMER16_INDEX]->handle); // Forward to timer interrupt handler

  uint32_t RxFifoITs = hfdcan2.Instance->IR & FDCAN_RX_FIFO_MASK;
           RxFifoITs &= hfdcan2.Instance->IE;

  if (RxFifoITs) // Any Fifo read interrupts?
    HAL_FDCAN_IRQHandler(&hfdcan2); // Forward call to FDCAN interrupt handler
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle) { // Called automatically, configure GPIO for FDCAN, enable interrupts
  GPIO_InitTypeDef GPIO_InitStruct       = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();

  // FDCAN1/2 clock enable (1 clock for both CAN devices)
  if (__HAL_RCC_FDCAN_IS_CLK_DISABLED())
    __HAL_RCC_FDCAN_CLK_ENABLE();

  if (__HAL_RCC_GPIOB_IS_CLK_DISABLED())
  __HAL_RCC_GPIOB_CLK_ENABLE(); // ENABLE GPIO B CLOCK
  // FDCAN2 GPIO Configuration
  // PB0     ------> FDCAN2_RX
  // PB1     ------> FDCAN2_TX

  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; // Pin PB0 and Pin PB1
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);              // Port B

  // FDCAN2 interrupt Init
  HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, 1, 1);   // Set interrupt priority
  HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);           // Enable interrupt handler
}

/*
uint32_t ClockDivider              FDCAN kernel clock divider (common to all FDCAN instances, applied only at initialisation of first FDCAN instance)
                                   1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30
uint32_t FrameFormat               FDCAN_FRAME_CLASSIC / FDCAN_FRAME_FD_NO_BRS / FDCAN_FRAME_FD_BRS (BRS=Bit Rate Switch)
uint32_t Mode;                     FDCAN_MODE_NORMAL / FDCAN_MODE_RESTRICTED_OPERATION / FDCAN_MODE_BUS_MONITORING / FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_EXTERNAL_LOOPBACK
FunctionalState AutoRetransmission Retransmit if fault is deteced, values: ENABLE / DISABLE
FunctionalState TransmitPause      Waits for 2 bit times before transmitting the next frame to allow other notes to cut in, values: ENABLE / DISABLE
FunctionalState ProtocolException  Protocol Exception Handling, values: ENABLE(start bus integration) or DISABLE(send error frame)
uint32_t NominalPrescaler          Clock divider for generating the bit time quanta (1-512)
uint32_t NominalSyncJumpWidth      Maximum number of time quanta the FDCAN allows to lengthen or shorten a bit to perform resynchronization (1-128)
uint32_t NominalTimeSeg1           Number of time quanta in Bit Segment 1 (2-256)
uint32_t NominalTimeSeg2           Number of time quanta in Bit Segment 2 (2-128)
uint32_t DataPrescaler             Clock divider for generating the data bit time quanta (1-32)
uint32_t DataSyncJumpWidth         Max of time quanta allowed to lengthen or shorten a data bit to perform resynchronization (1-16)
uint32_t DataTimeSeg1              Number of time quanta in Data Bit Segment 1. (1-32)
uint32_t DataTimeSeg2              Number of time quanta in Data Bit Segment 2. (1-16)
uint32_t StdFiltersNbr             Number of standard Message ID filters. (0-28)
uint32_t ExtFiltersNbr             Number of extended Message ID filters. (0-8)
uint32_t TxFifoQueueMode           Tx FIFO/Queue Mode selection FDCAN_TX_FIFO_OPERATION / FDCAN_TX_QUEUE_OPERATION
*/

HAL_StatusTypeDef FDCAN2_Start(void) { // START THE FDCAN BUS
  HAL_StatusTypeDef status = HAL_OK;

  // FDCAN baud rate = Device Clock Frequency / Clock Divider / Prescaler / SJW + TSG1 + TSG2)
  // Baud rate = 64M / 1 / 4 / (1 + 10 + 5) = 1M
  // Baud rate = 64M / 1 / 8 / (1 + 10 + 5) = 500k
  // http://www.bittiming.can-wiki.info
  // https://www.kvaser.com/support/calculators/can-fd-bit-timing-calculator
  hfdcan2.Instance                  = FDCAN2;                  // FDCAN2 device
  hfdcan2.Init.ClockDivider         = FDCAN_CLOCK_DIV1;        // Clock divider 1 2 4 6 8 10 12 14 16 18 20 22 24 26 28 30
  hfdcan2.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;     // FDCAN_FRAME_CLASSIC / FDCAN_FRAME_FD_BRS / FDCAN_FRAME_FD_NO_BRS (Bit Rate Switching)
  hfdcan2.Init.Mode                 = FDCAN_MODE_NORMAL;       // FDCAN_MODE_NORMAL / FDCAN_MODE_EXTERNAL_LOOPBACK / FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_BUS_MONITORING / FDCAN_MODE_RESTRICTED_OPERATION
  hfdcan2.Init.AutoRetransmission   = DISABLE;                 // Auto Retransmission of message if error occured, can cause lockup
  hfdcan2.Init.TransmitPause        = DISABLE;                 // Transmit Pause to allow for other device to send message
  hfdcan2.Init.ProtocolException    = DISABLE;                 // ProtocolException

  hfdcan2.Init.NominalPrescaler     = hfdcan2.Init.DataPrescaler     =  4; // Arbitration/data clock prescaler (classic only)
  hfdcan2.Init.NominalSyncJumpWidth = hfdcan2.Init.DataSyncJumpWidth =  1; // Arbitration/data sync jump width
  hfdcan2.Init.NominalTimeSeg1      = hfdcan2.Init.DataTimeSeg1      = 10; // Arbitration/data period 1
  hfdcan2.Init.NominalTimeSeg2      = hfdcan2.Init.DataTimeSeg2      =  5; // Arbitration period 2

  hfdcan2.Init.StdFiltersNbr        = 2;                       // Number of standard frame filters
  hfdcan2.Init.ExtFiltersNbr        = 2;                       // Number of extended frame filters
  hfdcan2.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION; // Queue mode: FDCAN_TX_FIFO_OPERATION / FDCAN_TX_QUEUE_OPERATION

  status = HAL_FDCAN_Init(&hfdcan2);
  if (status != HAL_OK)
    return status;

  FDCAN_FilterTypeDef sFilterConfig; // Configure RX message filter

  sFilterConfig.IdType       = FDCAN_EXTENDED_ID; // Filter to FIFO1 if higest ID bit is set
  sFilterConfig.FilterIndex  = 0;                 // Exteneded filter ID 0
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // Filter to FIFO1
  sFilterConfig.FilterID1    = 0x10000000; // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  sFilterConfig.FilterID2    = 0x10000000; // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

  sFilterConfig.IdType       = FDCAN_EXTENDED_ID; // Filter all remaining messages to FIFO0
  sFilterConfig.FilterIndex  = 1;                 // Exteneded filter ID 1
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Remaining messages go to FIFO1
  sFilterConfig.FilterID1    = 0; // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  sFilterConfig.FilterID2    = 0; // Range: 0 - 0x1FFF FFFF, 0=don'tcare
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

  sFilterConfig.IdType       = FDCAN_STANDARD_ID; // Filter to FIFO1 if higest ID bit is set
  sFilterConfig.FilterIndex  = 0;                 // Standard filter ID 0
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1    = 0b10000000000; // Range: 0 - 0x7FF, 0=don't care
  sFilterConfig.FilterID2    = 0b10000000000; // Range: 0 - 0x7FF, 0=don't care
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

  sFilterConfig.IdType       = FDCAN_STANDARD_ID; // Filter all remaining messages to FIFO0
  sFilterConfig.FilterIndex  = 1;                 // Standard filter ID 1
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK; // FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Remaining to FIFO1
  sFilterConfig.FilterID1    = 0; // Range: 0 - 0x7FF, 0=don't care
  sFilterConfig.FilterID2    = 0; // Range: 0 - 0x7FF, 0=don't care
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

// Configure global filter
// status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  // Start the FDCAN module
  status = HAL_FDCAN_Start(&hfdcan2); // ===== START FDCAN2 DEVICE =====

  // Activate RX FIFO0 new message interrupt
  if (status == HAL_OK)
    status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,  0); // Calls TIM16_IRQHandler

  // Activate RX FIFO0 message lost interrupt
  if (status == HAL_OK)
    status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0); // Calls TIM16_IRQHandler

  // Activate RX FIFO1 new message interrupt
  if (status == HAL_OK)
    status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,  0); // Calls TIM16_IRQHandler

  // Activate RX FIFO1 message lost interrupt
  if (status == HAL_OK)
    status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 0); // Calls TIM16_IRQHandler

  return status;
}

/* ---INTERRUPT HANDLERS-------------------------------------------------------------
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan);

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs);
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);

void HAL_FDCAN_ClockCalibrationCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ClkCalibrationITs);
void HAL_FDCAN_TimestampWraparoundCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_TimeoutOccurredCallback(FDCAN_HandleTypeDef *hfdcan);
void HAL_FDCAN_TT_ScheduleSyncCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TTSchedSyncITs);
void HAL_FDCAN_TT_TimeMarkCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TTTimeMarkITs);
void HAL_FDCAN_TT_StopWatchCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t SWTime, uint32_t SWCycleCount);
void HAL_FDCAN_TT_GlobalTimeCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TTGlobTimeITs); */

HAL_StatusTypeDef CAN_Send_Message(bool TempUpdate) { // Called from temperature ISR!
  // Send a IO/temp report from the HEAD to the HOST
  HAL_StatusTypeDef status = HAL_OK;
  FDCAN_TxHeaderTypeDef CanTxHeader; // Transmit CAN message buffer

  // Initialize standard TxHeader values
  CanTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  CanTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // FDCAN_NO_TX_EVENTS / FDCAN_STORE_TX_EVENTS
  CanTxHeader.MessageMarker       = 0;                  // 0-0xFF for tracking messages in FIFO buffer
  CanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // FDCAN_ESI_PASSIVE / FDCAN_ESI_ACTIVE ???
  CanTxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME
  CanTxHeader.IdType              = FDCAN_STANDARD_ID;  // FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  CanTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // FDCAN_BRS_OFF / FDCAN_BRS_ON

  uint8_t can_tx_buffer[8];
  if (TempUpdate) {                      // Add temperature to CAN message, 4 bytes
    float * fp = (float *)can_tx_buffer; // Point to CAN tx buffer
    *fp = thermalManager.degHotend(0);   // Copy temp to can_tx_buffer
    CanTxHeader.DataLength = FDCAN_DLC_BYTES_4; // Hotend temp in data only, bytes, FDCAN_DLC_BYTES_4 = (4 << 16)!
  }
  else
    CanTxHeader.DataLength = FDCAN_DLC_BYTES_0; // 0 data byte CAN message
  bool request_time_sync = CAN_request_time_sync;
  uint32_t VirtualIO = (Head_Not_configured       << CAN_REQUEST_SETUP_BIT) | // Report Head is not configured
                       (READ(Z_MIN_PROBE_PIN)     << CAN_PROBE_BIT)         | // Report probe status
                       (READ(FILAMENT_RUNOUT_PIN) << CAN_FILAMENT_BIT)      | // Report filament detector status
                       (request_time_sync         << CAN_REQUEST_TIME_SYNC_BIT) |
                       ((!!CAN_Error_Code)        << CAN_ERROR_BIT);          // Report error (if any)
 // IRON, ADD PINS YOU WANT TO MONITOR HERE

  CanTxHeader.Identifier = ((CanTxHeader.Identifier ^ STDID_FIFO_BIT) & STDID_FIFO_BIT) + VirtualIO; // Toggle FIFO bit for receiver filtering
  uint32_t ms = millis();
  while ((HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) && (millis() - ms < 25)) { } // BLOCKING! Wait max 50ms for free TX FIFO slot

  if (request_time_sync) { // For time sync, wait until all message TX slots are empty before sending
    CAN_request_time_sync = false;
    ms = millis();
    while ((HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) < 3) && (millis() - ms < 25)) { } // BLOCKING! Wait max 50ms for free TX FIFO slot
    t[0] = micros(); // Store time sync message send time
  }
  status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CanTxHeader, can_tx_buffer); // Send FDCAN message
  CAN_message_counter++;
  return status;
}

HAL_StatusTypeDef CAN_Send_String(const char * message) { // Send string message to host, string should end on '\n'
  HAL_StatusTypeDef status = HAL_OK;
  FDCAN_TxHeaderTypeDef CanTxHeader; // Transmit CAN message buffer

  // Initialize standard TxHeader values
  CanTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  CanTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // FDCAN_NO_TX_EVENTS / FDCAN_STORE_TX_EVENTS
  CanTxHeader.MessageMarker       = 0;                  // 0-0xFF for tracking messages in FIFO buffer
  CanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // FDCAN_ESI_PASSIVE / FDCAN_ESI_ACTIVE ???
  CanTxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME
  CanTxHeader.IdType              = FDCAN_STANDARD_ID;  // FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  CanTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // FDCAN_BRS_OFF / FDCAN_BRS_ON

  uint8_t can_tx_buffer[8];
  int remaining = strlen(message); // Length of string
  int index = 0;

  #ifdef CAN_DEBUG
    SERIAL_ECHOPGM(">>> SENDING STRING MESSAGE TO HOST: ", message); // IRON, SHOULD HAVE '\n' AT THE END, DEBUGGING
  #endif

  while (remaining > 0) { // Keep sending message if there are more characters to send
    int c = MIN(8, remaining); // Max 8 character at a time
    remaining -= c; // Reduce character counter
    CanTxHeader.DataLength = (c << DATALENGTH_OFFSET); // Max message length is 8 bytes, offset is 16 bits into the DataLength variable

    for (int i = 0; i < c; i++)
      can_tx_buffer[i] = message[index++]; // Copy string data to CAN buffer

    uint32_t VirtualIO = (READ(Z_MIN_PROBE_PIN) << CAN_PROBE_BIT)        | // Report probe status
                         (READ(FILAMENT_RUNOUT_PIN) << CAN_FILAMENT_BIT) | // Report filament detector status
                         ((!!CAN_Error_Code) << CAN_ERROR_BIT)           | // Report error (if any)
                         (1 << CAN_STRING_MESSAGE_BIT);                    // Set string message bit

    CanTxHeader.Identifier = ((CanTxHeader.Identifier ^ 0b10000000000) & 0b10000000000) + VirtualIO; // Toggle FIFO bit for receiver filtering
    uint32_t ms = millis();
    while ((HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) < 2) && (millis() - ms < 50)) { } // BLOCKING! Wait for free TX FIFO slot, with 1 spare slot for urgent messages

    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CanTxHeader, can_tx_buffer); // Send message
    CAN_message_counter++;
  }

  return status;
}

uint32_t last_led_flash = 0; // IRON, for error LED flashing
uint32_t last_error_message = 0;
uint32_t oldExtuder_auto_fan_speed = controllerFan.settings.extruder_auto_fan_speed;

void FDCAN_idle() { // Called from MarlinCore.cpp
// heart beat signal, send and receive
// error monitoring (hotend auto fan speed)

  if (oldExtuder_auto_fan_speed != controllerFan.settings.extruder_auto_fan_speed) {
    controllerFan.settings.extruder_auto_fan_speed = MAX(controllerFan.settings.extruder_auto_fan_speed, PROBING_AUTO_FAN_SPEED); // Should not be required, safeguard!
    oldExtuder_auto_fan_speed = controllerFan.settings.extruder_auto_fan_speed;
  }

  if (send_gcode_count) {
    MString<24> buffer(F("GCODES="), gcode_counter, '\n');
    CAN_Send_String(buffer); // Report number of gcodes received from host
    send_gcode_count = false;
    SERIAL_ECHOLNPGM(">>> CAN messages sent: ", CAN_message_counter);
  }
  // NTP style time sync
  // t[0] = local time sync request time
  // t[1] = host time sync receive time
  // t[2] = host time sync reply time
  // t[3] = local time stamp receive time
  if (t[3] != 0) {
    t[0] += time_offset; // Adjust local time stamps with time_offset
    t[3] += time_offset; // Adjust local time stamps with time_offset
    uint32_t local_time_adjustment = (t[1] - t[0] + t[2] - t[3]) >> 1;
    time_offset += local_time_adjustment;
    uint32_t Round_trip_delay = (t[3] - t[0] - t[2] + t[1]);
    SERIAL_ECHOLNPGM("t0: ", t[0], " us");
    SERIAL_ECHOLNPGM("t1: ", t[1], " us");
    SERIAL_ECHOLNPGM("t2: ", t[2], " us");
    SERIAL_ECHOLNPGM("t3: ", t[3], " us");
    SERIAL_ECHOPGM("Local time adjustment: ", local_time_adjustment, " us");
    if (t[4]) {
      SERIAL_ECHOLNPGM(" after ",  ftostr42_52(float(t[0] - t[4]) / 1000000.0), " seconds");
      SERIAL_ECHOLNPGM("Drift: ", local_time_adjustment / (float(t[0] - t[4]) / 1000000.0), " us/s");
    }
    else
      SERIAL_EOL();

    SERIAL_ECHOLNPGM("Time offset: ", time_offset, " us");
    SERIAL_ECHOLNPGM("Round_trip_delay: ", Round_trip_delay, " us");
    SERIAL_ECHOLNPGM("Host response time: ", (t[2] - t[1]), " us");

    t[4] = t[0]; // Store previous time sync request time
    t[3] = 0;
  }

  if (hfdcan2.ErrorCode) {
    uint32_t ms = millis();

    if ((ms - last_error_message) > 20000) { // 20 seconds repeat
      MString<40> buffer(F("Error: fdcan2.ErrorCode="), hfdcan2.ErrorCode, '\n');
      CAN_Send_String(buffer);
      last_error_message = ms;
    }
  }

  if (CAN_Error_Code) {
    uint32_t ms = millis();

    if ((ms - last_led_flash) > 100) { // Flash LED fast on error
      digitalToggle(LED_PIN);
      last_led_flash = ms;
    }

    if (CAN_Error_Code != Old_Error_Code) { // Show message on new error code
      switch(CAN_Error_Code) {
        case CAN_ERROR_FIFO_OVERFLOW:
          CAN_Send_String("Error: HEAD CAN FIFO OVERFLOW\n");
          SERIAL_ECHOLNPGM(">>> ERROR: HEAD CAN FIFO OVERFLOW");
          break;

        case CAN_ERROR_INCOMPLETE_GCODE_RECEIVED:
          CAN_Send_String("Error: HEAD INCOMPLETE GCODE MESSAGE\n");
          SERIAL_ECHOLNPGM(">>> ERROR: HEAD INCOMPLETE GCODE MESSAGE");
          break;

        case CAN_ERROR_MARLIN_CMD_BUFFER_OVERFLOW:
          CAN_Send_String("Error: HEAD MARLIN CMD BUF OVERFLOW\n");
          SERIAL_ECHOLNPGM(">>> ERROR: HEAD MARLIN CMD BUF OVERFLOW");
          break;

        default: { }
      } // Switch

      Old_Error_Code = CAN_Error_Code;
    } // New error code
  } // CAN_Error_Code
} // CAN_idle()

/*
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *hfdcan, uint32_t ITList, uint32_t InterruptLine);
HAL_StatusTypeDef HAL_FDCAN_TT_ConfigInterruptLines(FDCAN_HandleTypeDef *hfdcan, uint32_t TTITList, uint32_t InterruptLine);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t ActiveITs, uint32_t BufferIndexes);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t InactiveITs);
HAL_StatusTypeDef HAL_FDCAN_TT_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t ActiveTTITs);
HAL_StatusTypeDef HAL_FDCAN_TT_DeactivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t InactiveTTITs);
void              HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *hfdcan);
*/

#endif // CAN_TOOLHEAD