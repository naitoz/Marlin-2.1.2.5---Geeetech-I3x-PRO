#pragma once

// IMPORTANT NOTES
// ===============
// In \Users\<username>\.platformio\packages\framework-arduinoststm32@4.20600.231001\libraries\SrcWrapper\src\HardwareTimer.cpp
// Add function "__weak" in front of "void TIM16_IRQHandler(void)"
// NOTE: Every CAN message is accepted in FIFO0 if: 1. No Standard/Extended filters are set. 2. No global filters is set.

#define CAN_IO_MASK                               0b11111 // Virtual IO Mask, 5 bits are used for IO signalling
#define CAN_PROBE_BIT                                   0 // Virtual IO bit
#define CAN_FILAMENT_BIT                                1 // Virtual IO bit
#define CAN_X_ENDSTOP_BIT                               2 // Virtual IO bit
#define CAN_Y_ENDSTOP_BIT                               3 // Virtual IO bit
#define CAN_Z_ENDSTOP_BIT                               4 // Virtual IO bit
#define CAN_STRING_MESSAGE_BIT                          5 // Signals the head sends a string message
#define CAN_REQUEST_SETUP_BIT                           6 // Signals the head requests setup information
#define CAN_TMC_OT_BIT                                  7 // Signals the head encountered a TMC Over Temp error
#define CAN_REQUEST_TIME_SYNC_BIT                       8 // Signals a request for time sync
#define CAN_ERROR_BIT                                   9 // Signals the head encountered an error

#define GCODE_TYPE_D                                    0
#define GCODE_TYPE_G                                    1
#define GCODE_TYPE_M                                    2
#define GCODE_TYPE_T                                    3

#define IDENTIFIER_PARAMTER1_OFFSET                     0
#define IDENTIFIER_PARAMTER2_OFFSET                     5
#define IDENTIFIER_GCODE_NUMBER_OFFSET                 10 
#define IDENTIFIER_GCODE_TYPE_OFFSET                   23
#define IDENTIFIER_PARAMETER_COUNT_OFFSET              25

#define DATALENGTH_OFFSET                              16

#define IDENTIFIER_PARAMETER_COUNT_MASK             0b111
#define IDENTIFIER_PARAMETER_MASK                 0b11111
#define IDENTIFIER_GCODE_TYPE_MASK                   0b11 // GCODE TYPE
#define IDENTIFIER_GCODE_NUMBER_MASK      0b1111111111111 // GCODE NUMBER ONLY
#define IDENTIFIER_GCODE_MASK           0b111111111111111 // GCODE TYPE AND NUMBER

#define STDID_FIFO_BIT                      0b10000000000

// ERROR CODES
#define CAN_ERROR_FIFO_OVERFLOW                         1
#define CAN_ERROR_INCOMPLETE_GCODE_RECEIVED             2
#define CAN_ERROR_MARLIN_CMD_BUFFER_OVERFLOW            4

HAL_StatusTypeDef CAN_Send_String(const char * message); // Send CAN string to host
HAL_StatusTypeDef FDCAN2_Start(void); // Start the CAN bus