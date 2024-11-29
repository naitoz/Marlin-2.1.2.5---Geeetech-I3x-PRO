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
#pragma once

#include "../../inc/MarlinConfigPre.h"

//#define CAN_DEBUG // Define to show gcodes send to HEAD

#define SOUND_OK   880
#define SOUND_ERROR 40

extern uint32_t CAN_io_state; // CAN virtual IO variable

#define CAN_IO_MASK                0b11111 // Masks for the 5 virtual IO bits
#define CAN_PROBE_MASK                   1 // Virtual IO bit
#define CAN_FILAMENT_MASK                2 // Virtual IO bit
#define CAN_X_ENDSTOP_MASK               4 // Virtual IO bit
#define CAN_Y_ENDSTOP_MASK               8 // Virtual IO bit
#define CAN_Z_ENDSTOP_MAS               16 // Virtual IO bit
#define CAN_STRING_MESSAGE_MASK         32 // Signals the head sends a string message
#define CAN_REQUEST_SETUP_MASK          64 // Signals the head requests setup information
#define CAN_TMC_OT_MASK                128 // Signals the head signals a TMC Over Temp error
#define CAN_CAN_REQUEST_TIME_SYNC_MASK 256 // Signals the head requested a time sync
#define CAN_ERROR_MASK                 512 // Signals the head encountered an error

HAL_StatusTypeDef CAN1_Start(); // FUNCTION PROTOTYPES
HAL_StatusTypeDef CAN1_Stop();
HAL_StatusTypeDef CAN_Send_Gcode();
HAL_StatusTypeDef CAN_Send_Gcode_2params(uint32_t Gcode_type, uint32_t Gcode_no, uint32_t parameter1, float value1, uint32_t parameter2, float value2);
void CAN_Send_Setup();   // Send host configuration to head
void CAN_Idle();         // Idle CAN task
