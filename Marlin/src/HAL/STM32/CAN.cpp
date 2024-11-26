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

/**
 * Contributor Notes:
 * NOTE 1: For MKS Monster 8 V1/V2 on Arduino use: Board "Generic STM32F4 series", Board part number "Generic F407VETx"
 * NOTE 2: Requires `HAL_CAN_MODULE_ENABLED`, e.g., with `-DHAL_CAN_MODULE_ENABLED`
 *         For Arduino IDE use "hal_conf_extra.h" with `#define HAL_CAN_MODULE_ENABLED`
 * NOTE 3: To accept all CAN messages, enable 1 filter (FilterBank = 0) in "FilterMode = CAN_FILTERMODE_IDMASK", mask and ID = 0 (0=don't care)
 * NOTE 4: Serial communication in ISR causes issues! Hangs etc. so avoid this!
 * NOTE 5: A FIFO storage cell is called a "Mailbox" in STM32F4xx, FIFO0 and FiFO1 can hold 3 CAN messages each.
 * NOTE 6: The filter ID/mask numbers (LOW/HIGH) do not directly relate to the message ID numbers (See Figure 342 in RM0090)
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(CAN_MASTER)

#include "../platforms.h"
#include "../../gcode/parser.h"
#include "../../module/temperature.h"
#include "../../module/motion.h"  // For current_position variable
#include "../../module/planner.h" // For steps/mm parameters variables
#include "../../feature/tmc_util.h"
#include "../../module/endstops.h"
#include "../../feature/controllerfan.h" // For controllerFan settings
#include "../../libs/numtostr.h"  // For float to string conversion

#define CAN_EXTENDED_ID CAN_ID_EXT
#define CAN_STANDARD_ID CAN_ID_STD

#define STDID_FIFO_BIT       0b10000000000
#define EXTID_FIFO_BIT          0x10000000

#define CAN_IO_MASK                0b11111 // Masks for the 5 virtual IO bits (see below)
#define GCODE_NUMBER_MASK  0b1111111111111
#define PARAMETER_MASK             0b11111
#define PARAMETER_COUNT_MASK         0b111
#define GCODE_TYPE_MASK               0b11
#define CAN_PROBE_MASK                   1 // Virtual IO bit for probe
#define CAN_FILAMENT_MASK                2 // Virtual IO bit for filament
#define CAN_X_ENDSTOP_MASK               4 // Virtual IO bit for X-endstop
#define CAN_Y_ENDSTOP_MASK               8 // Virtual IO bit for Y-endstop
#define CAN_Z_ENDSTOP_MASK              16 // Virtual IO bit for Z-endstop
#define CAN_STRING_MESSAGE_MASK         32 // Signals the head sent a string message
#define CAN_REQUEST_SETUP_MASK          64 // Signals the head requests setup information
#define CAN_TMC_OT_MASK                128 // Signals the head signals a TMC Over Temp error
#define CAN_E0_TARGET_MASK             256 // Signals E0 or E1
#define CAN_ERROR_MASK                 512 // Signals the head encountered an error

#define PARAMETER1_OFFSET                0
#define PARAMETER2_OFFSET                5
#define GCODE_NUMBER_OFFSET             10
#define GCODE_TYPE_OFFSET               23
#define PARAMETER_COUNT_OFFSET          25

#define GCODE_TYPE_D                     0
#define GCODE_TYPE_G                     1
#define GCODE_TYPE_M                     2
#define GCODE_TYPE_T                     3

extern "C" void CAN1_RX0_IRQHandler(); // Override weak CAN FIFO0 interrupt handler
extern "C" void CAN1_RX1_IRQHandler(); // Override weak CAN FIFO1 interrupt handler
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan); // Override weak CAN interrupt callback for FIFO0
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan); // Override weak CAN interrupt callback for FIFO1
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);             // Override weak CAN interrupt callback

CAN_HandleTypeDef hcan1                = { 0 }; // The CAN1 handle
CAN_TxHeaderTypeDef TxHeader           = { 0 }; // Header to send a CAN message
volatile uint32_t CAN_io_state         = 0;     // Virtual IO state variable
volatile bool CAN_head_error           = 0;     // Register if an error was reported by the head
volatile bool CAN_head_setup_request   = false; // Signals the head requesting setup information
volatile uint32_t gcode_counter        = 0;     // Count amount of gcodes received
volatile uint32_t HAL_CAN_error_code   = 0;     // Record a host CAN error message

volatile bool FirstE0Error             = true;  // First CAN bus error, show warning only once
volatile bool string_message_complete  = false; // Signals a complete string message was received
volatile uint32_t string_message_index = 0;     // Index into the CAN string that is being received
uint32_t Next_CAN_Temp_Report          = 0;     // Track when the next head temperature report will be received
uint32_t Last_CAN_Error_Message        = 0;     // Track when the last CAN error messaget was shown
char string_message[128]               = "\0";  // CAN string message buffer for incoming message, max 128 characters

void CAN1_RX0_IRQHandler() { // CAN FIFO0 Interrupt handler overrides standard weak CAN1_RX0_IRQHandler
  HAL_CAN_IRQHandler(&hcan1); // Forwarded for callbacks --> HAL_CAN_RxFifo0MsgPendingCallback/HAL_CAN_ErrorCallback
  // OR
  //HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); // Call the required callback directly, faster but no error reporting
}

void CAN1_RX1_IRQHandler() { // CAN FIFO1 Interrupt handler overrides standard weak CAN1_RX0_IRQHandler
  HAL_CAN_IRQHandler(&hcan1); // Forwarded for callbacks --> HAL_CAN_RxFifo1MsgPendingCallback/HAL_CAN_ErrorCallback
  // OR
  //HAL_CAN_RxFifo1MsgPendingCallback(&hcan1); // Call the required callback directly, faster but no error reporting
}

// Send specified Gcode with max 2 parameters and 2 values via CAN bus
HAL_StatusTypeDef CAN_Send_Gcode_2params(uint32_t Gcode_type, uint32_t Gcode_no, uint32_t parameter1, float value1, uint32_t parameter2, float value2) {
  switch (Gcode_type) {
    case 'D':
      Gcode_type = GCODE_TYPE_D;
      break;

    case 'G':
      Gcode_type = GCODE_TYPE_G;
      break;

    case 'M':
      Gcode_type = GCODE_TYPE_M;

      #ifdef CAN_DEBUG
        SERIAL_ECHOPGM(">>> CAN TO HEAD: M", Gcode_no);
        if (parameter1) {
          SERIAL_CHAR(' ', parameter1);
          if (value1 == int(value1))
            SERIAL_ECHO(i16tostr3left(value1)); // Integer value
          else
            SERIAL_ECHO(p_float_t(value1, 4));  // Float with 4 digits
        }

        if (parameter2) {
          SERIAL_CHAR(' ', parameter2);
          if (value2 == int(value2))
            SERIAL_ECHO(i16tostr3left(value2)); // Integer value
          else
            SERIAL_ECHO(p_float_t(value2, 4));  // Float with 4 digits
        }
        SERIAL_EOL();
      #endif // CAN_DEBUG

      break;

    case 'T': Gcode_type = GCODE_TYPE_T;
      break;

    default: return HAL_ERROR; // UNKNOWN GCODE TYPE
  }

  HAL_StatusTypeDef status = HAL_OK;
  uint32_t TxMailbox;  // Stores which Mailbox (0-2) is used to store the Sent TX message

  if (parameter1 > 31)
    parameter1 -= 64; // Format 'A' = 1, 'B' = 2, etc.

  if (parameter2 > 31)
    parameter2 -= 64; // Format 'A' = 1, 'B' = 2, etc.

  TxHeader.IDE   = CAN_EXTENDED_ID;
  TxHeader.DLC   = 4 * (!!parameter1 + !!parameter2);                        // Amount of bytes to send (4 or 8)
  TxHeader.StdId = (TxHeader.StdId  ^ STDID_FIFO_BIT);                       // Toggle FIFO bit 10, keep FIFO toggling in sync
  TxHeader.ExtId = ((TxHeader.ExtId ^ EXTID_FIFO_BIT) & EXTID_FIFO_BIT) +    // Toggle FIFO bit 28
                   ((TxHeader.DLC >> 2) << PARAMETER_COUNT_OFFSET) +         // Data bytes (4 or 8)
                   (Gcode_type << GCODE_TYPE_OFFSET) +                       // G/M/T/D-code
                   ((Gcode_no & GCODE_NUMBER_MASK) << GCODE_NUMBER_OFFSET) + // Gcode number
                   ((parameter2 & PARAMETER_MASK) << PARAMETER2_OFFSET) +    // First parameter
                   ((parameter1 & PARAMETER_MASK) << PARAMETER1_OFFSET);     // Second parameter
  uint8_t CAN_tx_buffer[8];            // 8 bytes CAN data TX buffer
  float * fp = (float *)CAN_tx_buffer; // Point to TX buffer
  *fp++ = value1;
  *fp-- = value2;

  const uint32_t ms = millis(); // Don't send too fast!
  while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) && (millis() - ms < 25)) { } // BLOCKING! Wait max 25ms!
  //SERIAL_ECHOLNPGM(">>> Waited1: ", millis() - ms, " FreeTX: ", HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)); // IRON, DEBUGGING
  status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_tx_buffer, &TxMailbox); // Send message

  if (status == HAL_OK) // Count sent gcode messages
    gcode_counter++;

  return status;
}

HAL_StatusTypeDef CAN_Send_Setup() { // Send setup to HEAD
  CAN_head_setup_request = false;
  SERIAL_ECHOLNPGM(">>> CAN: SENDING CONFIG TO HEAD =====");
  // NOTE: Sending many command too fast will cause a Marlin command buffer overrun at the head, add delays if needed
  //CAN_Send_Gcode_2params('M', 104, 'S', 0, 0, 0); // M104 S0, switch off hotend heating, NOT NEEDED ANYMORE
  //CAN_Send_Gcode_2params('M', 107,   0, 0, 0, 0); // M107, switch off cooling fan, NOT NEEDED ANYMORE
  //CAN_Send_Gcode_2params('M',  18,   0, 0, 0, 0); // M18, switch off steppers, NOT NEEDED ANYMORE

  #if ENABLED(MPCTEMP)
    // M306 MPC settings (managed by host)
    MPC_t &mpc = thermalManager.temp_hotend[0].mpc;

    CAN_Send_Gcode_2params('M', 306, 'A', mpc.ambient_xfer_coeff_fan0, 'C', mpc.block_heat_capacity);          // M306 R<sensor_responsiveness> A<Ambient heat transfer coefficient>
    CAN_Send_Gcode_2params('M', 306, 'F', mpc.fanCoefficient(),        'H', mpc.filament_heat_capacity_permm); // M306 F<sensor_responsiveness> H<filament_heat_capacity_permm>
    CAN_Send_Gcode_2params('M', 306, 'P', mpc.heater_power,            'R', mpc.sensor_responsiveness);        // M306 P<heater_power> C<Heatblock Capacity (joules/kelvin)>
  #endif

  //CAN_Send_Gcode_2params('M', 150, 0, 0, 0, 0); // M150, SWITCH NEOPIXEL OFF

  /*
  extern Planner planner; // M92 Steps per mm
  CAN_Send_Gcode_2params('M', 92, 'X', planner.settings.axis_steps_per_mm[X_AXIS], 'Y', planner.settings.axis_steps_per_mm[Y_AXIS]);
  CAN_Send_Gcode_2params('M', 92, 'Z', planner.settings.axis_steps_per_mm[Z_AXIS], 'E', planner.settings.axis_steps_per_mm[E_AXIS]);

  // M200 Set filament diameter
  CAN_Send_Gcode_2params('M', 200, 'S', parser.volumetric_enabled, 'D', LINEAR_UNIT(planner.filament_size[0]));
  #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
     CAN_Send_Gcode_2params('M', 200, 'L', LINEAR_UNIT(planner.volumetric_extruder_limit[0])
  #endif

  // M201 Max acceleration
  CAN_Send_Gcode_2params('M', 201, 'X', planner.settings.max_acceleration_mm_per_s2[X_AXIS], 'Y', planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
  CAN_Send_Gcode_2params('M', 201, 'Z', planner.settings.max_acceleration_mm_per_s2[Z_AXIS], 'E', planner.settings.max_acceleration_mm_per_s2[E_AXIS]);

  // M203 Max feedrate
  CAN_Send_Gcode_2params('M', 203, 'X', planner.settings.max_feedrate_mm_s[X_AXIS], 'Y', planner.settings.max_feedrate_mm_s[Y_AXIS]);
  CAN_Send_Gcode_2params('M', 203, 'Z', planner.settings.max_feedrate_mm_s[Z_AXIS], 'E', planner.settings.max_feedrate_mm_s[E_AXIS]);

  // M204 Accelerations in units/sec^2, ENABLED BECAUSE IT INFORMS THE HEAD THE CONFIGURATION WAS SENT
  CAN_Send_Gcode_2params('M', 204, 'P', planner.settings.acceleration, 'R', planner.settings.retract_acceleration);
  CAN_Send_Gcode_2params('M', 204, 'T', planner.settings.travel_acceleration, 0, 0);

  // M205
  #if ENABLED(CLASSIC_JERK)
    CAN_Send_Gcode_2params('M', 205,'S')) planner.settings.min_feedrate_mm_s, 'T')) planner.settings.min_travel_feedrate_mm_s);
    CAN_Send_Gcode_2params('M', 205, M205_MIN_SEG_TIME_PARAM, planner.settings.min_segment_time_us, 'J', planner.junction_deviation_mm);
    CAN_Send_Gcode_2params('M', 205, 'X', LINEAR_UNIT(planner.max_jerk.x), 'Y', LINEAR_UNIT(planner.max_jerk.y));
    CAN_Send_Gcode_2params('M', 205, 'Z', LINEAR_UNIT(planner.max_jerk.z), 'E', LINEAR_UNIT(planner.max_jerk.e));
    CAN_Send_Gcode_2params('M', 205, 'J', LINEAR_UNIT(planner.junction_deviation_mm), 0, 0);
  #endif

  // M206 Home offset
  #if DISABLED(NO_HOME_OFFSETS)
    _CAN_Send_Gcode_2params('M', 206, 'X', LINEAR_UNIT(home_offset.x), 'Y', LINEAR_UNIT(home_offset.y));
    CAN_Send_Gcode_2params('M', 206, 'Z', LINEAR_UNIT(home_offset.z), 0, 0);
  #endif

  // M207 Set Firmware Retraction
  // M208 - Firmware Recover
  // M209 - Set Auto Retract

  // M220 Speed/feedrate
  CAN_Send_Gcode_2params('M', 220, 'S', feedrate_percentage, 0, 0);

  // M221 Flow percentage
  CAN_Send_Gcode_2params('M', 221, 'T', 0, 'S', planner.flow_percentage[0]);
  // CAN_Send_Gcode_2params('M', 221, 'T', 1, 'S', planner.flow_percentage[1]); // For 2nd extruder

  // M302 Cold extrude settings
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    CAN_Send_Gcode_2params('M', 302, 'P', '0' + thermalManager.allow_cold_extrude, 'S', thermalManager.extrude_min_temp); // P0 enable cold extrusion checking, P1 = disabled, S=Minimum temperature
  #endif

  // M569 TMC Driver StealthChop/SpreadCycle
  CAN_Send_Gcode_2params('M', 569, 'S', stepperE0.get_stored_stealthChop(), 'E', 0); // M569 S[0/1] E

  // M592 Nonlinear Extrusion Control

  // M916 TMC Motor current
  CAN_Send_Gcode_2params('M', 906, 'E', stepperE0.getMilliamps(), 0, 0);

  // M919 TMC Chopper timing for E only
  CAN_Send_Gcode_2params('M', 919, 'O', off, 'P' , Hysteresis End);
  CAN_Send_Gcode_2params('M', 919, 'S', Hysteresis Start, 0, 0);
  */
  #if USE_CONTROLLER_FAN
    /*
    CAN_Send_Gcode_2params('M', 710, 'E', 1, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 2, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 3, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 4, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 5, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 6, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 7, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 8, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 9, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 10, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    CAN_Send_Gcode_2params('M', 710, 'E', 11, 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
    */
    // IRON, M710 SIGNALS TO THE HEAD THAT THE CAN CONFIGURATION IS COMPLETE, USE IT AS THE LAST GCODE TO SEND
    return CAN_Send_Gcode_2params('M', 710, 'E', int(controllerFan.settings.extruder_auto_fan_speed), 'P', int(controllerFan.settings.probing_auto_fan_speed)); // M710 E<auto fan speed> P<probing fan speed>
  #endif
  return HAL_OK;
}

void CAN_Idle() { // Tasks that cannot be done in the ISR
  if (CAN_head_setup_request) // The head requested the setup data
    CAN_Send_Setup();

  if (string_message_complete) { // Received string message is complete
    BUZZ(1, SOUND_OK);
    SERIAL_ECHOPGM(">>> CAN MSG FROM HEAD: ");
    for (uint32_t i = 0; i < string_message_index; i++)
      SERIAL_CHAR(string_message[i]); // Show received string message, ends on '\n'

    string_message_complete = false;
    string_message_index    = 0;
  }

  if ((hcan1.ErrorCode || CAN_head_error || HAL_CAN_error_code) && (millis() - Last_CAN_Error_Message > 6000)) {
    BUZZ(1, SOUND_ERROR);
    if (CAN_head_error) SERIAL_ECHOLNPGM(">>> CAN Error reported by head");
    CAN_head_error = false; // Reset, but will be repeated by the head

    if (HAL_CAN_error_code)
      SERIAL_ECHOLNPGM(">>> HAL CAN Error reported: ", HAL_CAN_error_code);

    if (hcan1.ErrorCode)
      SERIAL_ECHOLNPGM(">>> hcan1.ErrorCode=", hcan1.ErrorCode);

    Last_CAN_Error_Message = millis();
  }

  if (ELAPSED(millis(), Next_CAN_Temp_Report)) { // IRON, ERROR, NO TEMP UPDATE RECEIVED IN 10 SECONDS, KILL PRINTER
    Next_CAN_Temp_Report = millis() +  10000;
    if (FirstE0Error) { // Send error notification
      BUZZ(1, SOUND_ERROR); // Warn with sound
      SERIAL_ECHOLNPGM("Error: No CAN E0 temp updates!");
    }
    else // Send only error message
      SERIAL_ECHOLNPGM(">>> CAN ERROR, No E0 temp updates!");

    FirstE0Error = false; // Warn only once

    #ifndef CAN_DEBUG // Only kill if not debugging
      kill(F("CAN ERROR, NO E0 TEMPERATURE UPDATES"));
    #endif
  }
}

HAL_StatusTypeDef CAN_Send_Gcode() { // Forward a Marlin Gcode via CAN (uses parser.command_letter, Gcode_no, parser.value_float())
  // Send a Gcode to the head with parameters and values
  // Gcode starts with extended frame which can send the Gcode with max 2 parameters and values.
  // Extended frames are send to complete all parameters and values (max 2 per extended message).
  // 1. Analyze Gcode command
  // 2. Ignore gcodes that do not need to be forwarded
  // 3. Send parameters and values
  // char s[] = "G0 X123.45678 Y124.45678 Z125.45678 E126.45678 F127.45678\n";

  HAL_StatusTypeDef status = HAL_OK;
  uint32_t TxMailbox; // Stores which Mailbox (0-2) is used to store the Sent TX message

  if (parser.command_letter != 'M') // Only forward Mxxx Gcode to head
    return HAL_OK;

  uint32_t Gcode_type = GCODE_TYPE_M; // M-code, fixed for now
  uint32_t Gcode_no = parser.codenum;

  if (Gcode_no == 109) // Convert M109(Hotend wait) to M104 (no wait) to keep the head responsive
    Gcode_no = 104;

  if ((Gcode_no == 501) || (Gcode_no == 502)) // M501=Restore settings, M502=Factory defaults
    CAN_head_setup_request = true; // Also update settings for the head

  if ((Gcode_no != 104) && // Set hotend target temp
      (Gcode_no != 106) && // Set cooling fan speed
      (Gcode_no != 107) && // Cooling fan off
      (Gcode_no != 150) && // Set NeoPixel values
      //(Gcode_no != 108) && // Break and Continue
      (Gcode_no != 280) && // Servo position
      (Gcode_no != 306) && // MPC settings/tuning
      (Gcode_no != 710) && // Control fan PWM
      (Gcode_no != 997))   // Reboot
    return HAL_OK;         // Nothing to do

  uint32_t index;
  uint32_t parameter_counter = 0;
  char letters[] = "XYZEFPSABCHIJKLOQRTUVW"; // All possible parameters (22), defines scan order, no "D G M N", includes 'T' for autotune (M306 T)
  static uint32_t parameters[8] = { 0 }; // Store found parameters, send max 7 parameters (send in pairs, so reserve 8), CodeA=1 (ASCII65), CodeE=5, CodeF=6, CodeX=88-64=24, CodeY=89-64=25, CodeZ=90-64=26
  static float values[8]        = { 0 }; // Store found values, send max 7 parameters (send in pairs, so reserve 8)

  uint8_t CAN_tx_buffer[8];   // 8 bytes CAN data TX buffer

  /*
  switch (parser.command_letter) // Filter/adjust Gcodes
  {
    case 'G': Gcode_type = GCODE_TYPE_G;
      switch (Gcode_no)
      {
        case 12: break; // No Nozzle cleaning support needed on head
        case 29: case 34: return HAL_OK; // No bedleveling/Z-syncing on head
        break;
      }
      break;

    case 'M': Gcode_type = GCODE_TYPE_M;
      switch (Gcode_no)
      { // Save Prog mem: M112, M48, M85, M105, M114, M155, M500, M501, M502, M503, M226, M422
        case 109: Gcode_no = 104; break;   // Replace M109 with M104
        case 112: Gcode_no = 104; break;   // Don't shutdown board, should stop heating with "M104"

        case  20: case 21: case 22: case 23: case 24: case 25: case 26: case 524: case 540: case 928:
        case  27: case 28: case 29: case 30: case 32: case 33: case 34: // No SD file commands
        case  43:                     // No pin debug
        case  48:                     // No repeatability test
        case  85:                     // No inactivity shutdown
        case 100:                     // No show free memory support
        case 108:                     // Break and Continue
        case 105:                     // No temperature reporting
        case 114:                     // Don't report position
        case 117: case 118: case 119: // Don't send strings
        case 140: case 190:           // Ignore bed temp commands
        case 150:                     // Set NeoPixel values
        case 154:                     // No auto position reporting
        case 155:                     // No tempeature reporting
        case 226:                     // Wait for pin state
        case 240:                     // No camera support
        case 250:                     // No LCD contrast support
        case 260: case 261:           // No I2C on head
        case 280:                     // Don't send servo angle, done via Servo.cpp already
        case 290:                     // No baby stepping
        case 300:                     // No tones
       // case 303:                     // No PID autotune (done on HEAD)
        case 304:                     // No bed PID settings
       // case 306:                     // MPC autotune (done on HEAD)
        case 350: case 351:           // No live microstepping adjustment
        case 380: case 381:           // No solenoid support
        case 401: case 402:           // No probe deploy/stow, done via M280 servo angles
        case 412:                     // Filament runout sensor done by MASTER
        case 420: case 421:           // No bed leveling state
        case 423:                     // No X Twist Compensation
        case 425:                     // No backlash compensation
        case 500: case 501: case 502: case 503: case 504: case 910: // No EEPROM on head, remove M50x commands to save Prog mem
        case 510: case 511: case 512: // No locking of the machine
        case 605:                     // No IDEX commands
        case 810: case 811: case 812: case 813: case 814: case 815: case 816: case 817: case 818: case 819:
        case 851:                     //
        case 871:                     // No Probe temp config
        case 876:                     // No handle prompt response
        case 913:                     // No Set Hybrid Threshold Speed
        case 914:                     // No TMC Bump Sensitivity
        case 997:                     // No remote reset
        case 998:                     // No ESP3D reset
        return HAL_OK;                // NO CAM MESSAGE
      }
    break;

    case 'T': Gcode_type = GCODE_TYPE_T;
      switch (Gcode_no)
      {
        case 0: case 1:
        break;
      }
    break;

    case 'D': Gcode_type = GCODE_TYPE_D;
      switch (Gcode_no)
      {
        case 0: case 1:
        break;
      }
    break;
    default: return HAL_OK; // Invalid command, nothing to do
  }
  */

  #ifdef CAN_DEBUG
    SERIAL_ECHOPGM(">>> CAN GCODE TO HEAD: "); // IRON, DEBUGGING
    SERIAL_CHAR(parser.command_letter);
    SERIAL_ECHO(Gcode_no); // IRON, DEBUGGING
  #endif

  if (strlen(parser.command_ptr) > 4) // "M107\0", ONLY SCAN FOR PARAMETERS IF STRING IS LONG ENOUGH
  for (index = 0; index < sizeof(letters); index++) { // Scan parameters
    if (parser.seen(letters[index])) {
      parameters[parameter_counter] = letters[index] - 64; // Store parameter letter, A=1, B=2...

      #ifdef CAN_DEBUG
        SERIAL_CHAR(' ', letters[index]); // IRON, DEBUGGING
      #endif

      if (parser.has_value()) { // Check if there is a value
        values[parameter_counter++] = parser.value_float();

        #ifdef CAN_DEBUG
          if (values[parameter_counter - 1] == int(values[parameter_counter - 1]))
            SERIAL_ECHO(i16tostr3left(values[parameter_counter - 1])); // Integer value
          else
            SERIAL_ECHO(p_float_t(values[parameter_counter - 1], 4));  // Float with 4 digits
        #endif

      }
      else // No value for parameter
        values[parameter_counter++] = NAN; // Not A Number, indicates no parameter value is present
    }

    if (parameter_counter == 8) { // Max 8 parameters
      parameter_counter--; // Max is 7 parameters
      SERIAL_ECHOLNPGM("\nError: TOO MANY PARAMETERS (> 7): ", parser.command_ptr);
      BUZZ(1, SOUND_ERROR);
      break;
    }
  }
  #ifdef CAN_DEBUG
    SERIAL_EOL();
  #endif

  parameters[parameter_counter] = 0; // Set next parameter to 0 (0=no parameter), send in pairs
  index = 0;
  float * fp = (float *)CAN_tx_buffer; // Points to TX buffer

  if ((Gcode_no == 710) && (parameters[0] == 3)) // "M710 C" INDICATES REQUEST FOR GCODE COUNT
    SERIAL_ECHOLNPGM(">>> GCODES SENT: ", gcode_counter);

  gcode_counter++;

  TxHeader.IDE = CAN_EXTENDED_ID; // Start with EXTENDED_ID then send STANDARD_ID if needed
  //TxHeader.ExtId &= EXTID_FIFO_BIT; // Clear ID, keep FIFO bit
  TxHeader.ExtId = (TxHeader.ExtId & EXTID_FIFO_BIT) +  // KEEP FIFO BIT
                   ((parameter_counter & PARAMETER_COUNT_MASK) << PARAMETER_COUNT_OFFSET) + // Parameter count
                   ((Gcode_type & GCODE_TYPE_MASK)   << GCODE_TYPE_OFFSET) +   // GCODE TYPE (G/M/T/D)
                   ((Gcode_no & GCODE_NUMBER_MASK)   << GCODE_NUMBER_OFFSET) + // GCODE NUMBER
                   ((parameters[1] & PARAMETER_MASK) << PARAMETER2_OFFSET) +   // PARAMETER2
                   ((parameters[0] & PARAMETER_MASK) << PARAMETER1_OFFSET);    // PARAMETER1
  uint32_t ms = millis(); // Record message send start time
  do {
    TxHeader.DLC = MIN(8, (parameter_counter - index) << 2); // Maximum 8 bytes, 4 bytes if there is only 1 parameter, 8 bytes if there are 2
    //TxHeader.StdId =  (TxHeader.StdId ^ STDID_FIFO_BIT) & STDID_FIFO_BIT;            // Toggle FIFO bit 10, clear other bits
    TxHeader.StdId = ((TxHeader.StdId ^ STDID_FIFO_BIT) & STDID_FIFO_BIT) +            // Toggle FIFO bit
                     ((parameters[index + 1] & PARAMETER_MASK) << PARAMETER2_OFFSET) + // Parameter 2
                     ((parameters[index    ] & PARAMETER_MASK) << PARAMETER1_OFFSET);  // Parameter 1
    TxHeader.ExtId ^= EXTID_FIFO_BIT; // Toggle FIFO bit 28
    *fp++ = values[index++]; // Copy first parameter value to data, move pointer to next 4 bytes
    *fp-- = values[index++]; // Copy 2nd parameter value to data, move pointer to beginning of data array for next round

    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) && (millis() - ms < 50)) { } // BLOCKING! Wait max 50ms
    status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_tx_buffer, &TxMailbox); // Send message

    if (status != HAL_OK)
      return status;

    TxHeader.IDE = CAN_STANDARD_ID; // All following messages have standard ID for parameter values, 11 bits identifier
  } while (index < parameter_counter);

  return HAL_OK;
}

void CAN_Send_Position() { // Send the X, Y, Z and E position to the HEAD
  CAN_Send_Gcode_2params('G', 92, 'X', current_position.x, 'Y', current_position.y); // M92 X<pos> Y<pos>
  CAN_Send_Gcode_2params('G', 92, 'Z', current_position.z, 'E', current_position.e); // M92 E<pos> Z<pos>
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) { // Called by HAL_CAN_Init
  GPIO_InitTypeDef GPIO_InitStruct       = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (canHandle->Instance == CAN1) {
    RCC_PeriphCLKInitTypeDef periphClkInit = { };
    HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);
    periphClkInit.PeriphClockSelection |= RCC_APB1ENR_CAN1EN; // DONE IN __HAL_RCC_CAN1_CLK_ENABLE?

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      Error_Handler();

    // CAN1 clock enable
    if (__HAL_RCC_CAN1_IS_CLK_DISABLED()); // Enable CAN1 clock
      __HAL_RCC_CAN1_CLK_ENABLE();         // Enable CAN1 clock

    if (__HAL_RCC_GPIOB_IS_CLK_DISABLED()) // Should be enabled by Marlin already
      __HAL_RCC_GPIOB_CLK_ENABLE();        // Enable GPIO B clock
    // CAN1 GPIO Configuration
    // PB8     ------> CAN1_RX
    // PB9     ------> CAN1_TX

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9; // Pin PB8 and Pin PB9
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;           // Alternate function 9
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);              // Port B

    // CAN1 interrupt Init
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);         // Enable CAN FIFO1 interrupt handler

    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);         // Enable CAN FIFO1 interrupt handler
  }
}

HAL_StatusTypeDef CAN1_Stop() {
  return HAL_CAN_Stop(&hcan1);
}

HAL_StatusTypeDef CAN1_Start() {
  HAL_StatusTypeDef status = HAL_OK;

  // Init TxHeader with constant values
  TxHeader.ExtId              = 0;
  TxHeader.StdId              = 0;
  TxHeader.RTR                = CAN_RTR_DATA; // Data transmission type: CAN_RTR_DATA / CAN_RTR_REMOTE
  TxHeader.TransmitGlobalTime = DISABLE; // Put timestamp in Data[6-7], requires Time Triggered Communication Mode

  // CAN baud rate = clock frequency / clock divider / prescaler / (1 + TSG1 + TSG2)
  // Baud rate = 42M / 3 / 1 / (1 + 11 + 2) = 1M baud
  // Baud rate = 42M / 3 / 2 / (1 + 11 + 2) = 500k baud
  // Baud rate = 42M / 3 / 4 / (1 + 11 + 2) = 250k baud
  hcan1.Instance                  = CAN1;
  hcan1.Init.Prescaler            = 3;               // 1-1024, 42MHz peripheral clock / 3 --> 14MHz -> 1M baud. 6 --> 500K baud. 12 --> 250K baud.
  hcan1.Init.AutoBusOff           = DISABLE;         // DISABLE: Software controlled Bus-off. ENABLE: Automatic hardware controlled (no send/receive)
  hcan1.Init.AutoWakeUp           = ENABLE;          // ENABLE: Automatic hardware controlled bus wakeup. DISABLE: Software controlled bus wakeup.
  hcan1.Init.AutoRetransmission   = ENABLE;          // DISABLE / ENABLE, resend if transmission failed, but locks up if communication fails/cable not connected!!!!!!!!!!!!!!!!!
  hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;     // CAN_SJW_1TQ
  hcan1.Init.TimeSeg1             = CAN_BS1_11TQ;    // CAN_BS1_11TQ
  hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;     // CAN_BS2_2TQ
  hcan1.Init.Mode                 = CAN_MODE_NORMAL; // CAN_MODE_NORMAL / CAN_MODE_SILENT / CAN_MODE_LOOPBACK / CAN_MODE_SILENT_LOOPBACK
  hcan1.Init.TimeTriggeredMode    = DISABLE;         // TTCAN is used to assign timeslot to the devices for real time applications
  hcan1.Init.ReceiveFifoLocked    = DISABLE;         // Handle RX FIFO overruns. DISABLE: Overwrite previous message with new one. ENABLE: Discard the new message.
  hcan1.Init.TransmitFifoPriority = ENABLE;          // Handle TX FIFO send order. ENABLE: Chronologically. DISABLE: Transmit lower ID number first.

  status = HAL_CAN_Init(&hcan1); // Calls HAL_CAN_MspInit
  if (status != HAL_OK)
    return status;

  CAN_FilterTypeDef  sFilterConfig;

  // Catch CAN messags with highest bit of StdId set in FIFO0
  sFilterConfig.FilterBank           = 0; // This filter bank ID number (0-13 for single CAN instances)
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK; // Accept if "Received ID" & Mask = ID, CAN_FILTERMODE_IDMASK / CAN_FILTERMODE_IDLIST (See Figure 342 in RM0090)
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT; // CAN_FILTERSCALE_16BIT / CAN_FILTERSCALE_32BIT (See Figure 342 in RM0090)
  sFilterConfig.FilterIdHigh         = 0b1000000000000000;    // ID MSB:   (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
  sFilterConfig.FilterIdLow          = 0;                     // ID LSB:   (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterMaskIdHigh     = 0b1000000000000000;    // Mask MSB: (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
  sFilterConfig.FilterMaskIdLow      = 0;                     // Mask LSB: (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // Store message in FIFO1 (CAN_FILTER_FIFO0 / CAN_FILTER_FIFO1)
  sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;     // CAN_FILTER_ENABLE / CAN_FILTER_DISABLE
  sFilterConfig.SlaveStartFilterBank = 14;                    // Start bank number for CAN slave instance (not used in single CAN setups)
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

 // Catch all remaining CAN messages in FIFO1
  sFilterConfig.FilterBank           = 1; // This filter bank ID number (0-13 for single CAN instances)
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK; // CAN_FILTERMODE_IDMASK / CAN_FILTERMODE_IDLIST (See Figure 342 in RM0090)
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT; // CAN_FILTERSCALE_16BIT / CAN_FILTERSCALE_32BIT (See Figure 342 in RM0090)
  sFilterConfig.FilterIdHigh         = 0b0000;                // ID MSB:   (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
  sFilterConfig.FilterIdLow          = 0x0000;                // ID LSB:   (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterMaskIdHigh     = 0b0000;                // Mask MSB: (0-0xFFFF) (StdId[10-0] [ExtId17-13]) (See Figure 342 in RM0090)
  sFilterConfig.FilterMaskIdLow      = 0x0000;                // Mask LSB: (0-0xFFFF) ([ExtId12-0][IDE][RTR]  0) (0="don't care")
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;      // Store message in FIFO0 (CAN_FILTER_FIFO0 / CAN_FILTER_FIFO1)
  sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;     // CAN_FILTER_ENABLE / CAN_FILTER_DISABLE
  sFilterConfig.SlaveStartFilterBank = 14;                    // Start bank number for CAN slave instance (not used in single CAN setups)
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  // Activate RX FIFO0/FIFO1 new message interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // Calls CAN1_RX0_IRQHandler -> HAL_CAN_IRQHandler -> HAL_CAN_RxFifo0MsgPendingCallback
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING); // Calls CAN1_RX1_IRQHandler -> HAL_CAN_IRQHandler -> HAL_CAN_RxFifo1MsgPendingCallback

// Activate RX FIFO0/FIFO1 overrun interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_OVERRUN); // Calls CAN1_RX0_IRQHandler -> HAL_CAN_ErrorCallback
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_OVERRUN); // Calls CAN1_RX1_IRQHandler -> HAL_CAN_ErrorCallback

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR); // Calls CAN1_RX0_IRQHandler -> HAL_CAN_ErrorCallback

  status = HAL_CAN_Start(&hcan1);   // Start the CAN1 module
  if (status != HAL_OK)
    return status;

  return CAN_Send_Gcode_2params('M', 997, 0, 0, 0, 0); // M997, reset head at host startup
}

void HAL_CAN_RxFifoMsgPending(CAN_HandleTypeDef *hcan, uint32_t RxFifo) { // ISR! New FIFO 0/1 message interrupt handler
  CAN_RxHeaderTypeDef RxHeader;  // RX Header buffer for FIFO0
  uint8_t CAN_RX_buffer_Fifo[8]; // CAN MESSAGE DATA BUFFER

  if (HAL_CAN_GetRxMessage(&hcan1, RxFifo, &RxHeader, CAN_RX_buffer_Fifo) == HAL_OK) { // Get message from CAN_RX_FIFO0
    if ((RxHeader.StdId & CAN_IO_MASK) != CAN_io_state) { // First handle time critical IO update
      CAN_io_state = (RxHeader.StdId & CAN_IO_MASK);
      endstops.update();
    }

    if (RxHeader.StdId & CAN_STRING_MESSAGE_MASK) { // Head sends a string message
      char * CAN_RX_p = (char *)CAN_RX_buffer_Fifo;
      for (uint32_t i = 0; i < RxHeader.DLC; i++) {
        string_message[string_message_index++ % 128] = CAN_RX_p[i]; // Copy message to global buffer

        if (CAN_RX_p[i] == '\n') {
          string_message_complete = true; // Print buffer
          string_message[string_message_index % 128] = 0; // Close string with \0
        }
      }
    }
    else if (RxHeader.DLC == 4) { // FIFO0, head sent a temperature update (DLC = Data Length Code == 4 bytes)
      float * fp = (float *)CAN_RX_buffer_Fifo;   // FIFO0
      thermalManager.temp_hotend[0].celsius = *fp; // Set E0 hotend temperature
      Next_CAN_Temp_Report = millis() + 10000;
      FirstE0Error = true; // Reset error status
    }

    CAN_head_setup_request = (RxHeader.StdId & CAN_REQUEST_SETUP_MASK) > 0; // FIFO0, head signals request for data
    CAN_head_error = (RxHeader.StdId & CAN_ERROR_MASK) > 0; // FIFO0, head signals an error
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { // ISR! New FIFO0 message interrupt handler
  HAL_CAN_RxFifoMsgPending(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) { // ISR! New FIFO1 message interrupt handler
  HAL_CAN_RxFifoMsgPending(hcan, CAN_RX_FIFO1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) { // Interrupt handler for any CAN error
  HAL_CAN_error_code = hcan->ErrorCode; // Store the received error code
}

/*
CAN Bus Control functions
  HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)                                Start the CAN module
  HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan)                                 Stop the CAN module
  HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan)                         Request sleep mode entry.
  HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan)                               Wake up from sleep mode.
  uint32_t HAL_CAN_IsSleepActive(CAN_HandleTypeDef *hcan)                                 Check is sleep mode is active.
  HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)  Add a message to the Tx mailboxes and activate the corresponding transmission request
  HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes) Abort transmission request
  uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)                       Return Tx mailboxes free level
  uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);Check if a transmission request is pending on the selected Tx mailbox
  uint32_t HAL_CAN_GetTxTimestamp(CAN_HandleTypeDef *hcan, uint32_t TxMailbox)            Return Tx Timestamp
  HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])  Get a CAN frame from the Rx FIFO
  uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo)           Return Rx FIFO fill level

CAN INTERRUPT FUNCTIONS (See STM32F4xx_hal_can.c)
  HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)     // Enable interrupts
  HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs) // Disable interrupts

CAN WEAK CALLBACKS WHEN USING STANDARD WEAK CAN1_RX0_IRQHandler------------> INTERRUPTS
  __weak void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) // CAN_IT_TX_MAILBOX_EMPTY
  __weak void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) // CAN_IT_TX_MAILBOX_EMPTY
  __weak void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) // CAN_IT_TX_MAILBOX_EMPTY
  __weak void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)    // CAN_IT_TX_MAILBOX_EMPTY
  __weak void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)    // CAN_IT_TX_MAILBOX_EMPTY
  __weak void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)    // CAN_IT_TX_MAILBOX_EMPTY
  __weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  // CAN_IT_RX_FIFO0_MSG_PENDING
  __weak void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)        // CAN_IT_RX_FIFO0_FULL
  __weak void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)  // CAN_IT_RX_FIFO1_MSG_PENDING
  __weak void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)        // CAN_IT_RX_FIFO1_FULL
  __weak void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)              // CAN_IT_SLEEP_ACK
  __weak void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)    // CAN_IT_WAKEUP
  __weak void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)              // CAN_IT_ERROR
*/

#endif // CAN_MASTER
