/*		
 * eeprom_addresses.c		
 *		
 *  Created on: 24.01.2020		
 *  Author: Yannick		
 *		
 *	/!\ Generated from the file memory_map.csv 
   / ! \ DO NOT EDIT THIS DIRECTLY !!!	
 */		
		
#include "eeprom_addresses.h"		

/*		
Add all used addresses to the VirtAddVarTab[] array. This is important for the eeprom emulation to correctly transfer between pages.		
This ensures that addresses that were once used are not copied again in a page transfer if they are not in this array.		
*/		
		
const uint16_t VirtAddVarTab[NB_OF_VAR] =		
{
// System variables
	ADR_HW_VERSION,
	ADR_SW_VERSION,
	ADR_FLASH_VERSION,
	ADR_CURRENT_CONFIG,
// Ports
	ADR_CANCONF1,
	ADR_I2CCONF1,
// FFBWheel
	ADR_FFBWHEEL_BUTTONCONF,
	ADR_FFBWHEEL_ANALOGCONF,
	ADR_FFBWHEEL_CONF1,
// Button Sources:
	ADR_SPI_BTN_1_CONF,
	ADR_SHIFTERANALOG_CONF,
	ADR_LOCAL_BTN_CONF, // Pin mask
	ADR_LOCAL_BTN_CONF_2, // Misc settings
	ADR_SPI_BTN_2_CONF,
	ADR_SPI_BTN_1_CONF_2,
	ADR_SPI_BTN_2_CONF_2,
	ADR_LOCAL_BTN_CONF_3, // Pulse mask
// Local encoder
	ADR_ENCLOCAL_CPR,
	ADR_ENCLOCAL_OFS,
// PWM
	ADR_PWM_MODE,
// Local analog source
	ADR_LOCALANALOG_MASK,
// Shifter Analog
	ADR_SHIFTERANALOG_X_12,
	ADR_SHIFTERANALOG_X_56,
	ADR_SHIFTERANALOG_Y_135,
	ADR_SHIFTERANALOG_Y_246,
	ADR_SHIFTERANALOG_CONF_2,
	ADR_SHIFTERANALOG_CONF_3,
// PCF buttons
	ADR_PCFBTN_CONF1,
// CAN port
	ADR_CANBTN_CONF1,
	ADR_CANBTN_CONF2, // CAN ID
// CAN analog
	ADR_CANANALOG_CONF1,
// FFB Engine flash area
	ADR_FFB_CF_FILTER, // Constant Force Lowpass
	ADR_FFB_FR_FILTER, // Friction Lowpass
	ADR_FFB_DA_FILTER, // Damper Lowpass
	ADR_FFB_IN_FILTER, // Inertia Lowpass
	ADR_FFB_EFFECTS1, // 0-7 inertia, 8-15 friction
	ADR_FFB_EFFECTS2, // 0-7 spring, 8-15 damper
	ADR_FFB_EFFECTS3, // 0-7 friction ramp up zone, 8-9 filterProfile
// Button Sources:
	ADR_ADS111X_CONF1,
// How many axis configured 1-3
	ADR_AXIS_COUNT,
// AXIS1
	ADR_AXIS1_CONFIG, // 0-2 ENC, 3-5 DRV
	ADR_AXIS1_POWER,
	ADR_AXIS1_DEGREES,
	ADR_AXIS1_MAX_SPEED, // Store the max speed
	ADR_AXIS1_MAX_ACCEL, // Store the max accel
	ADR_AXIS1_ENDSTOP, // 0-7 endstop margin, 8-15 endstop stiffness
	ADR_AXIS1_EFFECTS1, // 0-7 idlespring, 8-15 damper
	ADR_AXIS1_SPEEDACCEL_FILTER, // Speed/Accel filter Lowpass profile
	ADR_AXIS1_ENC_RATIO, // Accel filter Lowpass
// TMC1
	ADR_TMC1_MOTCONF, // 0-2: MotType 3-5: PhiE source 6-15: Poles
	ADR_TMC1_CPR,
	ADR_TMC1_ENCA, // Misc
	ADR_TMC1_ADC_I0_OFS,
	ADR_TMC1_ADC_I1_OFS,
	ADR_TMC1_ENC_OFFSET,
	ADR_TMC1_OFFSETFLUX,
	ADR_TMC1_TORQUE_P,
	ADR_TMC1_TORQUE_I,
	ADR_TMC1_FLUX_P,
	ADR_TMC1_FLUX_I,
	ADR_TMC1_PHIE_OFS,
	ADR_TMC1_TRQ_FILT,
// AXIS2
	ADR_AXIS2_CONFIG, // 0-2 ENC, 3-5 DRV
	ADR_AXIS2_POWER,
	ADR_AXIS2_DEGREES,
	ADR_AXIS2_MAX_SPEED, // Store the max speed
	ADR_AXIS2_MAX_ACCEL, // Store the max accel
	ADR_AXIS2_ENDSTOP, // 0-7 endstop margin, 8-15 endstop stiffness
	ADR_AXIS2_EFFECTS1, // 0-7 idlespring, 8-15 damper
	ADR_AXIS2_SPEEDACCEL_FILTER, // Speed/Accel filter Lowpass profile
	ADR_AXIS2_ENC_RATIO, // Store the encoder ratio for an axis
// TMC2
	ADR_TMC2_MOTCONF, // 0-2: MotType 3-5: PhiE source 6-15: Poles
	ADR_TMC2_CPR,
	ADR_TMC2_ENCA, // Misc
	ADR_TMC2_ADC_I0_OFS,
	ADR_TMC2_ADC_I1_OFS,
	ADR_TMC2_ENC_OFFSET,
	ADR_TMC2_OFFSETFLUX,
	ADR_TMC2_TORQUE_P,
	ADR_TMC2_TORQUE_I,
	ADR_TMC2_FLUX_P,
	ADR_TMC2_FLUX_I,
	ADR_TMC2_PHIE_OFS,
	ADR_TMC2_TRQ_FILT,
// AXIS3
	ADR_AXIS3_CONFIG, // 0-2 ENC, 3-5 DRV
	ADR_AXIS3_POWER,
	ADR_AXIS3_DEGREES,
	ADR_AXIS3_MAX_SPEED, // Store the max speed
	ADR_AXIS3_MAX_ACCEL, // Store the max accel
	ADR_AXIS3_ENDSTOP, // 0-7 endstop margin, 8-15 endstop stiffness
	ADR_AXIS3_EFFECTS1, // 0-7 idlespring, 8-15 damper
	ADR_AXIS3_SPEEDACCEL_FILTER, // Speed/Accel filter Lowpass profile
	ADR_AXIS3_ENC_RATIO, // Store the encoder ratio for an axis
// TMC3
	ADR_TMC3_MOTCONF, // 0-2: MotType 3-5: PhiE source 6-15: Poles
	ADR_TMC3_CPR,
	ADR_TMC3_ENCA, // Misc
	ADR_TMC3_ADC_I0_OFS,
	ADR_TMC3_ADC_I1_OFS,
	ADR_TMC3_ENC_OFFSET,
	ADR_TMC3_OFFSETFLUX,
	ADR_TMC3_TORQUE_P,
	ADR_TMC3_TORQUE_I,
	ADR_TMC3_FLUX_P,
	ADR_TMC3_FLUX_I,
	ADR_TMC3_PHIE_OFS,
	ADR_TMC3_TRQ_FILT,
// Odrive
	ADR_ODRIVE_CANID, //0-6 ID M0, 7-12 ID M1, 13-15 can speed
	ADR_ODRIVE_SETTING1_M0,
	ADR_ODRIVE_SETTING1_M1,
// VESC Section
	ADR_VESC1_CANID, //0-7 AxisCanID, 8-16 VescCanId
	ADR_VESC1_DATA, //0-2 can speed, 3 useVescEncoder
	ADR_VESC1_OFFSET, //16b offset
	ADR_VESC2_CANID, //0-8 AxisCanID, 8-16 VescCanId
	ADR_VESC2_DATA, //0-2 can speed, 3 useVescEncoder
	ADR_VESC2_OFFSET, //16b offset
	ADR_VESC3_CANID, //0-8 AxisCanID, 8-16 VescCanId
	ADR_VESC3_DATA, //0-2 can speed, 3 useVescEncoder
	ADR_VESC3_OFFSET, //16b offset
// CYBERGEAR Section
	ADR_CYBERGEAR1_CANID, //0-7 AxisCanID, 8-16 CyberGearCanId
	ADR_CYBERGEAR1_DATA, //0-2 can speed, 3 useCyberGearEncoder
	ADR_CYBERGEAR1_OFFSET, //16b offset
	ADR_CYBERGEAR2_CANID, //0-7 AxisCanID, 8-16 CyberGearCanId
	ADR_CYBERGEAR2_DATA, //0-2 can speed, 3 useCyberGearEncoder
	ADR_CYBERGEAR2_OFFSET, //16b offset
	ADR_CYBERGEAR3_CANID, //0-7 AxisCanID, 8-16 CyberGearCanId
	ADR_CYBERGEAR3_DATA, //0-2 can speed, 3 useCyberGearEncoder
	ADR_CYBERGEAR3_OFFSET, //16b offset

//MT Encoder
	ADR_MTENC_OFS,
	ADR_MTENC_CONF1,
// Biss-C
	ADR_BISSENC_CONF1,
	ADR_BISSENC_OFS,
// SSI
	ADR_SSI_CONF1,
	ADR_SSI_OFS,
// Analog min/max calibrations
	ADR_LOCALANALOG_MIN_0,
	ADR_LOCALANALOG_MAX_0,
	ADR_LOCALANALOG_MIN_1,
	ADR_LOCALANALOG_MAX_1,
	ADR_LOCALANALOG_MIN_2,
	ADR_LOCALANALOG_MAX_2,
	ADR_LOCALANALOG_MIN_3,
	ADR_LOCALANALOG_MAX_3,
	ADR_LOCALANALOG_MIN_4,
	ADR_LOCALANALOG_MAX_4,
	ADR_LOCALANALOG_MIN_5,
	ADR_LOCALANALOG_MAX_5,
	ADR_LOCALANALOG_MIN_6,
	ADR_LOCALANALOG_MAX_6,
	ADR_LOCALANALOG_MIN_7,
	ADR_LOCALANALOG_MAX_7,
	ADR_ADS111X_MIN_0,
	ADR_ADS111X_MAX_0,
	ADR_ADS111X_MIN_1,
	ADR_ADS111X_MAX_1,
	ADR_ADS111X_MIN_2,
	ADR_ADS111X_MAX_2,
	ADR_ADS111X_MIN_3,
	ADR_ADS111X_MAX_3,
};

/**		
 * Variables to be included in a flash dump		
 */		
const uint16_t exportableFlashAddresses[NB_EXPORTABLE_ADR] =		
{
// System variables
//	ADR_HW_VERSION,
//	ADR_SW_VERSION,
	ADR_FLASH_VERSION,
	ADR_CURRENT_CONFIG,
// Ports
	ADR_CANCONF1,
	ADR_I2CCONF1,
// FFBWheel
	ADR_FFBWHEEL_BUTTONCONF,
	ADR_FFBWHEEL_ANALOGCONF,
	ADR_FFBWHEEL_CONF1,
// Button Sources:
	ADR_SPI_BTN_1_CONF,
	ADR_SHIFTERANALOG_CONF,
	ADR_LOCAL_BTN_CONF, // Pin mask
	ADR_LOCAL_BTN_CONF_2, // Misc settings
	ADR_SPI_BTN_2_CONF,
	ADR_SPI_BTN_1_CONF_2,
	ADR_SPI_BTN_2_CONF_2,
	ADR_LOCAL_BTN_CONF_3, // Pulse mask
// Local encoder
	ADR_ENCLOCAL_CPR,
	ADR_ENCLOCAL_OFS,
// PWM
	ADR_PWM_MODE,
// Local analog source
	ADR_LOCALANALOG_MASK,
// Shifter Analog
	ADR_SHIFTERANALOG_X_12,
	ADR_SHIFTERANALOG_X_56,
	ADR_SHIFTERANALOG_Y_135,
	ADR_SHIFTERANALOG_Y_246,
	ADR_SHIFTERANALOG_CONF_2,
	ADR_SHIFTERANALOG_CONF_3,
// PCF buttons
	ADR_PCFBTN_CONF1,
// CAN port
	ADR_CANBTN_CONF1,
	ADR_CANBTN_CONF2, // CAN ID
// CAN analog
//	ADR_CANANALOG_CONF1,
// FFB Engine flash area
	ADR_FFB_CF_FILTER, // Constant Force Lowpass
	ADR_FFB_FR_FILTER, // Friction Lowpass
	ADR_FFB_DA_FILTER, // Damper Lowpass
	ADR_FFB_IN_FILTER, // Inertia Lowpass
	ADR_FFB_EFFECTS1, // 0-7 inertia, 8-15 friction
	ADR_FFB_EFFECTS2, // 0-7 spring, 8-15 damper
	ADR_FFB_EFFECTS3, // 0-7 friction ramp up zone, 8-9 filterProfile
// Button Sources:
	ADR_ADS111X_CONF1,
// How many axis configured 1-3
	ADR_AXIS_COUNT,
// AXIS1
	ADR_AXIS1_CONFIG, // 0-2 ENC, 3-5 DRV
	ADR_AXIS1_POWER,
	ADR_AXIS1_DEGREES,
	ADR_AXIS1_MAX_SPEED, // Store the max speed
	ADR_AXIS1_MAX_ACCEL, // Store the max accel
	ADR_AXIS1_ENDSTOP, // 0-7 endstop margin, 8-15 endstop stiffness
	ADR_AXIS1_EFFECTS1, // 0-7 idlespring, 8-15 damper
	ADR_AXIS1_SPEEDACCEL_FILTER, // Speed/Accel filter Lowpass profile
	ADR_AXIS1_ENC_RATIO, // Accel filter Lowpass
// TMC1
	ADR_TMC1_MOTCONF, // 0-2: MotType 3-5: PhiE source 6-15: Poles
	ADR_TMC1_CPR,
	ADR_TMC1_ENCA, // Misc
//	ADR_TMC1_ADC_I0_OFS,
//	ADR_TMC1_ADC_I1_OFS,
//	ADR_TMC1_ENC_OFFSET,
	ADR_TMC1_OFFSETFLUX,
	ADR_TMC1_TORQUE_P,
	ADR_TMC1_TORQUE_I,
	ADR_TMC1_FLUX_P,
	ADR_TMC1_FLUX_I,
//	ADR_TMC1_PHIE_OFS,
	ADR_TMC1_TRQ_FILT,
// AXIS2
	ADR_AXIS2_CONFIG, // 0-2 ENC, 3-5 DRV
	ADR_AXIS2_POWER,
	ADR_AXIS2_DEGREES,
	ADR_AXIS2_MAX_SPEED, // Store the max speed
	ADR_AXIS2_MAX_ACCEL, // Store the max accel
	ADR_AXIS2_ENDSTOP, // 0-7 endstop margin, 8-15 endstop stiffness
	ADR_AXIS2_EFFECTS1, // 0-7 idlespring, 8-15 damper
	ADR_AXIS2_SPEEDACCEL_FILTER, // Speed/Accel filter Lowpass profile
	ADR_AXIS2_ENC_RATIO, // Store the encoder ratio for an axis
// TMC2
	ADR_TMC2_MOTCONF, // 0-2: MotType 3-5: PhiE source 6-15: Poles
	ADR_TMC2_CPR,
	ADR_TMC2_ENCA, // Misc
//	ADR_TMC2_ADC_I0_OFS,
//	ADR_TMC2_ADC_I1_OFS,
//	ADR_TMC2_ENC_OFFSET,
	ADR_TMC2_OFFSETFLUX,
	ADR_TMC2_TORQUE_P,
	ADR_TMC2_TORQUE_I,
	ADR_TMC2_FLUX_P,
	ADR_TMC2_FLUX_I,
//	ADR_TMC2_PHIE_OFS,
	ADR_TMC2_TRQ_FILT,
// AXIS3
	ADR_AXIS3_CONFIG, // 0-2 ENC, 3-5 DRV
	ADR_AXIS3_POWER,
	ADR_AXIS3_DEGREES,
	ADR_AXIS3_MAX_SPEED, // Store the max speed
	ADR_AXIS3_MAX_ACCEL, // Store the max accel
	ADR_AXIS3_ENDSTOP, // 0-7 endstop margin, 8-15 endstop stiffness
	ADR_AXIS3_EFFECTS1, // 0-7 idlespring, 8-15 damper
	ADR_AXIS3_SPEEDACCEL_FILTER, // Speed/Accel filter Lowpass profile
	ADR_AXIS3_ENC_RATIO, // Store the encoder ratio for an axis
// TMC3
	ADR_TMC3_MOTCONF, // 0-2: MotType 3-5: PhiE source 6-15: Poles
	ADR_TMC3_CPR,
	ADR_TMC3_ENCA, // Misc
//	ADR_TMC3_ADC_I0_OFS,
//	ADR_TMC3_ADC_I1_OFS,
//	ADR_TMC3_ENC_OFFSET,
	ADR_TMC3_OFFSETFLUX,
	ADR_TMC3_TORQUE_P,
	ADR_TMC3_TORQUE_I,
	ADR_TMC3_FLUX_P,
	ADR_TMC3_FLUX_I,
//	ADR_TMC3_PHIE_OFS,
	ADR_TMC3_TRQ_FILT,
// Odrive
	ADR_ODRIVE_CANID, //0-6 ID M0, 7-12 ID M1, 13-15 can speed
	ADR_ODRIVE_SETTING1_M0,
	ADR_ODRIVE_SETTING1_M1,
// VESC Section
	ADR_VESC1_CANID, //0-7 AxisCanID, 8-16 VescCanId
	ADR_VESC1_DATA, //0-2 can speed, 3 useVescEncoder
	ADR_VESC1_OFFSET, //16b offset
	ADR_VESC2_CANID, //0-8 AxisCanID, 8-16 VescCanId
	ADR_VESC2_DATA, //0-2 can speed, 3 useVescEncoder
	ADR_VESC2_OFFSET, //16b offset
	ADR_VESC3_CANID, //0-8 AxisCanID, 8-16 VescCanId
	ADR_VESC3_DATA, //0-2 can speed, 3 useVescEncoder
	ADR_VESC3_OFFSET, //16b offset	
// CYBERGEAR Section
	ADR_CYBERGEAR1_CANID, //0-7 AxisCanID, 8-16 CyberGearCanId
	ADR_CYBERGEAR1_DATA, //0-2 can speed, 3 useCyberGearEncoder
	ADR_CYBERGEAR1_OFFSET, //16b offset
	ADR_CYBERGEAR2_CANID, //0-7 AxisCanID, 8-16 CyberGearCanId
	ADR_CYBERGEAR2_DATA, //0-2 can speed, 3 useCyberGearEncoder
	ADR_CYBERGEAR2_OFFSET, //16b offset
	ADR_CYBERGEAR3_CANID, //0-7 AxisCanID, 8-16 CyberGearCanId
	ADR_CYBERGEAR3_DATA, //0-2 can speed, 3 useCyberGearEncoder
	ADR_CYBERGEAR3_OFFSET, //16b offset

//MT Encoder
	ADR_MTENC_OFS,
	ADR_MTENC_CONF1,
// Biss-C
	ADR_BISSENC_CONF1,
	ADR_BISSENC_OFS,
// SSI
	ADR_SSI_CONF1,
	ADR_SSI_OFS,
// Analog min/max calibrations
	ADR_LOCALANALOG_MIN_0,
	ADR_LOCALANALOG_MAX_0,
	ADR_LOCALANALOG_MIN_1,
	ADR_LOCALANALOG_MAX_1,
	ADR_LOCALANALOG_MIN_2,
	ADR_LOCALANALOG_MAX_2,
	ADR_LOCALANALOG_MIN_3,
	ADR_LOCALANALOG_MAX_3,
	ADR_LOCALANALOG_MIN_4,
	ADR_LOCALANALOG_MAX_4,
	ADR_LOCALANALOG_MIN_5,
	ADR_LOCALANALOG_MAX_5,
	ADR_LOCALANALOG_MIN_6,
	ADR_LOCALANALOG_MAX_6,
	ADR_LOCALANALOG_MIN_7,
	ADR_LOCALANALOG_MAX_7,
	ADR_ADS111X_MIN_0,
	ADR_ADS111X_MAX_0,
	ADR_ADS111X_MIN_1,
	ADR_ADS111X_MAX_1,
	ADR_ADS111X_MIN_2,
	ADR_ADS111X_MAX_2,
	ADR_ADS111X_MIN_3,
	ADR_ADS111X_MAX_3,
};