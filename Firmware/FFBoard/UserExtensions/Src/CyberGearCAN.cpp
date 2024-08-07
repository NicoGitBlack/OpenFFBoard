/*
 * CyberGearCAN.cpp
 *
 *  Created on: Apr 24, 2024
 *      Author: BlackBird
 */
#include "target_constants.h"
#ifdef CYBERGEAR
#include <CyberGearCAN.h>

bool CyberGearCAN1::inUse = false;
ClassIdentifier CyberGearCAN1::info = {
		 .name = "CyberGear (M0)" ,
		 .id=CLSID_MOT_CYBG0,	// B
};
bool CyberGearCAN2::inUse = false;
ClassIdentifier CyberGearCAN2::info = {
		 .name = "CyberGear (M1)" ,
		 .id=CLSID_MOT_CYBG1,	// C
};



uint8_t master_can_id_ = 0;       //!< master can id
uint8_t target_can_id_ = 0 ;       //!< target can id
uint8_t run_mode_ = (uint8_t) CyberGearControlMode::MODE_MOTION ;            //!< run mode
unsigned long send_count_ = 0 ;    //!< send count
uint8_t receive_buffer_[64] = {0};  //!< receive buffer
MotorStatus motor_status_;    //!< current motor status
MotorParameter motor_param_;  //!< motor parameter




// No significant modif from ODrive
const ClassIdentifier CyberGearCAN1::getInfo(){
	return info;
}
// No significant modif from ODrive
const ClassIdentifier CyberGearCAN2::getInfo(){
	return info;
}
// No significant modif from ODrive
bool CyberGearCAN1::isCreatable(){
	return !CyberGearCAN1::inUse; // Creatable if not already in use for example by another axis
}
// No significant modif from ODrive
bool CyberGearCAN2::isCreatable(){
	return !CyberGearCAN2::inUse; // Creatable if not already in use for example by another axis
}

CyberGearCAN::CyberGearCAN(uint8_t id)  : CommandHandler("cybg", CLSID_MOT_CYBG0,id),
										Thread("CYBERGEAR", CYBERGEAR_THREAD_MEM, CYBERGEAR_THREAD_PRIO),
										motorId(id) {





	if(motorId == 0){
		nodeId = 0;
	}else if(motorId == 1){
		nodeId = 1; // defaults
	}
	restoreFlash();

	setCanFilter();

	if(port->getSpeedPreset() < 5){
		port->setSpeedPreset(5); // Minimum 1000k for cybergear
	}

	this->port->setSilentMode(false);
	this->registerCommands();
	this->port->takePort();
	this->Start();
}

CyberGearCAN::~CyberGearCAN() {
	this->setTorque(0.0);
	this->port->removeCanFilter(filterId);
	this->port->freePort();
}

void CyberGearCAN::setCanFilter(){
	// Set up a filter to receive cybergear commands
	/*
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0x0000;
//	sFilterConfig.FilterIdLow = 0x0000;
//	sFilterConfig.FilterMaskIdHigh = 0x0000;
//	sFilterConfig.FilterMaskIdLow = 0x0000;
	uint32_t filter_id = (nodeId & 0x7F) << 6;
	uint32_t filter_mask = 0x07E0;
	sFilterConfig.FilterIdHigh = filter_id << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = filter_mask << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;

//	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterFIFOAssignment = motorId % 2 == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	this->filterId = this->port->addCanFilter(sFilterConfig);
	*/
	this->port  = &canport;

	// Set up a filter to receive everything
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	txHeader.StdId = 12;
	txHeader.ExtId = 0;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;	//Use std id
	txHeader.DLC = 8;	// 8 bytes
	txHeader.TransmitGlobalTime = DISABLE;

	this->filterId = this->port->addCanFilter(sFilterConfig);
	conf1.enabled = true;
 

	this->port->setSilentMode(false);
	this->port->takePort();









}

void CyberGearCAN::registerCommands(){
	CommandHandler::registerCommands();
	registerCommand("canid", CyberGearCAN_commands::canid, "CAN id of CyberGear",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("canspd", CyberGearCAN_commands::canspd, "CAN baudrate",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("errors", CyberGearCAN_commands::errors, "CyberGear error flags",CMDFLAG_GET);
	registerCommand("state", CyberGearCAN_commands::state, "CyberGear state",CMDFLAG_GET);
	registerCommand("maxtorque", CyberGearCAN_commands::maxtorque, "Max torque to send for scaling",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("vbus", CyberGearCAN_commands::vbus, "CyberGear Vbus",CMDFLAG_GET);
	registerCommand("anticogging", CyberGearCAN_commands::anticogging, "Set 1 to start anticogging calibration",CMDFLAG_SET);
	registerCommand("connected", CyberGearCAN_commands::connected, "CyberGear connection state",CMDFLAG_GET);
}

void CyberGearCAN::restoreFlash(){
	uint16_t setting1addr = ADR_CYBERGEAR_SETTING1_M0;

	uint16_t canIds = 0x057f;
	if(true) { //Flash_Read(ADR_CYBERGEAR_CANID_M0, &canIds)){
		if(motorId == 0){
			if(Flash_Read(ADR_CYBERGEAR_CANID_M0, &canIds)){
				nodeId = canIds & 0x7f;   // 0x7f = 0b0111'1111
			}
		}else if(motorId == 1){
			if(Flash_Read(ADR_CYBERGEAR_CANID_M1, &canIds)){
				nodeId = canIds & 0x7f;   // 0x7f = 0b0111'1111
			}
			setting1addr = ADR_CYBERGEAR_SETTING1_M1;
		}
		uint8_t canspd = (canIds >> 8) & 0x7;
		this->setCanRate(CANSPEEDPRESET_1000);
	}
/*
	uint16_t settings1 = 0;
	if(Flash_Read(setting1addr, &settings1)){
		maxTorque = (float)clip(settings1 & 0xfff, 0, 0xfff) / 100.0;
	}*/
	maxTorque = 0.5
}

void CyberGearCAN::saveFlash(){
	uint16_t setting1addr = ADR_CYBERGEAR_SETTING1_M0;

	uint16_t canIds = 0x057f;
	/*Flash_Read(ADR_CYBERGEAR_CANID, &canIds); // Read again
	if(motorId == 0){
		canIds &= ~0x3F; // reset bits
		canIds |= nodeId & 0x3f;
	}else if(motorId == 1){
		setting1addr = ADR_CYBERGEAR_SETTING1_M1;
		canIds &= ~0xFC0; // reset bits
		canIds |= (nodeId & 0x3f) << 6;
	}
	canIds &= ~0x7000; // reset bits*/
//	canIds |= (this->baudrate & 0x7) << 12;
	Flash_Write(ADR_CYBERGEAR_CANID_M0,canIds);
	Flash_Write(ADR_CYBERGEAR_CANID_M1,canIds);
/*
	uint16_t settings1 = ((int32_t)(maxTorque*100) & 0xfff);
	Flash_Write(setting1addr, settings1);*/
}

void CyberGearCAN::Run(){
	while(true){
		this->Delay(500);
		state = CyberGearLocalState::START_RUNNING ; // overwrite any previous state

/*
		switch(state){
		case CyberGearLocalState::WAIT_READY:
			if(this->cybergearCurrentState == CyberGearState::AXIS_STATE_IDLE){
				this->state = CyberGearLocalState::WAIT_CALIBRATION; // Wait
				this->setState(CyberGearState::AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
			}

		break;

		// Calibration in progress. wait until its finished to enter torque mode
		case CyberGearLocalState::WAIT_CALIBRATION_DONE:
			if(cybergearCurrentState == CyberGearState::AXIS_STATE_IDLE){
				setState(CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL);
				state = CyberGearLocalState::START_RUNNING;

			}else if(cybergearCurrentState == CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL){
				state = CyberGearLocalState::START_RUNNING;
			}
		break;
*/
		case CyberGearLocalState::START_RUNNING:
			state = CyberGearLocalState::RUNNING;
			setState(CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL);
			// oybergear is active,
			// enable torque mode
			if(cybergearCurrentState == CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL){
				this->setPos(0);
				setMode(CyberGearControlMode::MODE_CURRENT ); // , CyberGearInputMode::INPUT_MODE_PASSTHROUGH);
			}

		break;
		default:

		break;
		}

		// If cybergear is currently performing any calibration task wait until finished
		if(cybergearCurrentState == CyberGearState::AXIS_STATE_FULL_CALIBRATION_SEQUENCE || cybergearCurrentState == CyberGearState::AXIS_STATE_MOTOR_CALIBRATION || cybergearCurrentState == CyberGearState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION || cybergearCurrentState == CyberGearState::AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION || cybergearCurrentState == CyberGearState::AXIS_STATE_ENCODER_INDEX_SEARCH){
			if(state != CyberGearLocalState::WAIT_CALIBRATION_DONE)
				state = CyberGearLocalState::WAIT_CALIBRATION_DONE;
		// If closed loop mode is on assume its ready and already calibrated
		}else if(cybergearCurrentState == CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL){
			if(state != CyberGearLocalState::START_RUNNING && state != CyberGearLocalState::RUNNING)
				state = CyberGearLocalState::RUNNING;
		}

		if(HAL_GetTick() - lastVoltageUpdate > 10000){
			requestMsg(ADDR_VBUS); // Update voltage
		}

		if(HAL_GetTick() - lastCanMessage > 20000){ // Timeout
			cybergearCurrentState = CyberGearState::AXIS_STATE_UNDEFINED;
			state = CyberGearLocalState::IDLE;
			waitReady = true;
			connected = false;
		}else{
			connected = true;
		}

	}
}

void CyberGearCAN::stopMotor(){
	active = false;
	this->setTorque(0.0);
	if(cybergearCurrentState == CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL)
		this->setState(CyberGearState::AXIS_STATE_IDLE);
}

void CyberGearCAN::startMotor(){
	active = true;
	if(cybergearCurrentState == CyberGearState::AXIS_STATE_IDLE)
		this->setState(CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

Encoder* CyberGearCAN::getEncoder(){
	return static_cast<Encoder*>(this);
}

/**
 * Must be in encoder cpr if not just used to zero the axis
 * void CybergearDriver::set_position_ref(float position)
 * {
 *  write_float_data(target_can_id_, ADDR_LOC_REF, position, P_MIN, P_MAX);
 * }
 */

void CyberGearCAN::setPos(int32_t pos){
	// Only change encoder count internally as offset
	posOffset = lastPos - ((float)pos / (float)getCpr());

}


void CyberGearCAN::requestMsg(uint16_t  cmd){


	uint8_t data[8] = {0x00};
	memcpy(&data[0], &cmd, 2);
	send_command(target_can_id_, CMD_RAM_READ, master_can_id_, 8, data);

}

float CyberGearCAN::getPos_f(){
	// Do not start a new request if already waiting and within timeout
	if(this->connected && (!posWaiting || HAL_GetTick() - lastPosTime > 5)){
		posWaiting = true;
		requestMsg( ADDR_MECH_POS );
		'''
		With cyberGear configuration tool AT MODE ??? see https://github.com/Tony607/Cybergear/blob/main/notebooks/cybergear.ipynb
		Start request_MECHPos:
		2466 [2024/05/06 17:41:23]  -> COM9: 41 54 50 07 eb fc 08 64 00 11 00 00 10 0e 00 0d 0a
		2467 [2024/05/06 17:41:23]  -> COM9: 41 54 50 0f eb fc 08 16 30 16 30 16 30 00 00 0d 0a
		Stop request_MECHPos:
		2468 [2024/05/06 17:41:23]  -> COM9: 41 54 50 17 eb fc 08 00 00 00 00 00 00 00 00 0d 0a
		2469 [2024/05/06 17:41:25]  -> COM9: 41 54 50 1f eb fc 08 00 00 00 00 00 00 00 00 0d 0a

		'''

	}
	return lastPos-posOffset;
}

bool CyberGearCAN::motorReady(){
	return state == CyberGearLocalState::RUNNING && (cybergearCurrentState == CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL);
}
int32_t CyberGearCAN::getPos(){
	return getCpr() * getPos_f();
}


uint32_t CyberGearCAN::getCpr(){
	return 0xffff;
}
void CyberGearCAN::setTorque(float torque){
	//if(motorReady()){
		//sendMsg<float>(0x0E,torque);
	    // void write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max)
	float current = torque / maxTorque * IQ_MAX ;
		write_float_data(target_can_id_, ADDR_IQ_REF, current, IQ_MIN, IQ_MAX);
	//}
}

EncoderType CyberGearCAN::getEncoderType(){
	return EncoderType::absolute;
}


void CyberGearCAN::setMode(CyberGearControlMode controlMode ){ /// ,CyberGearInputMode inputMode){
	// uint64_t mode = (uint64_t) controlMode | ((uint64_t)inputMode << 32);
	// sendMsg(0x0B,mode);

	// set class variable
	  // run_mode_ = controlMode; //uint8_t run_mode
	  uint8_t data[8] = {0x00};
	  data[0] = ADDR_RUN_MODE & 0x00FF;
	  data[1] = ADDR_RUN_MODE >> 8;
	  data[4] = (uint8_t) controlMode  ;
	  send_command(target_can_id_, CMD_RAM_WRITE, master_can_id_, 8, data);

}

void CyberGearCAN::setState(CyberGearState state){
	sendMsg(0x07,(uint32_t)state);
}

void CyberGearCAN::send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data)
{
	CAN_tx_msg msg;
	memcpy(&msg.data,&data, len );
	msg.header.RTR = CAN_RTR_DATA;
	msg.header.DLC =  len ;
	uint32_t id = cmd_id << 24 | option << 8 | can_id; // from cybergear
	msg.header.StdId = id ;
	if(!port->sendMessage(msg)){
		// Nothing
	}
	//++send_count_;
}
void CyberGearCAN::write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max)
{
  uint8_t data[8] = {0x00};
  data[0] = addr & 0x00FF;
  data[1] = addr >> 8;

  float val = (max < value) ? max : value;
  val = (min > value) ? min : value;
  memcpy(&data[4], &val, 4);
  send_command(can_id, CMD_RAM_WRITE, master_can_id_, 8, data);
}



void CyberGearCAN::turn(int16_t power){
	float torque = ((float)power / (float)0x7fff) * maxTorque;
	this->setTorque(torque);
}

//void CyberGearCAN::setCanRate(uint8_t canRate){
//	baudrate = clip<uint8_t,uint8_t>(canRate, 3, 5);
//	port->setSpeedPreset(baudrate);
//}

/**
 * Sends the start anticogging command
 */
void CyberGearCAN::startAnticogging(){
	// sendMsg(0x10, (uint8_t)0);
}


CommandStatus CyberGearCAN::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<CyberGearCAN_commands>(cmd.cmdId)){
	case CyberGearCAN_commands::vbus:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(lastVoltage*1000);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case CyberGearCAN_commands::errors:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint32_t)errors);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case CyberGearCAN_commands::canid:
		handleGetSet(cmd, replies, this->nodeId);
		canport.removeCanFilter(filterId);
		setCanFilter();
		break;
	case CyberGearCAN_commands::state:
		if(cmd.type == CMDtype::get){
			replies.emp 
			lace_back((uint32_t)cybergearCurrentState);
		}else{
			return CommandStatus::ERR;
		}
		break;
	case CyberGearCAN_commands::canspd:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(port->getSpeedPreset());
		}else if(cmd.type == CMDtype::set){
			port->setSpeedPreset(std::max<uint8_t>(3,cmd.val));
		}else{
			return CommandStatus::ERR;
		}
		break;
	case CyberGearCAN_commands::anticogging:
		if(cmd.type == CMDtype::set && cmd.val == 1){
			this->startAnticogging();
		}else{
			return CommandStatus::ERR;
		}
		break;
	case CyberGearCAN_commands::maxtorque:
	{
		if(cmd.type == CMDtype::get){
			int32_t val = maxTorque*100;
			replies.emplace_back(val);
		}else if(cmd.type == CMDtype::set){
			maxTorque = (float)clip(cmd.val, 0, 0xfff) / 100.0;
		}else{
			return CommandStatus::ERR;
		}
		break;
	}
	case CyberGearCAN_commands::connected:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(connected ? 1 : 0);
		}
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;

}

void CyberGearCAN::canErrorCallback(CAN_HandleTypeDef *hcan){
	//pulseErrLed();
}



void CyberGearCAN::canRxPendCallback(CAN_HandleTypeDef *hcan,uint8_t* rxBuf,CAN_RxHeaderTypeDef* rxHeader,uint32_t fifo){
	uint16_t node = (rxHeader->StdId >> 5) & 0x7F;
	if(node != this->nodeId){
		return;
	}

	uint64_t msg = *reinterpret_cast<uint64_t*>(rxBuf);
	// uint16_t cmd = rxHeader->StdId & 0x1F;

	// uint16_t index = data[1] << 8 | data[0];
	uint16_t cmd = rxBuf[1] << 8 | rxBuf[0];

	uint8_t uint8_data;
	memcpy(&uint8_data, &rxBuf[4], sizeof(uint8_t));

	int16_t int16_data;
	memcpy(&int16_data, &rxBuf[4], sizeof(int16_t));

	float float_data;
	memcpy(&float_data, &rxBuf[4], sizeof(float));

	bool is_updated = true;




	cybergearCurrentState =  CyberGearState::AXIS_STATE_IDLE;



	lastCanMessage = HAL_GetTick();

	switch(cmd){ 
 

	case ADDR_RUN_MODE:
	      motor_param_.run_mode = uint8_data;
	      //CG_DEBUG_PRINTF("Receive ADDR_RUN_MODE = [0x%02x]\n", uint8_data);
	      break;


	case ADDR_MECH_POS : // encoder pos float
	{
		motor_param_.mech_pos = float_data;
		//CG_DEBUG_PRINTF("Receive ADDR_MECH_POS = [%f]\n", float_data);
		memcpy(&lastPos,&float_data,sizeof(float));
		lastPosTime = HAL_GetTick();
		posWaiting = false;
		break;
	}

	case ADDR_VBUS: // voltage
	{
		motor_param_.vbus = float_data;
		// CG_DEBUG_PRINTF("Receive ADDR_VBUS = [%f]\n", float_data);
		lastVoltageUpdate = HAL_GetTick();
		memcpy(&lastVoltage,&float_data,sizeof(float));
		break;
	}



	default:
	break;
	}

}

#endif
