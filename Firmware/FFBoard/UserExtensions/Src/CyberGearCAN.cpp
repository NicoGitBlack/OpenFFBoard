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
		 .id=CLSID_MOT_ODRV0,	// 5
};
bool CyberGearCAN2::inUse = false;
ClassIdentifier CyberGearCAN2::info = {
		 .name = "CyberGear (M1)" ,
		 .id=CLSID_MOT_ODRV1,	// 6
};



const ClassIdentifier CyberGearCAN1::getInfo(){
	return info;
}

const ClassIdentifier CyberGearCAN2::getInfo(){
	return info;
}

bool CyberGearCAN1::isCreatable(){
	return !CyberGearCAN1::inUse; // Creatable if not already in use for example by another axis
}

bool CyberGearCAN2::isCreatable(){
	return !CyberGearCAN2::inUse; // Creatable if not already in use for example by another axis
}

CyberGearCAN::CyberGearCAN(uint8_t id)  : CommandHandler("odrv", CLSID_MOT_ODRV0,id),  Thread("CYBERGEAR", CYBERGEAR_THREAD_MEM, CYBERGEAR_THREAD_PRIO), motorId(id) {

	if(motorId == 0){
		nodeId = 0;
	}else if(motorId == 1){
		nodeId = 1; // defaults
	}
	restoreFlash();

	setCanFilter();

	if(port->getSpeedPreset() < 3){
		port->setSpeedPreset(3); // Minimum 250k
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
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0x0000;
//	sFilterConfig.FilterIdLow = 0x0000;
//	sFilterConfig.FilterMaskIdHigh = 0x0000;
//	sFilterConfig.FilterMaskIdLow = 0x0000;
	uint32_t filter_id = (nodeId & 0x3F) << 5;
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

	uint16_t canIds = 0x3040;
	if(Flash_Read(ADR_CYBERGEAR_CANID, &canIds)){
		if(motorId == 0){
			nodeId = canIds & 0x3f;
		}else if(motorId == 1){
			nodeId = (canIds >> 6) & 0x3f;
			setting1addr = ADR_CYBERGEAR_SETTING1_M1;
		}
//		uint8_t canspd = (canIds >> 12) & 0x7;
//		this->setCanRate(canspd);
	}

	uint16_t settings1 = 0;
	if(Flash_Read(setting1addr, &settings1)){
		maxTorque = (float)clip(settings1 & 0xfff, 0, 0xfff) / 100.0;
	}
}

void CyberGearCAN::saveFlash(){
	uint16_t setting1addr = ADR_CYBERGEAR_SETTING1_M0;

	uint16_t canIds = 0x3040;
	Flash_Read(ADR_CYBERGEAR_CANID, &canIds); // Read again
	if(motorId == 0){
		canIds &= ~0x3F; // reset bits
		canIds |= nodeId & 0x3f;
	}else if(motorId == 1){
		setting1addr = ADR_CYBERGEAR_SETTING1_M1;
		canIds &= ~0xFC0; // reset bits
		canIds |= (nodeId & 0x3f) << 6;
	}
	canIds &= ~0x7000; // reset bits
//	canIds |= (this->baudrate & 0x7) << 12;
	Flash_Write(ADR_CYBERGEAR_CANID,canIds);

	uint16_t settings1 = ((int32_t)(maxTorque*100) & 0xfff);
	Flash_Write(setting1addr, settings1);
}

void CyberGearCAN::Run(){
	while(true){
		this->Delay(500);

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

		case CyberGearLocalState::START_RUNNING:
			state = CyberGearLocalState::RUNNING;
			// oybergear is active,
			// enable torque mode
			if(cybergearCurrentState == CyberGearState::AXIS_STATE_CLOSED_LOOP_CONTROL){
				this->setPos(0);
				setMode(CyberGearControlMode::CONTROL_MODE_TORQUE_CONTROL, CyberGearInputMode::INPUT_MODE_PASSTHROUGH);
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

		if(HAL_GetTick() - lastVoltageUpdate > 1000){
			requestMsg(0x17); // Update voltage
		}

		if(HAL_GetTick() - lastCanMessage > 2000){ // Timeout
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
 */
void CyberGearCAN::setPos(int32_t pos){
	// Only change encoder count internally as offset
	posOffset = lastPos - ((float)pos / (float)getCpr());
}


void CyberGearCAN::requestMsg(uint8_t cmd){
	CAN_tx_msg msg;

	msg.header.RTR = CAN_RTR_REMOTE;
	msg.header.DLC = 0;
	msg.header.StdId = cmd | (nodeId << 5);
	port->sendMessage(msg);
}

float CyberGearCAN::getPos_f(){
	// Do not start a new request if already waiting and within timeout
	if(this->connected && (!posWaiting || HAL_GetTick() - lastPosTime > 5)){
		posWaiting = true;
		requestMsg(0x09);
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
		sendMsg<float>(0x0E,torque);
	//}
}

EncoderType CyberGearCAN::getEncoderType(){
	return EncoderType::absolute;
}


void CyberGearCAN::setMode(CyberGearControlMode controlMode,CyberGearInputMode inputMode){
	uint64_t mode = (uint64_t) controlMode | ((uint64_t)inputMode << 32);
	sendMsg(0x0B,mode);
}

void CyberGearCAN::setState(CyberGearState state){
	sendMsg(0x07,(uint32_t)state);
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
	sendMsg(0x10, (uint8_t)0);
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
			replies.emplace_back((uint32_t)cybergearCurrentState);
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
	uint16_t node = (rxHeader->StdId >> 5) & 0x3F;
	if(node != this->nodeId){
		return;
	}
	uint64_t msg = *reinterpret_cast<uint64_t*>(rxBuf);
	uint8_t cmd = rxHeader->StdId & 0x1F;

	lastCanMessage = HAL_GetTick();

	switch(cmd){
	case 1:
	{
		// TODO error handling
		errors = (msg & 0xffffffff);
		cybergearCurrentState = (CyberGearState)( (msg >> 32) & 0xff);
		cybergearMotorFlags = (msg >> 40) & 0xff;
		cybergearEncoderFlags = ((msg >> 48) & 0xff);
		cybergearControllerFlags = (msg >> 56) & 0xff;

		if(waitReady){
			waitReady = false;
			state = CyberGearLocalState::WAIT_READY;
		}
		break;
	}
	case 0x09: // encoder pos float
	{
		uint64_t tp = msg & 0xffffffff;
		memcpy(&lastPos,&tp,sizeof(float));

		uint64_t ts = (msg >> 32) & 0xffffffff;
		memcpy(&lastSpeed,&ts,sizeof(float));
		lastPosTime = HAL_GetTick();
		posWaiting = false;

		break;
	}

	case 0x17: // voltage
	{
		lastVoltageUpdate = HAL_GetTick();
		uint64_t t = msg & 0xffffffff;
		memcpy(&lastVoltage,&t,sizeof(float));

		break;
	}



	default:
	break;
	}

}

#endif
