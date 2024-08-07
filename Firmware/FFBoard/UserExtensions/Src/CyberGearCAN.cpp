/*
 * CyberGearCAN.cpp
 *
 *  Created on: Aug 08, 2021
 *      Author: Vincent Manoukian (manoukianv@gmail.com)
 */
#include "target_constants.h"
#ifdef CYBERGEAR
#include <CyberGearCAN.h>
#include "ClassIDs.h"
#include "CRC.h"

bool CyberGearCAN::crcTableInitialized = false;
std::array<uint16_t,256> CyberGearCAN::crc16_tab __attribute__((section (".ccmram"))); // Generate in ram to save some flash (512B)

// *****    static initializer for the CYBERGEAR_1 instance (extend CYBERGEAR_CAN) *****

bool CYBERGEAR_1::inUse = false;

ClassIdentifier CYBERGEAR_1::info = {
	 .name = "CYBERGEAR 1" ,
	 .id=CLSID_MOT_CYBERGEAR0
};

bool CYBERGEAR_1::isCreatable() {
	return !CYBERGEAR_1::inUse;
}

const ClassIdentifier CYBERGEAR_1::getInfo() {
	return info;
}

// *****    static initializer for the CYBERGEAR_2 instance (extend CYBERGEAR_CAN) *****

bool CYBERGEAR_2::inUse = false;

ClassIdentifier CYBERGEAR_2::info = {
	 .name = "CYBERGEAR 2" ,
	 .id=CLSID_MOT_CYBERGEAR1
};

bool CYBERGEAR_2::isCreatable() {
	return !CYBERGEAR_2::inUse;
}

const ClassIdentifier CYBERGEAR_2::getInfo() {
	return info;
}

// *****    				 CYBERGEAR_CAN							 *****

CyberGearCAN::CyberGearCAN(uint8_t address) :
		CommandHandler("cybergear", CLSID_MOT_CYBERGEAR0, address), Thread("CYBERGEAR", CYBERGEAR_THREAD_MEM,
				CYBERGEAR_THREAD_PRIO) {

	if(!crcTableInitialized){ // Generate a CRC16 table the first time a cybergear instance is created
		makeCrcTable(crc16_tab, crcpoly,16);
		crcTableInitialized = true;
	}

	setAddress(address);
	restoreFlash();

	// Set up a filter to receive cybergear commands
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
	this->filterId = this->port->addCanFilter(sFilterConfig);

	//this->setCanRate(this->baudrate);
	if(port->getSpeedPreset() < 3){
		port->setSpeedPreset(3); // Minimum 250k
	}
	this->port->takePort();

	registerCommands();
	this->Start();

}

CyberGearCAN::~CyberGearCAN() {
	this->stopMotor();
	this->port->freePort();
	this->state = CyberGearState::CYBERGEAR_STATE_UNKNOWN;
}

void CyberGearCAN::setAddress(uint8_t address) {
	if (address == 0) {
		this->flashAddrs = CyberGearFlashAddrs( { ADR_CYBERGEAR1_CANID, ADR_CYBERGEAR1_DATA, ADR_CYBERGEAR1_OFFSET });
	} else if (address == 1) {
		this->flashAddrs = CyberGearFlashAddrs( { ADR_CYBERGEAR2_CANID, ADR_CYBERGEAR2_DATA, ADR_CYBERGEAR2_OFFSET });
	} else if (address == 2) {
		this->flashAddrs = CyberGearFlashAddrs( { ADR_CYBERGEAR3_CANID, ADR_CYBERGEAR3_DATA, ADR_CYBERGEAR3_OFFSET });
	}
}

void CyberGearCAN::turn(int16_t power) {
	float torque = ((float) power / (float) 0x7fff);
	lastTorque = torque;
	this->setTorque(torque);
}

void CyberGearCAN::stopMotor() {
	this->setTorque(0.0);
	activeMotor = false;
}

void CyberGearCAN::startMotor() {
	activeMotor = true;
}

Encoder* CyberGearCAN::getEncoder() {
	if (useEncoder)
		return static_cast<Encoder*>(this);
	else
		return MotorDriver::getEncoder();
}

bool CyberGearCAN::hasIntegratedEncoder() {
	return useEncoder;
}

EncoderType CyberGearCAN::getEncoderType(){
	if (useEncoder)
		return EncoderType::absolute;
	else
		return MotorDriver::getEncoder()->getEncoderType();
}

bool CyberGearCAN::motorReady() {
	return state == CyberGearState::CYBERGEAR_STATE_READY;
}

void CyberGearCAN::restoreFlash() {
	uint16_t dataFlash = 0;

	if (Flash_Read(flashAddrs.canId, &dataFlash)) {
		this->OFFB_can_Id = dataFlash & 0xFF;
		this->CYBERGEAR_can_Id = (dataFlash >> 8) & 0xFF;
	}

	if (Flash_Read(flashAddrs.data, &dataFlash)) {

		//uint8_t canspd = dataFlash & 0b111;
		//this->baudrate = canspd;

		this->useEncoder = (dataFlash >> 3) & 0x1;
	}

	if (Flash_Read(flashAddrs.offset, &dataFlash)) {
		this->posOffset = (float) ((int16_t) dataFlash / 10000.0);

		this->posOffset = this->posOffset - (int) this->posOffset; // Remove the multi-turn value
		bool moreThanHalfTurn = fabs(this->posOffset) > 0.5 ? 1 : 0;
		if (moreThanHalfTurn) {
			// if delta is neg, turn is CCW, decrement multi turn pos... else increment it
			this->posOffset += (this->posOffset > 0) ? -1.0 : 1.0;
		}
	}

}

void CyberGearCAN::saveFlash() {
	uint16_t dataFlash = 0;

	// save the OpenFFBoard Can Id in 0..7 and the CyberGear_can_id in the 8..15
	dataFlash = (this->CYBERGEAR_can_Id << 8) | this->OFFB_can_Id;
	Flash_Write(flashAddrs.canId, dataFlash);

	dataFlash = 0;
	//dataFlash |= (this->baudrate & 0b111); // set the baudrate in 0..2 bit
	dataFlash |= (this->useEncoder & 0x1) << 3;  // set the encoder use in bit 3
	Flash_Write(flashAddrs.data, dataFlash);

	saveFlashOffset();
}

void CyberGearCAN::saveFlashOffset() {

	// store the -180°/180°offset
	float storedOffset = this->posOffset - (int) this->posOffset; // Remove the multi-turn value
	bool moreThanHalfTurn = fabs(storedOffset) > 0.5 ? 1 : 0;
	if (moreThanHalfTurn) {
		// if delta is neg, turn is CCW, decrement multi turn pos... else increment it
		storedOffset += (storedOffset > 0) ? -1.0 : 1.0;
	}

	uint16_t dataFlash = ((int16_t) (storedOffset * 10000) & 0xFFFF);
	Flash_Write(flashAddrs.offset, dataFlash);

}

/**
 * Must be in encoder cpr if not just used to zero the axis
 */
void CyberGearCAN::setPos(int32_t pos) {
	// Only change encoder count internally as offset
	posOffset = lastPos - ((float) pos / (float) getCpr());

	saveFlashOffset(); // save the new offset for next restart
}

float CyberGearCAN::getPos_f() {
	if (state == CyberGearState::CYBERGEAR_STATE_READY) {
		this->askPositionEncoder();
	}
	return lastPos - posOffset;
}

int32_t CyberGearCAN::getPos() {
	return getCpr() * getPos_f();
}

uint32_t CyberGearCAN::getCpr() {
	return 0xFFFF;
}

void CyberGearCAN::registerCommands() {
	CommandHandler::registerCommands();
	registerCommand("offbcanid", CyberGearCAN_commands::offbcanid, "CAN id of OpenFFBoard Axis", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("cybergearcanid", CyberGearCAN_commands::cybergearcanid, "CAN id of CYBERGEAR", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("canspd", CyberGearCAN_commands::canspd, "CAN baud (3=250k 4=500k 5=1M)", CMDFLAG_GET | CMDFLAG_SET); // Kept for backwards compatibility. Remove in the future
	registerCommand("errorflags", CyberGearCAN_commands::errorflags, "CYBERGEAR error state", CMDFLAG_GET);
	registerCommand("cybergearstate", CyberGearCAN_commands::cybergearstate, "CYBERGEAR state", CMDFLAG_GET);
	registerCommand("voltage", CyberGearCAN_commands::voltage, "CYBERGEAR supply voltage (mV)", CMDFLAG_GET);
	registerCommand("encrate", CyberGearCAN_commands::encrate, "Encoder update rate", CMDFLAG_GET);
	registerCommand("pos", CyberGearCAN_commands::pos, "CYBERGEAR position", CMDFLAG_GET);
	registerCommand("torque", CyberGearCAN_commands::torque, "Current CYBERGEAR torque", CMDFLAG_GET);
	registerCommand("forceposread", CyberGearCAN_commands::forcePosRead, "Force a position update", CMDFLAG_GET);
	registerCommand("useencoder", CyberGearCAN_commands::useEncoder, "Enable CYBERGEAR encoder", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("offset", CyberGearCAN_commands::offset, "Get or set encoder offset", CMDFLAG_GET | CMDFLAG_SET);
}

CommandStatus CyberGearCAN::command(const ParsedCommand &cmd,
		std::vector<CommandReply> &replies) {

	switch (static_cast<CyberGearCAN_commands>(cmd.cmdId)) {

	case CyberGearCAN_commands::offbcanid:
		return handleGetSet(cmd, replies, this->OFFB_can_Id);
	case CyberGearCAN_commands::cybergearcanid:
		return handleGetSet(cmd, replies, this->CYBERGEAR_can_Id);
	case CyberGearCAN_commands::errorflags:
		if (cmd.type == CMDtype::get)
			replies.emplace_back(cybergearErrorFlag);
		break;
	case CyberGearCAN_commands::canspd:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back(port->getSpeedPreset());
		} else if (cmd.type == CMDtype::set) {
			port->setSpeedPreset(std::max<uint8_t>(3,cmd.val));
		}
		break;
	case CyberGearCAN_commands::cybergearstate:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) this->state);
		break;
	case CyberGearCAN_commands::encrate:
		if (cmd.type == CMDtype::get)
			replies.emplace_back((uint32_t) this->encRate);
		break;
	case CyberGearCAN_commands::pos:
		if (cmd.type == CMDtype::get) {
			replies.push_back(
					CommandReply(
							(int32_t) ((lastPos - posOffset) * 1000000000)));
		}
		break;
	case CyberGearCAN_commands::torque:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int32_t) (lastTorque * 10000));
		}
		break;
	case CyberGearCAN_commands::forcePosRead:
		this->askPositionEncoder();
		break;
	case CyberGearCAN_commands::useEncoder:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back(useEncoder ? 1 : 0);
		} else if (cmd.type == CMDtype::set) {
			useEncoder = cmd.val != 0;
		}
		break;
	case CyberGearCAN_commands::offset: {
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int32_t) (posOffset * 10000));
		} else if (cmd.type == CMDtype::set) {
			posOffset = (float) cmd.val / 10000.0;
			this->saveFlashOffset(); // Really save to flash?
		}
	}
		break;
	case CyberGearCAN_commands::voltage:
		if (cmd.type == CMDtype::get)
			replies.emplace_back(voltage * 1000);
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;
}

/*
 * state 	= CYBERGEAR_UNKOWN 	=> ping until the state go to PONG
 * 			= CYBERGEAR_PONG		=> call a status message
 * 			= CYBERGEAR_ERROR	=> call a status message
 *
 * 	if error => state = CYBERGEAR_ERROR
 * 	   else=> state = CYBERGEAR_RESPOND
 *
 * 	if RespondCyberGearTime > 100 => CYBERGEAR_NOT_RESPOND
 *
 */
void CyberGearCAN::Run() {

	while (true) {
		Delay(500);
		// if status is UNKNOW, ask firmware info and wait next 500ms
		if (state == CyberGearState::CYBERGEAR_STATE_UNKNOWN) {
			this->getFirmwareInfo();
			// the return of getFirmwareInfo change the state to COMPATIBLE/INCOMPATIBLE
			// skip the follow control if we don't change from unknown state
			continue;
		}

		// if cybergear is compatible, ready or in error => check the status
		if (state >= CyberGearState::CYBERGEAR_STATE_COMPATIBLE) {
			this->askGetValue();
			// the return of askGetValue put the state in READY or ERROR is cybergear respond to status
		}

		// test is a ready/error cybergear is always respond
		uint32_t period = HAL_GetTick() - lastCyberGearResponse;
		if ((state == CyberGearState::CYBERGEAR_STATE_READY
				|| state == CyberGearState::CYBERGEAR_STATE_ERROR) && period > 1000) {
			state = CyberGearState::CYBERGEAR_STATE_UNKNOWN;
		}

		// when motor is active we call cybergear each 500ms, to disable the cybergear watchdog
		// (if cybergear not received message in 1sec, it cut off motor)
		if (lastTorque != 0.0 && activeMotor
				&& state == CyberGearState::CYBERGEAR_STATE_READY) {
			this->setTorque(lastTorque);
		}

		// compute encoderRate each second
		period = HAL_GetTick() - encStartPeriod;
		if (period > 1000) {
			encRate = encCount / (period / 1000.0);
			encCount = 0;
			encStartPeriod = HAL_GetTick();
		}

	}
}

/************************************************************************************************
 *
 * 										CAN SECTION
 */

//void CyberGearCAN::setCanRate(uint8_t canRate) {
//	this->baudrate = clip<uint8_t, uint8_t>(canRate, 3, 5);
//	this->port->setSpeedPreset(baudrate);
//}

/**
 * Get the firmware cybergear information
 */
void CyberGearCAN::getFirmwareInfo() {
	uint8_t msg[3];
	msg[0] = this->OFFB_can_Id;
	msg[1] = 0x00;								// ask cybergear to process the msg
	msg[2] = (uint8_t) CyberGearCmd::COMM_FW_VERSION;// sub action is to get FW version

	this->sendMsg((uint8_t) CyberGearCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER, msg,
			sizeof(msg));
}

/**
 * Send a ping command to the cybergear to check if it's here.
 */
void CyberGearCAN::sendPing() {
	uint8_t buffer[1];
	buffer[0] = this->OFFB_can_Id;

	this->sendMsg((uint8_t) CyberGearCANMsg::CAN_PACKET_PING, buffer,
			sizeof(buffer));
}

/**
 * Send a torque command to the cybergear : value is [-1,1] range relative to max torque settings
 * in the cybergear fw.
 */
void CyberGearCAN::setTorque(float torque) {

	// Check if cybergear CAN is ok, if it is, send a message
	if (!this->motorReady() || !activeMotor)
		return;

	uint8_t buffer[4];
	int32_t send_index = 0;
	this->buffer_append_float32(buffer, torque, 1e5, &send_index);

	this->sendMsg((uint8_t) CyberGearCANMsg::CAN_PACKET_SET_CURRENT_REL, buffer,
			sizeof(buffer));

}

/*
 * Get the telemetry : Status & Value
 */
void CyberGearCAN::askGetValue() {
	uint8_t msg[8];
	msg[0] = this->OFFB_can_Id;
	msg[1] = 0x00;					// ask cybergear to process the msg
	msg[2] = (uint8_t) CyberGearCmd::COMM_GET_VALUES_SELECTIVE;// sub action is COMM_GET_VALUES_SELECTIVE : ask wanted data
	uint32_t request = ((uint32_t) 1 << 15); // bit 15 = ask cybergear status (1byte)
	request |= ((uint32_t) 1 << 8);				// bit 8  = ask voltage	(2byte)

	int32_t index = 3;
	this->buffer_append_uint32(msg, request, &index);

	this->sendMsg((uint8_t) CyberGearCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER, msg,
			sizeof(msg));
}

/**
 * Ask a position with the new system
 * Call the new short response command with a 2 messages communications
 */
void CyberGearCAN::askPositionEncoder() {
	this->sendMsg((uint8_t) CyberGearCANMsg::CAN_PACKET_POLL_ROTOR_POS, nullptr, 0);
}

/*
 * Compute the new position (laspos) value from data received by cybergear:
 * 		newPos is a 0-360° angle position
 * 		if delta with previous and new position is > 180° then we count 1 turn
 * 		at 1kz, more than 180° mean 30000rpm, not admissible in simracing
 */
void CyberGearCAN::decodeEncoderPosition(const float newPos) {
	float delta = newPos - prevPos360; // Compute the delta position

	bool moreThanHalfTurn = fabs(delta) > 180.0 ? 1 : 0;
	if (moreThanHalfTurn) {
		// if delta is neg, turn is CCW, decrement multi turn pos... else increment it
		mtPos += (delta > 0) ? -1.0 : 1.0;
	}

	lastPos = (newPos / 360.0) + mtPos; // normalize the position
	prevPos360 = newPos;

	encCount++;
}

/**
 * send the message to the CAN with an Extended Can message
 */
void CyberGearCAN::sendMsg(uint8_t cmd, uint8_t *buffer, uint8_t len) {
	CAN_tx_msg msg;
	memcpy(&msg.data, buffer, len);
	msg.header.RTR = CAN_RTR_DATA;
	msg.header.DLC = len;
	msg.header.IDE = CAN_ID_EXT;
	msg.header.ExtId = this->CYBERGEAR_can_Id | (cmd << 8);
	port->sendMessage(msg);
}

void CyberGearCAN::decode_buffer(uint8_t *buffer, uint8_t len) {

	if (!len) {
		return;
	}

	CyberGearCmd command = (CyberGearCmd) (buffer[0] & 0xFF);
	buffer++;
	len--;

	switch (command) {

	case CyberGearCmd::COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t fw_major = buffer[ind++];
		uint8_t fw_min = buffer[ind++];

		std::string test((char*) buffer + ind);
		ind += test.length() + 1;

		uint8_t uuid[12] = { 0 };
		memcpy(buffer + ind, uuid, 12);
		ind += 12;

		//bool isPaired = buffer[ind++];
		uint8_t isTestFw = buffer[ind++];

		//uint8_t hwType = buffer[ind++];			// Comment because unused
		//uint8_t customConfig = buffer[ind++];		// Comment because unused

		// bool hasPhaseFilters = buffer[ind++];	// Comment because unused

		bool compatible = false;
		if (!isTestFw)
			compatible = ((fw_major << 8) | fw_min) >= (FW_MIN_RELEASE >> 8);
		else
			compatible = ((fw_major << 16) | (fw_min << 8) | isTestFw)
					>= FW_MIN_RELEASE;

		if (compatible) {
			this->state = CyberGearState::CYBERGEAR_STATE_COMPATIBLE;
		} else {
			this->state = CyberGearState::CYBERGEAR_STATE_INCOMPATIBLE;
		}

		break;
	}

	case CyberGearCmd::COMM_GET_VALUES_SELECTIVE: {
		int32_t ind = 0;
		uint32_t mask = this->buffer_get_uint32(buffer, &ind);

		if (mask & ((uint32_t) 1 << 8)) {
			voltage = this->buffer_get_float16(buffer, 1e1, &ind);
		}
		if (mask & ((uint32_t) 1 << 15)) {
			cybergearErrorFlag = buffer[ind++];

			// if the cybergear respond but with an error, we put the cybergear in error flag
			if (cybergearErrorFlag) {
				state = CyberGearState::CYBERGEAR_STATE_ERROR;
			} else {
				state = CyberGearState::CYBERGEAR_STATE_READY;
			}

		}
		if (mask & ((uint32_t) 1 << 16)) {
			float pos = this->buffer_get_float32(buffer, 1e6, &ind);
			this->decodeEncoderPosition(pos);
		}
		break;

	}

	default:
		break;

	}

}

/*
 *	Process the received message on the CAN bus.
 *	Message have several struct, depends on message type, check the comm_can.c and
 *	command.c in the cybergear source project to identify the struct format.
 *
 *	Quick tips :
 *	Msg struct for CAN_PACKET_PROCESS_RX_BUFFER and CAN_PACKET_PROCESS_SHORT_BUFFER :
 *	uint8_t[0] => can_emitter_id (cybergear id)
 *	uint8_t[1] => send bool
 *	uint8_t[2] => COMM_PACKET_ID command
 *	uint8_t[3] -> uint8_t[7] => DATA (5 bytes)
 *
 *	Msg struct for CAN_PACKET_POLL_ROTOR_POS
 *	uint8_t[0]..uint8_t[3] : uint32 f_pos / 10000
 *
 */
void CyberGearCAN::canRxPendCallback(CAN_HandleTypeDef *hcan, uint8_t *rxBuf,
		CAN_RxHeaderTypeDef *rxHeader, uint32_t fifo) {

	// we record the last time respond to check if cybergear is OK
	lastCyberGearResponse = HAL_GetTick();

	// Check if message is for the openFFBoard
	uint16_t destCanID = rxHeader->ExtId & 0xFF;

	// Extract the command encoded in the ExtId
	CyberGearCANMsg cmd = (CyberGearCANMsg) (rxHeader->ExtId >> 8);

	bool messageIsForThisCyberGearAxis = destCanID == this->OFFB_can_Id;
	messageIsForThisCyberGearAxis |= (cmd == CyberGearCANMsg::CAN_PACKET_POLL_ROTOR_POS) &&	// if it's a encoder position message
				(this->CYBERGEAR_can_Id == 0xFF); // and the cybergearCanId is not the default one
	messageIsForThisCyberGearAxis |= (cmd == CyberGearCANMsg::CAN_PACKET_POLL_ROTOR_POS)
			&&	// if it's a encoder position message
			(this->CYBERGEAR_can_Id != 0xFF) && // and the cybergearCanId is not the default one
			(destCanID == this->CYBERGEAR_can_Id); // we check that emiterId is the cybergearId for this axis

	if (!messageIsForThisCyberGearAxis)
		return;

	// Process the CAN message received
	switch (cmd) {

	case CyberGearCANMsg::CAN_PACKET_FILL_RX_BUFFER: {
		memcpy(buffer_rx + rxBuf[0], rxBuf + 1, rxHeader->DLC - 1);
		break;
	}

	case CyberGearCANMsg::CAN_PACKET_FILL_RX_BUFFER_LONG: {
		uint32_t rxbuf_ind = (unsigned int) rxBuf[0] << 8;
		rxbuf_ind |= rxBuf[1];
		if (rxbuf_ind < BUFFER_RX_SIZE) {
			memcpy(buffer_rx + rxbuf_ind, rxBuf + 2, rxHeader->DLC - 2);
		}
		break;
	}

	case CyberGearCANMsg::CAN_PACKET_PROCESS_RX_BUFFER: {

		// remove 2 first byte to exclude "can emitter id" and the "send bool" flag from buffer
		int32_t index = 2;

		uint32_t rxbuf_length = (uint32_t) rxBuf[index++] << 8;
		rxbuf_length |= (uint32_t) rxBuf[index++];

		if (rxbuf_length > BUFFER_RX_SIZE) {
			break;
		}

		uint8_t crc_high = rxBuf[index++];
		uint8_t crc_low = rxBuf[index++];

		if(calculateCrc16_8(crc16_tab,buffer_rx,rxbuf_length)
		//if (crc16(buffer_rx, rxbuf_length)
				== ((uint8_t) crc_high << 8 | (uint8_t) crc_low)) {

			this->decode_buffer(buffer_rx, rxbuf_length);

		}

		break;
	}
	case CyberGearCANMsg::CAN_PACKET_PROCESS_SHORT_BUFFER: {

		// remove 2 first byte to exclude "can emitter id" and the "send bool" flag from buffer
		this->decode_buffer(rxBuf + 2, rxHeader->DLC - 2);

		break;
	}

	case CyberGearCANMsg::CAN_PACKET_PONG:
		state = CyberGearState::CYBERGEAR_STATE_PONG;
		break;

	case CyberGearCANMsg::CAN_PACKET_POLL_ROTOR_POS: { // decode encoder data
		int32_t index = 0;
		float pos = this->buffer_get_int32(rxBuf, &index) / 100000.0; // extract the 0-360 float position
		this->decodeEncoderPosition(pos);
		break;
	}

	default:
		break;
	}

}

void CyberGearCAN::buffer_append_float32(uint8_t *buffer, float number, float scale,
		int32_t *index) {
	this->buffer_append_int32(buffer, (int32_t) (number * scale), index);
}

void CyberGearCAN::buffer_append_int32(uint8_t *buffer, int32_t number,
		int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void CyberGearCAN::buffer_append_uint32(uint8_t *buffer, uint32_t number,
		int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

uint32_t CyberGearCAN::buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
	uint32_t res = ((uint32_t) buffer[*index]) << 24
			| ((uint32_t) buffer[*index + 1]) << 16
			| ((uint32_t) buffer[*index + 2]) << 8
			| ((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

int32_t CyberGearCAN::buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res = ((uint32_t) buffer[*index]) << 24
			| ((uint32_t) buffer[*index + 1]) << 16
			| ((uint32_t) buffer[*index + 2]) << 8
			| ((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

int16_t CyberGearCAN::buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res = ((uint16_t) buffer[*index]) << 8
			| ((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

float CyberGearCAN::buffer_get_float32(const uint8_t *buffer, float scale,
		int32_t *index) {
	return (float) buffer_get_int32(buffer, index) / scale;
}

float CyberGearCAN::buffer_get_float16(const uint8_t *buffer, float scale,
		int32_t *index) {
	return (float) buffer_get_int16(buffer, index) / scale;
}

#endif
