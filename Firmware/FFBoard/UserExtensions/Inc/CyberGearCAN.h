/*
 * CyberGearCAN.h
 *
 *  Created on: Aug 08, 2021
 *      Author: Vincent Manoukian (manoukianvg@gmail.com)
 */

#ifndef USEREXTENSIONS_SRC_CYBERGEARCAN_H_
#define USEREXTENSIONS_SRC_CYBERGEARCAN_H_
#include "MotorDriver.h"
#include "cpp_target_config.h"
#include "CAN.h"
#include "Encoder.h"
#include "thread.hpp"
#include "CanHandler.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"
#include <math.h>

#ifdef CYBERGEAR
#define CYBERGEAR_THREAD_MEM 512
#define CYBERGEAR_THREAD_PRIO 25 // Must be higher than main thread
#define BUFFER_RX_SIZE	64

#define FW_MIN_RELEASE ((5 << 16) | (3 << 8) | 51)

enum class CyberGearState : uint32_t {
	CYBERGEAR_STATE_UNKNOWN = 0,
	CYBERGEAR_STATE_INCOMPATIBLE = 1,
	CYBERGEAR_STATE_PONG = 2,
	CYBERGEAR_STATE_COMPATIBLE = 3,
	CYBERGEAR_STATE_READY = 4,
	CYBERGEAR_STATE_ERROR = 5

};

enum class CyberGearCANMsg : uint8_t {
	CAN_PACKET_FILL_RX_BUFFER = 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
	CAN_PACKET_PROCESS_RX_BUFFER = 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
	CAN_PACKET_SET_CURRENT_REL = 10,
	CAN_PACKET_PING = 17,
	CAN_PACKET_PONG = 18,
	CAN_PACKET_POLL_ROTOR_POS = 56
};

enum class CyberGearCmd : uint8_t {
	COMM_FW_VERSION = 0,
	COMM_ROTOR_POSITION = 22,
	COMM_GET_VALUES_SELECTIVE = 50
};
enum class CyberGearEncoderMode : uint8_t {
	ENCODER_OFF = 0, ENCODER_ON = 3
};

enum class CyberGearCAN_commands : uint32_t {
	offbcanid,cybergearcanid,canspd,errorflags,cybergearstate,voltage,encrate,pos,torque,forcePosRead,useEncoder,offset
};

struct CyberGearFlashAddrs{
	uint16_t canId = 	ADR_CYBERGEAR1_CANID;
	uint16_t data = 	ADR_CYBERGEAR1_DATA;
	uint16_t offset = 	ADR_CYBERGEAR1_OFFSET;
};

class CyberGearCAN: public MotorDriver,
		public PersistentStorage,
		public Encoder,
		public CanHandler,
		public CommandHandler,
		cpp_freertos::Thread {
public:
	CyberGearCAN(uint8_t address);
	virtual ~CyberGearCAN();
	void setAddress(uint8_t address);

	// MotorDriver impl
	void turn(int16_t power) override;
	void stopMotor() override;
	void startMotor() override;
	Encoder* getEncoder() override;
	bool hasIntegratedEncoder() override;
	EncoderType getEncoderType() override;
	bool motorReady() override;

	// PersistentStorage impl
	void saveFlash() override;
	void restoreFlash() override;

	// Encoder impl
	float getPos_f() override;
	uint32_t getCpr() override;
	int32_t getPos() override;
	void setPos(int32_t pos) override;

	// CanHandler impl
	void canRxPendCallback(CAN_HandleTypeDef *hcan, uint8_t *rxBuf,
			CAN_RxHeaderTypeDef *rxHeader, uint32_t fifo) override;

	// CommandHandler impl
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	void registerCommands(); // Function reserved to register commands in the command list.
	virtual std::string getHelpstring() {
		return "CYBERGEAR CAN interface";
	};

	// Thread impl
	void Run();

private:

	// CyberGear interface and motor state
	CyberGearFlashAddrs flashAddrs;
	volatile CyberGearState state = CyberGearState::CYBERGEAR_STATE_UNKNOWN;
	volatile uint8_t cybergearErrorFlag;
	bool activeMotor = false;
	float lastTorque;
	float voltage;

	// Encoder section

	bool useEncoder = true;
	float lastPos = 0;				// last motor position known in "turn unit"
	float mtPos = 0;				// number of turn known
	float prevPos360 = 0;			// previous position in [0-360] position
	float posOffset = 0;			// used to center the wheel
	volatile uint32_t encCount = 0;			// count incoming message
	volatile uint32_t encStartPeriod = 0;	// start time of period to compute encoder Rate
	volatile float encRate = 0;				// encoder rate to test if can speed setting is OK
	volatile uint32_t lastCyberGearResponse = 0;	// record last time when cybergear respond

	void saveFlashOffset();

	// CAN section

	CANPort *port = &canport;
	int32_t filterId = 0;
	//uint8_t baudrate = CANSPEEDPRESET_500; 	// 250000, 500000, 1M
	uint8_t OFFB_can_Id = 0x40; 			// Default OpenFFBoard CAN ID
	uint8_t CYBERGEAR_can_Id = 0xFF;				// Default CYBERGEAR CAN id
	uint8_t buffer_rx[BUFFER_RX_SIZE];		// Used to store multi-frame can message

	//void setCanRate(uint8_t canRate);
	void getFirmwareInfo();
	bool isFirmwareCompatible();
	void sendPing();
	void setTorque(float torque);
	void decodeEncoderPosition(float newPos);
	void askGetValue();
	void askPositionEncoder();
	void sendMsg(uint8_t cmd, uint8_t *buffer, uint8_t len);
	void decode_buffer(uint8_t *buffer, uint8_t len);

	void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);
	void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
	void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
	uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
	int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
	int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
	float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
	float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);

	static std::array<uint16_t,256> crc16_tab;
	static const uint16_t crcpoly = 0x1021;
	static bool crcTableInitialized;

};

// *** Creation of the first concrete class (CYBERGEAR_1) used by OpenFFBoard of CyberGearCan interface ***
class CYBERGEAR_1 : public CyberGearCAN {
public:
	CYBERGEAR_1() : CyberGearCAN{0} { inUse = true; }
	~CYBERGEAR_1() { inUse = false; }

	static ClassIdentifier info;
	static bool inUse;

	static bool isCreatable();
	const ClassIdentifier getInfo();
};

// *** Creation of the second concrete class (CYBERGEAR_2) used by OpenFFBoard of CyberGearCan interface ***
class CYBERGEAR_2 : public CyberGearCAN {
public:
	CYBERGEAR_2() : CyberGearCAN{1} { inUse = true; }
	~CYBERGEAR_2() { inUse = false; }

	static ClassIdentifier info;
	static bool inUse;

	static bool isCreatable();
	const ClassIdentifier getInfo();
};

#endif /*             CYBERGEAR               */
#endif /*  USEREXTENSIONS_SRC_CYBERGEARCAN_H_ */