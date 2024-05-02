/*
 * CyberGearCAN.h
 *
 *  Created on: Apr 24, 2024
 *      Author: BlackBird
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

#ifdef CYBERGEAR
#define CYBERGEAR_THREAD_MEM 256
#define CYBERGEAR_THREAD_PRIO 25 // Must be higher than main thread


#define CMD_POSITION                    1
#define CMD_RESPONSE                    2
#define CMD_ENABLE                      3
#define CMD_RESET                       4
#define CMD_SET_MECH_POSITION_TO_ZERO   6
#define CMD_CHANGE_CAN_ID               7
#define CMD_RAM_READ                    17
#define CMD_RAM_WRITE                   18
#define CMD_GET_MOTOR_FAIL              21


#define ADDR_RUN_MODE              0x7005
#define ADDR_IQ_REF                0x7006
#define ADDR_SPEED_REF             0x700A
#define ADDR_LIMIT_TORQUE          0x700B
#define ADDR_CURRENT_KP            0x7010
#define ADDR_CURRENT_KI            0x7011
#define ADDR_CURRENT_FILTER_GAIN   0x7014
#define ADDR_LOC_REF               0x7016
#define ADDR_LIMIT_SPEED           0x7017
#define ADDR_LIMIT_CURRENT         0x7018
#define ADDR_MECH_POS              0x7019
#define ADDR_IQF                   0x701A
#define ADDR_MECH_VEL              0x701B
#define ADDR_VBUS                  0x701C
#define ADDR_ROTATION              0x701D
#define ADDR_LOC_KP                0x701E
#define ADDR_SPD_KP                0x701F
#define ADDR_SPD_KI                0x7020

/* in CyberGearControlMode
#define MODE_MOTION                0x00
#define MODE_POSITION              0x01
#define MODE_SPEED                 0x02
#define MODE_CURRENT               0x03
*/


enum class CyberGearState : uint32_t {AXIS_STATE_UNDEFINED=0,AXIS_STATE_IDLE=1,AXIS_STATE_STARTUP_SEQUENCE=2,AXIS_STATE_FULL_CALIBRATION_SEQUENCE=3,AXIS_STATE_MOTOR_CALIBRATION=4,AXIS_STATE_ENCODER_INDEX_SEARCH=6,AXIS_STATE_ENCODER_OFFSET_CALIBRATION=7,AXIS_STATE_CLOSED_LOOP_CONTROL=8,AXIS_STATE_LOCKIN_SPIN=9,AXIS_STATE_ENCODER_DIR_FIND=10,AXIS_STATE_HOMING=11,AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION=12,AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION=13};
// enum class CyberGearControlMode : uint32_t {CONTROL_MODE_VOLTAGE_CONTROL = 0,CONTROL_MODE_TORQUE_CONTROL = 1,CONTROL_MODE_VELOCITY_CONTROL = 2,CONTROL_MODE_POSITION_CONTROL = 3};
enum class CyberGearInputMode : uint32_t {INPUT_MODE_INACTIVE = 0,INPUT_MODE_PASSTHROUGH = 1,INPUT_MODE_VEL_RAMP = 2,INPUT_MODE_POS_FILTER = 3,INPUT_MODE_MIX_CHANNELS = 4,INPUT_MODE_TRAP_TRAJ = 5,INPUT_MODE_TORQUE_RAMP =6,INPUT_MODE_MIRROR =7};
enum class CyberGearLocalState : uint32_t {IDLE,WAIT_READY,WAIT_CALIBRATION_DONE,WAIT_CALIBRATION,RUNNING,START_RUNNING};

enum class CyberGearEncoderFlags : uint32_t {ERROR_NONE = 0,ERROR_UNSTABLE_GAIN = 0x01,ERROR_CPR_POLEPAIRS_MISMATCH = 0x02, ERROR_NO_RESPONSE = 0x04, ERROR_UNSUPPORTED_ENCODER_MODE = 0x08, ERROR_ILLEGAL_HALL_STATE = 0x10, ERROR_INDEX_NOT_FOUND_YET = 0x20, ERROR_ABS_SPI_TIMEOUT = 0x40, ERROR_ABS_SPI_COM_FAIL = 0x80, ERROR_ABS_SPI_NOT_READY = 0x100, ERROR_HALL_NOT_CALIBRATED_YET = 0x200};
enum class CyberGearAxisError : uint32_t {AXIS_ERROR_NONE = 0x00000000,AXIS_ERROR_INVALID_STATE  = 0x00000001, AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000800,AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x00001000, AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x00002000,AXIS_ERROR_ESTOP_REQUESTED = 0x00004000,AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x00020000,AXIS_ERROR_OVER_TEMP = 0x00040000,AXIS_ERROR_UNKNOWN_POSITION = 0x00080000};

enum class CyberGearCAN_commands : uint32_t{
	canid,canspd,errors,state,maxtorque,vbus,anticogging,connected
};
/*
enum class CyberGearCAN_commands : uint32_t{CMD_POSITION = 1,
											CMD_RESPONSE = 2,
											CMD_ENABLE = 3,
											CMD_RESET = 4,
											CMD_SET_MECH_POSITION_TO_ZERO = 6,
											CMD_CHANGE_CAN_ID = 7,
											CMD_RAM_READ = 17,
											CMD_RAM_WRITE = 18,
											CMD_GET_MOTOR_FAIL = 21
};*/


enum class CyberGearControlMode : uint8_t {MODE_MOTION = 0,MODE_POSITION = 1,MODE_SPEED = 2,MODE_CURRENT = 3};


#define P_MIN     -12.5f
#define P_MAX      12.5f
#define V_MIN     -30.0f
#define V_MAX      30.0f
#define KP_MIN      0.0f
#define KP_MAX    500.0f
#define KD_MIN      0.0f
#define KD_MAX      5.0f
#define T_MIN     -12.0f
#define T_MAX      12.0f
#define IQ_MIN     -27.0f
#define IQ_MAX      27.0f
#define CURRENT_FILTER_GAIN_MIN 0.0f
#define CURRENT_FILTER_GAIN_MAX 1.0f

#define IQ_REF_MAX 23.0f
#define IQ_REF_MIN -23.0f
#define SPD_REF_MAX 30.0f
#define SPD_REF_MIN -30.0f
#define LIMIT_TORQUE_MAX 0.0f
#define LIMIT_TORQUE_MIN 12.0f
#define CUR_KP_MAX  200.0f
#define CUR_KP_MIN  0.0f
#define CUR_KI_MAX  200.0f
#define CUR_KI_MIN  0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_SPD_MIN 0.0f


#define DEFAULT_CURRENT_KP           0.125f
#define DEFAULT_CURRENT_KI           0.0158f
#define DEFAULT_CURRENT_FINTER_GAIN  0.1f
#define DEFAULT_POSITION_KP          30.0f
#define DEFAULT_VELOCITY_KP          2.0f
#define DEFAULT_VELOCITY_KI          0.002f
#define DEFAULT_VELOCITY_LIMIT       2.0f
#define DEFAULT_CURRENT_LIMIT        27.0f
#define DEFAULT_TORQUE_LIMIT         12.0f

#define RET_CYBERGEAR_OK              0x00
#define RET_CYBERGEAR_MSG_NOT_AVAIL   0x01
#define RET_CYBERGEAR_INVALID_CAN_ID  0x02
#define RET_CYBERGEAR_INVALID_PACKET  0x03

#define CW      1
#define CCW     -1




/**
 * @brief MotorStatus class
 */
struct MotorStatus
{
  unsigned long stamp_usec;     //< timestamp
  uint8_t motor_id;             //!< motor id
  float position;               //!< encoder position (-4pi to 4pi)
  float velocity;               //!< motor velocity (-30rad/s to 30rad/s)
  float effort;                 //!< motor effort (-12Nm - 12Nm)
  float temperature;            //!< temperature
  uint16_t raw_position;        //!< raw position (for sync data)
  uint16_t raw_velocity;        //!< raw velocity (for sync data)
  uint16_t raw_effort;          //!< raw effort (for sync data)
  uint16_t raw_temperature;     //!< raw temperature (for sync data)
};


struct MotorFault
{
  bool encoder_not_calibrated;
  bool over_current_phase_a;
  bool over_current_phase_b;
  bool over_voltage;
  bool under_voltage;
  bool driver_chip;
  bool motor_over_tempareture;
};


struct MotorParameter
{
  unsigned long stamp_usec;
  uint16_t run_mode;
  float iq_ref;
  float spd_ref;
  float limit_torque;
  float cur_kp;
  float cur_ki;
  float cur_filt_gain;
  float loc_ref;
  float limit_spd;
  float limit_cur;
  float mech_pos;
  float iqf;
  float mech_vel;
  float vbus;
  int16_t rotation;
  float loc_kp;
  float spd_kp;
  float spd_ki;
};


class CyberGearCAN : public MotorDriver,public PersistentStorage, public Encoder, public CanHandler, public CommandHandler, cpp_freertos::Thread{
public:
	CyberGearCAN(uint8_t id);
	virtual ~CyberGearCAN();


	//static ClassIdentifier info;
	const ClassIdentifier getInfo() = 0;
	//static bool isCreatable();

	void turn(int16_t power) override;
	void stopMotor() override;
	void startMotor() override;
	Encoder* getEncoder() override;
	bool hasIntegratedEncoder() {return true;}

	#define ADDR_IQ_REF                0x7006
	#define CMD_RAM_WRITE                   18
	void send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data);
	void write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max);




	template<class T>
	void sendMsg(uint8_t cmd,T value){
		CAN_tx_msg msg;
		memcpy(&msg.data,&value,sizeof(T));
		msg.header.RTR = CAN_RTR_DATA;
		msg.header.DLC = sizeof(T);
		msg.header.StdId = cmd | (nodeId << 5);
		if(!port->sendMessage(msg)){
			// Nothing
		}
	}

	void sendMsg(uint8_t cmd,float value);
	void requestMsg(uint16_t  cmd);
	bool motorReady() override;
	void startAnticogging();

	void Run();

	void setTorque(float torque);

	void canRxPendCallback(CAN_HandleTypeDef *hcan,uint8_t* rxBuf,CAN_RxHeaderTypeDef* rxHeader,uint32_t fifo) override;
	void canErrorCallback(CAN_HandleTypeDef *hcan);


	float getPos_f() override;
	uint32_t getCpr() override;
	int32_t getPos() override;
	void setPos(int32_t pos) override;
	EncoderType getEncoderType() override;

	void setMode(CyberGearControlMode controlMode ) ; //,CyberGearInputMode inputMode);
	void setState(CyberGearState state);

	void readyCb();

	//void setCanRate(uint8_t canRate);

	void saveFlash() override; 		// Write to flash here
	void restoreFlash() override;	// Load from flash

	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies) override;
	void registerCommands();
	std::string getHelpstring(){return "CyberGear motor driver with CAN";};

	void setCanFilter();

private:
	CANPort* port = &canport;
	float lastPos = 0;
	float lastSpeed = 0;
	float posOffset = 0;
	float lastVoltage = 0;
	uint32_t lastVoltageUpdate = 0;
	uint32_t lastCanMessage = 0;

	uint32_t lastPosTime = 0;
	bool posWaiting = false;

	int8_t nodeId = 0; // 6 bits can ID
	int8_t motorId = 0;

	volatile CyberGearState cybergearCurrentState = CyberGearState::AXIS_STATE_UNDEFINED;
	volatile uint32_t errors = 0; // Multiple flag bits can be set
	// Not yet used by cybergear (0.5.4):
	volatile uint32_t cybergearMotorFlags = 0;
	volatile uint32_t cybergearEncoderFlags = 0;
	volatile uint32_t cybergearControllerFlags = 0;

	float maxTorque = 1.0; // range how to scale the torque output
	volatile bool waitReady = true;

	bool active = false;

	int32_t filterId = 0;
	volatile CyberGearLocalState state = CyberGearLocalState::IDLE;
	bool connected = false;

	uint8_t baudrate = CANSPEEDPRESET_500; // 250000, 500000, 1M
};

/**
 * Instance 1 of CyberGear
 * Use for M0 output
 */
class CyberGearCAN1 : public CyberGearCAN{
public:
	CyberGearCAN1() : CyberGearCAN{0} {inUse = true;}
	const ClassIdentifier getInfo();
	~CyberGearCAN1(){inUse = false;}
	static bool isCreatable();
	static ClassIdentifier info;
	static bool inUse;
};

/**
 * Instance 2 of CyberGear
 * Use for M1 output
 */
class CyberGearCAN2 : public CyberGearCAN{
public:
	CyberGearCAN2() : CyberGearCAN{1} {inUse = true;}
	const ClassIdentifier getInfo();
	~CyberGearCAN2(){inUse = false;}
	static bool isCreatable();
	static ClassIdentifier info;
	static bool inUse;
};

/*
uint8_t master_can_id_;       //!< master can id
uint8_t target_can_id_;       //!< target can id
uint8_t run_mode_;            //!< run mode
unsigned long send_count_;    //!< send count
uint8_t receive_buffer_[64];  //!< receive buffer
MotorStatus motor_status_;    //!< current motor status
MotorParameter motor_param_;  //!< motor parameter
*/


#endif /* USEREXTENSIONS_SRC_CYBERGEARCAN_H_ */
#endif
