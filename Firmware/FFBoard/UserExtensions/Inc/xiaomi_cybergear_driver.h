// #include "driver/twai.h"
#include "CAN.h"
#include "xiaomi_cybergear_defs.h"

/*
    typedef struct
    {
      uint32_t StdId;    //!< Specifies the standard identifier
                         //     This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF.

      uint32_t ExtId;    //!< Specifies the extended identifier.
                         //     This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF.

      uint32_t IDE;      //!< Specifies the type of identifier for the message that will be transmitted.
                         //     This parameter can be a value of @ref CAN_identifier_type

      uint32_t RTR;      //!< Specifies the type of frame for the message that will be transmitted.
                         //     This parameter can be a value of @ref CAN_remote_transmission_request

      uint32_t DLC;      //!< Specifies the length of the frame that will be transmitted.
                          //    This parameter must be a number between Min_Data = 0 and Max_Data = 8.

      uint32_t Timestamp; //!< Specifies the timestamp counter value captured on start of frame reception.
                          //    @note: Time Triggered Communication Mode must be enabled.
                           //   This parameter must be a number between Min_Data = 0 and Max_Data = 0xFFFF.

      uint32_t FilterMatchIndex; //!< Specifies the index of matching acceptance filter element.
                            //  This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF.

    } CAN_RxHeaderTypeDef;
    typedef struct
    {
      uint32_t StdId;    //!< Specifies the standard identifier.
                          //    This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF.

      uint32_t ExtId;    //!< Specifies the extended identifier.
                          //    This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF.

      uint32_t IDE;      //!< Specifies the type of identifier for the message that will be transmitted.
                          //    This parameter can be a value of @ref CAN_identifier_type

      uint32_t RTR;      //!< Specifies the type of frame for the message that will be transmitted.
                         //     This parameter can be a value of @ref CAN_remote_transmission_request

      uint32_t DLC;      //!< Specifies the length of the frame that will be transmitted.
                         //     This parameter must be a number between Min_Data = 0 and Max_Data = 8.

      FunctionalState TransmitGlobalTime; //!< Specifies whether the timestamp counter value captured on start
                            //  of frame transmission, is sent in DATA6 and DATA7 replacing pData[6] and pData[7].
                           //   @note: Time Triggered Communication Mode must be enabled.
                            //  @note: DLC must be programmed as 8 bytes, in order these 2 bytes are sent.
                           //   This parameter can be set to ENABLE or DISABLE.

    }
    ;*/


struct XiaomiCyberGearStatus {
    float position;
    float speed;
    float torque;
    uint16_t temperature;
};

struct XiaomiCyberGearMotionCommand {
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
};

class XiaomiCyberGearDriver {
    public:
        XiaomiCyberGearDriver();
        XiaomiCyberGearDriver(uint8_t master_can_id, uint8_t cybergear_can_id);
        virtual ~XiaomiCyberGearDriver();

        /**
         * @retval -1 Error
         * @retval 0 OK
         */
        // int init_twai(uint8_t rx_pin, uint8_t tx_pin, bool serial_debug=false);
        void init_motor(uint8_t mode);
        void enable_motor();
        void stop_motor();
        void set_run_mode(uint8_t mode);

        void set_limit_speed(float speed);
        void set_limit_current(float current);
        void set_limit_torque(float torque);

        // MODE MOTION
        void send_motion_control(XiaomiCyberGearMotionCommand cmd);

        // MODE_CURRENT
        void set_current_kp(float kp);
        void set_current_ki(float ki);
        void set_current_filter_gain(float gain);
        void set_current_ref(float current);

        // MODE_POSITION
        void set_position_kp(float kp);
        void set_position_ref(float position);

        // MODE_SPEED
        void set_speed_kp(float kp);
        void set_speed_ki(float ki);
        void set_speed_ref(float speed);

        uint8_t get_run_mode() const;
        uint8_t get_motor_can_id() const;
        void set_motor_can_id(uint8_t can_id);

        void request_status();
        // void process_message(twai_message_t& message);
        void process_message(CAN_rx_msg& message);
        XiaomiCyberGearStatus get_status() const;
        
    private:
        uint16_t _float_to_uint(float x, float x_min, float x_max, int bits);
        float _uint_to_float(uint16_t x, float x_min, float x_max);
        void _send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data);
        void _send_can_float_package(uint8_t can_id, uint16_t addr, float value, float min, float max);

        uint8_t _cybergear_can_id;
        uint8_t _master_can_id;
        uint8_t _run_mode;
        bool _use_serial_debug;
        XiaomiCyberGearStatus _status;
};
