#ifndef UART_APP_H
#define UART_APP_H

#define MSG_PAKAGE_HEAD 0xa5
#define MSG_PAKAGE_TAIL 0x5a

#define CONTROLLER_PACKAGE_BEGINNING 0xff
#define GET_CONTROLLER_PACKAGE_MSG 0xfe

#define DEAL_DATA_AS_SPEED_IN_LOCAL_COORDINATE_SYSTEM 0x01
#define DEAL_DATA_AS_SPEED_IN_WORLD_COORDINATE_SYSTEM 0x02
#define DEAL_DATA_AS_DISPLACEMENT 0x03
#define DATA_REQUEST 0x04
#define MISTAKE_FROM_CRC 0x05

#define CIRCUIT_SEND_ENABLE 0x06
#define CIRCUIT_SEND_DISABLE 0x07

#define CYBER_GEAR_STEP_ONE 0x08
#define CYBER_GEAR_STEP_TWO 0x09

#define MOVEMENT_COMPELETE 0x0a

#define DELTA_ENABLE 0x0b
#define DELTA_DISABLE 0x0c

#define DELTA_RISE 0x0d
#define DELTA_FALL 0x0e


typedef struct{
	
	float roll;
	float pitch;
	float yaw;
	
	float filtered_roll;
	float filtered_pitch;
	float filtered_yaw;
	
	float roll_omega;
	float pitch_omega;
	float yaw_omega;
	
	float filtered_roll_omega;
	float filtered_pitch_omega;
	float filtered_yaw_omega;
	
	float x_acc;
	float y_acc;
	float z_acc;
	
	float filtered_x_acc;
	float filtered_y_acc;
	float filtered_z_acc;
	
}angle_measure;


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern uint8_t step;

extern uint8_t run_mode;
extern uint8_t delta_mode;
extern uint8_t msg_mode;
extern uint8_t controller;

extern uint8_t buffer[4];
extern uint8_t angle_buffer[10];
extern uint8_t rx_buffs_ykq[12];
extern uint8_t tx_buffer[6];
extern float receive_data[6];

extern uint8_t crc_code;
extern uint8_t declearation;
extern uint8_t count;

extern uint8_t cyber_gear_knock_flag;
extern angle_measure my_angle_measure;

extern uint8_t sum_ykq;
extern int step_ykq;
extern int cnt_ykq;

extern uint8_t change_mode_flag;

// void send_command_package(uint8_t command_type);
// void not_got_it_the_point_yet(void);
// void got_it_the_point_already(void);
class UARTApp {
public:
  
    // 成员函数
    
    void send_command_package(uint8_t command);
    void not_got_it_the_point_yet();
    void got_it_the_point_already();
    void send_angle_velocity();

private:
    // 私有辅助函数
    void waiting_for_pack_head();
    void judgement_of_declearation();
    void receiving_datas();
    void crc8_inspecting();
    void waiting_for_package_tail();
    void commander_package_analysis(uint8_t command);
    void gyro_position_filter(angle_measure *angle);
    void gyro_omega_filter(angle_measure *angle);
    void gyro_acc_filter(angle_measure *angle);
    void crc(uint8_t data);
};

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

#endif 
