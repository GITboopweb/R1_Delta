#ifndef FUNCTION_FOR_UART_H
#define FUNCTION_FOR_UART_H

#ifdef __cplusplus
extern "C" {
#endif
#include "include.h"

extern float ykq_xyw[3];
extern uint8_t send_flag;
class UARTFunction {
    public:
   
        void crc(uint8_t bye);
        float get_float_num(uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4);
        void waiting_for_pack_head(void);
        void judgement_of_declearation(void);
        void receiving_datas(void);
        void crc8_inspecting(void);
        void waiting_for_package_tail(void);
        void controller_data_analysis(uint8_t* controller_datas);
        void commander_package_analysis(uint8_t commander);
        void gyro_position_filter(angle_measure*this);
        void gyro_omega_filter(angle_measure*this);
        void gyro_acc_filter(angle_measure*this);
        void toresult();
        float input_map(float input,float max);
};
#ifdef __cplusplus
}
#endif
#endif


