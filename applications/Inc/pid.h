#ifndef __PID_H_
#define __PID_H_

#ifdef __cplusplus
extern "C" 
{
#endif
#include <stdint.h>

typedef enum
{
    PID_POSITION = 0,
    PID_DELTA
}PID_MODE;
typedef struct 
{
    uint8_t mode;
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float max_out;
    float max_iout;
    float max_fout;
    float set;
    float fdb;
    float out;
    float Pout;
    float Iout;
    float Dout;
    float Fout;
    float Dbuf[3];
    float error[3];
}pid_type_def;

class PID_Controller
{
public:
    void All_Device_Init();

    class CORE
    {
    public:
        void PID_init(pid_type_def *pid, uint8_t mode, const float PID[4], float max_out, float max_iout, float max_fout);
        float PID_calc(pid_type_def *pid, float ref, float set);
        float vel_PID_calc(pid_type_def *pid, float ff, float ref, float set);
        void PID_clear(pid_type_def *pid);
        float pid_calc_MI(MI_motor_PID*pid ,float angle,float set_angle);
        void PID_Init_MI(MI_motor_PID*pid,float kp,float ki,float kd,float i_max,float out_max);

    } core;
    class CAN_MOTOR
    {
	public:
		void PID_devices_Init(void);
        void PID_delta_init(void);

        float PID_velocity_realize_1(float set_speed,int i);
        float PID_velocity_realize_1_nonfilter(float set_speed,int i);
        float PID_position_realize_1(float set_pos,int i);
        float pid_call_1(float position,int i);

        float PID_velocity_realize_2(float set_speed,int i);
        float PID_position_realize_2(float set_pos,int i);
        float pid_call_2(float position,int i);


        float PID_delta_call_up(float position,int i);
        float PID_delta_call_down(float position,int i);

    }can_motor;
};
#ifdef __cplusplus
}
#endif

#endif // __PID_H_