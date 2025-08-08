/**
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  * @file       MI_motor_drive.c
  * @brief      С�׵��CyberGear����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-11-2023     С���           1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  */
 
/* Includes -------------------------------------------------------------------*/
#include "include.h"
#include "Mi.h"
#include "can.h"
#include "pid.h"

extern MI_Motor mi_motor;
PID_Controller pid_controller;

MI_Motor_s MI_Motor[9];//С�׵���ṹ��,0�Ų���
CAN_TxHeaderTypeDef CAN_TxHeader_MI;

uint8_t MI_MASTERID = 1; //master id ����ָ��ʱEXTID��bit8:15,������bit0:7
OutputData_s OutputData;


/**
  * @brief          floatתint�����ݴ����
  * @param[in]      x float��ֵ
  * @param[in]      x_min float��ֵ����Сֵ
  * @param[in]      x_max float��ֵ�����ֵ
  * @param[in]      bits  int������λ��
  * @retval         none
  */
uint32_t FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x=x_min;
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          ���뷶Χ����
  * @param[in]      x ������ֵ
  * @param[in]      x_min ������ֵ����Сֵ
  * @param[in]      x_max ������ֵ�����ֵ
  * @retval         none
  */
float RangeRestrict(float x, float x_min, float x_max)
{
    float res;
    if(x > x_max) res=x_max;
    else if(x < x_min) res=x_min;
    else res = x;
    return res;
}

/**
  * @brief          С�׵����ʼ��
  * @param[out]     hmotor ����ṹ��
  * @param[in]      phcan can���߾��
  * @retval         none
  */
void MI_Motor::MI_motor_Init(MI_Motor_s* hmotor,CAN_HandleTypeDef *phcan, uint8_t motor_id)
{
    hmotor->phcan = phcan;
    hmotor->motor_id = motor_id;
}

/**
  * @brief          С�׵��CANͨ�ŷ���
  * @param[in]      hmotor ����ṹ��
  * @retval         none
  */
void MI_motor_CanTx(MI_Motor_s* hmotor)
{
    CAN_TxHeader_MI.DLC = 8;
    CAN_TxHeader_MI.IDE = CAN_ID_EXT;
    CAN_TxHeader_MI.RTR = CAN_RTR_DATA;
    CAN_TxHeader_MI.ExtId = *((uint32_t*)&(hmotor->EXT_ID));
    /*�����õķ�������*/
    uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(hmotor->phcan);//����Ƿ��п�������
    while (free_TxMailbox==0){//�ȴ������������ﵽ3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(hmotor->phcan);
    }
    /* ��������Ϣ���ӵ����������� */
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hmotor->phcan, &CAN_TxHeader_MI, hmotor->txdata, &mailbox);//�����͵��������ӵ�����������
}

/*-------------------- ����С�׵���ĵ�д�ĸ���ͨ������ --------------------*/

/**
  * @brief          ��ȡ�豸ID ��ͨ������0�������ڵ��ʹ��ǰʹ��
  * @param[in]      hmotor ����ṹ��
  * @retval         none
  */
void MI_Motor::MI::MI_motor_GetID(MI_Motor_s* hmotor)
{
    hmotor->EXT_ID.mode = 0;
    hmotor->EXT_ID.data = 0;
    hmotor->EXT_ID.motor_id = 0;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    
    MI_motor_CanTx(hmotor);
}

/**
  * @brief          �˿�ģʽ�������ָ�ͨ������1��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      torque Ŀ������
  * @param[in]      MechPosition 
  * @param[in]      speed 
  * @param[in]      kp 
  * @param[in]      kd 
  * @retval         none
  */
void MI_Motor::CONTROL::MI_motor_Control(MI_Motor_s* hmotor, float torque, float MechPosition , float speed , float kp , float kd)
{
    hmotor->EXT_ID.mode = 1;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = FloatToUint(torque,T_MIN,T_MAX,16);
    hmotor->EXT_ID.res = 0;
 
    hmotor->txdata[0]=FloatToUint(MechPosition,P_MIN,P_MAX,16)>>8;
    hmotor->txdata[1]=FloatToUint(MechPosition,P_MIN,P_MAX,16);
    hmotor->txdata[2]=FloatToUint(speed,V_MIN,V_MAX,16)>>8;
    hmotor->txdata[3]=FloatToUint(speed,V_MIN,V_MAX,16);
    hmotor->txdata[4]=FloatToUint(kp,KP_MIN,KP_MAX,16)>>8;
    hmotor->txdata[5]=FloatToUint(kp,KP_MIN,KP_MAX,16);
    hmotor->txdata[6]=FloatToUint(kd,KD_MIN,KD_MAX,16)>>8;
    hmotor->txdata[7]=FloatToUint(kd,KD_MIN,KD_MAX,16);

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          С�׵������֡���루ͨ������2��
  * @param[in]      Rx_can_info ���ܵ��ĵ�����ݽṹ��
  * @param[in]      rx_data[8] CAN�߽��յ�������
  * @note           �����յ���CAN�����ݽ��뵽������ݽṹ����
  * @retval         none
  */
void MI_Motor::MI::MI_motor_RxDecode(RxCAN_info_type_2_s* RxCAN_info,uint8_t rx_data[8])
{
    uint16_t decode_temp_mi;//С�׵���������ݽ��뻺��
    decode_temp_mi = (rx_data[0] << 8 | rx_data[1]);
    RxCAN_info->angle = ((float)decode_temp_mi-32768.7f)/32768.7f*4*3.1415926f;
//-32767.5f)/32767.5f*4*3.1415926f;;
    decode_temp_mi = (rx_data[2] << 8 | rx_data[3]);
    RxCAN_info->speed = ((float)decode_temp_mi-32767.5f)/32767.5f*30.0f;

    decode_temp_mi = (rx_data[4] << 8 | rx_data[5]);
    RxCAN_info->torque = ((float)decode_temp_mi-32767.5f)/32767.5f*12.0f;

    decode_temp_mi = (rx_data[6] << 8 | rx_data[7]);
    RxCAN_info->temperature = (float)decode_temp_mi/10.0f;
 }

/**
  * @brief          С�׵��ʹ�ܣ�ͨ������ 3��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      id ���id
  * @retval         none
  */
void MI_Motor::MI::MI_motor_Enable(MI_Motor_s* hmotor)
{
    hmotor->EXT_ID.mode = 3;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          ���ֹͣ����֡��ͨ������4��
  * @param[in]      hmotor ����ṹ��
  * @retval         none
  */
void MI_Motor::MI::MI_motor_Stop(MI_Motor_s* hmotor)
{
    hmotor->EXT_ID.mode = 4;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          ���õ����е��λ��ͨ������6����ѵ�ǰ���λ����Ϊ��е��λ�����綪ʧ��
  * @param[in]      hmotor ����ṹ��
  * @retval         none
  */
void MI_Motor::MI::MI_motor_SetMechPositionToZero(MI_Motor_s* hmotor)
{
    hmotor->EXT_ID.mode = 6;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    hmotor->txdata[0]=1;
 
    for(uint8_t i=1; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          ���õ��CAN_ID��ͨ������7�����ĵ�ǰ���CAN_ID , ������Ч�����ڵ��ʹ��ǰʹ��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      Now_ID ������ڵ�ID
  * @param[in]      Target_ID ��Ҫ�ĳɵĵ��ID
  * @retval         none
  */
void MI_Motor::MI::MI_motor_ChangeID(MI_Motor_s* hmotor,uint8_t Now_ID,uint8_t Target_ID)
{
    hmotor->motor_id = Now_ID;

    hmotor->EXT_ID.mode = 7;	
    hmotor->EXT_ID.motor_id = Now_ID;
    hmotor->EXT_ID.data = Target_ID << 8 | MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          ����������ȡ��ͨ������17��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      index ������
  * @retval         none
  */
void MI_Motor::MI::MI_motor_ReadParam(MI_Motor_s* hmotor,uint16_t index)
{
    hmotor->EXT_ID.mode = 17;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    
    memcpy(&hmotor->txdata[0],&index,2);

    for(uint8_t i=2; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          С�׵������ģʽ�л�
  * @param[in]      hmotor ����ṹ��
  * @param[in]      run_mode ���ĵ�ģʽ
  * @note           ͨ������18 �����綪ʧ��
  * @retval         none
  */
void MI_Motor::MI::MI_motor_ModeSwitch(MI_Motor_s* hmotor, uint8_t run_mode)
{
    uint16_t index = 0X7005;

    hmotor->EXT_ID.mode = 18;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;

    for(uint8_t i=0;i<8;i++){
      hmotor->txdata[i]=0;
    }

    memcpy(&hmotor->txdata[0],&index,2);
    memcpy(&hmotor->txdata[4],&run_mode, 1);

    MI_motor_CanTx(hmotor);
}

/**
  * @brief          С�׵�����Ʋ���д��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      index ������
  * @param[in]      param д��Ĳ���
  * @note           ͨ������18 �����綪ʧ��
  * @retval         none
  */
 void MI_Motor::MI::MI_motor_WritePram(MI_Motor_s* hmotor, uint16_t index, float param)
 {
    hmotor->EXT_ID.mode = 18;
    hmotor->EXT_ID.motor_id = hmotor->motor_id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;

    memcpy(&hmotor->txdata[0],&index,2);
    hmotor->txdata[2]=0;
    hmotor->txdata[3]=0;
    memcpy(&hmotor->txdata[4],&param, 4);

    MI_motor_CanTx(hmotor);
 }
void PID_Controller::CORE::PID_Init_MI(MI_motor_PID*pid,float kp,float ki,float kd,float i_max,float out_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_max = i_max;
	pid->out_max = out_max;
}
void LIMIT_MIN_MAX_MI(float value,float min_value,float max_value)
{
	if(value<min_value)
	{
		value = min_value;
	}
	else if(value>max_value)
	{
		value = max_value;
	}
}

float PID_Controller::CORE::pid_calc_MI(MI_motor_PID*pid ,float angle,float set_angle)
{
	pid->angle = angle;
	pid->set_angle = set_angle;
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->angle-pid->set_angle;
	
	pid->p_out = pid->kp*pid->err[0];
	pid->i_out += pid->ki*pid->err[0];
	pid->d_out = pid->kd*(pid->err[0] - pid->err[1]);
	LIMIT_MIN_MAX_MI(pid->i_out,-pid->i_max,pid->i_max);
	pid->output = pid->p_out+pid->i_out+pid->d_out;
	LIMIT_MIN_MAX_MI(pid->output,-pid->out_max,pid->out_max);
	return pid->output;
}
/*-------------------- ��װ��һЩ���ƺ��� --------------------*/

/**
  * @brief          С�׵�����ؿ���ģʽ����ָ��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      torque Ŀ������
  * @retval         none
  */
void MI_Motor::CONTROL::MI_motor_TorqueControl(MI_Motor_s* hmotor, float torque)
{
    MI_Motor::CONTROL::MI_motor_Control(hmotor, torque, 0, 0, 0, 0);
}

/**
  * @brief          С�׵��λ��ģʽ����ָ��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      location ����λ�� rad
  * @param[in]      kp ��Ӧ�ٶ�(����λ�ÿ���)��һ��ȡ1-10
  * @param[in]      kd ������ᣬ��С���𵴣��������������ԡ�һ��ȡ0.5����
  * @retval         none
  */
void MI_Motor::CONTROL::MI_motor_LocationControl(MI_Motor_s* hmotor, float location, float kp, float kd)
{
    MI_Motor::CONTROL::MI_motor_Control(hmotor, 0, location, 0, kp, kd);
}

/**
  * @brief          С�׵���ٶ�ģʽ����ָ��
  * @param[in]      hmotor ����ṹ��
  * @param[in]      speed �����ٶ�
  * @param[in]      kd ��Ӧ�ٶȣ�һ��ȡ0.1-1
  * @retval         none
  */
void MI_Motor::CONTROL::MI_motor_SpeedControl(MI_Motor_s* hmotor, float speed, float kd)
{
    MI_Motor::CONTROL::MI_motor_Control(hmotor, 0, 0, speed, 0, kd);
}

// /**
//   * @brief          С�׵������ģʽ����ָ��
//   * @param[in]      hmotor ����ṹ��
//   * @param[in]      iq_ref ���Ƶ���
//   * @retval         none
//   */
// void MI_motor_CurrentMode(MI_Motor_s* hmotor, float iq_ref)
// {
//     MI_motor_ModeSwitch(hmotor, CURRENT_MODE);
//     MI_motor_Enable(hmotor);
//     MI_motor_WritePram(hmotor, IQ_REF, RangeRestrict(iq_ref, IQ_REF_MIN, IQ_REF_MAX));
// }




/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */

