#ifndef DELTA_CLAC_H
#define DELTA_CLAC_H

#ifdef __cplusplus
extern "C" 
{
#endif
#include "math.h"

#define delta_PI 3.415926f
#include "include.h"

// ���峣��
extern const float under_R; // ����뾶��������������ĵľ��룬��λ��m��
extern const float move_r ; // ��ƽ̨�뾶����λ��m
extern const float u_L ; // �����۵ĳ��ȣ���λ��m
extern const float a_l ; // ���˵ĳ��ȣ���λ��m

//˿��
extern const float s_S ;//�ݾ�
extern float delta_position[3];
extern float D_theta[3];//����Ƕ�����
extern float zero_theta;//����Ƕ�

// void get_delta_position(void);
// float deltaLimitMax(float input);
// extern void DeltaInversekinematic(float x, float y, float z, float *theta) ;
// extern void move_delta(float theta , uint8_t theta_flag);



class DeltaCalculator {
public:
    

    static void get_delta_position(void)
    static float limitMax(float input);
    static void move_delta(float theta , uint8_t theta_flag);
    static void DeltaInversekinematic(float x, float y, float z, float *theta);
};
#ifdef __cplusplus
}
#endif
#endif




