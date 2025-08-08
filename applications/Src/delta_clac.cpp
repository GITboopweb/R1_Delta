#include "CAN_receive.h"
#include "include.h"
#include "delta_clac.h"

float delta_position[3]={0};
CAN_BUS can_bus;
void DeltaCalculator::get_delta_position(void)
{
	for(uint8_t i=0;i<3;++i)
	{
		
		delta_position[i]=-motor_can1[5+i].total_angle* 360.0f * 187.0f / (8191.0f * 3591.0f) - 22.0f;
		
	}
	
}

// ����delta����
const float under_R = 0.09857; // ����뾶��������������ĵľ��룬��λ��m��
const float move_r = 0.11;  // ��ƽ̨�뾶����λ��m
const float u_L = 0.192; // �����۵ĳ��ȣ���λ��m
const float u_l = 0.242; // ���˵ĳ��ȣ���λ��m

//˿��
const float s_S = 4.0f;//�ݾ�


float D_theta[3];
float zero_theta = 0.0f;

float DeltaCalculator::deltaLimitMax(float input){                          
        if (input > 90.0)     
        {                      
            return 90.0;      
        }                      
        else if (input < -15)  
        {                      
            return -15;       
        } 
        else {
	          return input;
        }  	
}
		
/**
 * Delta ����������㺯��
 * ������ƽ̨��Ŀ��λ�� (x, y, z)�������ÿ������ĽǶ� theta1, theta2, theta3
 *
 * @param x Ŀ��λ�õ� x ����
 * @param y Ŀ��λ�õ� y ����
 * @param z Ŀ��λ�õ� z ����
 * @param theta1 ���1�ĽǶȣ�����ֵ��
 * @param theta2 ���2�ĽǶȣ�����ֵ��
 * @param theta3 ���3�ĽǶȣ�����ֵ��
 * @return true �������ɹ���false ���û�п��н�
 */

void DeltaCalculator::DeltaInversekinematic(float x, float y, float z, float *theta) {
    // ���ڼ�����������Ƕȵķ����У����֮�����λ��
    float phi_list[3] = { 0, 2.0 / 3.0 * delta_PI, 4.0 / 3.0 * delta_PI };  // ���������װλ�õ���λ�����ֱ�λ�ڵȱ������ε������ǣ�,�·���ʼ��ʱ������

    // ���ڴ洢ÿ�����������ĽǶ�
    float theta_list[3];

    // �����������
    for (int i = 0; i < 3; i++) {
        float phi = phi_list[i]; // ��ǰ�������λ�Ƕ�

        // ���ݼ��ι�ʽ���� a, b, c����Ϊ����ǶȵĲ���
        float a = 2 * u_L * (under_R - move_r - x * cosf(phi) - y * sinf(phi));
        float b = -2 * u_L * z;
        float c = x * x + y * y + z * z + u_L * u_L + (under_R - move_r) * (under_R - move_r) - u_l * u_l - 2 * (under_R - move_r) * (x * cosf(phi) + y * sinf(phi));

        // ������⹫ʽ�������ĽǶ�
        theta_list[i] = (-b - sqrt(a * a + b * b - c * c)) / (c - a);
    }

    // �ֱ��ȡ��������ĽǶ�
    theta[0] = deltaLimitMax(2 * atanf(theta_list[0]) * 180.0 / delta_PI);
    theta[1] = deltaLimitMax(2 * atanf(theta_list[1]) * 180.0 / delta_PI);
    theta[2] = deltaLimitMax(2 * atanf(theta_list[2]) * 180.0 / delta_PI);

    if (isnan(theta[0]) || isnan(theta[1]) || isnan(theta[2])) {
       theta[0] = zero_theta * 180.0 / delta_PI;
       theta[1] = zero_theta * 180.0 / delta_PI;
       theta[2] = zero_theta * 180.0 / delta_PI;  // �������Ч���Ƕȹ���  
    }


}

//*********************************************************
/*ִ�д���

����Ŀ��λ��
�������Ƕ�
delta����
DeltaInversekinematic(0.0f,0.0f,0.00f,D_theta);
CAN1_CMD_1(pid_call_1(-(D_theta[0]+78)/360*8191*3591/187,1),
           pid_call_1(-(D_theta[1]+78)/360*8191*3591/187,2),
           pid_call_1(-(D_theta[2]+78)/360*8191*3591/187,3),0);
�趨��ʱ0.1s
delta����

*/

//*************************************************************************
//��ִ̨�д���
/*
//
//�����ʺϻ���Ƕ�
//����
//
//
//
*/
void DeltaCalculator::move_delta(float theta , uint8_t theta_flag){
	if(theta_flag == 1){
		//if������Ҫ���ifѡ��ִ�нǶ�
		can_bus.cmd.CAN2_CMD_1(0,pid_call_2(theta*8191*36,2),0,0);
	}
	else{
		//������Ҫ��ʱ�ȴ��������½��ձ�־λ
		can_bus.cmd.CAN2_CMD_1(0,0,0,0);
	}
	
}//��Ҫ��ִ�к������ʱ����pid����










