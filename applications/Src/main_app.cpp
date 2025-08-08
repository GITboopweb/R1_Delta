#include "stm32f4xx_hal.h" // Add this line for HAL_Delay declaration

#include "bsp_delay.h"
#include "include.h"
#include "can.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Mi.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern MI_Motor_s MI_Motor[9];
extern TIM_HandleTypeDef htim2;

extern VelocityCalculator velocity_calculator;
extern PID_Controller pid_controller;
extern CAN_BUS can_bus;
BSP_Delay bsp_delay;
extern DisplacementPID displacement_pid;
extern DeltaCalculator delta_calculator;
extern MI_Motor mi_motor;
extern SpatiumDesigner spatiunm_designer;


void user_init(void)
{
	HAL_Delay(3000);
	pid_controller.core.PID_devices_Init();
	pid_controller.core.PID_delta_init();

	can_bus.bsp.CAN1_Filter_Init();
	can_bus.bsp.CAN2_Filter_Init();

	can_bus.bsp.CAN_Start(&hcan1);
	can_bus.bsp.CAN_Start(&hcan2);

	HAL_UART_Receive_IT(&huart1,buffer,1);
	HAL_UART_Receive_IT(&huart2,angle_buffer,1);
	HAL_UART_Receive_IT(&huart4,rx_buffs_ykq,1);
	bsp_delay.delay.delay_init();

	velocity_calculator.inverse_solution_matrix_init();
	velocity_calculator.solution_matrix_init();
	HAL_TIM_Base_Start_IT(&htim2);

	displacement_pid.xy_displacement_pid_config(1,0,0,150,2000);//kp,kd,ki,max_output
	displacement_pid.z_displacement_pid_config(2,0,0,184,3000);//�Ƕȵ�������
	
	velocity_calculator.vel_designer_init(body_acc_design,1500);
	displacement_pid.xy_spatium_designer_init(xyw_spatium_designer,4800,2400,1200);//���Ƕ�xy��Ĺ滮����λ�����������ת�����������ϵķ�����Ҳ����rpm�����ת�ӵ�ÿ2.41rpm,��Ӧ���������ٶ�1mm/s
	spatiunm_designer.spatium_designer_init(&xyw_spatium_designer[2],2000,500,125);//�Ƕȹ켣��������
	mi_motor.MI_motor_Init(&MI_Motor[2],&MI_CAN_2,2);
	mi_motor.mi.MI_motor_Enable(&MI_Motor[2]);
}

void local_velocity_mode_run_with_angle_displacement(void)
{
	calc_buffer[0]=receive_data[0]+ykq_xyw[0];
	calc_buffer[1]=receive_data[1]+ykq_xyw[1];
	calc_buffer[2]=angular_displacement_PID(receive_data[2]+ykq_xyw[2]);
	feedforword_control(body_acc_design,calc_buffer);
	velocity_calculator.vel_control(calc_buffer);
}

void world_velocity_mode_run(void)
{	
	velocity_calculator.rotate_matrix_calc(-my_yaw*PI/180.0f);
	calc_buffer[0]=ykq_xyw[0];
	calc_buffer[1]=ykq_xyw[1];
	calc_buffer[2]=ykq_xyw[2];
	velocity_calculator.matrix_multiply(rotate_matrix,calc_buffer,calc_buffer_2nd);
	velocity_calculator.vel_control(calc_buffer_2nd);
}

void world_displacement_mode_run(void)
{
	
		static float last_target[3]={0};
		static const float threshold = 1.0f;
		
		if(change_mode_flag==0x04)
		{
			can_bus.dji_encoder.reset_motor_position(motor_can1,3);
			displacement_pid.rnd_count_and_diaplacement_reset();
			
			last_target[0]=0;
			last_target[1]=0;
			last_target[2]=0;
			
			receive_data[0]=0;
			receive_data[1]=0;
			receive_data[2]=0;
			
			change_mode_flag=0;
		}
		
		displacement_pid.get_rnd_count_and_diaplacement();
		
		if(fabsf(last_target[0] - receive_data[0]) > threshold||fabsf(last_target[1] - receive_data[1]) > threshold||fabsf(last_target[2] - receive_data[2]) > threshold)
		{
			
			for(uint8_t i=0;i<2;++i)//�Ƕȿ��Ʋ����������Ǹ�����ת�������Ŀ��ƣ�Ҫ����д����������� i ���� <2 �ı��� 
			{
				spatiunm_designer.spatium_designer_set_target(&xyw_spatium_designer[i],(receive_data[i]/*Ŀ������*/-displace_buffer[i])/*��ǰ����*/);
				last_target[i]=receive_data[i];
			}
			spatiunm_designer.spatium_designer_set_target(&xyw_spatium_designer[2],angle_error(receive_data[2]/*Ŀ������*/,displace_buffer[2]/*��ǰ����*/));//���鵽+-180
		}

		spatiunm_designer.spatium_designer_update(&xyw_spatium_designer[0]);//�ȸ����ټ�⣬���˶�ĩβ�����ȼ���ٸ��¿�����Ӧ�ٶȿ���һ����������
		if(xyw_spatium_designer[0].finished==True)
		{
			displacement_pid.x_displacement_control(receive_data[0]);
			calc_buffer[0]=my_displacement_pid.out[0];
		}else calc_buffer[0]=xyw_spatium_designer[0].v_out;//x�����

		spatiunm_designer.spatium_designer_update(&xyw_spatium_designer[1]);
		if(xyw_spatium_designer[1].finished==True)
		{
			displacement_pid.y_displacement_control(receive_data[1]);
			calc_buffer[1]=my_displacement_pid.out[1];
		}else calc_buffer[1]=xyw_spatium_designer[1].v_out;//y

		spatiunm_designer.spatium_designer_update(&xyw_spatium_designer[2]);
		if(xyw_spatium_designer[2].finished==True)
		{
			displacement_pid.z_displacement_control(receive_data[2]);
			calc_buffer[2]=my_displacement_pid.out[2];
		}else calc_buffer[2]=xyw_spatium_designer[2].v_out;//z

		velocity_calculator.vel_control(calc_buffer);

		if(xyw_spatium_designer[0].finished==False||xyw_spatium_designer[1].finished==False||xyw_spatium_designer[2].finished==False)
		{
			if(msg_mode==CIRCUIT_SEND_ENABLE) not_got_it_the_point_yet();
		}else if(msg_mode==CIRCUIT_SEND_ENABLE) got_it_the_point_already();
		
}

void pure_local_velocity_mode_run(void)
{
//	static float curren_angle_speed=0;
//	static float current_angle=0;
//	
//	calc_buffer[0]=receive_data[0]+ykq_xyw[0];
//	calc_buffer[1]=receive_data[1]+ykq_xyw[1];
//	
//	curren_angle_speed=(receive_data[2]+ykq_xyw[2])*6*187/3591;//ͨ�Ż������ת���ٶȵ�λrpm��ת��Ϊ���ӵĶ�ÿ�롣
//	current_angle+=curren_angle_speed*DT;
//	current_angle = fmodf(current_angle + 180.0f, 360.0f);
//	if (current_angle < 0) current_angle += 360.0f;
//	current_angle -= 180.0f;
//	
//	calc_buffer[2]=angular_displacement_PID(current_angle);
	velocity_calculator.vel_control(ykq_xyw);
}

void cybergear_control(void)
{
	switch(cyber_gear_knock_flag)
	{
		case 1:{
			mi_motor.control.MI_motor_Control(&MI_Motor[2],-0.4,-3.6,28.2,21,2);
		}break;
		
		case 2:{
			mi_motor.control.MI_motor_Control(&MI_Motor[2],2,0.7,29,300,4.4);
		}break;
		
		default:break;
	}
}

void msg_control(void)
{
	
//		if(msg_mode==CIRCUIT_SEND_ENABLE)//msg_mode==CIRCUIT_SEND_ENABLE
//	  {
//		  
//			  tx_buffer[0]=motor_can1[0].speed_rpm;
//			  tx_buffer[1]=motor_can1[0].speed_rpm >> 8;
//			  tx_buffer[2]=motor_can1[1].speed_rpm;
//			  tx_buffer[3]=motor_can1[1].speed_rpm >> 8;
//			  tx_buffer[4]=motor_can1[2].speed_rpm;
//			  tx_buffer[5]=motor_can1[2].speed_rpm >> 8;
//			  HAL_UART_Transmit_IT(&huart1,tx_buffer,6);
//		  
//	  }
	switch (send_flag){
		
		case 2:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		case 3:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		case 4:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		case 5:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		default:{send_flag=0;break;}
		send_flag=0;
	}
	
}

void delta_control(void)
{
	static TimerNonBlocking delta_rise_timer={0};
	static TimerNonBlocking delta_fall_timer={0};
    if(delta_mode==1)
    {
//        DeltaInversekinematic(receive_data[3],receive_data[4],receive_data[5],D_theta);
        if (receive_data[3]>90&&receive_data[4]>90&&receive_data[5]>90) {
			receive_data[3] = 90;
			receive_data[4] = 90;
			receive_data[5] = 90;  // �������Ч���Ƕȹ���  
             can_bus.bsp.CAN1_CMD_2
           (

            pid_controller.can_motor.pid_call_1(-(receive_data[3]+12.0f)/360.0f*8191.0f*3591.0f/187.0f,5),

            pid_controller.can_motor.pid_call_1(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),

            pid_controller.can_motor.pid_call_1(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),

            0
        
           );
//			float num1 = -(receive_data[3]+22)/360*8191*3591/187;
     }
        else if (receive_data[3]<-12 && receive_data[3]<-12 && receive_data[3]<-12) {
		   receive_data[3] = -12;
		   receive_data[4] = -12;
		   receive_data[5] = -12;  // �������Ч���Ƕȹ���  
             can_bus.bsp.CAN1_CMD_2
           (
        
            pid_controller.can_motor.pid_call_1(-(receive_data[3]+12.0f)/360.0f*8191.0f*3591.0f/187.0f,5),

            pid_controller.can_motor.pid_call_1(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),

            pid_controller.can_motor.pid_call_1(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),

            0
        
           );
     }
		else{

			if(delta_position[0]-receive_data[3]>0)/*delta fall*/
			{

				can_bus.bsp.CAN1_CMD_2
			 (
			
			  pid_controller.can_motor.PID_delta_call_down(-(receive_data[3]+12.0f)/360.0f*8191.0f*3591.0f/187.0f,5),

			  pid_controller.can_motor.PID_delta_call_down(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),

			  pid_controller.can_motor.PID_delta_call_down(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),

			  0
			
			 );
			}else/*delta rise*/ 
			{
				can_bus.bsp.CAN1_CMD_2
			 (
			
			  pid_controller.can_motor.PID_delta_call_up(-(receive_data[3]+12.0f)/360.0f*8191.0f*3591.0f/187.0f,5),

			  pid_controller.can_motor.PID_delta_call_up(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),

			  pid_controller.can_motor.PID_delta_call_up(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),

			  0
			
			 );
             

			delta_calculator.get_delta_position();
        }
        
    }
	
	else if(delta_mode==2)
	{

		if (!delta_rise_timer.active) {
			bsp_delay.timer.Timer_Start(&delta_rise_timer, 700000); // .7����ʱ
		}

		can_bus.bsp.CAN1_CMD_2(
			pid_controller.can_motor.PID_delta_call_up(-(81+22.0f)/360.0f*8191.0f*3591.0f/187.0f,5),
			pid_controller.can_motor.PID_delta_call_up(-(90+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),
			pid_controller.can_motor.PID_delta_call_up(-(90+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),
			0
		);

		if (bsp_delay.timer.Timer_Expired(&delta_rise_timer)) {
			delta_mode = 3;
			delta_rise_timer.active = 0;
		}
	
	}
	else if (delta_mode==3)
	{
		if (!delta_fall_timer.active) {
			bsp_delay.timer.Timer_Start(&delta_fall_timer, 500000); // .5����ʱ
		}
		can_bus.bsp.CAN1_CMD_2(
			pid_controller.can_motor.PID_delta_call_up(-(32.0f)/360.0f*8191.0f*3591.0f/187.0f,5),
			pid_controller.can_motor.PID_delta_call_up(-(32.0f)/360.0f*8191.0f*3591.0f/187.0f,6),
			pid_controller.can_motor.PID_delta_call_up(-(32.0f)/360.0f*8191.0f*3591.0f/187.0f,7),
			0
		);
		if (bsp_delay.timer.Timer_Expired(&delta_fall_timer)) {
			delta_fall_timer.active = 0;
			can_bus.bsp.CAN1_CMD_2(0,0,0,0);
			delta_mode=0;
		}
	}
}
