#include "include.h"
#include "uart_app.h"

extern motor_measure_t motor_can1[8];

uint8_t step=0;

uint8_t run_mode=0x01;
uint8_t controller=0;// 0->��ͬ���� | 1->�ֶ�����
uint8_t delta_mode=0;
uint8_t msg_mode=CIRCUIT_SEND_DISABLE;

uint8_t crc_code=0;
uint8_t declearation=0;
uint8_t count=0;

uint8_t tx_buffer[6];
uint8_t buffer[4]={0};
uint8_t angle_buffer[10]={0};
uint8_t rx_buffs_ykq[12];
float receive_data[6]={0};

uint8_t cyber_gear_knock_flag=0;

angle_measure my_angle_measure={0};

uint8_t sum_ykq=0;
int step_ykq =0;
int cnt_ykq=0;

uint8_t change_mode_flag=0x00;

uint8_t commander=0;
UARTApp uartapp;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		switch(step)
		{
			
			case 0:
			{
				uartapp.waiting_for_pack_head();
			}break;
			
			case 1:
			{
				uartapp.judgement_of_declearation();
				uartapp.receiving_datas();

			}break;
			
			case 2:
			{
				
				uartapp.receiving_datas();
				
			}break;
			
			case 3:
			{
				uartapp.crc8_inspecting();
				
			}break;
			
			case 4:
			{
				uartapp.waiting_for_package_tail();
			}break;

			case CONTROLLER_PACKAGE_BEGINNING:
			{
				commander=buffer[0];
				uartapp.commander_package_analysis(buffer[0]);
				step=4;
				HAL_UART_Receive_IT(&huart1,buffer,1);
				
			}break;
			
			default:step=0;break;
		}
		
	}
	
	else if(huart->Instance==USART2)
	{
		
		static uint8_t step_uart2=0;
		static uint8_t data_type=0;
		
		switch(step_uart2)
		{
		
			case 0:
			{
				
				if(angle_buffer[0]==0x55)
				{
					
					step_uart2=1;
					HAL_UART_Receive_IT(&huart2,angle_buffer,1);
						
				}else HAL_UART_Receive_IT(&huart2,angle_buffer,1);
				
			}break;
			
			case 1:
			{
				
				if(angle_buffer[0]==0x53)
				{
					
					step_uart2=2;
					data_type=0x53;
					HAL_UART_Receive_IT(&huart2,angle_buffer,9);
					
				}
				
				else if(angle_buffer[0]==0x52)
				{
					
					step_uart2=2;
					data_type=0x52;
					HAL_UART_Receive_IT(&huart2,angle_buffer,9);
					
				}
				
				else if(angle_buffer[0]==0x51)
				{
					
					step_uart2=2;
					data_type=0x51;
					HAL_UART_Receive_IT(&huart2,angle_buffer,9);
					
				}
					
				else HAL_UART_Receive_IT(&huart2,angle_buffer,1);
				
			}break;
			case 2:
			{
				if(data_type==0x53)
				{
					my_angle_measure.roll  =  ((int16_t)( angle_buffer[1]<<8 )|angle_buffer[0])/32768.0f*180.0f;
					my_angle_measure.pitch =  ((int16_t)( angle_buffer[3]<<8 )|angle_buffer[2])/32768.0f*180.0f;
					my_angle_measure.yaw   =  ((int16_t)( angle_buffer[5]<<8 )|angle_buffer[4])/32768.0f*180.0f;
					uartapp.gyro_position_filter(&my_angle_measure);
					HAL_UART_Receive_IT(&huart2,angle_buffer,1);
					step_uart2=0;
					
				}
				
				else if(data_type==0x52)
				{
					
					my_angle_measure.roll_omega  =  ((int16_t)( angle_buffer[1]<<8 )|angle_buffer[0])/32768.0f*180.0f;
					my_angle_measure.pitch_omega =  ((int16_t)( angle_buffer[3]<<8 )|angle_buffer[2])/32768.0f*180.0f;
					my_angle_measure.yaw_omega   =  ((int16_t)( angle_buffer[5]<<8 )|angle_buffer[4])/32768.0f*180.0f;			
					uartapp.gyro_omega_filter(&my_angle_measure);
					HAL_UART_Receive_IT(&huart2,angle_buffer,1);
					step_uart2=0;
					
				}
				
				else if(data_type==0x51)
				{
					
					my_angle_measure.x_acc = ((int16_t)( angle_buffer[1]<<8 )|angle_buffer[0])/32768.0f*16*9.8f;
					my_angle_measure.y_acc = ((int16_t)( angle_buffer[3]<<8 )|angle_buffer[2])/32768.0f*16*9.8f;
					my_angle_measure.z_acc = ((int16_t)( angle_buffer[5]<<8 )|angle_buffer[4])/32768.0f*16*9.8f;
					uartapp.gyro_acc_filter(&my_angle_measure);
					HAL_UART_Receive_IT(&huart2,angle_buffer,1);
					step_uart2=0;
					
				}
				
			}
			
			default:break;
				
		}
		
	}
	
	else if(huart->Instance==UART4)
	{
		
		uartapp.crc(rx_buffs_ykq[0]);
		HAL_UART_Receive_IT(&huart4,rx_buffs_ykq,1);
		
	}

	
}

void send_command_package(uint8_t command)
{
	
//	tx_buffer[0]=0xa5;
//	tx_buffer[1]=0x00;
//	tx_buffer[2]=command;
//	tx_buffer[3]=0x5a;
//	HAL_UART_Transmit_IT(&huart1,tx_buffer,4);
	
}

void UARTApp::got_it_the_point_already(void)
{
	
	tx_buffer[0]=0x01;
	tx_buffer[1]=0x00;
	tx_buffer[2]=0x01;
	tx_buffer[3]=0x00;
	tx_buffer[4]=0x01;
	tx_buffer[5]=0x00;
	HAL_UART_Transmit_IT(&huart1,tx_buffer,6);
	
}

void UARTApp::not_got_it_the_point_yet(void)
{
	
	tx_buffer[0]=0x00;
	tx_buffer[1]=0x00;
	tx_buffer[2]=0x00;
	tx_buffer[3]=0x00;
	tx_buffer[4]=0x00;
	tx_buffer[5]=0x00;
	HAL_UART_Transmit_IT(&huart1,tx_buffer,6);
	
}

void UARTApp::send_angle_velocity(void)
{
	
//	tx_buffer[0]=;
//	tx_buffer[1]=;
//	tx_buffer[2]=;
//	tx_buffer[3]=;
//	tx_buffer[4]=;
//	tx_buffer[5]=;
//	HAL_UART_Transmit_IT(&huart1,tx_buffer,6);
	
}
/*	
					my_angle_measure.roll= ((int16_t)( angle_buffer[2]<<8 )|angle_buffer[1])/32768.0f*180.0f;
					my_angle_measure.pitch=((int16_t)( angle_buffer[4]<<8 )|angle_buffer[3])/32768.0f*180.0f;
					my_angle_measure.yaw=  ((int16_t)( angle_buffer[6]<<8 )|angle_buffer[5])/32768.0f*180.0f;
*/
