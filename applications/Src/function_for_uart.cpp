#include "include.h"
#include "function_for_uart.h"

uint8_t send_flag=0;
static uint8_t Buf[30]={0};//��λ�����ݰ�
static uint8_t last_anjian;
uint8_t anjian;
static uint8_t mode_c=0x01;
static uint8_t linux_navgt=0;

float ykq_xyw[3]={0};

float UARTFunction::input_map(float input,float max)
{
    float x_norm = fabsf(input / 2048.0f);           // [-1, 1]
    float output = powf(x_norm, 2.0f) * max;  // ң����������ӳ�䵽+-8000
    if (input < 0) output = -output;
    return output;
}

void UARTFunction::toresult()//*********************ң�������ݽ���
{
	static float xyzf[4]={0};
	
	xyzf[0]=(float)(rx_buffs_ykq[7]<<8|rx_buffs_ykq[8]);
	xyzf[1]=(float)(rx_buffs_ykq[5]<<8|rx_buffs_ykq[6]);
	xyzf[2]=(float)(rx_buffs_ykq[3]<<8|rx_buffs_ykq[4]);
		
			last_anjian=anjian;
			 anjian=rx_buffs_ykq[9];
			switch(anjian){
				
				case 0x00:{}break;
				case 0x01:{
					run_mode=0x01;
				}break;
				
				case 0x02:{
					run_mode=0x02;
				}break;
				
				case 0x03:{
					run_mode=0x03;//�������̵��˶�ģʽ
					change_mode_flag=0x04;
				}break;
				case 0x04:{
					run_mode=0x04;
				}break;
				case 0x05:{}break;
				case 0x06:{
					send_flag=2;
				}break;
				case 0x07:{
					send_flag=3;
				}break;
				case 0x08:{
					send_flag=4;
				}break;
				case 0x09:{
					send_flag=5;
				}break;
				case 0x0a:{
					
					delta_mode=2;
					
				}break;
				
				case 0x0b:{
					
					delta_mode=3;
					
				}break;
				
				case 0x0c:{
					cyber_gear_knock_flag=0x01;
				}break;
				
				case 0x0d:{
					cyber_gear_knock_flag=0x02;
				}break;
				
				case 0x0e:{
					if(last_anjian==0x10){
						if(controller){
							controller=0x00;
							for(uint8_t i =0;i<3;++i) receive_data[i]=0;
					}else controller=0x01;}
					
				}break;
				
				default:{
					linux_navgt=anjian;
					HAL_UART_Transmit(&huart4,&anjian,1,100);
				}break;
			}
			
				ykq_xyw[0]=input_map(xyzf[0]-2048,4000);
				ykq_xyw[1]=input_map(xyzf[1]-2048,4000);
				ykq_xyw[2]=-(xyzf[2]-2048);
				
}

void UARTFunction::crc(uint8_t bye)//ң��������
{
   
    switch(step_ykq)
   {
       case 0://֡ͷ0x5a
       {
         if(bye==0x5A)
         {
             rx_buffs_ykq[cnt_ykq++]=0x5A;
             step_ykq++;
             sum_ykq+=0x5A;
         }
         else 
         {
             cnt_ykq=0;
             step_ykq=0;
             sum_ykq=0;
         }
         break;
        }
       case 1://����
       {
           if(step_ykq==1&&cnt_ykq<=9&&cnt_ykq>=1)
           {
               rx_buffs_ykq[cnt_ykq++]=bye;
               sum_ykq+=bye;
               if(cnt_ykq==10)
                   step_ykq++;
           }
           else 
           {
               step_ykq=0;
               sum_ykq=0;
               cnt_ykq=0;
           }
           break;
       }
       case 2://crc����
       {
           if(bye==sum_ykq)
           {
               rx_buffs_ykq[cnt_ykq]=bye;
               step_ykq++;
               cnt_ykq++;
           }
           else 
           {
               step_ykq=0;
               cnt_ykq=0;
               sum_ykq=0;
           }
           break;
       }
       case 3:
       {
           if(bye==0xA5)
           {
            rx_buffs_ykq[cnt_ykq]=bye;
            toresult();
             cnt_ykq=0;
             step_ykq=0;
              sum_ykq=0;               
           }
           else 
           {
               cnt_ykq=0;
               step_ykq=0;
               sum_ykq=0;
           }
           break;
       }
   }
    
}

float UARTFunction::get_float_num(uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4)
{
	float temp=0;
	uint8_t* ptr_for_float=(uint8_t*)&temp;
	ptr_for_float[0]=data1;
	ptr_for_float[1]=data2;
	ptr_for_float[2]=data3;
	ptr_for_float[3]=data4;
	return temp;
}

void UARTFunction::waiting_for_pack_head(void)
{
	
	if(buffer[0]==0xa5)
				{
					++step;
					count=0;
					crc_code=0;
					HAL_UART_Receive_IT(&huart1,buffer,1);
					
				}
				
				else
				{
					HAL_UART_Receive_IT(&huart1,buffer,1);
					step=0;
				}
	
}

void UARTFunction::judgement_of_declearation(void)
{
	
	if(buffer[0]!=0)
	{
					
		declearation=buffer[0];
		if(declearation>6) declearation=6;
		++step;
		HAL_UART_Receive_IT(&huart1,buffer,4);
					
	}
				
	else
	{
					
		step=CONTROLLER_PACKAGE_BEGINNING;
		HAL_UART_Receive_IT(&huart1,buffer,1);
					
	}
	
}

void UARTFunction::receiving_datas(void)
{
	
	receive_data[count]=get_float_num(buffer[0],buffer[1],buffer[2],buffer[3]);
	crc_code+=(buffer[0]+buffer[1]+buffer[2]+buffer[3]);
	if(controller){
		if(count<3) receive_data[count]=0;
	}++count;
				
	if(count<declearation)
	{
					
		HAL_UART_Receive_IT(&huart1,buffer,4);
					
	}
				
				
	else
	{
					
		++step;
		HAL_UART_Receive_IT(&huart1,buffer,1);
	}
	
}

void UARTFunction::crc8_inspecting(void)
{
	if(crc_code==buffer[0])
	{
		++step;
	}
				
	else
	{
		step=0;	
		for(uint8_t i=0;i<4;++i)
		{
			receive_data[i]=0;
		}		
	}		
	HAL_UART_Receive_IT(&huart1,buffer,1);
	
}

void UARTFunction::waiting_for_package_tail(void)
{
	
	if(buffer[0]==0x5a)
	{
		step=0;
	}
				
//	else 
//	{
//		step=0;
//	//				
//	//	for(uint8_t i=0;i<4;++i)
//	//	{
//	//		receive_data[i]=0;
//	//	}
//					
//		run_mode=0x01;
//					
//	}
				
	HAL_UART_Receive_IT(&huart1,buffer,1);
	
}

void UARTFunction::commander_package_analysis(uint8_t commander)
{
	
	switch(commander)
				{
					
					case DEAL_DATA_AS_SPEED_IN_LOCAL_COORDINATE_SYSTEM:
					{
						
						change_mode_flag=0x01;
						run_mode=DEAL_DATA_AS_SPEED_IN_LOCAL_COORDINATE_SYSTEM;
						
					}break;
					
					case DEAL_DATA_AS_SPEED_IN_WORLD_COORDINATE_SYSTEM:
					{
						
						change_mode_flag=0x02;
						run_mode=DEAL_DATA_AS_SPEED_IN_WORLD_COORDINATE_SYSTEM;
						
					}break;
					
					case DEAL_DATA_AS_DISPLACEMENT:
					{
						change_mode_flag=0x04;
						run_mode=DEAL_DATA_AS_DISPLACEMENT;
						
					}break;
					
					case DATA_REQUEST:
					{
						
						tx_buffer[0]=motor_can1[0].speed_rpm;
						tx_buffer[1]=motor_can1[0].speed_rpm >> 8;
						tx_buffer[2]=motor_can1[1].speed_rpm;
						tx_buffer[3]=motor_can1[1].speed_rpm >> 8;
						tx_buffer[4]=motor_can1[2].speed_rpm;
						tx_buffer[5]=motor_can1[2].speed_rpm >> 8;
						
						HAL_UART_Transmit(&huart1,tx_buffer,6,10);
						step=0;
						break;
						
					}break;
					
					case MISTAKE_FROM_CRC:
					{
						
						
						
					}break;
					
					case CIRCUIT_SEND_ENABLE:
					{
						
						msg_mode=CIRCUIT_SEND_ENABLE;
						
					}break;
					
					case CIRCUIT_SEND_DISABLE:
					{
						
						msg_mode=CIRCUIT_SEND_DISABLE;
						
					}break;
					
					case CYBER_GEAR_STEP_ONE:
					{
						
						cyber_gear_knock_flag=1;
						
					}break;
					
					case CYBER_GEAR_STEP_TWO:
					{
						
						cyber_gear_knock_flag=2;
						
					}break;
					
					case DELTA_ENABLE:
					{
						
						delta_mode=1;
						
					}break;
					
					case  DELTA_DISABLE:
					{
						
						delta_mode=0;
						
					}break;
					
					case DELTA_RISE:
					{
						delta_mode=2;
						
					}break;
					
					case DELTA_FALL:
					{
						delta_mode=3;
						
					}
					
					default:break;
					
				}
	
}
void UARTFunction::gyro_position_filter(angle_measure*this)
{
	
	static float filter_buffer_yaw[3]={0};
	filter_buffer_yaw[2]=filter_buffer_yaw[1];
	filter_buffer_yaw[1]=filter_buffer_yaw[0];
	filter_buffer_yaw[0]=this->yaw;
	
	this->filtered_yaw = (2*filter_buffer_yaw[0]+filter_buffer_yaw[1]+filter_buffer_yaw[2])/4;	
	
}

void UARTFunction::gyro_omega_filter(angle_measure*this)
{
	
	static float filter_buffer_yaw_omega[3]={0};
	filter_buffer_yaw_omega[2]=filter_buffer_yaw_omega[1];
	filter_buffer_yaw_omega[1]=filter_buffer_yaw_omega[0];
	filter_buffer_yaw_omega[0]=this->yaw_omega;
	
	this->filtered_yaw_omega = (2*filter_buffer_yaw_omega[0]+filter_buffer_yaw_omega[1]+filter_buffer_yaw_omega[2])/4;
	
}

void UARTFunction::gyro_acc_filter(angle_measure*this)
{
	
	static float filtered_buffer_xacc[3]={0};
	filtered_buffer_xacc[2]=filtered_buffer_xacc[1];
	filtered_buffer_xacc[1]=filtered_buffer_xacc[0];
	filtered_buffer_xacc[0]=this->x_acc;
	this->filtered_x_acc=(2*filtered_buffer_xacc[0]+filtered_buffer_xacc[1]+filtered_buffer_xacc[0])/4;
	
	static float filtered_buffer_yacc[3]={0};
	filtered_buffer_yacc[2]=filtered_buffer_yacc[1];
	filtered_buffer_yacc[1]=filtered_buffer_yacc[0];
	filtered_buffer_yacc[0]=this->y_acc;
	this->filtered_y_acc=(2*filtered_buffer_yacc[0]+filtered_buffer_yacc[1]+filtered_buffer_yacc[0])/4;
	
	static float filtered_buffer_zacc[3]={0};
	filtered_buffer_zacc[2]=filtered_buffer_zacc[1];
	filtered_buffer_zacc[1]=filtered_buffer_zacc[0];
	filtered_buffer_zacc[0]=this->z_acc;
	this->filtered_z_acc=(2*filtered_buffer_zacc[0]+filtered_buffer_zacc[1]+filtered_buffer_zacc[0])/4;
	
}
