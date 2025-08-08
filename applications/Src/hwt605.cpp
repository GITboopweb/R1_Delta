#include "include.h"
#include "hwt605.h"

float my_yaw=0;
uint8_t hwt_tx_buffer[5]={0};
angle_PID_config angular_displacement_PID_config={30,0,0,200,2800};//kp,ki,kd,max_iout,max_out//184输出对应1rad/s的角速度
angle_PID_config angular_velocity_PID_config={40,0,0,400,1200};
angle_PID_struct angular_displacement_PID_struct={0};
angle_PID_struct angular_velocity_PID_struct={0};
//破陀螺仪调零要发两条命令，还要原地死等100ms，而且九轴算法根本不能清理我真正想要的清理的y轴角量.所以这里的角度参考指令被我删了。

float angular_displacement_PID(float angular_displacement)
{
	angular_displacement_PID_struct.error[1]=angular_displacement_PID_struct.error[0];
	angular_displacement_PID_struct.error[0]=angular_displacement-my_angle_measure.yaw;
	
	angular_displacement_PID_struct.se+=angular_displacement_PID_struct.error[0];
	angular_displacement_PID_struct.de=angular_displacement_PID_struct.error[0]-angular_displacement_PID_struct.error[1];
	
	angular_displacement_PID_struct.dout=angular_displacement_PID_config.kd*angular_displacement_PID_struct.de;
	angular_displacement_PID_struct.iout=angular_displacement_PID_config.ki*angular_displacement_PID_struct.se;
	angular_displacement_PID_struct.pout=angular_displacement_PID_config.kp*angular_displacement_PID_struct.error[0];
	
	
	if(angular_displacement_PID_struct.iout>angular_displacement_PID_config.max_iout)
	{
		
		angular_displacement_PID_struct.iout=angular_displacement_PID_config.max_iout;
		
	}
	
	else if(angular_displacement_PID_struct.iout<-angular_displacement_PID_config.max_iout)
	{
		
		angular_displacement_PID_struct.iout=-angular_displacement_PID_config.max_iout;
		
	}
	
	angular_displacement_PID_struct.out=(angular_displacement_PID_struct.pout+angular_displacement_PID_struct.dout+angular_displacement_PID_struct.iout);
	
	if(angular_displacement_PID_struct.out>angular_displacement_PID_config.max_out)
	{
		
		angular_displacement_PID_struct.out=angular_displacement_PID_config.max_out;
		
	}
	
	else if(angular_displacement_PID_struct.out<-angular_displacement_PID_config.max_out)
	{
		
		angular_displacement_PID_struct.out=-angular_displacement_PID_config.max_out;
		
	}
	
	return angular_displacement_PID_struct.out;
	
}

float angular_velocity_PID(float angular_velocity)
{
	
	angular_velocity_PID_struct.error[1]=angular_velocity_PID_struct.error[0];
	angular_velocity_PID_struct.error[0]=angular_velocity-my_angle_measure.yaw_omega;
	
	angular_velocity_PID_struct.se+=angular_velocity_PID_struct.error[0];
	angular_velocity_PID_struct.de=angular_velocity_PID_struct.error[0]-angular_velocity_PID_struct.error[1];
	
	angular_velocity_PID_struct.dout=angular_velocity_PID_config.kd*angular_velocity_PID_struct.de;
	angular_velocity_PID_struct.iout=angular_velocity_PID_config.ki*angular_velocity_PID_struct.se;
	angular_velocity_PID_struct.pout=angular_velocity_PID_config.kp*angular_velocity_PID_struct.error[0];
	
	if(angular_velocity_PID_struct.iout>angular_velocity_PID_config.max_iout)
	{
		
		angular_velocity_PID_struct.iout=angular_velocity_PID_config.max_iout;
		
	}
	
	else if(angular_velocity_PID_struct.iout<-angular_velocity_PID_config.max_iout)
	{
		
		angular_velocity_PID_struct.iout=-angular_velocity_PID_config.max_iout;
		
	}
	
	angular_velocity_PID_struct.out=(angular_velocity_PID_struct.pout+angular_velocity_PID_struct.dout+angular_velocity_PID_struct.iout);
	
	if(angular_velocity_PID_struct.out>angular_velocity_PID_config.max_out)
	{
		
		angular_velocity_PID_struct.out=angular_velocity_PID_config.max_out;
		
	}
	
	else if(angular_velocity_PID_struct.out<-angular_velocity_PID_config.max_out)
	{
		
		angular_velocity_PID_struct.out=-angular_velocity_PID_config.max_out;
		
	}
	
	return angular_velocity_PID_struct.out;
	
}

float hwt_angle_to_radian(float angle)
{
	
	return((angle*2*3.1415926535)/180);
	
}

void yaw_updater(void)
{
	static float last_yaw=0;
	if(fabsf(last_yaw-my_angle_measure.yaw)>0.1)
	{
		my_yaw=0.8f*my_angle_measure.yaw+0.2f*my_yaw;
	}
	else 
	{
		my_yaw+=my_angle_measure.filtered_yaw_omega*DT;
	}
	last_yaw=my_angle_measure.yaw;
}
