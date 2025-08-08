#include "include.h"
#include "displacement_PID.h"

float motors_rnd[3]={0};
displacement_PID my_displacement_pid;
float displace_buffer[3];

void DisplacementPID::get_rnd_count_and_diaplacement(void)
{
	
	motors_rnd[0]=motor_can1[0].total_angle;
	motors_rnd[1]=motor_can1[1].total_angle;
	motors_rnd[2]=motor_can1[2].total_angle;
	
	motors_rnd[0]=motors_rnd[0]*((2*PI/8192.0)*(187.0/3591.0)*76);
	motors_rnd[1]=motors_rnd[1]*((2*PI/8192.0)*(187.0/3591.0)*76);
	motors_rnd[2]=motors_rnd[2]*((2*PI/8192.0)*(187.0/3591.0)*76);
	
	matrix_multiply(solution_matrix,motors_rnd,displace_buffer);
	displace_buffer[2]=my_yaw;//��������z���滻Ϊ�����ǵĲ���
	
}

void DisplacementPID::rnd_count_and_diaplacement_reset(void)
{
	motors_rnd[0]=0;
	motors_rnd[1]=0;
	motors_rnd[2]=0;
	displace_buffer[0]=0;
	displace_buffer[1]=0;
	displace_buffer[2]=0;
}

float DisplacementPID::angle_error(float target, float current) 
{
    float diff = target - current;
    diff = fmodf(diff + 180.0f, 360.0f);
    if (diff < 0) diff += 360.0f;
    return diff - 180.0f;
}

void DisplacementPID::xy_displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out)
{
	
	my_displacement_pid.xy_kp=kp;
	my_displacement_pid.xy_kd=kd;
	my_displacement_pid.xy_ki=ki;
	my_displacement_pid.xy_max_iout=max_iout;
	my_displacement_pid.xy_max_out=max_out;
	
}

void DisplacementPID::z_displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out)
{
	my_displacement_pid.z_kp=kp;
	my_displacement_pid.z_kd=kd;
	my_displacement_pid.z_ki=ki;
	my_displacement_pid.z_max_iout=max_iout;
	my_displacement_pid.z_max_out=max_out;
}

void DisplacementPID::x_displacement_control(float x)
{
	
	my_displacement_pid.x_error[1]=my_displacement_pid.x_error[0];
	my_displacement_pid.x_error[0]=x-displace_buffer[0];
	
	my_displacement_pid.s_x_error+=my_displacement_pid.x_error[0];
	my_displacement_pid.iout[0]=my_displacement_pid.s_x_error*my_displacement_pid.xy_ki;
	
		if(my_displacement_pid.iout[0]>my_displacement_pid.xy_max_iout)
		{
			
			my_displacement_pid.iout[0]=my_displacement_pid.xy_max_iout;
			
		}
		else if(my_displacement_pid.iout[0]<-my_displacement_pid.xy_max_iout)
		{
			
			my_displacement_pid.iout[0]=-my_displacement_pid.xy_max_iout;
			
		}

	my_displacement_pid.pout[0]=my_displacement_pid.x_error[0]*my_displacement_pid.xy_kp;
	
	my_displacement_pid.dout[0]=my_displacement_pid.xy_kd*(my_displacement_pid.x_error[0]-my_displacement_pid.x_error[1]);
	
	my_displacement_pid.out[0]= my_displacement_pid.pout[0]+my_displacement_pid.dout[0]+my_displacement_pid.iout[0];
	
	
		if(my_displacement_pid.out[0]>my_displacement_pid.xy_max_out)
		{
			
			my_displacement_pid.out[0]=my_displacement_pid.xy_max_out;
			
		}
		
		else if(my_displacement_pid.out[0]<-my_displacement_pid.xy_max_out)
		{
			
			my_displacement_pid.out[0]=-my_displacement_pid.xy_max_out;
			
		}
}

void DisplacementPID::y_displacement_control(float y)
{
	
	my_displacement_pid.y_error[1]=my_displacement_pid.y_error[0];
	my_displacement_pid.y_error[0]=y-displace_buffer[1];
	
	my_displacement_pid.s_y_error+=my_displacement_pid.y_error[0];
	my_displacement_pid.iout[1]=my_displacement_pid.s_y_error*my_displacement_pid.xy_ki;
	
		if(my_displacement_pid.iout[1]>my_displacement_pid.xy_max_iout)
		{
			
			my_displacement_pid.iout[1]=my_displacement_pid.xy_max_iout;
			
		}
		else if(my_displacement_pid.iout[1]<-my_displacement_pid.xy_max_iout)
		{
			
			my_displacement_pid.iout[1]=-my_displacement_pid.xy_max_iout;
			
		}

	my_displacement_pid.pout[1]=my_displacement_pid.y_error[0]*my_displacement_pid.xy_kp;
	
	my_displacement_pid.dout[1]=my_displacement_pid.xy_kd*(my_displacement_pid.y_error[0]-my_displacement_pid.y_error[1]);
	
	my_displacement_pid.out[1]= my_displacement_pid.pout[1]+my_displacement_pid.dout[1]+my_displacement_pid.iout[1];
	
	
		if(my_displacement_pid.out[1]>my_displacement_pid.xy_max_out)
		{
			
			my_displacement_pid.out[1]=my_displacement_pid.xy_max_out;
			
		}
		
		else if(my_displacement_pid.out[1]<-my_displacement_pid.xy_max_out)
		{
			
			my_displacement_pid.out[1]=-my_displacement_pid.xy_max_out;
			
		}
}

void DisplacementPID::theta_displacement_control(float theta)
{
	my_displacement_pid.omega_error[1]=my_displacement_pid.omega_error[0];
	my_displacement_pid.omega_error[0]= /*������һ������*/ angle_error(theta,displace_buffer[2])/*ע��:�����Ƕ����鵽+-180*/ ;
	
	my_displacement_pid.s_omega_error+=my_displacement_pid.omega_error[0];
	my_displacement_pid.iout[2]=my_displacement_pid.s_omega_error*my_displacement_pid.z_ki;
	
	my_displacement_pid.pout[2]=my_displacement_pid.omega_error[0]*my_displacement_pid.z_kp;
	my_displacement_pid.dout[2]=my_displacement_pid.z_kd*(my_displacement_pid.omega_error[0]-my_displacement_pid.omega_error[1]);
	my_displacement_pid.out[2]= my_displacement_pid.pout[2]+my_displacement_pid.dout[2]+my_displacement_pid.iout[2];
	
	if(my_displacement_pid.out[2]>my_displacement_pid.z_max_out)
		{
			
			my_displacement_pid.out[2]=my_displacement_pid.z_max_out;
			
		}
		
		else if(my_displacement_pid.out[2]<-my_displacement_pid.z_max_out)
		{
			
			my_displacement_pid.out[2]=-my_displacement_pid.z_max_out;
			
		}

}
