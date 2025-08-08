#include "include.h"
#include "automatic_control_promote.h"

SpeedDesigner body_acc_design[3];
SpeedDesigner speed_designer;
float body_feedfowword[3];
float body_accs[3];
SpatiumDesigner xyw_spatium_designer[3];
SpatiumDesigner delta_motion_planner[3];
 VelocityCalculator velocity_calculator;
 SpatiumDesigner spatiunm_designer;
float low_pass_filter(float input, float last_output,float alpha)
{
	
	return alpha*input+(1.0f-alpha)*last_output;
}

SpeedDesigner body_acc_design[3]={0};
float body_feedforword[3]={0};
float body_accs[3]={0};

void SpeedDesigner::vel_designer_init(SpeedDesigner* this,float max_acc)
{
	
	for(uint8_t i=0;i<3;++i)
	{
		
		this[i].max_acc=max_acc;
		
	}
	
}

void SpeedDesigner::designer_update(SpeedDesigner* this)
{
	
	float delta_v = this->expected_vel - this->current_vel;//delta��ʾ�仯���������Ǹ�delta����
    float max_delta = this->max_acc * DT;

    if (fabsf(delta_v) <= max_delta) {
        this->current_vel = this->expected_vel;
        this->current_acc = 0.0f;
    } else {
        float step = (delta_v > 0.0f) ? max_delta : -max_delta;
        this->current_vel += step;
        this->current_acc = step / DT;
    }
	
}

void feedforword_control(SpeedDesigner* this,float *exp_vel)
{
	for(uint8_t i=0;i<3;++i)
	{
		this[i].expected_vel=exp_vel[i];
		 speed_designer.designer_update(&this[i]);
		body_feedforword[i]=this[i].current_acc;
	}
	
	velocity_calculator.matrix_multiply(inverse_solution_matrix,body_feedforword,body_accs);
	
}

SpatiumDesigner xyw_spatium_designer[3]={0};
SpatiumDesigner delta_motion_planner[3]={0};

void SpatiumDesigner::spatium_designer_init(SpatiumDesigner*this,float v_m,float a_m,float j_m)
{
	
	this->v_max=v_m;
	this->a_max=a_m;
	this->j_max=j_m;
	
	this->current_s=0;
	this->current_v=0;
	this->current_a=0;
	this->current_t=0;
	
}

void SpatiumDesigner::xy_spatium_designer_init(SpatiumDesigner*this,float v_m,float a_m,float j_m)
{
	
	for(uint8_t i=0;i<2;++i)
	{

		spatiunm_designer.spatium_designer_init(&this[i],v_m,a_m,j_m);

	}
	
}

void SpatiumDesigner::spatium_designer_set_target(SpatiumDesigner* this,float total_spatium)
{
	
	this->total_spatium=fabsf(total_spatium);
	this->finished=False;
	this->current_s=0;
	this->current_v=0;
	this->current_a=0;
	this->current_t=0;
	this->towards=total_spatium>0? 1:-1;
	
	this->t[0]=this->a_max/this->j_max;//����ٶ�ʱ��
	this->spatium_acc_rise=this->j_max*this->t[0]*this->t[0]*this->t[0]/6.0f;//�Ӽ��ٶ�λ��
	this->t[1]=(this->v_max-this->j_max*DT*DT)/this->a_max;//�ȼ��ٶ�ʱ��
	this->spatium_acc_fall=0.5f*this->j_max*this->t[0]*this->t[0]*this->t[0]+this->a_max*this->t[0]*this->t[1]+0.5f*this->a_max*this->t[0]*this->t[0];//�����ٶ�λ��
	this->spatium_acc_const=0.5f*this->j_max*this->t[0]*this->t[0]*this->t[1]+0.5f*this->a_max*this->t[1]*this->t[1];//�ȱ��ٶ�λ��
	
	
	if(this->total_spatium>2.0f*(this->spatium_acc_rise+this->spatium_acc_const+this->spatium_acc_fall))//v_m���Դﵽ
	{
		this->spatium_vel_const=this->total_spatium-2.0f*(this->spatium_acc_rise+this->spatium_acc_const+this->spatium_acc_fall);//���ٶ�λ��
		this->t[2]=this->spatium_vel_const/this->v_max;//���ٶ�ʱ��
		
		this->velocity_rise_acc_const=0.5f*this->j_max*this->t[0]*this->t[0];//�ȼ��ٶγ��ٶ�
		this->velocity_rise_acc_fall=0.5f*this->j_max*this->t[0]*this->t[0]+this->a_max*this->t[1];//�����ٶ�
		this->velocity_fall_acc_const=this->v_max-0.5f*this->j_max*this->t[0]*this->t[0];//�ȼ��ٶ�
		this->velocity_fall_acc_fall=this->v_max-0.5f*this->j_max*this->t[0]*this->t[0]-this->a_max*this->t[1];//�����ٶ�
	}
	else if(this->total_spatium>2.0f*(this->spatium_acc_rise+this->spatium_acc_fall))/*v_m���ɵ���,��������ʽ�켣*/
	{
		this->spatium_acc_const=0.5f*this->total_spatium-(this->spatium_acc_rise+this->spatium_acc_fall);
		this->t[1]=this->spatium_acc_const/this->a_max;
		this->t[2]=0;
	}
	else//a_m���ɵ�����������͹켣
	{
		this->t[0]=0;
		this->t[1]=sqrt(this->total_spatium/this->a_max);
	}
	
	this->t[6]=4.0f*this->t[0]+2.0f*this->t[1]+this->t[2];
	this->t[5]=3.0f*this->t[0]+2.0f*this->t[1]+this->t[2];
	this->t[4]=3.0f*this->t[0]+this->t[1]+this->t[2];
	this->t[3]=2.0f*this->t[0]+this->t[1]+this->t[2];
	this->t[2]=2.0f*this->t[0]+this->t[1];
	this->t[1]=this->t[0]+this->t[1];
	this->total_t=this->t[0]+this->t[1]+this->t[2]+this->t[3]+this->t[4]+this->t[5]+this->t[6];
}

void SpatiumDesigner::spatium_designer_update(SpatiumDesigner* this)
{
	if(this->finished==True) return;//��׳�Ը���
	
	if(this->current_t<this->t[0])
	{
		this->current_s+=this->j_max*DT*DT*DT/6.0f;
		this->current_v+=0.5f*this->j_max*DT*DT;
		this->current_a+=this->j_max*DT;
	}//�Ӽ��ٶ�
	else if(this->current_t<this->t[1])
	{
		this->current_s+=this->velocity_rise_acc_const*DT+0.5f*this->a_max*DT*DT;
		this->current_v+=this->a_max;
	}//�ȼ��ٶ�
	else if(this->current_t<this->t[2])
	{
		this->current_s+=this->velocity_rise_acc_fall*DT+0.5f*this->a_max*DT*DT-this->j_max*DT*DT*DT/6.0f;
		this->current_v+=(this->a_max*DT-0.5f*this->j_max*DT*DT);
		this->current_a-=this->j_max*DT;
	}//�����ٶ�
	else if(this->current_t<this->t[3])
	{
		this->current_s+=this->v_max*DT;
	}//���ٶ�
	else if(this->current_t<this->t[4])
	{
		this->current_s+=this->v_max*DT-this->j_max*DT*DT*DT/6.0f;
		this->current_v-=0.5f*this->j_max*DT*DT;
		this->current_a-=this->j_max*DT;
	}//�Ӽ��ٶ�
	else if(this->current_t<this->t[5])
	{
		this->current_s+=this->velocity_fall_acc_const*DT-0.5f*this->a_max*DT*DT;
		this->current_v-=this->a_max*DT;
	}//�ȼ��ٶ�
		
	else
	{
		this->current_s+=this->velocity_fall_acc_fall*DT-this->j_max*DT*DT*DT/6.0f;
		this->current_v-=(this->current_a*DT+0.5f*this->j_max*DT*DT);
		this->current_a+=this->j_max*DT;
	}//�����ٶ�
		
	if(this->current_s >= this->total_spatium*0.999f/*�����ݲ�*/ || this->current_t > this->total_t)
	{
		this->current_s = this->total_spatium;
		this->current_v = 0;
		this->current_t = 0;
		this->current_a = 0;
		this->finished=True;
	}
		
	this->v_out=this->towards>0? this->current_v:-this->current_v;
	this->current_t+=DT;
	//s������,�߶ι滮
}

