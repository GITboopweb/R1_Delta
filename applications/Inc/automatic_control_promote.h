#ifndef FILTER_AND_OBSERVER_H
#define FILTER_AND_OBSERVER_H
#ifdef __cplusplus
extern "C" {
#endif
#define DT 0.001f  // 采样周期为1ms

enum MyBool {
    False = 0,
    True
};

class StateObserver {
public:
    int temp;
};

class SpeedDesigner {
public:
    float expected_vel;
    float current_vel;
    float max_acc;
    float current_acc;
    
   void vel_designer_init(SpeedDesigner* this,float max_acc);

	void designer_update(SpeedDesigner* this);
};

class SpatiumDesigner {
public:
    float total_spatium;
    float v_max;
    float a_max;
    float j_max;
    float expected_acc;
    float expected_vel;
    float velocity_rise_acc_const;  // 上升加速度段
    float velocity_rise_acc_fall;   // 上升减速度段
    float velocity_fall_acc_const;  // 下降加速度段
    float velocity_fall_acc_fall;   // 下降减速度段
    float spatium_acc_rise;         // 加速度上升段位移
    float spatium_acc_fall;         // 加速度下降段位移
    float spatium_acc_const;        // 恒定加速度段位移
    float spatium_vel_const;        // 恒定速度段位移
    float current_a;
    float current_v;
    float current_s;
    float total_t;
    float t[7];     // 7段时间
    float current_t;
    int8_t towards; 
    float v_out;
    MyBool finished;
    void spatium_designer_init(SpatiumDesigner*this,float v_m,float a_m,float j_m);

    void xy_spatium_designer_init(SpatiumDesigner*this,float v_m,float a_m,float j_m);
   void spatium_designer_set_target(SpatiumDesigner* this,float total_spatium);
    void spatium_designer_update(SpatiumDesigner* this);
};

// 全局实例声明
extern SpeedDesigner body_acc_design[3];
extern float body_feedfowword[3];
extern float body_accs[3];
extern SpatiumDesigner xyw_spatium_designer[3];
extern SpatiumDesigner delta_motion_planner[3];

// 函数声明
float low_pass_filter(float input, float last_output,float alpha);
void vel_designer_init(SpeedDesigner* this,float max_acc);
void designer_update(SpeedDesigner* this);
void feedforword_control(SpeedDesigner* this,float *exp_vel);
#ifdef __cplusplus
}
#endif
#endif