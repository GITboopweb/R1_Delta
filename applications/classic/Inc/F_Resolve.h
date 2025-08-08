#ifndef F_RESOLVE_H
#define F_RESOLVE_H
#define PI 3.1415926f

typedef struct {
	float v_x;
	float v_y;
}velocity;

extern float vl_wheel_ave[4] ;//����ٶ�����
extern float offset_angle;//�Ƕ�ƫ����
class VelocityResolver {
public:
   extern void set_velocity(velocity *tool_v, float x, float y);
extern void set_vl_omega(float vl_omega, float val);
extern void set_vl_r(float vl_r, float val);
extern void set_offset_angle(float angle, float val);

extern void set_vl_wheel(float *w_arr,velocity v,float omega,float r);

};

extern VelocityResolver velocity_resolver;

extern velocity tool_v;
extern velocity *ptr;





#endif
