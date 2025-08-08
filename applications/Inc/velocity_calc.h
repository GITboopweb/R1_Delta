#ifndef MATRIX_CALC_H
#define MATRIX_CALC_H
#ifdef __cplusplus
extern "C" {
#endif
class VelocityCalculator {
public:
    static constexpr float RADIUS = 1.0f;
    static constexpr float MAX_VEL = 8500.0f;
    
    static float solution_matrix[3][3];
    static float inverse_solution_matrix[3][3];
    static float rotate_matrix[3][3];
    static float inverse_transform_matrix[3][3];
    static float calculated_velocity[3];
    static float calc_buffer[3];
    static float theta;
    static float calc_buffer_2nd[3];
    
    static void inverse_solution_matrix_init();
    static void solution_matrix_init();
    static void matrix_multiply(float matrix[3][3], float organized_vector[3], float solution_vector[3]);
    static void rotate_matrix_calc(float theta);
    static void get_theta(float omega, float dt);
    static void vel_control(float exp_vel[3]);
};
#ifdef __cplusplus
}
#endif
#endif