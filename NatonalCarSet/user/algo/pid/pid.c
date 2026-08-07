#include "pid.h"
#include <math.h>

void PID_Init(PID_TypeDef *pid, float p, float i, float d,
              float max_out, float max_i, float sep_thresh, float d_alpha) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_integral = max_i;
    pid->integral_sep_thresh = sep_thresh;
    pid->d_alpha = d_alpha;
    pid->integral = 0;
    pid->last_err = 0;
    pid->d_filtered = 0;
    pid->out = 0;
}

float PID_Calc(PID_TypeDef *pid, float target, float measured) {
    pid->err = target - measured;

    /* 积分分离: 大偏差时清零积分防止超调 */
    if (fabsf(pid->err) > pid->integral_sep_thresh) {
        pid->integral = 0;
    } else {
        pid->integral += pid->err;
        /* 积分限幅 */
        if (pid->integral > pid->max_integral)
            pid->integral = pid->max_integral;
        else if (pid->integral < -pid->max_integral)
            pid->integral = -pid->max_integral;
    }

    /* 微分项带一阶低通滤波，抑制电机转速高频噪声 */
    float d_err = pid->err - pid->last_err;
    pid->d_filtered = pid->d_alpha * d_err + (1.0f - pid->d_alpha) * pid->d_filtered;

    pid->out = pid->Kp * pid->err +
               pid->Ki * pid->integral +
               pid->Kd * pid->d_filtered;

    pid->last_err = pid->err;

    /* 输出限幅 (3508通过C620控制，电流最大值为16384) */
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    else if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    return pid->out;
}

void PID_Clear(PID_TypeDef *pid) {
    pid->integral = 0;
    pid->last_err = 0;
    pid->d_filtered = 0;
    pid->out = 0;
}
