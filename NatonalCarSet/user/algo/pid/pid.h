#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float err;
    float last_err;

    float integral;
    float max_integral;         /* 积分限幅 */
    float max_out;              /* 输出限幅 */
    float integral_sep_thresh;  /* 积分分离阈值: |err|>此值时清零积分 */

    float d_filtered;           /* 微分项低通滤波值 */
    float d_alpha;              /* 微分滤波系数 (0~1, 越小滤波越强) */

    float out;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d,
              float max_out, float max_i, float sep_thresh, float d_alpha);
float PID_Calc(PID_TypeDef *pid, float target, float measured);
void PID_Clear(PID_TypeDef *pid);

#endif
