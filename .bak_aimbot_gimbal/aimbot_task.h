#ifndef _AIMBOT_H_
#define _AIMBOT_H_

#include "main.h"
#include "struct_typedef.h"

#define AIMBOT_CONTROL_TIME 100
#define AIMBOT_AUTO_FIRE_THRE   20.0f

// ===== 调试开关: 1=旁路视觉, 用固定目标测下位机环路; 调试完改回 0 =====
#define AIMBOT_DEBUG_BYPASS   0

// ===== 调试: 1=冻结pitch目标(力矩恒为0), 只让视觉给yaw; 测完改回 0 =====
#define AIMBOT_DEBUG_FREEZE_PITCH  1
// ===== 调试: 1=同样冻结yaw目标(两轴都纹丝不动); 测完改回 0 =====
#define AIMBOT_DEBUG_FREEZE_YAW    1
#define AIMBOT_DEBUG_YAW      0.0f
#define AIMBOT_DEBUG_PITCH    -0.2f

typedef enum
{
    AIMBOT_OFF = 0,
    AIMBOT_ON = 1,
    AIMBOT_RESET = 2,   // reset_tracker

    AIMBOT_GET = 3,
    AIMBOT_OUT = 4,
} aimbot_mode_e;

typedef struct
{
    uint8_t aimbot_mode;
    uint8_t aimbot_state;

    fp32 target_yaw;
    fp32 target_pitch;
} aimbot_task_t;

extern void aimbot_mode_set(uint8_t tracker_state);
extern void aimbot_task(void const *pvParameters);
extern void aimbot_init(aimbot_task_t *aimbot_task_init);
extern const aimbot_task_t *get_aimbot_point(void);

#endif
