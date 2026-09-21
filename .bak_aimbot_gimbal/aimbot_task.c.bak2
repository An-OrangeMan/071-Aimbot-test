#include "aimbot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "referee.h"
#include <math.h>

// 自瞄任务周期 (ms)
#define AIMBOT_TASK_TIME  40

aimbot_task_t aimbot;

// 调试: 判断 INS_quat 的真实顺序. 与 INS_angle[0] 对比, 哪个相等就是哪个
fp32 aimbot_dbg_yaw_wxyz = 0.0f;	// 假设 INS_quat = [w,x,y,z]
fp32 aimbot_dbg_yaw_xyzw = 0.0f;	// 假设 INS_quat = [x,y,z,w]

// 自瞄任务
void aimbot_task(void const *pvParameters)
{
    vTaskDelay(1000);
    aimbot_init(&aimbot);
    while (1)
    {
        const vision_cmd_t *vc = get_vision_cmd_point();
        const fp32 *quat = get_INS_quat_point();

        // 视觉下发 control=1 时接管目标角度; 否则退出自瞄
        if (vc->control)
        {
#if AIMBOT_DEBUG_FREEZE_YAW
            // 调试: yaw 目标 = 当前角度, yaw 轴不出力
            aimbot.target_yaw   = get_INS_angle_point()[YAW_CHANNEL];
#else
            aimbot.target_yaw   = vc->yaw;
#endif
            // 视觉 pitch 与 INS_angle[1] 同为"上为正", 直接用, 不取反
#if AIMBOT_DEBUG_FREEZE_PITCH
            // 调试: pitch 目标 = 当前角度, 误差恒为0, pitch 轴不出力
            aimbot.target_pitch = get_INS_angle_point()[PITCH_CHANNEL];
#else
            aimbot.target_pitch = vc->pitch;
#endif
            aimbot.aimbot_state = AIMBOT_GET;
        }
        else
        {
            aimbot.aimbot_state = AIMBOT_OUT;
        }

        // 上报四元数 [x,y,z,w] 给视觉做姿态/EKF
#if AIMBOT_DEBUG_BYPASS
        // 调试: 强制固定目标, 完全忽略视觉下发的 yaw/pitch
        aimbot.target_yaw   = AIMBOT_DEBUG_YAW;
        aimbot.target_pitch = AIMBOT_DEBUG_PITCH;
        aimbot.aimbot_state = AIMBOT_GET;
#endif
        // 调试: 两种四元数顺序各解一次 yaw
        aimbot_dbg_yaw_wxyz = atan2f(2.0f*(quat[0]*quat[3] + quat[1]*quat[2]), 1.0f - 2.0f*(quat[2]*quat[2] + quat[3]*quat[3]));
        aimbot_dbg_yaw_xyzw = atan2f(2.0f*(quat[3]*quat[2] + quat[0]*quat[1]), 1.0f - 2.0f*(quat[1]*quat[1] + quat[2]*quat[2]));

        CAN_cmd_vision_quat(quat);

        // 上报弹速/模式反馈给视觉做弹道解算
//        CAN_cmd_vision_feedback(get_bullet_speed(), aimbot.aimbot_mode, 0, 0.0f);
				CAN_cmd_vision_feedback(25.0f, 1, 0, 0.0f);  

        vTaskDelay(AIMBOT_TASK_TIME);
    }
}

void aimbot_init(aimbot_task_t *aimbot_task_init)
{
    aimbot_task_init->aimbot_mode = AIMBOT_OFF;
    aimbot_task_init->aimbot_state = AIMBOT_OUT;
    aimbot_task_init->target_yaw = 0.0f;
    aimbot_task_init->target_pitch = 0.0f;
}

void aimbot_mode_set(uint8_t aimbot_mode)
{
    aimbot.aimbot_mode = aimbot_mode;
}

const aimbot_task_t *get_aimbot_point(void)
{
    return &aimbot;
}
