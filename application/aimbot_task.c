#include "aimbot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "referee.h"

// 自瞄任务周期 (ms)
#define AIMBOT_TASK_TIME  40

aimbot_task_t aimbot;

// 自瞄任务
void aimbot_task(void const *pvParameters)
{
    vTaskDelay(1000);
    aimbot_init(&aimbot);
    static uint8_t freeze_pitch_latched = 0;   // FREEZE_PITCH 用: 只在刚接管时锁存一次

    while (1)
    {
        const vision_cmd_t *vc = get_vision_cmd_point();
        const fp32 *quat = get_INS_quat_point();

        // 视觉下发 control=1 时接管目标角度; 否则退出自瞄
        // 注意: 真正的接管与否由 gimbal_set_mode() 决定(还要开关在自瞄位), 这里只准备目标值
        if (vc->control)
        {
#if AIMBOT_DEBUG_FREEZE_YAW
            // 调试: yaw 目标 = 当前角度, yaw 轴不出力
            aimbot.target_yaw   = get_INS_angle_point()[YAW_CHANNEL];
#else
            aimbot.target_yaw   = vc->yaw;
#endif
            // sp_vision 送来的 pitch 是"向上为负"
            //   (aimer.cpp: pitch = -(traj.pitch + offset); trajectory.hpp: pitch 抬头为正)
            // 而 INS_angle[1] 是"向上为正", 所以必须取反, 否则 pitch 会正反馈往下暴冲
#if AIMBOT_DEBUG_FREEZE_PITCH
            // 调试: 只在刚接管那一刻锁存一次当前角度,之后保持这个绝对目标
            // (原来的写法是个带延迟的负反馈,没有位置刚度,重力会把 pitch 匀速压下去)
            if (!freeze_pitch_latched)
            {
                aimbot.target_pitch = get_INS_angle_point()[PITCH_CHANNEL];
                freeze_pitch_latched = 1;
            }
#else
            aimbot.target_pitch = -vc->pitch;
#endif
            aimbot.aimbot_state = AIMBOT_GET;
        }
        else
        {
            aimbot.aimbot_state = AIMBOT_OUT;
#if AIMBOT_DEBUG_FREEZE_PITCH
            freeze_pitch_latched = 0;   // 视觉松手后重新锁存
#endif
        }

#if AIMBOT_DEBUG_BYPASS
        // 调试: 强制固定目标, 完全忽略视觉下发的 yaw/pitch
        aimbot.target_yaw   = AIMBOT_DEBUG_YAW;
        aimbot.target_pitch = AIMBOT_DEBUG_PITCH;
        aimbot.aimbot_state = AIMBOT_GET;
#endif
				

        // 上报四元数 [x,y,z,w] 给视觉做姿态/EKF
        CAN_cmd_vision_quat(quat);

        // 上报弹速/模式反馈给视觉做弹道解算
        // mode: cboard 的 enum Mode { idle=0, auto_aim=1 }, 自瞄中才报 auto_aim
        // shoot_mode: ShootMode { left_shoot=0, right_shoot=1, both_shoot=2 }, 两轮都转就报 both_shoot
        // ft_angle: 哨兵/无人机专有量, 这里不用
        CAN_cmd_vision_feedback(get_bullet_speed(), (aimbot.aimbot_mode == AIMBOT_ON) ? 1 : 0, 2, 0.0f);

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
