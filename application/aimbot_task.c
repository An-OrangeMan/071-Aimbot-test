#include "aimbot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "referee.h"

// 自瞄任务运行周期 (ms)
#define AIMBOT_TASK_TIME  40

aimbot_task_t aimbot;



// 任务函数
void aimbot_task(void const *pvParameters)
{
    vTaskDelay(1000);
    aimbot_init(&aimbot);

    static uint8_t freeze_pitch_latched = 0;   // FREEZE_PITCH 调试: 只在刚接管时锁存一次

    while (1)
    {
        const vision_cmd_t *vc = get_vision_cmd_point();
        const fp32 *quat = get_INS_quat_point();

        // 视觉下发 control=1 时接管目标角度; 否则退出接管
        // 注意: 最终的接管判定在 gimbal_set_mode() 里做(需判遥控模式位), 这里只准备目标值
        if (vc->control)
        {
            aimbot.target_yaw = vc->yaw;
            aimbot.target_pitch = -vc->pitch;

            aimbot.aimbot_state = AIMBOT_GET;
        }
        else
        {
            aimbot.aimbot_state = AIMBOT_OUT;
        }


				

        // 上报四元数 [x,y,z,w] 给视觉做姿态/EKF
        CAN_cmd_vision_quat(quat);

        // 上报子弹/模式反馈给视觉下位机
        // mode: cboard 的 enum Mode { idle=0, auto_aim=1 }, 接管才标 auto_aim
        // shoot_mode: ShootMode { left_shoot=0, right_shoot=1, both_shoot=2 }, 手动发弹就标 both_shoot
        // ft_angle: 特定机型专用, 这里置 0 不用
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
