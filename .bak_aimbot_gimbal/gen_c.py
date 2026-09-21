# -*- coding: utf-8 -*-
# 生成 application/aimbot_gimbal.c
# 中文全部写在 UTF-8 的 .py 里，输出时 encode('gbk')，不会出现 U+FFFD
# 原样搬运的函数用 grab() 从原 gimbal_task.c 解码文本里按函数签名抓取，注释字节级一致

import io, sys

SRC = 'application/gimbal_task.c'
DST = 'application/aimbot_gimbal.c'

raw = open(SRC, 'rb').read().decode('gbk')
lines = raw.replace('\r\n', '\n').split('\n')

def grab(sig):
    """从函数签名行抓到第 0 列的 } 为止（原文件函数都是全下风格，} 顶格）"""
    start = None
    for i, l in enumerate(lines):
        if l.startswith(sig):
            start = i
            break
    if start is None:
        sys.exit('NOT FOUND: ' + sig)
    for j in range(start + 1, len(lines)):
        if lines[j] == '}':
            return '\n'.join(lines[start:j + 1])
    sys.exit('NO CLOSE: ' + sig)

def flt(name):
    """抓取 static 变量声明行"""
    for l in lines:
        if l.startswith(name):
            return l
    sys.exit('NOT FOUND: ' + name)

P = []

P.append('#include "user_lib.h"')
P.append('#include "aimbot_gimbal.h"')
P.append('#include "CAN_receive.h"')
P.append('#include "Chassis_task.h"')
P.append('#include <math.h>')
P.append('#include "cmsis_os.h"')
P.append('#include "arm_math.h"')
P.append('#include "INS_task.h"')
P.append('#include "hatch.h"')
P.append('#include "bsp_laser.h"')
P.append('')
P.append('#define FRIC_INSURANCE_MOUSE\t\t1')
P.append('#define FRIC_INSURANCE_RC\t\t\t100')
P.append('#define AIMBOT_INSURANCE\t1')
P.append('')
P.append('gimbal_control_t gimbal_control;')
P.append('')

# ---------- 工具函数 ----------
P.append('''
//电流值下发前限幅, 防止超出 int16 范围回绕反号
static int16_t gimbal_current_limit(fp32 current)
{
\tif(current > 32767.0f)
\t\treturn 32767;
\telse if(current < -32768.0f)
\t\treturn -32768;
\telse
\t\treturn (int16_t)current;
}
''')

P.append('''/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */''')
P.append(grab('void gimbal_PID_init('))
P.append('')
P.append('//云台pid计算函数')
P.append(grab('fp32 gimbal_PID_calc('))
P.append('')
P.append(grab('void gimbal_limit('))
P.append('')

# ---------- 主任务 ----------
P.append('//gimbal主要运行程序')
P.append('''void gimbal_task(void const *pvParameters)
{
\tvTaskDelay(GIMBAL_TASK_INIT_TIME);
\t
\tgimbal_init(&gimbal_control);
\t
\twhile(1)
\t{
\t\t//云台模式设定
\t\tgimbal_set_mode(&gimbal_control);
\t\t//云台数据获取
\t\tgimbal_feedback_update(&gimbal_control);
\t\tgimbal_set_control(&gimbal_control);
\t\tgimbal_control_loop(&gimbal_control);
\t\t
\t\t//无力模式
\t\tif(gimbal_control.gimbal_mode == CHASSIS_NO_MOVE)
\t\t{
\t\t\tCAN_cmd_gimbal(0, 0);
\t\t}
\t\t//手动或视觉自瞄模式，给云台赋予电流
\t\telse
\t\t{
\t\t\tCAN_cmd_gimbal(gimbal_current_limit(gimbal_control.gimbal_yaw_motor.give_current), gimbal_current_limit(gimbal_control.gimbal_pitch_motor.give_current));
\t\t}
\t\t//发射机构不受云台模式牵连，拨到无力位摩擦轮照样能转
\t\tCAN_cmd_shoot(gimbal_current_limit(gimbal_control.gimbal_trigger_motor.give_current), gimbal_current_limit(gimbal_control.gimbal_fric_motor[0].give_current), gimbal_current_limit(gimbal_control.gimbal_fric_motor[1].give_current));
\t\t
\t\tvTaskDelay(GIMBAL_CONTROL_TIME);
\t}
}''')
P.append('')

# ---------- 初始化 ----------
P.append('''void gimbal_init(gimbal_control_t *gimbal_task_init)
{
\tif (gimbal_task_init == NULL)
\t{
\t\treturn;
\t}
\t
\tshoot_init();
\t//起始值赋0
\tgimbal_task_init->yaw = 0;
\tgimbal_task_init->aimbot_mode = AIMBOT_OFF;
\t//云台、底盘初始化无力
\tgimbal_task_init->gimbal_mode = CHASSIS_NO_MOVE;
\tgimbal_task_init->shoot_switch.fric_flag = FRIC_READY;
\tgimbal_task_init->shoot_switch.fric_mode = FRIC_STOP;\t//发射机构初始化无力
\tgimbal_task_init->shoot_switch.shoot_mode = SHOOT_STOP;
\tgimbal_task_init->shoot_switch.shoot_state = SHOOT_SINGLE_START_STATE;
\tgimbal_task_init->hatch_state = HATCH_CLOSE;
\t
\t//陀螺仪数据指针获取
\tgimbal_task_init->gimbal_INT_angle_point = get_INS_angle_point();
\tgimbal_task_init->gimbal_INT_gyro_point = get_gyro_data_point();
\t//获取自瞄数据指针
\tgimbal_task_init->gimbal_aimbot_point = get_aimbot_point();
\t//遥控器数据指针获取
\tgimbal_task_init->gimbal_rc_ctrl = get_remote_control_point();
\t//电机数据指针获取
\tgimbal_task_init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(0);
\tgimbal_task_init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(1);
\tgimbal_task_init->gimbal_trigger_motor.shoot_motor_measure = get_shoot_motor_measure_point(0);
\tgimbal_task_init->gimbal_fric_motor[0].shoot_motor_measure = get_shoot_motor_measure_point(1);
\tgimbal_task_init->gimbal_fric_motor[1].shoot_motor_measure = get_shoot_motor_measure_point(2);
\t
\t//初始化云台电机PID参数
\tconst static fp32 aimbot_speed_pitch_pid[3] = {AIMBOT_PITCH_SPEED_PID_KP, AIMBOT_PITCH_SPEED_PID_KI, AIMBOT_PITCH_SPEED_PID_KD};
\tconst static fp32 aimbot_speed_yaw_pid[3] = {AIMBOT_YAW_SPEED_PID_KP, AIMBOT_YAW_SPEED_PID_KI, AIMBOT_YAW_SPEED_PID_KD};
\tconst static fp32 gimbal_speed_yaw_ins_pid[3] = {YAW_INS_SPEED_PID_KP, YAW_INS_SPEED_PID_KI, YAW_INS_SPEED_PID_KD};
\tconst static fp32 gimbal_speed_pitch_ins_pid[3] = {PITCH_INS_SPEED_PID_KP, PITCH_INS_SPEED_PID_KI, PITCH_INS_SPEED_PID_KD};
\tconst static fp32 gimbal_speed_trigger_pid[3] = {TRIGGER_ENCODE_SPEED_PID_KP, TRIGGER_ENCODE_SPEED_PID_KI, TRIGGER_ENCODE_SPEED_PID_KD};
\tconst static fp32 gimbal_speed_fric_pid[3] = {FRIC_ENCODE_SPEED_PID_KP, FRIC_ENCODE_SPEED_PID_KI, FRIC_ENCODE_SPEED_PID_KD};
\t//初始化自瞄PID
\tgimbal_PID_init(&gimbal_task_init->aimbot_angle_pitch_pid, AIMBOT_PITCH_ANGLE_PID_MAX_OUT, AIMBOT_PITCH_ANGLE_PID_MAX_IOUT, AIMBOT_PITCH_ANGLE_PID_KP, AIMBOT_PITCH_ANGLE_PID_KI, AIMBOT_PITCH_ANGLE_PID_KD);
\tPID_init(&gimbal_task_init->aimbot_speed_pitch_pid, PID_POSITION, aimbot_speed_pitch_pid, AIMBOT_PITCH_SPEED_PID_MAX_OUT, AIMBOT_PITCH_SPEED_PID_MAX_IOUT);
\tgimbal_PID_init(&gimbal_task_init->aimbot_angle_yaw_pid, AIMBOT_YAW_ANGLE_PID_MAX_OUT, AIMBOT_YAW_ANGLE_PID_MAX_IOUT, AIMBOT_YAW_ANGLE_PID_KP, AIMBOT_YAW_ANGLE_PID_KI, AIMBOT_YAW_ANGLE_PID_KD);
\tPID_init(&gimbal_task_init->aimbot_speed_yaw_pid, PID_POSITION, aimbot_speed_yaw_pid, AIMBOT_YAW_SPEED_PID_MAX_OUT, AIMBOT_YAW_SPEED_PID_MAX_IOUT);
\t//初始化陀螺仪yaw电机PID
\tgimbal_PID_init(&gimbal_task_init->gimbal_angle_yaw_ins_pid, YAW_INS_ANGLE_PID_MAX_OUT, YAW_INS_ANGLE_PID_MAX_IOUT, YAW_INS_ANGLE_PID_KP, YAW_INS_ANGLE_PID_KI, YAW_INS_ANGLE_PID_KD);
\tPID_init(&gimbal_task_init->gimbal_speed_yaw_ins_pid, PID_POSITION, gimbal_speed_yaw_ins_pid, YAW_INS_SPEED_PID_MAX_OUT, YAW_INS_SPEED_PID_MAX_IOUT);
\t//初始化陀螺仪pitch电机PID
\tgimbal_PID_init(&gimbal_task_init->gimbal_angle_pitch_ins_pid, PITCH_INS_ANGLE_PID_MAX_OUT, PITCH_INS_ANGLE_PID_MAX_IOUT, PITCH_INS_ANGLE_PID_KP, PITCH_INS_ANGLE_PID_KI, PITCH_INS_ANGLE_PID_KD);
\tPID_init(&gimbal_task_init->gimbal_speed_pitch_ins_pid, PID_POSITION, gimbal_speed_pitch_ins_pid, PITCH_INS_SPEED_PID_MAX_OUT, PITCH_INS_SPEED_PID_MAX_IOUT);
\t//初始化trigger电机PID
\tPID_init(&gimbal_task_init->gimbal_speed_trigger_pid, PID_POSITION, gimbal_speed_trigger_pid, TRIGGER_ENCODE_SPEED_PID_MAX_OUT, TRIGGER_ENCODE_SPEED_PID_MAX_IOUT);
\t//初始化摩擦轮电机PID
\tfor(int i=0; i<2; i++)
\t\tPID_init(&gimbal_task_init->gimbal_speed_fric_pid[i], PID_POSITION, gimbal_speed_fric_pid, FRIC_ENCODE_SPEED_PID_MAX_OUT, FRIC_ENCODE_SPEED_PID_MAX_IOUT);
\t//初始化舱门PWM
\thatch_init();
\t//初始化陀螺仪模式起始值（需要加一个PI，因为起始值默认有一个-PI）
\tgimbal_task_init->ins_data.yaw_target = gimbal_task_init->ins_data.yaw+PI;
\tgimbal_task_init->ins_data.pitch_target = gimbal_task_init->ins_data.pitch;
\t//初始化激光
\tlaser_init();
}''')
P.append('')

# ---------- 模式 ----------
P.append('''void gimbal_set_mode(gimbal_control_t *gimbal_mode_set)
{
\tif (gimbal_mode_set == NULL)
\t{
\t\treturn;
\t}
\t//底盘模式切换: CH1拨到下=无力, 其余组合全部=手动(底盘跟随云台, 陀螺仪控制)
\tif (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]))
\t{
\t\t//底盘无力
\t\tgimbal_mode_set->gimbal_mode = CHASSIS_NO_MOVE;
\t}
\telse
\t{
\t\t//底盘跟随云台, 陀螺仪控制
\t\tgimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
\t}
\t
\t//自瞄权限: 开关必须在自瞄位(CH1上+CH2上), 并且视觉要求接管(control=1), 两个条件同时成立才交权
\t//遥控器任何时候把开关拨走就能拿回控制权
\tif (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]) &&
\t\tswitch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]) &&
\t\tget_vision_cmd_point()->control)
\t{
\t\tgimbal_mode_set->aimbot_mode = AIMBOT_ON;
\t}
\telse
\t{
\t\tgimbal_mode_set->aimbot_mode = AIMBOT_OFF;
\t}
\t// 调试: 无条件强制进入自瞄模式
#if AIMBOT_DEBUG_BYPASS
\tgimbal_mode_set->aimbot_mode = AIMBOT_ON;
#endif
\t//把自瞄模式同步给自瞄任务, 供 0x101 反馈帧的 mode 字节使用(只有这一个地方决定自瞄)
\taimbot_mode_set(gimbal_mode_set->aimbot_mode);
\t
\t//发射机构模式切换
\t//获得拨弹开关编码器值
\tgimbal_mode_set->shoot_switch.encoding_switch_ecd = gimbal_mode_set->gimbal_rc_ctrl->rc.ch[SHOOT_MODE_CHANNEL];
\t
\tif(gimbal_mode_set->gimbal_mode == CHASSIS_NO_MOVE)
\t{
\t\t//摩擦轮停止
\t\tgimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
\t\tgimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
\t\t//拨弹轮停止
\t\tgimbal_mode_set->shoot_switch.shoot_mode = SHOOT_STOP;
\t}
\t
\tstatic uint16_t shoot_i;''')
P.append('''\tif((gimbal_mode_set->shoot_switch.fric_mode == FRIC_STOP) && (gimbal_mode_set->shoot_switch.fric_flag == FRIC_READY) && (gimbal_mode_set->shoot_switch.encoding_switch_ecd<=-400))
\t{
\t\tshoot_i++;
\t\tif(shoot_i >= FRIC_INSURANCE_RC)
\t\t{
\t\t\tgimbal_mode_set->shoot_switch.fric_mode = FRIC_START;
\t\t\tgimbal_mode_set->shoot_switch.fric_flag = FRIC_START;
\t\t\tshoot_i=0;
\t\t}
\t}
\telse if((gimbal_mode_set->shoot_switch.fric_mode == FRIC_START) && (gimbal_mode_set->shoot_switch.fric_flag == FRIC_READY) && (gimbal_mode_set->shoot_switch.encoding_switch_ecd<=-400))
\t{
\t\tgimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
\t\tgimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
\t}
\telse if(ABS(gimbal_mode_set->shoot_switch.encoding_switch_ecd)<10)
\t{
\t\tgimbal_mode_set->shoot_switch.fric_flag = FRIC_READY;
\t}
\t//鼠标开启关闭摩擦轮
\tif(gimbal_mode_set->shoot_switch.fric_mode == FRIC_STOP && (gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 1))
\t{
\t\tshoot_i++;
\t\tif(shoot_i>= FRIC_INSURANCE_MOUSE)
\t\t{
\t\t\tgimbal_mode_set->shoot_switch.fric_mode = FRIC_START;
\t\t\tgimbal_mode_set->shoot_switch.fric_flag = FRIC_START;
\t\t}
\t}
\telse if(gimbal_mode_set->shoot_switch.fric_mode == FRIC_START && gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].e == 1)
\t{
\t\tgimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
\t\tgimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
\t\tshoot_i=0;
\t}
\t
\t//拨弹轮控制
\tif(gimbal_mode_set->shoot_switch.encoding_switch_ecd>200&&gimbal_mode_set->shoot_switch.encoding_switch_ecd<450)\t//遥控器编码轮
\t\tgimbal_mode_set->shoot_switch.shoot_mode = SHOOT_SINGLE_START;
\telse if(gimbal_mode_set->shoot_switch.encoding_switch_ecd>450)
\t\tgimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;
\telse if(gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 1&&fabs(gimbal_mode_set->gimbal_fric_motor[0].speed)>10)\t\t\t\t//鼠标左键
\t\tswitch(gimbal_mode_set->shoot_switch.shoot_state)
\t\t{
\t\t\tcase SHOOT_CONTINUOUS_START_STATE:gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;
\t\t\t\tbreak;
\t\t\tcase SHOOT_SINGLE_START_STATE:gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_SINGLE_START;
\t\t\t\tbreak;
\t\t\tcase SHOOT_WITHOUT_CONTROL_STATE:gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_WITHOUT_CONTROL;
\t\t\t\tbreak;
\t\t}
\telse
\t\tgimbal_mode_set->shoot_switch.shoot_mode = SHOOT_STOP;
\t
\t// 视觉自瞄自动开火(优先级最高, 覆盖遥控拨轮/鼠标)
\tif(get_vision_cmd_point()->shoot && gimbal_mode_set->aimbot_mode == AIMBOT_ON && gimbal_mode_set->shoot_switch.fric_mode == FRIC_START)
\t\tgimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;
}''')
P.append('')

# ---------- 反馈 ----------
P.append('//陀螺仪过零点计算')
P.append(grab('void gimbal_ins_data_count('))
P.append('')
P.append('''void gimbal_feedback_update(gimbal_control_t *gimbal_task_update)
{
\tif (gimbal_task_update == NULL)
\t{
\t\treturn;
\t}
\t//获取陀螺仪数据
\tgimbal_task_update->ins_data.roll = gimbal_task_update->gimbal_INT_angle_point[ROLL_CHANNEL];
\tgimbal_task_update->ins_data.pitch = gimbal_task_update->gimbal_INT_angle_point[PITCH_CHANNEL];
\tgimbal_task_update->ins_data.yaw = gimbal_task_update->gimbal_INT_angle_point[YAW_CHANNEL] - gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER;
\tgimbal_ins_data_count(&gimbal_task_update->ins_data);
\t//获得yaw轴6020数据
\tgimbal_task_update->gimbal_yaw_motor.speed = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
\tgimbal_task_update->gimbal_yaw_motor.absolute_angle = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
\t//获得pitch轴6020数据
\tgimbal_task_update->gimbal_pitch_motor.speed = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
\tgimbal_task_update->gimbal_pitch_motor.absolute_angle = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
\t//获得发射机构电机数据
\tgimbal_task_update->gimbal_trigger_motor.speed = gimbal_task_update->gimbal_trigger_motor.shoot_motor_measure->speed_rpm;
\tgimbal_task_update->gimbal_trigger_motor.relative_angle = gimbal_task_update->gimbal_trigger_motor.shoot_motor_measure->ecd;
\t
\t//获得摩擦轮电机数据
\tfor(int i=0; i<2; i++)
\t{
\t\tgimbal_task_update->gimbal_fric_motor[i].speed = gimbal_task_update->gimbal_fric_motor[i].shoot_motor_measure->speed_rpm;
\t\tgimbal_task_update->gimbal_fric_motor[i].relative_angle = gimbal_task_update->gimbal_fric_motor[i].shoot_motor_measure->ecd;
\t}
\t
\t//自瞄状态(自瞄任务里解算)
\tgimbal_task_update->aimbot_state = gimbal_task_update->gimbal_aimbot_point->aimbot_state;
\t
\t//yaw轴陀螺仪数据, 用pitch角把机体系z轴投影到世界系
\tgimbal_task_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_task_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
\t                                                        - arm_sin_f32(gimbal_task_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
\t//pitch轴陀螺仪数据
\tgimbal_task_update->gimbal_pitch_motor.motor_gyro = *(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
}''')
P.append('')

# ---------- 遥控器 -> 目标 ----------
P.append('''void gimbal_set_control(gimbal_control_t *gimbal_set_control_t)
{
\tif (gimbal_set_control_t == NULL)
\t{
\t\treturn;
\t}
\t
\t//遥控器对yaw,pitch控制赋值
\tif(gimbal_set_control_t->gimbal_mode != CHASSIS_NO_MOVE)
\t{
\t\tif(gimbal_set_control_t->aimbot_mode == AIMBOT_ON)
\t\t{
\t\t\t//自瞄接管时这两个值只用来观察, 实际控制量由 aimbot_gimbal_control() 从 gimbal_aimbot_point 直接取
\t\t\tgimbal_set_control_t->yaw_set = gimbal_set_control_t->gimbal_aimbot_point->target_yaw;
\t\t\tgimbal_set_control_t->pitch_set = gimbal_set_control_t->gimbal_aimbot_point->target_pitch;
\t\t}
\t\telse
\t\t{
//神秘小办法解决零漂
\t\t\tgimbal_set_control_t->yaw_set = -(gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_YAW_CHANNEL] * GIMBAL_OPEN_RC_YAW_SEN+gimbal_set_control_t->gimbal_rc_ctrl->mouse.x*GIMBAL_OPEN_X_YAW_SEN);
\t\t\tif(gimbal_set_control_t->yaw_set<=25&&gimbal_set_control_t->yaw_set>=-25)
\t\t\t\tgimbal_set_control_t->yaw_set=0;
\t\t\tgimbal_set_control_t->pitch_set = (gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_PITCH_CHANNEL] * GIMBAO_OPEN_RC_PITCH_FAST_SEN-gimbal_set_control_t->gimbal_rc_ctrl->mouse.y*GIMBAL_OPEN_Y_PITCH_SEN);
\t\t}
\t}
\t
\t//摩擦轮目标转速
\tif(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START)
\t{
\t\tgimbal_control.gimbal_fric_motor[0].speed_set = FRIC_SHOOT_LEFT_SPEED;
\t\tgimbal_control.gimbal_fric_motor[1].speed_set = -FRIC_SHOOT_RIGHT_SPEED;
\t}
\telse if(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_STOP)
\t{
\t\tgimbal_control.gimbal_fric_motor[0].speed_set = 0;
\t\tgimbal_control.gimbal_fric_motor[1].speed_set = 0;
\t\tgimbal_control.gimbal_trigger_motor.speed_set = 0;
\t}
\t
\t//拨弹轮trigger控制赋值
\t//trigger电机逻辑部分
\tif((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_SINGLE_START)&&(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
\t\tgimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_SLOW;
\telse if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_CONTINUOUS_START)&&(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
\t\tgimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_FAST;
\telse if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_WITHOUT_CONTROL)&&(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
\t\tgimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_FAST;
\telse
\t{
\t\tgimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_STOP;
\t}
\tshoot_task(&gimbal_set_control_t->shoot_switch.trigger_state , gimbal_set_control_t->shoot_switch.shoot_mode, gimbal_set_control_t->shoot_switch.fric_mode, &gimbal_set_control_t->gimbal_trigger_motor.speed_set, gimbal_set_control_t->gimbal_trigger_motor.shoot_delay);
\t
}''')
P.append('')

# ---------- 底盘差值 ----------
P.append('//底盘舵向起转判断')
P.append(grab('void gimbal_to_chassis_follow_yaw_determine('))
P.append('')

# ---------- 陀螺仪云台yaw ----------
P.append('//陀螺仪云台yaw')
P.append(grab('void gimbal_ins_yaw_control('))
P.append('')

# ---------- 自瞄云台 ----------
P.append('''//自瞄云台, 视觉目标即绝对角度目标值
//视觉目标进 PID 之前先过一道合法性闸门: 只有落在 ±PI 之内(即真正像个角度)才采信
//这样上游把目标写坏成离谱数值时, 云台只会停在上一帧合法位置, 不会一头砸向机械限位
static uint16_t gimbal_aimbot_delay;
static fp32 gimbal_aimbot_pitch_hold;	//上一次合法的视觉pitch目标
static fp32 gimbal_aimbot_yaw_hold;		//上一次合法的视觉yaw目标
void aimbot_gimbal_control(gimbal_control_t *aimbot_gimbal_control_point)
{
\tif(aimbot_gimbal_control_point->aimbot_mode == AIMBOT_ON)
\t{
\t\tfp32 target;
\t\tif(gimbal_aimbot_delay == 0)
\t\t{
\t\t\t//刚接管: 用当前姿态做初值, 防止第一帧目标异常时云台跳变
\t\t\tgimbal_aimbot_pitch_hold = aimbot_gimbal_control_point->ins_data.pitch;
\t\t\tgimbal_aimbot_yaw_hold = aimbot_gimbal_control_point->ins_data.yaw;
\t\t}
\t\t//封顶防回绕: uint16 涨到 65535 会绕回 0, 那样会重新触发上面的初值逻辑, 自瞄中云台会抖一下
\t\tif(gimbal_aimbot_delay < 0xFFFF)
\t\t\tgimbal_aimbot_delay++;
\t\tif(gimbal_aimbot_delay >= AIMBOT_INSURANCE)
\t\t{
\t\t\t//pitch
\t\t\t//视觉目标即目标值, 去掉45°跳变保护
\t\t\ttarget = aimbot_gimbal_control_point->gimbal_aimbot_point->target_pitch;
\t\t\t//NaN 与比较结果是 false, 一并挡住
\t\t\tif(target <= PI && target >= -PI)
\t\t\t\tgimbal_aimbot_pitch_hold = target;
\t\t\taimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set = gimbal_aimbot_pitch_hold;
\t\t\t//机械限位保护, 防止顶死
\t\t\tgimbal_limit(&aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, PITCH_INS_MIN);
\t\t\t//PID——pitch
\t\t\taimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_pitch_pid , aimbot_gimbal_control_point->ins_data.pitch,
\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\taimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro);
\t\t\taimbot_gimbal_control_point->gimbal_pitch_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_pitch_pid, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro,
\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\taimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set);
\t\t\t//yaw
\t\t\t//视觉目标即目标值, 去掉22.5°跳变保护
\t\t\ttarget = aimbot_gimbal_control_point->gimbal_aimbot_point->target_yaw;
\t\t\tif(target <= PI && target >= -PI)
\t\t\t\tgimbal_aimbot_yaw_hold = target;
\t\t\taimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set = loop_fp32_constrain(gimbal_aimbot_yaw_hold, -PI, PI);
\t\t\t//PID——yaw
\t\t\taimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_yaw_pid , aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL] - aimbot_gimbal_control_point->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER,
\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\taimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro);
\t\t\taimbot_gimbal_control_point->gimbal_yaw_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_yaw_pid, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro,
\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\taimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set);
\t\t}
\t}
\telse
\t\tgimbal_aimbot_delay = 0;
}''')
P.append('')

# ---------- 发射 ----------
P.append(grab('void fric_motor_PID_control('))
P.append('')
P.append('//云台发射机构控制')
P.append(grab('void trigger_motor_PID_control('))
P.append('')

# ---------- 陀螺仪pitch ----------
P.append('''void pitch_ins_PID_control(gimbal_control_t *pitch_ins_control)
{
\tif(pitch_ins_control->aimbot_mode == AIMBOT_OFF)
\t{
\t\t//只在还没顶到限位时继续累加, 到了限位就不让 add 继续涨, 防止松手后回不来
\t\tif(pitch_ins_control->gimbal_pitch_motor.absolute_angle_set < PITCH_INS_MAX && pitch_ins_control->gimbal_pitch_motor.absolute_angle_set > PITCH_INS_MIN)
\t\t\tpitch_ins_control->gimbal_pitch_motor.add += pitch_ins_control->pitch_set*GIMBAL_PITCH_INS_RATE;
\t\tpitch_ins_control->gimbal_pitch_motor.absolute_angle_set = pitch_ins_control->gimbal_pitch_motor.add + pitch_ins_control->ins_data.pitch_target;
\t\t//pitch轴
\t\tgimbal_limit(&pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, PITCH_INS_MIN);
\t\t
\t\tpitch_ins_control->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_ins_control->gimbal_angle_pitch_ins_pid , pitch_ins_control->ins_data.pitch,
\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tpitch_ins_control->gimbal_pitch_motor.absolute_angle_set, pitch_ins_control->gimbal_pitch_motor.motor_gyro);
\t\tpitch_ins_control->gimbal_pitch_motor.current_set = PID_calc(&pitch_ins_control->gimbal_speed_pitch_ins_pid, pitch_ins_control->gimbal_pitch_motor.motor_gyro,
\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tpitch_ins_control->gimbal_pitch_motor.motor_gyro_set);
\t}
}''')
P.append('')

# ---------- 控制环 ----------
P.append('''void gimbal_control_loop(gimbal_control_t *gimbal_control_loop_t)
{
\t//底盘舵向起转判断(只有"底盘跟随云台"一种模式)
\tgimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);
\t
\t//自瞄接管时, add 和 target 归位, 这样退出自瞄切回手动时云台不会跳
\tif(gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)
\t{
\t\tgimbal_control_loop_t->gimbal_yaw_motor.add = 0;
\t\tgimbal_control_loop_t->gimbal_pitch_motor.add = 0;
\t\tgimbal_control_loop_t->ins_data.yaw_target = gimbal_control_loop_t->ins_data.yaw;
\t\tgimbal_control_loop_t->ins_data.pitch_target = gimbal_control_loop_t->ins_data.pitch;
\t\tgimbal_control_loop_t->ins_data.yaw_last = 0;
\t}
\t
\t//二选一: 视觉自瞄接管 / 遥控器接管, 一眼能看出谁在驱动云台
\tif(gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)
\t{
\t\t//视觉接管两轴
\t\taimbot_gimbal_control(gimbal_control_loop_t);
\t}
\telse
\t{
\t\t//遥控器接管两轴, 走陀螺仪绝对角度环
\t\tgimbal_ins_yaw_control(gimbal_control_loop_t);
\t\tpitch_ins_PID_control(gimbal_control_loop_t);
\t}
\t
\t//trigger电机PID运算
\ttrigger_motor_PID_control(gimbal_control_loop_t);
\t//fric电机PID运算
\tfric_motor_PID_control(gimbal_control_loop_t);
\t
\t//云台pitch轴角度环控制电流赋值
\tgimbal_control_loop_t->gimbal_pitch_motor.give_current = gimbal_control_loop_t->gimbal_pitch_motor.current_set;
\t//云台yaw轴角度环控制电流赋值
\tgimbal_control_loop_t->gimbal_yaw_motor.give_current = gimbal_control_loop_t->gimbal_yaw_motor.current_set;
\t
\t//打开激光
\tif(gimbal_control_loop_t->shoot_switch.fric_mode == FRIC_START)
\t\tlaser_on();
\telse if(gimbal_control_loop_t->shoot_switch.fric_mode == FRIC_STOP)
\t\tlaser_off();
}''')
P.append('')
P.append('''void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
\tHAL_GPIO_EXTI_Callback_INS(GPIO_Pin);
}''')
P.append('')
P.append('''/**
  * @brief          获取云台数据指针
  * @param[in]      none
  * @retval         获取云台数据指针
  */
const gimbal_control_t *get_gimbal_move_point(void)
{
    return &gimbal_control;
}''')
P.append('')

txt = '\n'.join(P)

# 统一换行 + 行首 4 空格转 Tab
out = []
for l in txt.split('\n'):
    n = 0
    while l[n:n + 4] == '    ':
        n += 4
    out.append('\t' * (n // 4) + l[n:])
txt = '\r\n'.join(out)

data = txt.encode('gbk')
open(DST, 'wb').write(data)
print('bytes', len(data), 'CRLF', data.count(b'\r\n'), 'bareLF',
      data.count(b'\n') - 2 * data.count(b'\r\n'), 'FFFD', data.count(b'\xef\xbf\xbd'))
