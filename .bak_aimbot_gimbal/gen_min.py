# -*- coding: utf-8 -*-
# 生成"精简版" aimbot_gimbal.h / aimbot_gimbal.c
# 云台只做三件事: 1) 电机 PID 闭环  2) 遥控器控制  3) 视觉上位机控制
# 发射/弹舱/激光/键盘 全部删除
#
# 只保留同名函数, 未改动的函数体直接从 gimbal_task.c 按"内容标记"整段搬运,
# 绝不按行号切片(行号切片以前出过错).
import os
import sys

ROOT = 'D:/Document/Code/Aimbot_test/ShaoBing1'


def dec(p):
    return open(os.path.join(ROOT, p), 'rb').read().decode('gbk')


def lines_of(p):
    return [l.rstrip('\r') for l in dec(p).replace('\r\n', '\n').split('\n')]


def write_gbk(p, text):
    text = text.replace('\r\n', '\n').replace('\n', '\r\n')
    data = text.encode('gbk')
    n = data.count(b'\xef\xbf\xbd')
    assert n == 0, 'U+FFFD x%d in %s' % (n, p)
    open(os.path.join(ROOT, p), 'wb').write(data)
    print('wrote %-40s %6d bytes' % (p, len(data)))


def grab_doc(src, sig):
    """连同紧邻在函数签名上方的注释一起返回.
    只往上跳过连续的 // 行, 或者紧贴的 /** */ 块 —— 否则会一路走到很远的注释块上去"""
    i = src.index(sig)
    k = i
    while k - 1 >= 0 and src[k - 1].strip().startswith('//'):
        k -= 1
    if k - 1 >= 0 and src[k - 1].strip().endswith('*/'):
        j = k - 1
        while j >= 0 and not src[j].strip().startswith('/**'):
            assert not src[j].strip().startswith('/**'), src[j]
            j -= 1
        k = j
    j = i + 1
    while src[j] != '}':
        j += 1
    return src[k:j + 1]


def grab(src, sig):
    i = src.index(sig)
    j = i + 1
    while src[j] != '}':
        j += 1
    return src[i:j + 1]


G = lines_of('application/gimbal_task.c')

pid_init = grab_doc(G, 'void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)')
pid_calc = grab(G, 'fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)')
limit = grab(G, 'void gimbal_limit(fp32 *num, fp32 max_limix, fp32 min_limit)')
ins_cnt = grab(G, 'void gimbal_ins_data_count(ins_data_t *ins_data_count)')
follow_yaw = grab_doc(G, 'void gimbal_to_chassis_follow_yaw_determine(gimbal_control_t *gimbal_chassis_follow_yaw)')

# ============================================================================
#                             头文件
# ============================================================================
# 注意: 宏和 PID 参数从"备份的完整版"里切片, 不能读本脚本自己的输出,
# 否则第二次运行会把上一次的裁剪结果再裁一遍
H = lines_of('.bak_aimbot_gimbal/aimbot_gimbal.h.bloated')

INC = H[3:9]          # main.h ... aimbot_task.h   (去掉 referee.h / shoot.h)
MAC1 = H[14:20]       # 初始化时间 / 控制周期 / yaw补偿
MAC2 = H[29:42] + H[43:48]   # 去掉 SHOOT_MODE_CHANNEL
PIDM = H[49:105]      # PID 宏: 自瞄两轴 + 遥控陀螺仪两轴
PIDA = H[128:147]     # gimbal_PID_t
EXT = H[249:257]      # extern 函数声明

h_parts = []
h_parts.append("""#ifndef _AIMBOT_GIMBAL_H_
#define _AIMBOT_GIMBAL_H_
""")
h_parts.append('\n'.join(INC))
h_parts.append("""
//=====================================================
// 云台只做三件事: 电机PID闭环 / 遥控器控制 / 视觉上位机控制
// 发射, 弹舱, 激光, 键盘相关代码全部删除
//=====================================================
""")
h_parts.append('\n'.join(MAC1))
h_parts.append('')
h_parts.append('\n'.join(MAC2))
h_parts.append('')
h_parts.append('\n'.join(PIDM))
h_parts.append('')
h_parts.append('\n'.join(PIDA))
h_parts.append("""
//云台电机
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;

	fp32 add;				//遥控器积分量
	fp32 absolute_angle;	//rad, 编码器绝对角度
	fp32 absolute_angle_set;//rad, 目标绝对角度
	fp32 motor_gyro;		//rad/s, 陀螺仪角速度
	fp32 motor_gyro_set;
	fp32 speed;				//rpm
	fp32 current_set;
	int16_t give_current;
} gimbal_motor_t;

//陀螺仪数据
typedef struct
{
	fp32 yaw_target;	//云台yaw轴绝对角度目标
	fp32 pitch_target;	//云台pitch轴绝对角度目标

	fp32 roll;
	fp32 pitch;
	fp32 yaw;
	fp32 yaw_last;		//上一帧yaw, 用于解缠绕
}ins_data_t;

//云台控制结构体
typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;			//遥控器
	const aimbot_task_t *gimbal_aimbot_point;	//视觉上位机给的目标角度
	const fp32 *gimbal_INT_angle_point;			//陀螺仪角度
	const fp32 *gimbal_INT_gyro_point;			//陀螺仪角速度

	uint8_t aimbot_mode;	//AIMBOT_OFF: 遥控器控制, AIMBOT_ON: 视觉上位机控制
	uint8_t gimbal_mode;	//CHASSIS_NO_MOVE: 无力, CHASSIS_FOLLOW_INS_YAW: 陀螺仪控制
	ins_data_t	ins_data;

	fp32 yaw;	//云台与底盘差值, 给底盘跟随用

	//云台与底盘差值
	fp32 err_gimbal_chassis_ecd;

	fp32 yaw_set;		//遥控器给的目标
	fp32 pitch_set;		//遥控器给的目标

	gimbal_motor_t gimbal_yaw_motor;	//yaw电机
	gimbal_motor_t gimbal_pitch_motor;	//pitch电机

	//视觉上位机自瞄PID
	pid_type_def aimbot_speed_pitch_pid;
	gimbal_PID_t aimbot_angle_pitch_pid;
	pid_type_def aimbot_speed_yaw_pid;
	gimbal_PID_t aimbot_angle_yaw_pid;
	//遥控器陀螺仪yaw轴PID
	pid_type_def gimbal_speed_yaw_ins_pid;
	gimbal_PID_t gimbal_angle_yaw_ins_pid;
	//遥控器陀螺仪pitch轴PID
	pid_type_def gimbal_speed_pitch_ins_pid;
	gimbal_PID_t gimbal_angle_pitch_ins_pid;
} gimbal_control_t;
""")
h_parts.append('\n'.join(EXT))
h_parts.append("""
#endif
""")

write_gbk('application/aimbot_gimbal.h', '\n'.join(h_parts))

# ============================================================================
#                             源文件
# ============================================================================
c_parts = []

c_parts.append("""#include "user_lib.h"
#include "aimbot_gimbal.h"
#include "CAN_receive.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"

//自瞄接管前的保护延时
#define AIMBOT_INSURANCE	1

gimbal_control_t gimbal_control;


//下发前给电流值限幅, 防止超出 int16 范围回绕反号
static int16_t gimbal_current_limit(fp32 current)
{
	if(current > 32767.0f)
		return 32767;
	else if(current < -32768.0f)
		return -32768;
	else
		return (int16_t)current;
}
""")
c_parts.append('\n'.join(pid_init))
c_parts.append('')
c_parts.append('\n'.join(pid_calc))
c_parts.append('')
c_parts.append('\n'.join(limit))
c_parts.append('')
# ins_cnt 放在 gimbal_init 前面, gimbal_init 里要用它把初始角度对齐
c_parts.append('\n'.join(ins_cnt))
c_parts.append('')

c_parts.append("""
//gimbal要进行的操作
void gimbal_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);

	gimbal_init(&gimbal_control);

	while(1)
	{
		//云台模式设定
		gimbal_set_mode(&gimbal_control);
		//云台数据获取
		gimbal_feedback_update(&gimbal_control);
		gimbal_set_control(&gimbal_control);
		gimbal_control_loop(&gimbal_control);

		//无力模式
		if(gimbal_control.gimbal_mode == CHASSIS_NO_MOVE)
		{
			CAN_cmd_gimbal(0, 0);
		}
		//使能, 下发PID算出来的电流
		else
		{
			CAN_cmd_gimbal(gimbal_current_limit(gimbal_control.gimbal_yaw_motor.give_current),
							gimbal_current_limit(gimbal_control.gimbal_pitch_motor.give_current));
		}

		vTaskDelay(GIMBAL_CONTROL_TIME);
	}
}


void gimbal_init(gimbal_control_t *gimbal_task_init)
{
	if (gimbal_task_init == NULL)
    {
        return;
    }

	//初始值置0
	gimbal_task_init->yaw = 0;
	gimbal_task_init->err_gimbal_chassis_ecd = 0;
	gimbal_task_init->gimbal_mode = CHASSIS_NO_MOVE;	//云台跟随底盘初始化，无力
	gimbal_task_init->aimbot_mode = AIMBOT_OFF;

	//陀螺仪角度和角速度指针获取
	gimbal_task_init->gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_task_init->gimbal_INT_gyro_point = get_gyro_data_point();
	//获取视觉上位机的目标角度指针
	gimbal_task_init->gimbal_aimbot_point = get_aimbot_point();
	//遥控器数据指针获取
	gimbal_task_init->gimbal_rc_ctrl = get_remote_control_point();
	//电机数据指针获取
	gimbal_task_init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(0);
	gimbal_task_init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(1);

	//初始化云台相关PID参数
	const static fp32 aimbot_speed_pitch_pid[3] = {AIMBOT_PITCH_SPEED_PID_KP, AIMBOT_PITCH_SPEED_PID_KI, AIMBOT_PITCH_SPEED_PID_KD};
	const static fp32 aimbot_speed_yaw_pid[3] = {AIMBOT_YAW_SPEED_PID_KP, AIMBOT_YAW_SPEED_PID_KI, AIMBOT_YAW_SPEED_PID_KD};
	const static fp32 gimbal_speed_yaw_ins_pid[3] = {YAW_INS_SPEED_PID_KP, YAW_INS_SPEED_PID_KI, YAW_INS_SPEED_PID_KD};
	const static fp32 gimbal_speed_pitch_ins_pid[3] = {PITCH_INS_SPEED_PID_KP, PITCH_INS_SPEED_PID_KI, PITCH_INS_SPEED_PID_KD};

	//初始化视觉上位机自瞄的PID
	gimbal_PID_init(&gimbal_task_init->aimbot_angle_pitch_pid, AIMBOT_PITCH_ANGLE_PID_MAX_OUT, AIMBOT_PITCH_ANGLE_PID_MAX_IOUT, AIMBOT_PITCH_ANGLE_PID_KP, AIMBOT_PITCH_ANGLE_PID_KI, AIMBOT_PITCH_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->aimbot_speed_pitch_pid, PID_POSITION, aimbot_speed_pitch_pid, AIMBOT_PITCH_SPEED_PID_MAX_OUT, AIMBOT_PITCH_SPEED_PID_MAX_IOUT);
	gimbal_PID_init(&gimbal_task_init->aimbot_angle_yaw_pid, AIMBOT_YAW_ANGLE_PID_MAX_OUT, AIMBOT_YAW_ANGLE_PID_MAX_IOUT, AIMBOT_YAW_ANGLE_PID_KP, AIMBOT_YAW_ANGLE_PID_KI, AIMBOT_YAW_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->aimbot_speed_yaw_pid, PID_POSITION, aimbot_speed_yaw_pid, AIMBOT_YAW_SPEED_PID_MAX_OUT, AIMBOT_YAW_SPEED_PID_MAX_IOUT);
	//初始化遥控器陀螺仪yaw轴PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_yaw_ins_pid, YAW_INS_ANGLE_PID_MAX_OUT, YAW_INS_ANGLE_PID_MAX_IOUT, YAW_INS_ANGLE_PID_KP, YAW_INS_ANGLE_PID_KI, YAW_INS_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_yaw_ins_pid, PID_POSITION, gimbal_speed_yaw_ins_pid, YAW_INS_SPEED_PID_MAX_OUT, YAW_INS_SPEED_PID_MAX_IOUT);
	//初始化遥控器陀螺仪pitch轴PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_pitch_ins_pid, PITCH_INS_ANGLE_PID_MAX_OUT, PITCH_INS_ANGLE_PID_MAX_IOUT, PITCH_INS_ANGLE_PID_KP, PITCH_INS_ANGLE_PID_KI, PITCH_INS_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_pitch_ins_pid, PID_POSITION, gimbal_speed_pitch_ins_pid, PITCH_INS_SPEED_PID_MAX_OUT, PITCH_INS_SPEED_PID_MAX_IOUT);

	//先把当前姿态读进来, 目标角度对齐当前角度, 防止开机第一帧猛冲一下
	gimbal_task_init->ins_data.roll = gimbal_task_init->gimbal_INT_angle_point[ROLL_CHANNEL];
	gimbal_task_init->ins_data.pitch = gimbal_task_init->gimbal_INT_angle_point[PITCH_CHANNEL];
	gimbal_task_init->ins_data.yaw = gimbal_task_init->gimbal_INT_angle_point[YAW_CHANNEL];
	gimbal_ins_data_count(&gimbal_task_init->ins_data);
	gimbal_task_init->ins_data.yaw_target = gimbal_task_init->ins_data.yaw;
	gimbal_task_init->ins_data.pitch_target = gimbal_task_init->ins_data.pitch;
}


//云台模式设定: 只区分 无力 / 遥控器 / 视觉上位机
void gimbal_set_mode(gimbal_control_t *gimbal_mode_set)
{
	if (gimbal_mode_set == NULL)
    {
        return;
    }

	//CH1下 = 云台无力, 其余组合 = 遥控器控制
	if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]))
	{
		//云台无力
		gimbal_mode_set->gimbal_mode = CHASSIS_NO_MOVE;
	}
	else
	{
		//云台陀螺仪控制
		gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
	}

	//视觉上位机接管的条件: 开关拨到自瞄位(CH1上+CH2上) 且 上位机要求接管(control=1)
	//两个条件同时成立才接管, 保证任何时候拨一下开关就能用遥控器夺回控制权
	if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]) &&
		switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]) &&
		get_vision_cmd_point()->control)
	{
		gimbal_mode_set->aimbot_mode = AIMBOT_ON;
	}
	else
	{
		gimbal_mode_set->aimbot_mode = AIMBOT_OFF;
	}

	//把当前的云台模式同步给上位机
	aimbot_mode_set(gimbal_mode_set->aimbot_mode);
}
""")

c_parts.append("""
void gimbal_feedback_update(gimbal_control_t *gimbal_task_update)
{
	if (gimbal_task_update == NULL)
    {
        return;
    }
	//获取陀螺仪数据
	gimbal_task_update->ins_data.roll = gimbal_task_update->gimbal_INT_angle_point[ROLL_CHANNEL];
	gimbal_task_update->ins_data.pitch = gimbal_task_update->gimbal_INT_angle_point[PITCH_CHANNEL];
	gimbal_task_update->ins_data.yaw = gimbal_task_update->gimbal_INT_angle_point[YAW_CHANNEL] - gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER;
	gimbal_ins_data_count(&gimbal_task_update->ins_data);
	//计算yaw轴6020数据
	gimbal_task_update->gimbal_yaw_motor.speed = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
	gimbal_task_update->gimbal_yaw_motor.absolute_angle = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
	//计算pitch轴6020数据
	gimbal_task_update->gimbal_pitch_motor.speed = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
	gimbal_task_update->gimbal_pitch_motor.absolute_angle = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;

	//陀螺仪角速度, 直接取yaw轴和pitch轴的分量
	gimbal_task_update->gimbal_yaw_motor.motor_gyro = *(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	gimbal_task_update->gimbal_pitch_motor.motor_gyro = *(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
}


//遥控器给yaw,pitch目标赋值
void gimbal_set_control(gimbal_control_t *gimbal_set_control_t)
{
	if (gimbal_set_control_t == NULL)
    {
        return;
    }

	//只有遥控器控制的时候才需要算遥控器的目标值, 自瞄的目标值由上位机直接给
	if(gimbal_set_control_t->gimbal_mode == CHASSIS_NO_MOVE || gimbal_set_control_t->aimbot_mode == AIMBOT_ON)
	{
		return;
	}

	//遥控器摇杆和鼠标给目标角度
	gimbal_set_control_t->yaw_set = -(gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_YAW_CHANNEL] * GIMBAL_OPEN_RC_YAW_SEN + gimbal_set_control_t->gimbal_rc_ctrl->mouse.x*GIMBAL_OPEN_X_YAW_SEN);
	//死区, 防止摇杆回中不准导致漂移
	if(gimbal_set_control_t->yaw_set<=25&&gimbal_set_control_t->yaw_set>=-25)
		gimbal_set_control_t->yaw_set=0;
	gimbal_set_control_t->pitch_set = (gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_PITCH_CHANNEL] * GIMBAO_OPEN_RC_PITCH_FAST_SEN - gimbal_set_control_t->gimbal_rc_ctrl->mouse.y*GIMBAL_OPEN_Y_PITCH_SEN);
}
""")
c_parts.append('\n'.join(follow_yaw))

c_parts.append("""
//遥控器控制云台yaw
void gimbal_ins_yaw_control(gimbal_control_t *gimbal_ins)
{
	//遥控器的积分量累加, 再叠加到陀螺仪当前角度上
	gimbal_ins->gimbal_yaw_motor.add += gimbal_ins->yaw_set*YAW_INS_OPEN_RC_YAW_SEN;
	gimbal_ins->gimbal_yaw_motor.absolute_angle_set = gimbal_ins->ins_data.yaw_target + gimbal_ins->gimbal_yaw_motor.add;

	//yaw轴角度环PID
	gimbal_ins->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_ins->gimbal_angle_yaw_ins_pid , gimbal_ins->ins_data.yaw,
																			gimbal_ins->gimbal_yaw_motor.absolute_angle_set, gimbal_ins->gimbal_yaw_motor.motor_gyro);
	//yaw轴速度环PID
	gimbal_ins->gimbal_yaw_motor.current_set = PID_calc(&gimbal_ins->gimbal_speed_yaw_ins_pid, gimbal_ins->gimbal_yaw_motor.motor_gyro,
																	gimbal_ins->gimbal_yaw_motor.motor_gyro_set);
}


//视觉上位机接管云台
static uint16_t gimbal_aimbot_delay;
void aimbot_gimbal_control(gimbal_control_t *aimbot_gimbal_control_point)
{
	if(aimbot_gimbal_control_point->aimbot_mode != AIMBOT_ON)
	{
		gimbal_aimbot_delay = 0;
		return;
	}

	if(gimbal_aimbot_delay < AIMBOT_INSURANCE)
		gimbal_aimbot_delay++;
	if(gimbal_aimbot_delay < AIMBOT_INSURANCE)
		return;

	//pitch, 上位机给的目标就是绝对角度目标
	aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_aimbot_point->target_pitch;
	//机械限位保护, 防止撞限位
	gimbal_limit(&aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, PITCH_INS_MIN);
	//pitch轴角度环PID
	aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_pitch_pid , aimbot_gimbal_control_point->ins_data.pitch,
																			aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro);
	//pitch轴速度环PID
	aimbot_gimbal_control_point->gimbal_pitch_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_pitch_pid, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro,
																			aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set);

	//yaw, 上位机给的目标就是绝对角度目标
	aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_aimbot_point->target_yaw;
	aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set = loop_fp32_constrain(aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set, -PI, PI);
	//yaw轴角度环PID
	aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_yaw_pid , aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL] - aimbot_gimbal_control_point->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER,
																				aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro);
	//yaw轴速度环PID
	aimbot_gimbal_control_point->gimbal_yaw_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_yaw_pid, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro,
																				aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set);
}


//遥控器控制云台pitch
void pitch_ins_PID_control(gimbal_control_t *pitch_ins_control)
{
	//在机械行程内才允许继续积分
	if(pitch_ins_control->gimbal_pitch_motor.absolute_angle_set < PITCH_INS_MAX && pitch_ins_control->gimbal_pitch_motor.absolute_angle_set > PITCH_INS_MIN)
		pitch_ins_control->gimbal_pitch_motor.add += pitch_ins_control->pitch_set*GIMBAL_PITCH_INS_RATE;
	pitch_ins_control->gimbal_pitch_motor.absolute_angle_set = pitch_ins_control->gimbal_pitch_motor.add + pitch_ins_control->ins_data.pitch_target;
	//pitch机械限位
	gimbal_limit(&pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, PITCH_INS_MIN);

	//pitch轴角度环PID
	pitch_ins_control->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_ins_control->gimbal_angle_pitch_ins_pid , pitch_ins_control->ins_data.pitch,
																			pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, pitch_ins_control->gimbal_pitch_motor.motor_gyro);
	//pitch轴速度环PID
	pitch_ins_control->gimbal_pitch_motor.current_set = PID_calc(&pitch_ins_control->gimbal_speed_pitch_ins_pid, pitch_ins_control->gimbal_pitch_motor.motor_gyro,
																	pitch_ins_control->gimbal_pitch_motor.motor_gyro_set);
}


void gimbal_control_loop(gimbal_control_t *gimbal_control_loop_t)
{
	//算云台和底盘的差值, 给底盘跟随用
	gimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);

	//无力, 或者被视觉接管的时候, 遥控器的积分量清零
	if(gimbal_control_loop_t->gimbal_mode == CHASSIS_NO_MOVE || gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)
	{
		gimbal_control_loop_t->gimbal_yaw_motor.add = 0;
		gimbal_control_loop_t->gimbal_pitch_motor.add = 0;
		//把遥控器的target值同步到当前角度, 切回遥控器的时候不会跳转头
		gimbal_control_loop_t->ins_data.yaw_target = gimbal_control_loop_t->ins_data.yaw;
		gimbal_control_loop_t->ins_data.pitch_target = gimbal_control_loop_t->ins_data.pitch;
	}

	//显式二选一: 视觉上位机接管 还是 遥控器接管
	if(gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)
	{
		//视觉上位机控制云台
		aimbot_gimbal_control(gimbal_control_loop_t);
	}
	else
	{
		//遥控器控制云台
		gimbal_ins_yaw_control(gimbal_control_loop_t);
		pitch_ins_PID_control(gimbal_control_loop_t);
	}

	//云台yaw轴最终输出电流
	gimbal_control_loop_t->gimbal_yaw_motor.give_current = gimbal_control_loop_t->gimbal_yaw_motor.current_set;
	//云台pitch轴最终输出电流
	gimbal_control_loop_t->gimbal_pitch_motor.give_current = gimbal_control_loop_t->gimbal_pitch_motor.current_set;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_EXTI_Callback_INS(GPIO_Pin);
}


/**
  * @brief          获取云台控制指针
  * @param[in]      none
  * @retval         获取云台控制指针
  */
const gimbal_control_t *get_gimbal_move_point(void)
{
    return &gimbal_control;
}
""")

write_gbk('application/aimbot_gimbal.c', '\n'.join(c_parts))
print('OK')
