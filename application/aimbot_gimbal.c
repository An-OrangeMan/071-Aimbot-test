#include "user_lib.h"
#include "aimbot_gimbal.h"
#include "CAN_receive.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"

//����ӹ�ǰ�ı�����ʱ
#define AIMBOT_INSURANCE	1
#define FRIC_INSURANCE_RC		100

// trigger speed rate
#define TRIGGER_SHOOT_SPEED_RATE_FAST	80
#define TRIGGER_SHOOT_SPEED_RATE_SLOW	130
#define TRIGGER_SHOOT_STOP				0
// friction wheel speed (rpm)
#define FRIC_SHOOT_LEFT_SPEED			6000
#define FRIC_SHOOT_RIGHT_SPEED			6000

// trigger speed loop PID
#define TRIGGER_SPEED_PID_KP			15.0f
#define TRIGGER_SPEED_PID_KI			0.0f
#define TRIGGER_SPEED_PID_KD			0.0f
#define TRIGGER_SPEED_PID_MAX_OUT		10000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT		5000.0f
// friction wheel speed loop PID
#define FRIC_SPEED_PID_KP				15.0f
#define FRIC_SPEED_PID_KI				0.0f
#define FRIC_SPEED_PID_KD				0.0f
#define FRIC_SPEED_PID_MAX_OUT			10000.0f
#define FRIC_SPEED_PID_MAX_IOUT			1000.0f

gimbal_control_t gimbal_control;


//�·�ǰ������ֵ�޷�, ��ֹ���� int16 ��Χ���Ʒ���
static int16_t gimbal_current_limit(fp32 current)
{
	if(current > 32767.0f)
		return 32767;
	else if(current < -32768.0f)
		return -32768;
	else
		return (int16_t)current;
}

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
//    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    pid->err = set - get;
//    pid->err = loop_fp32_constrain(err, -PI, PI);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

void gimbal_limit(fp32 *num, fp32 max_limix, fp32 min_limit)
{
	if(*num > max_limix)
	{
		*num = max_limix;
	}
	else if(*num < min_limit)
	{
		*num = min_limit;
	}
}

void gimbal_ins_data_count(ins_data_t *ins_data_count)
{
	
	ins_data_count->yaw += PI;
	while(ins_data_count->yaw_last-ins_data_count->yaw>PI)
		ins_data_count->yaw += PI*2;
	while(ins_data_count->yaw_last-ins_data_count->yaw<-PI)
		ins_data_count->yaw -= PI*2;
	ins_data_count->yaw_last = ins_data_count->yaw;
}


//gimbalҪ���еĲ���
void gimbal_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);

	gimbal_init(&gimbal_control);

	while(1)
	{
		//��̨ģʽ�趨
		gimbal_set_mode(&gimbal_control);
		//��̨���ݻ�ȡ
		gimbal_feedback_update(&gimbal_control);
		gimbal_set_control(&gimbal_control);
		gimbal_control_loop(&gimbal_control);

		//����ģʽ
		if(gimbal_control.gimbal_mode == CHASSIS_NO_MOVE)
		{
			CAN_cmd_gimbal(0, 0);
			CAN_cmd_shoot(0, 0, 0);
		}
		//ʹ��, �·�PID������ĵ���
		else
		{
			CAN_cmd_gimbal(gimbal_current_limit(gimbal_control.gimbal_yaw_motor.give_current),
							gimbal_current_limit(gimbal_control.gimbal_pitch_motor.give_current));
			CAN_cmd_shoot(gimbal_control.gimbal_trigger_motor.give_current,
							gimbal_control.gimbal_fric_motor[0].give_current,
							gimbal_control.gimbal_fric_motor[1].give_current);
//			CAN_cmd_shoot(50000, 5000, 5000);
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

	//��ʼֵ��0
	gimbal_task_init->yaw = 0;
	gimbal_task_init->err_gimbal_chassis_ecd = 0;
	gimbal_task_init->gimbal_mode = CHASSIS_NO_MOVE;	//��̨������̳�ʼ��������
	gimbal_task_init->aimbot_mode = AIMBOT_OFF;

	// shooting init
	shoot_init();
	gimbal_task_init->shoot_switch.fric_flag = FRIC_READY;
	gimbal_task_init->shoot_switch.fric_mode = FRIC_STOP;
	gimbal_task_init->shoot_switch.shoot_mode = SHOOT_STOP;
	gimbal_task_init->shoot_switch.shoot_state = SHOOT_SINGLE_START_STATE;

	//�����ǽǶȺͽ��ٶ�ָ���ȡ
	gimbal_task_init->gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_task_init->gimbal_INT_gyro_point = get_gyro_data_point();
	//��ȡ�Ӿ���λ����Ŀ��Ƕ�ָ��
	gimbal_task_init->gimbal_aimbot_point = get_aimbot_point();
	//ң��������ָ���ȡ
	gimbal_task_init->gimbal_rc_ctrl = get_remote_control_point();
	//�������ָ���ȡ
	gimbal_task_init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(0);
	gimbal_task_init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(1);
	gimbal_task_init->gimbal_trigger_motor.shoot_motor_measure = get_shoot_motor_measure_point(0);
	gimbal_task_init->gimbal_fric_motor[0].shoot_motor_measure = get_shoot_motor_measure_point(1);
	gimbal_task_init->gimbal_fric_motor[1].shoot_motor_measure = get_shoot_motor_measure_point(2);

	//��ʼ����̨���PID����
	const static fp32 aimbot_speed_pitch_pid[3] = {AIMBOT_PITCH_SPEED_PID_KP, AIMBOT_PITCH_SPEED_PID_KI, AIMBOT_PITCH_SPEED_PID_KD};
	const static fp32 aimbot_speed_yaw_pid[3] = {AIMBOT_YAW_SPEED_PID_KP, AIMBOT_YAW_SPEED_PID_KI, AIMBOT_YAW_SPEED_PID_KD};
	const static fp32 gimbal_speed_yaw_ins_pid[3] = {YAW_INS_SPEED_PID_KP, YAW_INS_SPEED_PID_KI, YAW_INS_SPEED_PID_KD};
	const static fp32 gimbal_speed_pitch_ins_pid[3] = {PITCH_INS_SPEED_PID_KP, PITCH_INS_SPEED_PID_KI, PITCH_INS_SPEED_PID_KD};

	//��ʼ���Ӿ���λ�������PID
	gimbal_PID_init(&gimbal_task_init->aimbot_angle_pitch_pid, AIMBOT_PITCH_ANGLE_PID_MAX_OUT, AIMBOT_PITCH_ANGLE_PID_MAX_IOUT, AIMBOT_PITCH_ANGLE_PID_KP, AIMBOT_PITCH_ANGLE_PID_KI, AIMBOT_PITCH_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->aimbot_speed_pitch_pid, PID_POSITION, aimbot_speed_pitch_pid, AIMBOT_PITCH_SPEED_PID_MAX_OUT, AIMBOT_PITCH_SPEED_PID_MAX_IOUT);
	gimbal_PID_init(&gimbal_task_init->aimbot_angle_yaw_pid, AIMBOT_YAW_ANGLE_PID_MAX_OUT, AIMBOT_YAW_ANGLE_PID_MAX_IOUT, AIMBOT_YAW_ANGLE_PID_KP, AIMBOT_YAW_ANGLE_PID_KI, AIMBOT_YAW_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->aimbot_speed_yaw_pid, PID_POSITION, aimbot_speed_yaw_pid, AIMBOT_YAW_SPEED_PID_MAX_OUT, AIMBOT_YAW_SPEED_PID_MAX_IOUT);
	//��ʼ��ң����������yaw��PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_yaw_ins_pid, YAW_INS_ANGLE_PID_MAX_OUT, YAW_INS_ANGLE_PID_MAX_IOUT, YAW_INS_ANGLE_PID_KP, YAW_INS_ANGLE_PID_KI, YAW_INS_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_yaw_ins_pid, PID_POSITION, gimbal_speed_yaw_ins_pid, YAW_INS_SPEED_PID_MAX_OUT, YAW_INS_SPEED_PID_MAX_IOUT);
	//��ʼ��ң����������pitch��PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_pitch_ins_pid, PITCH_INS_ANGLE_PID_MAX_OUT, PITCH_INS_ANGLE_PID_MAX_IOUT, PITCH_INS_ANGLE_PID_KP, PITCH_INS_ANGLE_PID_KI, PITCH_INS_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_pitch_ins_pid, PID_POSITION, gimbal_speed_pitch_ins_pid, PITCH_INS_SPEED_PID_MAX_OUT, PITCH_INS_SPEED_PID_MAX_IOUT);

	// trigger / friction wheel speed loop PID init
	const static fp32 gimbal_speed_trigger_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
	const static fp32 gimbal_speed_fric_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
	PID_init(&gimbal_task_init->gimbal_speed_trigger_pid, PID_POSITION, gimbal_speed_trigger_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
	for(int i = 0; i < 2; i++)
	{
		PID_init(&gimbal_task_init->gimbal_speed_fric_pid[i], PID_POSITION, gimbal_speed_fric_pid, FRIC_SPEED_PID_MAX_OUT, FRIC_SPEED_PID_MAX_IOUT);
	}

	//�Ȱѵ�ǰ��̬������, Ŀ��Ƕȶ��뵱ǰ�Ƕ�, ��ֹ������һ֡�ͳ�һ��
	gimbal_task_init->ins_data.roll = gimbal_task_init->gimbal_INT_angle_point[ROLL_CHANNEL];
	gimbal_task_init->ins_data.pitch = gimbal_task_init->gimbal_INT_angle_point[PITCH_CHANNEL];
	gimbal_task_init->ins_data.yaw = gimbal_task_init->gimbal_INT_angle_point[YAW_CHANNEL];
	gimbal_ins_data_count(&gimbal_task_init->ins_data);
	gimbal_task_init->ins_data.yaw_target = gimbal_task_init->ins_data.yaw;
	gimbal_task_init->ins_data.pitch_target = gimbal_task_init->ins_data.pitch;
}


//��̨ģʽ�趨: ֻ���� ���� / ң���� / �Ӿ���λ��
void gimbal_set_mode(gimbal_control_t *gimbal_mode_set)
{
	if (gimbal_mode_set == NULL)
    {
        return;
    }

	//CH1�� = ��̨����, ������� = ң��������
	if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]))
	{
		//��̨����
		gimbal_mode_set->gimbal_mode = CHASSIS_NO_MOVE;
	}
	else
	{
		//��̨�����ǿ���
		gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
	}

	//�Ӿ���λ���ӹܵ�����: ���ز�������λ(CH1��+CH2��) �� ��λ��Ҫ��ӹ�(control=1)
	//��������ͬʱ�����Žӹ�, ��֤�κ�ʱ��һ�¿��ؾ�����ң������ؿ���Ȩ
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

	//�ѵ�ǰ����̨ģʽͬ������λ��
	// read dial (encoding switch) value from RC channel
	gimbal_mode_set->shoot_switch.encoding_switch_ecd = gimbal_mode_set->gimbal_rc_ctrl->rc.ch[SHOOT_MODE_CHANNEL];

	// NO_MOVE: stop friction wheels and shooting
	if(gimbal_mode_set->gimbal_mode == CHASSIS_NO_MOVE)
	{
		gimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_STOP;
	}

	// dial toggles friction wheels on/off (debounced)
	static uint16_t shoot_i;
	if((gimbal_mode_set->shoot_switch.fric_mode == FRIC_STOP) && (gimbal_mode_set->shoot_switch.fric_flag == FRIC_READY) && (gimbal_mode_set->shoot_switch.encoding_switch_ecd<=-400))
	{
		shoot_i++;
		if(shoot_i >= FRIC_INSURANCE_RC)
		{
			gimbal_mode_set->shoot_switch.fric_mode = FRIC_START;
			gimbal_mode_set->shoot_switch.fric_flag = FRIC_START;
			shoot_i=0;
		}
	}
	else if((gimbal_mode_set->shoot_switch.fric_mode == FRIC_START) && (gimbal_mode_set->shoot_switch.fric_flag == FRIC_READY) && (gimbal_mode_set->shoot_switch.encoding_switch_ecd<=-400))
	{
		gimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
	}
	else if(ABS(gimbal_mode_set->shoot_switch.encoding_switch_ecd)<10)
	{
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_READY;
	}

	// dial selects shoot mode
	if(gimbal_mode_set->shoot_switch.encoding_switch_ecd>200&&gimbal_mode_set->shoot_switch.encoding_switch_ecd<450)
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_SINGLE_START;
	else if(gimbal_mode_set->shoot_switch.encoding_switch_ecd>450)
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;
	else
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_STOP;

	// 视觉自动开火: 优先级最高, 覆盖拨码轮手动发弹; 仅自瞄接管 + 摩擦轮已启动时生效
	if(get_vision_cmd_point()->shoot && gimbal_mode_set->aimbot_mode == AIMBOT_ON && gimbal_mode_set->shoot_switch.fric_mode == FRIC_START)
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;

	aimbot_mode_set(gimbal_mode_set->aimbot_mode);
}


void gimbal_feedback_update(gimbal_control_t *gimbal_task_update)
{
	if (gimbal_task_update == NULL)
    {
        return;
    }
	//��ȡ����������
	gimbal_task_update->ins_data.roll = gimbal_task_update->gimbal_INT_angle_point[ROLL_CHANNEL];
	gimbal_task_update->ins_data.pitch = gimbal_task_update->gimbal_INT_angle_point[PITCH_CHANNEL];
	gimbal_task_update->ins_data.yaw = gimbal_task_update->gimbal_INT_angle_point[YAW_CHANNEL] - gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER;
	gimbal_ins_data_count(&gimbal_task_update->ins_data);
	//����yaw��6020����
	gimbal_task_update->gimbal_yaw_motor.speed = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
	gimbal_task_update->gimbal_yaw_motor.absolute_angle = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
	//����pitch��6020����
	gimbal_task_update->gimbal_pitch_motor.speed = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
	gimbal_task_update->gimbal_pitch_motor.absolute_angle = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;

	// trigger / friction wheel feedback
	gimbal_task_update->gimbal_trigger_motor.speed = gimbal_task_update->gimbal_trigger_motor.shoot_motor_measure->speed_rpm;
	gimbal_task_update->gimbal_trigger_motor.relative_angle = gimbal_task_update->gimbal_trigger_motor.shoot_motor_measure->ecd;
	for(int i = 0; i < 2; i++)
	{
		gimbal_task_update->gimbal_fric_motor[i].speed = gimbal_task_update->gimbal_fric_motor[i].shoot_motor_measure->speed_rpm;
		gimbal_task_update->gimbal_fric_motor[i].relative_angle = gimbal_task_update->gimbal_fric_motor[i].shoot_motor_measure->ecd;
	}

	//�����ǽ��ٶ�, ֱ��ȡyaw���pitch��ķ���
	gimbal_task_update->gimbal_yaw_motor.motor_gyro = *(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	gimbal_task_update->gimbal_pitch_motor.motor_gyro = *(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
}


//ң������yaw,pitchĿ�긳ֵ
void gimbal_set_control(gimbal_control_t *gimbal_set_control_t)
{
	if (gimbal_set_control_t == NULL)
    {
        return;
    }

	//ֻ��ң�������Ƶ�ʱ�����Ҫ��ң������Ŀ��ֵ, �����Ŀ��ֵ����λ��ֱ�Ӹ�
	// fric wheel speed set
	if(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START)
	{
		gimbal_set_control_t->gimbal_fric_motor[0].speed_set = FRIC_SHOOT_LEFT_SPEED;
		gimbal_set_control_t->gimbal_fric_motor[1].speed_set = -FRIC_SHOOT_RIGHT_SPEED;
	}
	else if(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_STOP)
	{
		gimbal_set_control_t->gimbal_fric_motor[0].speed_set = 0;
		gimbal_set_control_t->gimbal_fric_motor[1].speed_set = 0;
		gimbal_set_control_t->gimbal_trigger_motor.speed_set = 0;
	}

	// trigger shoot delay
	if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_SINGLE_START) && (gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_SLOW;
	else if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_CONTINUOUS_START) && (gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_FAST;
	else if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_WITHOUT_CONTROL) && (gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_FAST;
	else
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_STOP;

	shoot_task(&gimbal_set_control_t->shoot_switch.trigger_state, gimbal_set_control_t->shoot_switch.shoot_mode, gimbal_set_control_t->shoot_switch.fric_mode, &gimbal_set_control_t->gimbal_trigger_motor.speed_set, gimbal_set_control_t->gimbal_trigger_motor.shoot_delay);

	if(gimbal_set_control_t->gimbal_mode == CHASSIS_NO_MOVE || gimbal_set_control_t->aimbot_mode == AIMBOT_ON)
	{
		return;
	}

	//ң����ҡ�˺�����Ŀ��Ƕ�
	gimbal_set_control_t->yaw_set = -(gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_YAW_CHANNEL] * GIMBAL_OPEN_RC_YAW_SEN + gimbal_set_control_t->gimbal_rc_ctrl->mouse.x*GIMBAL_OPEN_X_YAW_SEN);
	//����, ��ֹҡ�˻��в�׼����Ư��
	if(gimbal_set_control_t->yaw_set<=25&&gimbal_set_control_t->yaw_set>=-25)
		gimbal_set_control_t->yaw_set=0;
	gimbal_set_control_t->pitch_set = (gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_PITCH_CHANNEL] * GIMBAO_OPEN_RC_PITCH_FAST_SEN - gimbal_set_control_t->gimbal_rc_ctrl->mouse.y*GIMBAL_OPEN_Y_PITCH_SEN);
}

//���̶�����ת�ж�
void gimbal_to_chassis_follow_yaw_determine(gimbal_control_t *gimbal_chassis_follow_yaw)
{	
	//���̸���ģʽ�����������ֵ�������
	if(gimbal_chassis_follow_yaw->gimbal_yaw_motor.absolute_angle - M6020_GIMBAL_ANGLE_YAW != 0)
		gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd = gimbal_chassis_follow_yaw->gimbal_yaw_motor.absolute_angle - M6020_GIMBAL_ANGLE_YAW;
	//���ӻ�����
	if(gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd>HALF_ECD_RANGE)			
		gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd -= ECD_RANGE;
	else if(gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd<-HALF_ECD_RANGE)		
		gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd += ECD_RANGE;
	//��������
//	abs_limit(&gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd, GIMBAO_TURN_LIMIT);
	
	
		gimbal_chassis_follow_yaw->yaw = gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd;
}

//ң����������̨yaw
void gimbal_ins_yaw_control(gimbal_control_t *gimbal_ins)
{
	//ң�����Ļ������ۼ�, �ٵ��ӵ������ǵ�ǰ�Ƕ���
	gimbal_ins->gimbal_yaw_motor.add += gimbal_ins->yaw_set*YAW_INS_OPEN_RC_YAW_SEN;
	gimbal_ins->gimbal_yaw_motor.absolute_angle_set = gimbal_ins->ins_data.yaw_target + gimbal_ins->gimbal_yaw_motor.add;

	//yaw��ǶȻ�PID
	gimbal_ins->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_ins->gimbal_angle_yaw_ins_pid , gimbal_ins->ins_data.yaw,
																			gimbal_ins->gimbal_yaw_motor.absolute_angle_set, gimbal_ins->gimbal_yaw_motor.motor_gyro);
	//yaw���ٶȻ�PID
	gimbal_ins->gimbal_yaw_motor.current_set = PID_calc(&gimbal_ins->gimbal_speed_yaw_ins_pid, gimbal_ins->gimbal_yaw_motor.motor_gyro,
																	gimbal_ins->gimbal_yaw_motor.motor_gyro_set);
}


//�Ӿ���λ���ӹ���̨
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

	//pitch, ��λ������Ŀ����Ǿ��ԽǶ�Ŀ��
	aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_aimbot_point->target_pitch;
	//��е��λ����, ��ֹײ��λ
	gimbal_limit(&aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, PITCH_INS_MIN);
	//pitch��ǶȻ�PID
	aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_pitch_pid , aimbot_gimbal_control_point->ins_data.pitch,
																			aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro);
	//pitch���ٶȻ�PID
	aimbot_gimbal_control_point->gimbal_pitch_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_pitch_pid, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro,
																			aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set);

	//yaw, ��λ������Ŀ����Ǿ��ԽǶ�Ŀ��
	aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_aimbot_point->target_yaw;
	aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set = loop_fp32_constrain(aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set, -PI, PI);
	//yaw��ǶȻ�PID
	aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_yaw_pid , aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL] - aimbot_gimbal_control_point->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER,
																				aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro);
	//yaw���ٶȻ�PID
	aimbot_gimbal_control_point->gimbal_yaw_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_yaw_pid, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro,
																				aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set);
}


//ң����������̨pitch
void pitch_ins_PID_control(gimbal_control_t *pitch_ins_control)
{
	//�ڻ�е�г��ڲ�������������
	if(pitch_ins_control->gimbal_pitch_motor.absolute_angle_set < PITCH_INS_MAX && pitch_ins_control->gimbal_pitch_motor.absolute_angle_set > PITCH_INS_MIN)
		pitch_ins_control->gimbal_pitch_motor.add += pitch_ins_control->pitch_set*GIMBAL_PITCH_INS_RATE;
	pitch_ins_control->gimbal_pitch_motor.absolute_angle_set = pitch_ins_control->gimbal_pitch_motor.add + pitch_ins_control->ins_data.pitch_target;
	//pitch��е��λ
	gimbal_limit(&pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, PITCH_INS_MIN);

	//pitch��ǶȻ�PID
	pitch_ins_control->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_ins_control->gimbal_angle_pitch_ins_pid , pitch_ins_control->ins_data.pitch,
																			pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, pitch_ins_control->gimbal_pitch_motor.motor_gyro);
	//pitch���ٶȻ�PID
	pitch_ins_control->gimbal_pitch_motor.current_set = PID_calc(&pitch_ins_control->gimbal_speed_pitch_ins_pid, pitch_ins_control->gimbal_pitch_motor.motor_gyro,
																	pitch_ins_control->gimbal_pitch_motor.motor_gyro_set);
}


void fric_motor_PID_control(gimbal_control_t *fric_motore_control)
{
	for(int i = 0; i < 2; i++)
	{
		fric_motore_control->gimbal_fric_motor[i].current_set = PID_calc(&fric_motore_control->gimbal_speed_fric_pid[i], fric_motore_control->gimbal_fric_motor[i].speed, fric_motore_control->gimbal_fric_motor[i].speed_set);
		fric_motore_control->gimbal_fric_motor[i].give_current = fric_motore_control->gimbal_fric_motor[i].current_set;
	}
}

void trigger_motor_PID_control(gimbal_control_t *trigger_motore_control)
{
	trigger_motore_control->gimbal_trigger_motor.target_ecd += trigger_motore_control->gimbal_trigger_motor.speed_set;
	trigger_motore_control->gimbal_trigger_motor.current_set = PID_calc(&trigger_motore_control->gimbal_speed_trigger_pid, trigger_motore_control->gimbal_trigger_motor.speed, trigger_motore_control->gimbal_trigger_motor.speed_set);
	trigger_motore_control->gimbal_trigger_motor.give_current = trigger_motore_control->gimbal_trigger_motor.current_set;
}

void gimbal_control_loop(gimbal_control_t *gimbal_control_loop_t)
{
	static uint8_t aimbot_last_mode = AIMBOT_OFF;
	//����̨�͵��̵Ĳ�ֵ, �����̸�����
	gimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);

	//����, ���߱��Ӿ��ӹܵ�ʱ��, ң�����Ļ���������
		// �Ӿ�������һ˲��ҲҪ����Ŀ���, �����ֶ��������̨���ؿ�������Ƕ�
	if(gimbal_control_loop_t->gimbal_mode == CHASSIS_NO_MOVE ||
	   gimbal_control_loop_t->aimbot_mode == AIMBOT_ON ||
	   aimbot_last_mode == AIMBOT_ON)
	{
		gimbal_control_loop_t->gimbal_yaw_motor.add = 0;
		gimbal_control_loop_t->gimbal_pitch_motor.add = 0;
		//��ң������targetֵͬ������ǰ�Ƕ�, �л�ң������ʱ�򲻻���תͷ
		gimbal_control_loop_t->ins_data.yaw_target = gimbal_control_loop_t->ins_data.yaw;
		gimbal_control_loop_t->ins_data.pitch_target = gimbal_control_loop_t->ins_data.pitch;
	}
	aimbot_last_mode = gimbal_control_loop_t->aimbot_mode;


	//��ʽ��ѡһ: �Ӿ���λ���ӹ� ���� ң�����ӹ�
	if(gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)
	{
		//�Ӿ���λ��������̨
		aimbot_gimbal_control(gimbal_control_loop_t);
	}
	else
	{
		//ң����������̨
		gimbal_ins_yaw_control(gimbal_control_loop_t);
		pitch_ins_PID_control(gimbal_control_loop_t);
	}

	//��̨yaw�������������
	gimbal_control_loop_t->gimbal_yaw_motor.give_current = gimbal_control_loop_t->gimbal_yaw_motor.current_set;
	//��̨pitch�������������
	gimbal_control_loop_t->gimbal_pitch_motor.give_current = gimbal_control_loop_t->gimbal_pitch_motor.current_set;

	// trigger / friction wheel PID control
	fric_motor_PID_control(gimbal_control_loop_t);
	trigger_motor_PID_control(gimbal_control_loop_t);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_EXTI_Callback_INS(GPIO_Pin);
}


/**
  * @brief          ��ȡ��̨����ָ��
  * @param[in]      none
  * @retval         ��ȡ��̨����ָ��
  */
const gimbal_control_t *get_gimbal_move_point(void)
{
    return &gimbal_control;
}
