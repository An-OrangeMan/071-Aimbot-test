#ifndef _AIMBOT_GIMBAL_H_
#define _AIMBOT_GIMBAL_H_

#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "aimbot_task.h"
#include "shoot.h"

#define ABS(x) (((x)>0)?(x):(-(x)))



//�����ʼ�� 
#define GIMBAL_TASK_INIT_TIME 500

#define GIMBAL_CONTROL_TIME 1
//С������̨yaw�Ჹ��
#define GIMBAL_SPIN_YAW_BUFFER	0.0005f

//ң���� ������
#define GIMBAL_OPEN_RC_YAW_SEN		3.0f		//Ӱ����������̸���ģʽ�µ����ƫת�Ƕ�
#define GIMBAO_OPEN_RC_PITCH_FAST_SEN	1.70f		//0.0024f
//������PITCH��ת������
#define GIMBAL_PITCH_INS_RATE			0.0000013f
//������yaw��������
#define YAW_INS_OPEN_RC_YAW_SEN		0.000002f			//������ģʽ������
//���	������
#define GIMBAL_OPEN_X_YAW_SEN		27.0f
#define GIMBAL_OPEN_Y_PITCH_SEN		27.0f
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define RC_YAW_CHANNEL   0
#define RC_PITCH_CHANNEL 1
#define SHOOT_MODE_CHANNEL 4

//yaw����ʼλ��
#define M6020_GIMBAL_ANGLE_YAW			   354//1394
#define PITCH_INS_MAX				0.483058602f//0.5f
#define PITCH_INS_MIN				-0.404820025f//0.25f

//=================== PID ���� ===================
//aimbot pitch�ǶȻ�PID
#define AIMBOT_PITCH_ANGLE_PID_KP        	25.0f	//25000.0f
#define AIMBOT_PITCH_ANGLE_PID_KI        	0.0f   //0.0f
#define AIMBOT_PITCH_ANGLE_PID_KD        	0.0f 	//200000.0f
// ��������ԭ�������ٻ����˦������ pitch(rad/s), 10000 = ������
#define AIMBOT_PITCH_ANGLE_PID_MAX_OUT   	5//2.5f   //300000.0f
#define AIMBOT_PITCH_ANGLE_PID_MAX_IOUT  	50.0f   //50.	0f
                                             
//aimbot pitch�ٶȻ� PID                                          
#define AIMBOT_PITCH_SPEED_PID_KP        	7000.0f//4500.0f   //2.4f
#define AIMBOT_PITCH_SPEED_PID_KI        	0.0f   //0.0f
#define AIMBOT_PITCH_SPEED_PID_KD        	40.0f   //200.0f
#define AIMBOT_PITCH_SPEED_PID_MAX_OUT   	30000.0f   //10000.0f
#define AIMBOT_PITCH_SPEED_PID_MAX_IOUT  	1000.0f   //1000.0f
                                            
//aimbot yaw�ǶȻ�PID                         
#define AIMBOT_YAW_ANGLE_PID_KP        				25.0f  //8000.0f	//25000.0f
#define AIMBOT_YAW_ANGLE_PID_KI        				0.0f  //0.0f   //0.0f
#define AIMBOT_YAW_ANGLE_PID_KD        				0.0f  //2000.0f 	//200000.0f
#define AIMBOT_YAW_ANGLE_PID_MAX_OUT   				20000.0f   //300000.0f
#define AIMBOT_YAW_ANGLE_PID_MAX_IOUT  				30.0f   //50.0f
                                                     
//aimbot yaw�ٶȻ� PID                                                  
#define AIMBOT_YAW_SPEED_PID_KP        				5000.0f//1200.0f  //50000.0f   //2.4f
#define AIMBOT_YAW_SPEED_PID_KI        				0.0f  //0.0f   //0.0f
#define AIMBOT_YAW_SPEED_PID_KD        				0.0f  //20.0f   //200.0f
#define AIMBOT_YAW_SPEED_PID_MAX_OUT   				30000.0f   //10000.0f
#define AIMBOT_YAW_SPEED_PID_MAX_IOUT  				1000.0f   //1000.0f
                                            
//������yaw �ǶȻ� PID                              
#define YAW_INS_ANGLE_PID_KP       		 		5.0f	//25000.0f
#define YAW_INS_ANGLE_PID_KI       				0.0f   //0.0f
#define YAW_INS_ANGLE_PID_KD        			0.0f 	//200000.0f
#define YAW_INS_ANGLE_PID_MAX_OUT   			300000.0f   //300000.0f
#define YAW_INS_ANGLE_PID_MAX_IOUT  			30.0f   //50.0f
                                                 
//������yaw �ٶȻ� PID                                             
#define YAW_INS_SPEED_PID_KP        		  2000.0f//24000.0f   
#define YAW_INS_SPEED_PID_KI        			0.0f   //0.0f
#define YAW_INS_SPEED_PID_KD        			0.0f   //200.0f
#define YAW_INS_SPEED_PID_MAX_OUT   			30000.0f   //10000.0f
#define YAW_INS_SPEED_PID_MAX_IOUT  			1000.0f   //1000.0f
                                            
//������pitch �ǶȻ� PID                              
#define PITCH_INS_ANGLE_PID_KP       		 	25.0f	//25000.0f
#define PITCH_INS_ANGLE_PID_KI       			0.0f   //0.0f
#define PITCH_INS_ANGLE_PID_KD        			0.0f 	//200000.0f
#define PITCH_INS_ANGLE_PID_MAX_OUT   			300000.0f   //300000.0f
#define PITCH_INS_ANGLE_PID_MAX_IOUT  			50.0f   //50.	0f
                                                 
//������pitch �ٶȻ� PID                                             
#define PITCH_INS_SPEED_PID_KP        			4500    //11500.0f   
#define PITCH_INS_SPEED_PID_KI        			0.0f   //0.0f
#define PITCH_INS_SPEED_PID_KD        			20.0f   //200.0f
#define PITCH_INS_SPEED_PID_MAX_OUT   			20000.0f   //10000.0f
#define PITCH_INS_SPEED_PID_MAX_IOUT  			1000.0f   //1000.0f

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

//��̨���
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;

	fp32 add;				//ң����������
	fp32 absolute_angle;	//rad, ���������ԽǶ�
	fp32 absolute_angle_set;//rad, Ŀ����ԽǶ�
	fp32 motor_gyro;		//rad/s, �����ǽ��ٶ�
	fp32 motor_gyro_set;
	fp32 speed;				//rpm
	fp32 current_set;
	int16_t give_current;
} gimbal_motor_t;

//����������
typedef struct
{
	fp32 yaw_target;	//��̨yaw����ԽǶ�Ŀ��
	fp32 pitch_target;	//��̨pitch����ԽǶ�Ŀ��

	fp32 roll;
	fp32 pitch;
	fp32 yaw;
	fp32 yaw_last;		//��һ֡yaw, ���ڽ����
}ins_data_t;

//��̨���ƽṹ��
typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;			//ң����
	const aimbot_task_t *gimbal_aimbot_point;	//�Ӿ���λ������Ŀ��Ƕ�
	const fp32 *gimbal_INT_angle_point;			//�����ǽǶ�
	const fp32 *gimbal_INT_gyro_point;			//�����ǽ��ٶ�

	uint8_t aimbot_mode;	//AIMBOT_OFF: ң��������, AIMBOT_ON: �Ӿ���λ������
	uint8_t gimbal_mode;	//CHASSIS_NO_MOVE: ����, CHASSIS_FOLLOW_INS_YAW: �����ǿ���
	ins_data_t	ins_data;

	fp32 yaw;	//��̨����̲�ֵ, �����̸�����

	//��̨����̲�ֵ
	fp32 err_gimbal_chassis_ecd;

	fp32 yaw_set;		//ң��������Ŀ��
	fp32 pitch_set;		//ң��������Ŀ��

	gimbal_motor_t gimbal_yaw_motor;	//yaw���
	gimbal_motor_t gimbal_pitch_motor;	//pitch���

	//�Ӿ���λ������PID
	pid_type_def aimbot_speed_pitch_pid;
	gimbal_PID_t aimbot_angle_pitch_pid;
	pid_type_def aimbot_speed_yaw_pid;
	gimbal_PID_t aimbot_angle_yaw_pid;
	//ң����������yaw��PID
	pid_type_def gimbal_speed_yaw_ins_pid;
	gimbal_PID_t gimbal_angle_yaw_ins_pid;
	//ң����������pitch��PID
	pid_type_def gimbal_speed_pitch_ins_pid;
	gimbal_PID_t gimbal_angle_pitch_ins_pid;

	// shooting (trigger + friction wheels)
	encoding_switch_t shoot_switch;
	shoot_motor_t gimbal_trigger_motor;
	shoot_motor_t gimbal_fric_motor[2];
	pid_type_def gimbal_speed_trigger_pid;
	pid_type_def gimbal_speed_fric_pid[2];
} gimbal_control_t;

extern void gimbal_task(void const *pvParameters);
extern void gimbal_init(gimbal_control_t *gimbal_init);
extern void gimbal_set_mode(gimbal_control_t *gimbal_mode_set);
extern void gimbal_set_control(gimbal_control_t *gimbal_set_control_t);
extern void gimbal_feedback_update(gimbal_control_t *feedback_update);
extern void gimbal_control_loop(gimbal_control_t *gimbal_control_loop);
extern void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
extern const gimbal_control_t *get_gimbal_move_point(void);

#endif
