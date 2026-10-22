/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
motor_measure_t motor_chassis[8];
motor_measure_t motor_gimbal[8];

static CAN_TxHeaderTypeDef  steer_tx_message;
static uint8_t              steer_can_send_data[8];
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];
static CAN_TxHeaderTypeDef  vision_tx_message;
static uint8_t              vision_can_send_data[8];

vision_cmd_t vision_cmd;

	
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
   //can1 ����
	if(hcan->Instance == hcan1.Instance)
	{
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,rx_data) == HAL_OK)
		{
			switch (rx_header.StdId)
			{
//				case CAN_3508_M1_ID:
//					get_motor_measure(&motor_chassis[0], rx_data);
//					break;
//				case CAN_3508_M2_ID:
//					get_motor_measure(&motor_chassis[1], rx_data);
//					break;
//				case CAN_3508_M3_ID:
//					get_motor_measure(&motor_chassis[2], rx_data);
//					break;
//				case CAN_3508_M4_ID:
//					get_motor_measure(&motor_chassis[3], rx_data);
//					break;
				case CAN_TRIGGER_ID:
					get_motor_measure(&motor_gimbal[2], rx_data);
					break;
				case CAN_FRIC_LEFT:
					get_motor_measure(&motor_gimbal[3], rx_data);
					break;
				case CAN_FRIC_RIGHT:
					get_motor_measure(&motor_gimbal[4], rx_data);
					break;
				case CAN_GIMBAL_M1_ID:
					get_motor_measure(&motor_gimbal[0], rx_data);
					break;
				case CAN_GIMBAL_M2_ID:
					get_motor_measure(&motor_gimbal[1], rx_data);
					break;
				case CAN_VISION_CMD_ID:
					vision_cmd.control = rx_data[0];
					vision_cmd.shoot   = rx_data[1];
					vision_cmd.yaw     = (fp32)(int16_t)((rx_data[2] << 8) | rx_data[3]) / 1e4f;
					vision_cmd.pitch   = (fp32)(int16_t)((rx_data[4] << 8) | rx_data[5]) / 1e4f;
					vision_cmd.horizon_distance = (fp32)(int16_t)((rx_data[6] << 8) | rx_data[7]) / 1e4f;
					break;

				default:
					break;
			}
		}
	}
	//can2����
	else if(hcan->Instance == hcan2.Instance)
	{

		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,rx_data) == HAL_OK)
		{
			switch (rx_header.StdId)
			{
				case CAN_STEER_M1_ID:
					get_motor_measure(&motor_chassis[4], rx_data);
					break;
				case CAN_STEER_M2_ID:
					get_motor_measure(&motor_chassis[5], rx_data);
					break;
				case CAN_STEER_M3_ID:
					get_motor_measure(&motor_chassis[6], rx_data);
					break;
				case CAN_STEER_M4_ID:
					get_motor_measure(&motor_chassis[7], rx_data);
					break;
				case CAN_TRIGGER_ID:
					get_motor_measure(&motor_gimbal[2], rx_data);
					break;
				case CAN_FRIC_LEFT:
					get_motor_measure(&motor_gimbal[3], rx_data);
					break;
				case CAN_FRIC_RIGHT:
					get_motor_measure(&motor_gimbal[4], rx_data);
					break;
				case CAN_VISION_CMD_ID:
					vision_cmd.control = rx_data[0];
					vision_cmd.shoot   = rx_data[1];
					vision_cmd.yaw     = (fp32)(int16_t)((rx_data[2] << 8) | rx_data[3]) / 1e4f;
					vision_cmd.pitch   = (fp32)(int16_t)((rx_data[4] << 8) | rx_data[5]) / 1e4f;
					vision_cmd.horizon_distance = (fp32)(int16_t)((rx_data[6] << 8) | rx_data[7]) / 1e4f;
					break;

				default:
					break;
			}
		}

	}
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      motor1: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      motor2: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      motor3: (0x207) 6020 motor control current, range [-30000,30000]
  * @param[in]      motor4: (0x208) 6020 motor control current, range [-30000,30000]
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      motor2: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      motor3: (0x207) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      motor4: (0x208) 6020������Ƶ���, ��Χ [-30000,30000]
  * @retval         none
  */
void CAN_cmd_chassis_steer(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    steer_tx_message.StdId = CAN_STEER_ALL_ID;
    steer_tx_message.IDE = CAN_ID_STD;
    steer_tx_message.RTR = CAN_RTR_DATA;
    steer_tx_message.DLC = 0x08;
    steer_can_send_data[0] = (motor1 >> 8);
    steer_can_send_data[1] = motor1;
    steer_can_send_data[2] = (motor2 >> 8);
    steer_can_send_data[3] = motor2;
    steer_can_send_data[4] = (motor3 >> 8);
    steer_can_send_data[5] = motor3;
    steer_can_send_data[6] = (motor4 >> 8);
    steer_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&STEER_CAN, &steer_tx_message, steer_can_send_data, &send_mail_box);
}

/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x205) 2006������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      motor2: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (motor1 >> 8);
    gimbal_can_send_data[1] = motor1;
    gimbal_can_send_data[2] = (motor2 >> 8);
    gimbal_can_send_data[3] = motor2;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          ���͵�����Ƶ���(0x204,0x202,0x203)
  * @param[in]      motor1: (0x205) triggerfric������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      motor2: (0x206) fric������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      motor3: (0x207) fric������Ƶ���, ��Χ [-30000,30000]
  * @retval         none
  */
void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (motor1 >> 8);
    shoot_can_send_data[1] = motor1;
    shoot_can_send_data[2] = (motor2 >> 8);
    shoot_can_send_data[3] = motor2;
    shoot_can_send_data[4] = (motor3 >> 8);
    shoot_can_send_data[5] = motor3;
    shoot_can_send_data[6] = 0;
    shoot_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ص���6020���������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_steer_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ������̨6020���������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i)
{
    return &motor_gimbal[i];
}

/**
  * @brief          i=0ʱ��trigger��i=1ʱ��fric_left��i=2ʱ��fric_right
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_shoot_motor_measure_point(uint8_t i)
{
    return &motor_gimbal[i+2];
}
/* ========== �Ӿ�(��λ�� NUC) CAN ͨ�� ========== */

const vision_cmd_t *get_vision_cmd_point(void)
{
    return &vision_cmd;
}

static void vision_tx_message_set(uint32_t std_id, uint8_t dlc)
{
    vision_tx_message.StdId = std_id;
    vision_tx_message.IDE = CAN_ID_STD;
    vision_tx_message.RTR = CAN_RTR_DATA;
    vision_tx_message.DLC = dlc;
}

// �ϱ���Ԫ�� 0x100: [x,y,z,w] �� int16 ���, ���� x1e4
void CAN_cmd_vision_quat(const fp32 quat[4])
{
    uint32_t send_mail_box;
    // ��Ԫ��˳��: INS_quat=[w,x,y,z](DJI Mahony), �Ӿ�cboardҪ[x,y,z,w]
    int16_t x = (int16_t)(quat[1] * 1e4f);
    int16_t y = (int16_t)(quat[2] * 1e4f);
    int16_t z = (int16_t)(quat[3] * 1e4f);
    int16_t w = (int16_t)(quat[0] * 1e4f);
    vision_tx_message_set(CAN_VISION_QUAT_ID, 0x08);
    vision_can_send_data[0] = (uint8_t)(x >> 8);
    vision_can_send_data[1] = (uint8_t)x;
    vision_can_send_data[2] = (uint8_t)(y >> 8);
    vision_can_send_data[3] = (uint8_t)y;
    vision_can_send_data[4] = (uint8_t)(z >> 8);
    vision_can_send_data[5] = (uint8_t)z;
    vision_can_send_data[6] = (uint8_t)(w >> 8);
    vision_can_send_data[7] = (uint8_t)w;
    HAL_CAN_AddTxMessage(&VISION_CAN, &vision_tx_message, vision_can_send_data, &send_mail_box);
}

// �ϱ����� 0x101: ����x1e2(int16���), mode(u8), shoot_mode(u8), ft_angle x1e4(int16���)
void CAN_cmd_vision_feedback(fp32 bullet_speed, uint8_t mode, uint8_t shoot_mode, fp32 ft_angle)
{
    uint32_t send_mail_box;
    int16_t bs = (int16_t)(bullet_speed * 1e2f);
    int16_t ft = (int16_t)(ft_angle * 1e4f);
    vision_tx_message_set(CAN_VISION_FEED_ID, 0x06);
    vision_can_send_data[0] = (uint8_t)(bs >> 8);
    vision_can_send_data[1] = (uint8_t)bs;
    vision_can_send_data[2] = mode;
    vision_can_send_data[3] = shoot_mode;
    vision_can_send_data[4] = (uint8_t)(ft >> 8);
    vision_can_send_data[5] = (uint8_t)ft;
    HAL_CAN_AddTxMessage(&VISION_CAN, &vision_tx_message, vision_can_send_data, &send_mail_box);
}

