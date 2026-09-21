/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN	hcan1
#define STEER_CAN	hcan2
#define GIMBAL_CAN	hcan1
#define SHOOT_CAN	hcan2
/* CAN send and receive ID */
typedef enum
{

    CAN_CHASSIS_ALL_ID = 0x200,	//CAN1
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
	CAN_GIMBAL_ALL_ID = 0x1FF,	//CAN1
	CAN_GIMBAL_M1_ID = 0x205,
	CAN_GIMBAL_M2_ID = 0x206,
	
	
	CAN_SHOOT_ALL_ID = 0x200,	//CAN2
	CAN_TRIGGER_ID = 0x201,
	CAN_FRIC_LEFT = 0x202,
	CAN_FRIC_RIGHT = 0x203,
	
    CAN_STEER_ALL_ID = 0x1FF,	//CAN2
    CAN_STEER_M1_ID = 0x205,
    CAN_STEER_M2_ID = 0x206,
    CAN_STEER_M3_ID = 0x207,
	CAN_STEER_M4_ID = 0x208,
	
	
	
} can_msg_id_e;

/* 视觉(上位机 NUC)通信 CAN 协议 —— 与视觉 cboard.cpp 严格一致
   0x0ff:视觉下发命令, 0x100:上报四元数, 0x101:上报弹速/模式 */
#define VISION_CAN            hcan1   // 视觉NUC接入的CAN(若接hcan2请改这里)
#define CAN_VISION_CMD_ID    0x0ff
#define CAN_VISION_QUAT_ID   0x100
#define CAN_VISION_FEED_ID   0x101

// 视觉下发的自瞄命令(0x0ff 解析结果)
typedef struct
{
    uint8_t control;           // 1=有目标,接管云台自瞄; 0=无目标
    uint8_t shoot;             // 1=允许射击(视觉auto_fire)
    fp32    yaw;               // 目标yaw角 (rad)
    fp32    pitch;             // 目标pitch角 (rad)
    fp32    horizon_distance;  // 水平距离 (m)
} vision_cmd_t;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_chassis_steer(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t motor1, int16_t motor2);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      motor1: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      motor2: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      motor3: (0x207) 6020 motor control current, range [-30000,30000]
  * @param[in]      motor4: (0x208) 6020 motor control current, range [-30000,30000]
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      motor2: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      motor3: (0x207) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      motor4: (0x208) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回底盘6020电机的数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_steer_measure_point(uint8_t i);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回云台6020电机的数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回云台6020电机的数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);

/* 视觉通信接口 */
extern vision_cmd_t vision_cmd;
extern const vision_cmd_t *get_vision_cmd_point(void);
extern void CAN_cmd_vision_quat(const fp32 quat[4]);
extern void CAN_cmd_vision_feedback(fp32 bullet_speed, uint8_t mode, uint8_t shoot_mode, fp32 ft_angle);

#endif
