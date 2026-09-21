h = open('application/gimbal_task.h','rb').read().decode('gbk').splitlines(True)
c = open('application/gimbal_task.c','rb').read().decode('gbk').splitlines(True)

out = []
def add(lines):
    if isinstance(lines, str):
        out.append(lines)
    else:
        out.extend(lines)

add('#ifndef _AIMBOT_GIMBAL_H_\r\n')
add('#define _AIMBOT_GIMBAL_H_\r\n')
add('\r\n')
add(h[3:9])            # main.h .. aimbot_task.h
add(h[10:12])          # referee.h, shoot.h   (skip chassis_cruise.h)
add(h[12])             # blank
add(h[13])             # #define ABS(x)
add(h[14])             # blank
add(h[15:22])          # 任务初始化注释 .. TRIGGER_SHOOT_STOP
add(h[22:25])          # 三个 TRIGGER_SHOOT 宏
add(h[27:29])          # 面对哨兵注释 + FRIC_SHOOT_LEFT_SPEED
add(h[30])             # FRIC_SHOOT_RIGHT_SPEED
add(h[31:33])          # blank, blank
add(h[40:42])          # 遥控器灵敏度注释 + GIMBAL_OPEN_RC_YAW_SEN
add(h[43])             # GIMBAO_OPEN_RC_PITCH_FAST_SEN
add(h[44])             # //陀螺仪PITCH轴转换比例
add(h[46])             # GIMBAL_PITCH_INS_RATE
add(h[47:49])          # 陀螺仪yaw轴灵敏度注释 + YAW_INS_OPEN_RC_YAW_SEN
add(h[49:52])          # 鼠标灵敏度注释 + 两个 27.0f
add(h[55:59])          # RC 通道注释 + 三个通道宏
add(h[59:62])          # blank + yaw轴起始位置注释 + M6020_GIMBAL_ANGLE_YAW
add(h[67:69])          # PITCH_INS_MAX / MIN
add('\r\n')
add('//=================== PID 参数 ===================\r\n')
add(c[19:61] + c[89:103] + c[110:124])         # 原 gimbal_task.c 里的全部 PID 宏（含中文注释）
add('\r\n')
add(h[81:87])          # hatch_state_e
add('\r\n')
add(h[89:108])         # gimbal_PID_t
add('\r\n')
add(h[109:138])        # gimbal_motor_t
add('\r\n')
add(h[140:156])        # 陀螺仪数据注释 + ins_data_t
add('\r\n')
drop = {162,166,167,169,175,179,180,185,186,188,193,194,212,213,214,215,216,217,223}
body = [ln for i, ln in enumerate(h[158:230]) if (i + 158) not in drop]
add(body)
add('\r\n')
add(h[231:239])        # extern 声明
add('\r\n')
add('\r\n')
add('#endif\r\n')

txt = ''.join(out).replace('\r\n', '\n').replace('\n', '\r\n')
data = txt.encode('gbk')
open('application/aimbot_gimbal.h','wb').write(data)
print('bytes', len(data), 'CRLF', data.count(b'\r\n'), 'bareLF', data.count(b'\n')-data.count(b'\r\n'), 'FFFD', data.count(b'\xef\xbf\xbd'))
