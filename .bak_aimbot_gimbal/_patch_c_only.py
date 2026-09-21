# -*- coding: utf-8 -*-
# aimbot_task.c / .h 适配
# 1) 关掉三个调试开关 (FREEZE_PITCH / FREEZE_YAW 之前是 1, 云台在自瞄模式下根本不出力)
# 2) 删掉四元数顺序调试残留 (aimbot_dbg_yaw_wxyz / _xyzw + <math.h>)
# 3) 0x101 反馈帧: 弹速改成真实弹速 get_bullet_speed(), mode 跟着 aimbot_mode 走
# 逐条精确匹配替换, 无一命中即报错退出; 写完校验 U+FFFD == 0

import sys

def patch(path, edits, expect_nl):
    d = open(path, 'rb').read()
    t = d.decode('gbk')
    for old, new in edits:
        n = t.count(old)
        if n != 1:
            sys.exit('EDIT HIT %d (expected 1) in %s:\n%r' % (n, path, old[:120]))
        t = t.replace(old, new)
    b = t.encode('gbk')
    assert b.count(b'\xef\xbf\xbd') == 0, path
    # 只约束"那条 CRLF 还在"以及"没有裸 CR", 行数会因为删改而变化
    assert b.count(b'\r\n') == expect_nl[0], (path, b.count(b'\r\n'))
    assert b.count(b'\r') == b.count(b'\r\n'), (path, 'stray CR')
    open(path, 'wb').write(b)
    print('OK %-28s CRLF=%d bareLF=%d FFFD=%d' % (path, b.count(b'\r\n'),
          b.count(b'\n') - b.count(b'\r\n'), b.count(b'\xef\xbf\xbd')))


# ---------------- aimbot_task.c : 1 CRLF + 86 bare LF ----------------
patch('application/aimbot_task.c', [
    # 1) <math.h> 只被 atan2f 调试用, 一并删掉
    ('#include "referee.h"\n#include <math.h>\n', '#include "referee.h"\n'),

    # 2) 四元数顺序调试全局变量
    ('aimbot_task_t aimbot;\n'
     '\n'
     '// 调试: 判断 INS_quat 的真实顺序. 与 INS_angle[0] 对比, 哪个相等就是哪个\n'
     'fp32 aimbot_dbg_yaw_wxyz = 0.0f;\t// 假设 INS_quat = [w,x,y,z]\n'
     'fp32 aimbot_dbg_yaw_xyzw = 0.0f;\t// 假设 INS_quat = [x,y,z,w]\n',
     'aimbot_task_t aimbot;\n'),

    # 3) 两轴调试注释挪到真正的调试分支上, 并删掉 atan2f 调试解算
    ('        // 视觉下发 control=1 时接管目标角度; 否则退出自瞄\n',
     '        // 视觉下发 control=1 时接管目标角度; 否则退出自瞄\n'
     '        // 注意: 真正的接管与否由 gimbal_set_mode() 决定(还要开关在自瞄位), 这里只准备目标值\n'),

    ('        // 上报四元数 [x,y,z,w] 给视觉做姿态/EKF\n'
     '#if AIMBOT_DEBUG_BYPASS\n'
     '        // 调试: 强制固定目标, 完全忽略视觉下发的 yaw/pitch\n'
     '        aimbot.target_yaw   = AIMBOT_DEBUG_YAW;\n'
     '        aimbot.target_pitch = AIMBOT_DEBUG_PITCH;\n'
     '        aimbot.aimbot_state = AIMBOT_GET;\n'
     '#endif\n'
     '        // 调试: 两种四元数顺序各解一次 yaw\n'
     '        aimbot_dbg_yaw_wxyz = atan2f(2.0f*(quat[0]*quat[3] + quat[1]*quat[2]), 1.0f - 2.0f*(quat[2]*quat[2] + quat[3]*quat[3]));\n'
     '        aimbot_dbg_yaw_xyzw = atan2f(2.0f*(quat[3]*quat[2] + quat[0]*quat[1]), 1.0f - 2.0f*(quat[1]*quat[1] + quat[2]*quat[2]));\n'
     '\n'
     '        CAN_cmd_vision_quat(quat);\n',
     '#if AIMBOT_DEBUG_BYPASS\n'
     '        // 调试: 强制固定目标, 完全忽略视觉下发的 yaw/pitch\n'
     '        aimbot.target_yaw   = AIMBOT_DEBUG_YAW;\n'
     '        aimbot.target_pitch = AIMBOT_DEBUG_PITCH;\n'
     '        aimbot.aimbot_state = AIMBOT_GET;\n'
     '#endif\n'
     '\n'
     '        // 上报四元数 [x,y,z,w] 给视觉做姿态/EKF\n'
     '        CAN_cmd_vision_quat(quat);\n'),

    # 4) 反馈帧: 弹速写死 25 会让视觉弹道解算直接算错; mode 要对齐 cboard 的 enum Mode
    ('        // 上报弹速/模式反馈给视觉做弹道解算\n'
     '//        CAN_cmd_vision_feedback(get_bullet_speed(), aimbot.aimbot_mode, 0, 0.0f);\n'
     '\t\t\t\tCAN_cmd_vision_feedback(25.0f, 1, 0, 0.0f);  \n',
     '        // 上报弹速/模式反馈给视觉做弹道解算\n'
     '        // mode: cboard 的 enum Mode { idle=0, auto_aim=1 }, 自瞄中才报 auto_aim\n'
     '        // shoot_mode: ShootMode { left_shoot=0, right_shoot=1, both_shoot=2 }, 两轮都转就报 both_shoot\n'
     '        // ft_angle: 哨兵/无人机专有量, 这里不用\n'
     '        CAN_cmd_vision_feedback(get_bullet_speed(), (aimbot.aimbot_mode == AIMBOT_ON) ? 1 : 0, 2, 0.0f);\n'),
], (1, 86))
