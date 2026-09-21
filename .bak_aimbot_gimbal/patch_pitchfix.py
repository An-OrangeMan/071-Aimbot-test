# -*- coding: utf-8 -*-
# 这一轮的两处修复, 做成可重跑的脚本.
#   !! 注意 !!  如果重新跑 gen_min.py 重新生成 aimbot_gimbal.{h,c},
#   这两处修复会被覆盖掉, 必须再跑一次本脚本.
# 用法:  cd D:/Document/Code/Aimbot_test/ShaoBing1 && python .bak_aimbot_gimbal/patch_pitchfix.py
CR = chr(13)
LF = chr(10)
TAB = chr(9)
H = 'application/aimbot_gimbal.h'
C = 'application/aimbot_gimbal.c'

# 中文一律用码点构造, 避免任何编码/转义在不同 shell 下被改坏
Z = lambda *c: ''.join(chr(x) for x in c)
CMT_H = '// ' + Z(0x81ea, 0x7784, 0x9650, 0x901f, 0x539f, 0x6765, 0x4e0d, 0x9650, 0x901f,
                  0x4f1a, 0x8fc7, 0x51b2, 0x7529, 0x51fa, 0x753b, 0x9762) \
        + ' pitch(rad/s), 10000 = ' + Z(0x4e0d, 0x9650, 0x901f)
CMT_C = TAB + '// ' + Z(0x89c6, 0x89c9, 0x653e, 0x624b, 0x90a3, 0x4e00, 0x77ac, 0x95f4,
                       0x4e5f, 0x8981, 0x91cd, 0x9501, 0x76ee, 0x6807, 0x89d2) \
        + ', ' + Z(0x5426, 0x5219, 0x624b, 0x52a8, 0x73af, 0x4f1a, 0x628a, 0x4e91, 0x53f0,
                   0x62c9, 0x56de, 0x5f00, 0x673a, 0x9501, 0x5b58, 0x89d2, 0x5ea6)


def wr(p, t):
    d = t.encode('gbk')
    assert d.count(b'\xef\xbf\xbd') == 0, 'U+FFFD!'
    open(p, 'wb').write(d)
    print('  wrote %s  %d bytes' % (p, len(d)))


# ---------- 修复1: aimbot pitch 角度环限速 ----------
# 角度环输出就是 motor_gyro_set, 单位 rad/s. 原来 10000 等于不限速,
# 捕获时 10 度以上误差会命令 300 度/秒, 必然过冲把装甲板甩出画面 -> control=0
t = open(H, 'rb').read().decode('gbk')
KEY = '#define AIMBOT_PITCH_ANGLE_PID_MAX_OUT'
if '2.5f' in t[t.index(KEY):t.index(CR + LF, t.index(KEY))]:
    print('修复1 已应用, 跳过')
else:
    i = t.index(KEY)
    e = t.index(CR + LF, i)
    line = t[i:e]
    assert '10000.0f' in line, repr(line)
    t = t[:i] + CMT_H + CR + LF + line.replace('10000.0f', '2.5f') + t[e:]
    wr(H, t)
    print('修复1 OK')

# ---------- 修复2: 视觉放手那一拍也要重锁目标角 ----------
# 原来只在 aimbot ON 或 CHASSIS_NO_MOVE 时把 ins_data.pitch_target 重锁到当前角度,
# 视觉放手的瞬间不重锁, 手动环就把云台拽回 gimbal_init 时锁存的开机角度 -> 来回抬头/低头
t = open(C, 'rb').read().decode('gbk')
if 'aimbot_last_mode' in t:
    print('修复2 已应用, 跳过')
else:
    FN = 'void gimbal_control_loop(gimbal_control_t *gimbal_control_loop_t)'
    f = t.index(FN)
    be = t.index(CR + LF, t.index('{', f)) + 2
    t = t[:be] + TAB + 'static uint8_t aimbot_last_mode = AIMBOT_OFF;' + CR + LF + t[be:]

    i = t.index('if(gimbal_control_loop_t->gimbal_mode == CHASSIS_NO_MOVE || '
                'gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)')
    TAIL = 'gimbal_control_loop_t->ins_data.pitch_target = gimbal_control_loop_t->ins_data.pitch;'
    j = t.index(TAIL, i)
    j = t.index(CR + LF, j) + 2
    assert t[j:j + 2] == TAB + '}', repr(t[j:j + 4])
    j += 2
    assert t[j:j + 2] == CR + LF
    body = t[t.index('{', i) + 1:j - 2]          # 原样切出来, 中文字节不变
    assert 'add = 0;' in body and 'yaw_target' in body
    NEW = (CMT_C + CR + LF
           + TAB + 'if(gimbal_control_loop_t->gimbal_mode == CHASSIS_NO_MOVE ||' + CR + LF
           + TAB + '   gimbal_control_loop_t->aimbot_mode == AIMBOT_ON ||' + CR + LF
           + TAB + '   aimbot_last_mode == AIMBOT_ON)' + CR + LF
           + TAB + '{' + body + TAB + '}' + CR + LF
           + TAB + 'aimbot_last_mode = gimbal_control_loop_t->aimbot_mode;' + CR + LF)
    t = t[:i] + NEW + t[j:]
    assert t.count('aimbot_last_mode') == 3, t.count('aimbot_last_mode')
    wr(C, t)
    print('修复2 OK')

print('done.')
