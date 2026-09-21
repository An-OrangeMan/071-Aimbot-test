# -*- coding: utf-8 -*-
# oled_task.c: 云台结构体里删掉了 shoot_switch / hatch_state 之后,
# oled 里读这两个成员的代码要跟着删掉, 否则编不过.
# 准心颜色改成跟 aimbot_mode 走(自瞄中=青色, 否则=黄色).
import os

P = 'D:/Document/Code/Aimbot_test/ShaoBing1/application/oled_task.c'
t = open(P, 'rb').read().decode('gbk')
orig = t

# ---- 1) 定位 ----
k = t.index('shoot_switch.fric_mode == FRIC_START')
l84 = t.rindex('\n', 0, k) + 1            # 第 84 行行首
l83 = t.rindex('\n', 0, l84 - 1) + 1      # 第 83 行行首(注释行)

g2 = t.index('Line_Draw(&Alignment_G2', l84)   # 从第84行往后找, 前面还有两处同名调用
l95 = t.rindex('\n', 0, g2) + 1           # 第 95 行行首
l96 = t.index('\n', l95) + 1              # 第 96 行行首

g1 = t.index('Line_Draw(&Alignment_G1', l95)
l101 = t.rindex('\n', 0, g1) + 1          # 第 101 行行首

assert l83 < l84 < l95 < l96 < l101, (l83, l84, l95, l96, l101)
assert 'hatch_state' in t[l96:l101], '未找到弹舱状态块'

NEW = ('\t\t/****************自瞄状态*************/\r\n'
       '\t\tif(oled.oled_gimbal_point->aimbot_mode == AIMBOT_ON)\r\n'
       '\t\t\tALIGNMENT_COLOR = UI_Color_Cyan;\r\n'
       '\t\telse\r\n'
       '\t\t\tALIGNMENT_COLOR = UI_Color_Yellow;\r\n')

t = t[:l83] + NEW + t[l95:l96] + t[l101:]

assert 'shoot_switch' not in t, 'shoot_switch 还有残留'
assert 'hatch_state' not in t, 'hatch_state 还有残留'
assert 'HATCH_OPEN' not in t, 'HATCH_OPEN 还有残留'

data = t.encode('gbk')
assert data.count(b'\xef\xbf\xbd') == 0, 'U+FFFD!'
open(P, 'wb').write(data)
print('patched %s  %d -> %d bytes' % (P, len(orig.encode('gbk')), len(data)))

# 复核改动位置
L = data.decode('gbk').replace('\r\n', '\n').split('\n')
for i in range(77, 96):
    print(i + 1, L[i].rstrip())
