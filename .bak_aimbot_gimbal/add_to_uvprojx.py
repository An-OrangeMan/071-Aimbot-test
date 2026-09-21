# -*- coding: utf-8 -*-
# 把 aimbot_gimbal.c 加进 chassis_task.uvprojx 的 application 组
# 用 chr(92) 拼反斜杠，免得被 shell 吃掉，也免得 Python 报 invalid escape sequence
B = chr(92)

p = 'chassis_task/MDK-ARM/chassis_task.uvprojx'
t = open(p, 'rb').read().decode('gbk')

def filedef(name):
    return ('            <File>\n'
            '              <FileName>' + name + '</FileName>\n'
            '              <FileType>1</FileType>\n'
            '              <FilePath>..' + B + '..' + B + 'application' + B + name + '</FilePath>\n'
            '            </File>\n')

anchor = filedef('oled_task.c') + '          </Files>\n'
assert t.count(anchor) == 1, 'anchor count = %d' % t.count(anchor)

t2 = t.replace(anchor, filedef('oled_task.c') + filedef('aimbot_gimbal.c') + '          </Files>\n')
b = t2.encode('gbk')

assert b.count(b'\xef\xbf\xbd') == 0
assert b.count(b'\r\n') == 0
open(p, 'wb').write(b)

import xml.etree.ElementTree as ET
ET.fromstring(b.decode('gbk'))

print('inserted. LF', b.count(b'\n'), 'CRLF', b.count(b'\r\n'))
print('XML well-formed OK')
print('aimbot_gimbal.c count:', b.count(b'aimbot_gimbal.c'))
print('gimbal_task.c   count:', b.count(b'gimbal_task.c'))
