# -*- coding: utf-8 -*-
# 没有编译器时的类型名检查：新 .c 里所有出现在"类型位置"的标识符，
# 必须是已知 typedef / 结构体名 / 内建类型。
import os
import re

def dec(p):
    try:
        return open(p, 'rb').read().decode('gbk')
    except Exception:
        return None

# 1) 收集所有头文件里的 typedef 名和 struct 标签
typedefs = set()
tags = set()
for root in ['application', 'components', 'Src', 'Inc', 'bsp', 'Middlewares', 'Drivers', 'CMSIS']:
    if not os.path.isdir(root):
        continue
    for dp, _, fns in os.walk(root):
        for fn in fns:
            if not fn.endswith('.h'):
                continue
            t = dec(os.path.join(dp, fn))
            if not t:
                continue
            typedefs |= set(re.findall(r'}\s*(\w+)\s*;', t))
            typedefs |= set(re.findall(r'typedef\s+[\w\*]+\s+(\w+)\s*;', t))
            tags |= set(re.findall(r'\b(?:struct|union|enum)\s+(\w+)', t))

builtin = {'void', 'char', 'short', 'int', 'long', 'float', 'double', 'signed',
           'unsigned', 'const', 'volatile', 'static', 'struct', 'union', 'enum',
           'return', 'if', 'else', 'for', 'while', 'do', 'switch', 'case', 'break',
           'continue', 'sizeof', 'NULL', 'uint8_t', 'uint16_t', 'uint32_t',
           'uint64_t', 'int8_t', 'int16_t', 'int32_t', 'int64_t', 'float32_t',
           'float64_t'}

src = open('application/aimbot_gimbal.c', 'rb').read().decode('gbk')

# 2) 声明形态： [const|static|volatile]* TYPE NAME (=|;|[|)|,)
pat = re.compile(r'^\s*(?:const\s+|static\s+|volatile\s+)*([A-Za-z_]\w*)\s+[*&]*\s*([A-Za-z_]\w*)\s*(?=[=;\[\),])', re.M)
bad = []
for m in pat.finditer(src):
    ty = m.group(1)
    if ty in builtin or ty in typedefs or ty in tags:
        continue
    line = src[:m.start()].count('\n') + 1
    bad.append((line, ty, m.group(0).strip()[:60]))
print('可疑的类型名:')
for b in bad:
    print('  line %-5d %-18s %s' % b)
if not bad:
    print('  (无)')

# 3) 强制类型转换
casts = set(re.findall(r'\(\s*([A-Za-z_]\w*)\s*\)\s*[A-Za-z_0-9\.\-]', src))
badc = sorted(c for c in casts if c not in builtin and c not in typedefs and c not in tags)
print('可疑的强制转换:', badc if badc else '(无)')
