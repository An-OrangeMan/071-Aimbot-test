# -*- coding: utf-8 -*-
# 静态检查：没有编译器可用时的替代验证
# 1) 括号配平（先剥掉注释和字符串）
# 2) 所有被调用的函数/宏必须"在本文件中先定义"或"在某个头文件里声明"
# 3) 所有 .成员 / ->成员 访问必须能在相关结构体定义里找到
import os
import re

ROOT = '.'
NEW_C = 'application/aimbot_gimbal.c'
NEW_H = 'application/aimbot_gimbal.h'


def dec(p):
    try:
        return open(p, 'rb').read().decode('gbk')
    except Exception:
        return None


def strip(src):
    """剥掉注释和字符串字面量，长度用空格占位以保持行号"""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == '/' and i + 1 < n and src[i + 1] == '/':
            while i < n and src[i] != '\n':
                out.append(' ')
                i += 1
        elif c == '/' and i + 1 < n and src[i + 1] == '*':
            out.append(' ')
            out.append(' ')
            i += 2
            while i + 1 < n and not (src[i] == '*' and src[i + 1] == '/'):
                out.append('\n' if src[i] == '\n' else ' ')
                i += 1
            out.append(' ')
            out.append(' ')
            i += 2
        elif c in '"\'':
            q = c
            out.append(' ')
            i += 1
            while i < n and src[i] != q:
                if src[i] == '\\':
                    out.append(' ')
                    i += 1
                out.append('\n' if i < n and src[i] == '\n' else ' ')
                i += 1
            out.append(' ')
            i += 1
        else:
            out.append(c)
            i += 1
    return ''.join(out)


src = open(NEW_C, 'rb').read().decode('gbk')
clean = strip(src)

# ---- 1) 括号配平 ----
bal = {'{': '}', '(': ')', '[': ']'}
rev = {v: k for k, v in bal.items()}
stack = []
line = 1
for ch in clean:
    if ch == '\n':
        line += 1
    elif ch in bal:
        stack.append((ch, line))
    elif ch in rev:
        if not stack or stack[-1][0] != rev[ch]:
            print('BRACE MISMATCH: %r at line %d' % (ch, line))
            break
        stack.pop()
else:
    if stack:
        print('UNCLOSED:', stack[-5:])
    else:
        print('brace balance OK')

# ---- 2) 被调用者是否可见 ----
# 本文件里先出现的函数定义
defined = set(re.findall(r'^[A-Za-z_][\w \*]*?\b(\w+)\s*\([^;]*\)\s*$', clean, re.M))
defined |= set(re.findall(r'^static\s+[\w\*]+\s+(\w+)\s*\(', clean, re.M))
defined |= set(re.findall(r'^#define\s+(\w+)\(', clean, re.M))

exports = set(re.findall(r'^extern\s+[\w\* ]+\b(\w+)\s*\(', dec(NEW_H), re.M))
exports |= set(re.findall(r'^\s*#define\s+(\w+)\(', dec(NEW_H), re.M))
exports |= set(re.findall(r'^\s*#define\s+(\w+)\s', dec(NEW_H), re.M))

KEYWORDS = {'if', 'for', 'while', 'switch', 'return', 'sizeof', 'else', 'do',
            'case', 'break', 'continue', 'define', 'ifdef', 'ifndef', 'endif',
            'include', 'void', 'int', 'char', 'float', 'static', 'const'}
calls = set(re.findall(r'\b(\w+)\s*\(', clean))
external = set()
for root in ['application', 'components', 'Src', 'Inc', 'bsp', 'Middlewares', 'Drivers']:
    if not os.path.isdir(root):
        continue
    for dp, _, fns in os.walk(root):
        for fn in fns:
            if not fn.endswith(('.c', '.h')):
                continue
            t = dec(os.path.join(dp, fn))
            if not t:
                continue
            external |= set(re.findall(r'^extern\s+[\w\* ]+\b(\w+)\s*\(', t, re.M))
            external |= set(re.findall(r'^\s*#define\s+(\w+)\(', t, re.M))
            external |= set(re.findall(r'^\s*#define\s+(\w+)\s*\(', t, re.M))
            external |= set(re.findall(r'^\s*#define\s+(\w+)\s+\S', t, re.M))

unknown = sorted(c for c in calls
                 if c not in KEYWORDS and c not in defined
                 and c not in exports and c not in external)
print('unknown calls:', unknown)

# ---- 3) 成员访问 ----
known_members = set()
for t in [dec(NEW_H), dec('application/shoot.h'), dec('application/CAN_receive.h'),
          dec('application/remote_control.h'), dec('application/INS_task.h'),
          dec('components/controller/pid.h')]:
    if not t:
        continue
    known_members |= set(re.findall(r'^\s*[\w\* ]+?\b(\w+)\s*(?:\[\d*\])?\s*;', t, re.M))
    known_members |= set(re.findall(r'^\s*#define\s+(\w+)', t, re.M))
known_members |= {'rc', 'mouse', 'key', 'ch', 's', 'b', 'v', 'press_l', 'press_r',
                  'x', 'y', 'z', 'w', 'ecd', 'speed_rpm', 'given_current',
                  'temperate', 'last_ecd'}

members = set(re.findall(r'(?:->|\.)(\w+)\b', clean))
bad = sorted(m for m in members if m not in known_members)
print('members not found in any header:', bad)
