# -*-coding:utf-8-*-
# @Author : PoLo
# @Time   : 2024/11/14 16:43
# @File   : plot.py

import matplotlib.pyplot as plt
import numpy as np
import os
file_name = 'Bumpsensors_2.txt'
# 读取数据
left_bump = []
right_bump = []
left_speed = []
right_speed = []

with open(file_name, 'r') as file:
    # 标志位，用于区分读取哪一部分数据
    reading_bump_values = True

    for line in file:
        # 跳过空行或标签行
        if not line.strip() or "LeftBumpValue & RightBumpValue" in line or "LeftAddSpeed & RightAddSpeed" in line:
            # 切换标志位以开始读取速度数据
            if "LeftAddSpeed & RightAddSpeed" in line:
                reading_bump_values = False
            continue

        # 处理不同数据块
        if reading_bump_values:
            # 读取 LeftBumpValue 和 RightBumpValue 数据
            left_val, right_val = map(float, line.strip().split(','))
            left_bump.append(left_val)
            right_bump.append(right_val)
        else:
            # 读取 LeftAddSpeed 和 RightAddSpeed 数据
            left_speed_val, right_speed_val = map(float, line.strip().split(','))
            left_speed.append(left_speed_val)
            right_speed.append(right_speed_val)

# 转换为numpy数组
left_bump = np.array(left_bump)
right_bump = np.array(right_bump)
left_speed = np.array(left_speed)
right_speed = np.array(right_speed)

# 创建图表和两个y轴
fig, ax1 = plt.subplots()

# 左侧y轴，用于 BumpValue 数据
ax1.set_xlabel('Time Step')
ax1.set_ylabel('Bump Value', color='tab:blue')
ax1.plot(left_bump, label='LeftBumpValue', color='tab:blue')
ax1.plot(right_bump, label='RightBumpValue', color='tab:cyan')
ax1.tick_params(axis='y', labelcolor='tab:blue')

# 创建右侧y轴，用于 AddSpeed 数据
ax2 = ax1.twinx()
ax2.set_ylabel('Add Speed', color='tab:red')
ax2.plot(left_speed, label='LeftAddSpeed', color='tab:red')
ax2.plot(right_speed, label='RightAddSpeed', color='tab:orange')
ax2.tick_params(axis='y', labelcolor='tab:red')

# 添加图例
fig.legend(loc='upper left', bbox_to_anchor=(0.1, 0.9))

plt.savefig(os.path.splitext(file_name)[0] + '_plot.png')
# 显示图表
plt.show()
