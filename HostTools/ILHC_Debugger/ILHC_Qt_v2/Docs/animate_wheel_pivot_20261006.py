"""Animate the recorded integer-RPM body response around its front left wheel."""
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import core
from pivot_turns import wheel_anchor
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
row = json.loads((ROOT / 'Docs/smooth_wheel_pivot_four_corners_20261006.json').read_text(encoding='utf-8'))[0]
pivot = row['pivot']
arc = [point for point in row['trajectory'] if point.get('pivot_wheel') == 'FL']
points = [arc[round(index * (len(arc) - 1) / 44)] for index in range(45)]
frames = [points[0]] * 8 + points + [points[-1]] * 10
fig, ax = plt.subplots(figsize=(6, 6), dpi=110)
ax.set(xlim=(1770, 2330), ylim=(1740, 2280), aspect='equal',
       xlabel='布局 X / mm', ylabel='布局 Y / mm')
ax.grid(alpha=.2)
cx, cy = pivot['center_mm']
ax.scatter([cx], [cy], color='#d85d16', marker='+', s=260, linewidths=3,
           label='固定左前轮支点', zorder=10)
ax.annotate('左前轮支点', (cx, cy), xytext=(cx-185, cy+90),
            arrowprops={'arrowstyle': '->', 'color': '#d85d16'}, color='#d85d16')
path = [core.field_to_layout(point['x_mm'], point['y_mm']) for point in arc]
ax.plot(*zip(*path), color='#087f5b', linestyle=':', label='车心轨迹')
body = Polygon([(0, 0)] * 4, closed=True, facecolor='#3176c0',
               edgecolor='#3176c0', alpha=.2)
ax.add_patch(body)
wheels = ax.scatter([0]*4, [0]*4, s=90, c=['#d85d16', '#3176c0', '#3176c0', '#3176c0'], zorder=8)
nose = ax.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1,
                 color='#3176c0', width=.007)
ax.legend(loc='upper left', fontsize=9)
title = ax.set_title('左直角弯：绕左前轮旋转车身\n206 mm 轴距 / 218 mm 轮心距 · 离线实际响应')


def update(point):
    x, y = core.field_to_layout(point['x_mm'], point['y_mm'])
    yaw = -90 - point['field_yaw_deg']
    radians = math.radians(yaw)
    c, s = math.cos(radians), math.sin(radians)
    # Original full body rectangle: length 280 mm, width 260 mm.
    body.set_xy([(x + c*f - s*l, y + s*f + c*l)
                 for f, l in ((140, 130), (140, -130), (-140, -130), (-140, 130))])
    wheels.set_offsets([wheel_anchor((x, y), yaw, pivot['wheel_geometry'], wheel)
                        for wheel in ('FL', 'FR', 'BL', 'BR')])
    nose.set_offsets([(x, y)])
    nose.set_UVC([70*c], [70*s])
    turned = (yaw - pivot['entry_yaw_deg']) % 360
    if turned > 180:
        turned -= 360
    title.set_text('左直角弯：绕左前轮旋转车身（%.1f°）\n206 mm 轴距 / 218 mm 轮心距 · 离线实际响应' % turned)
    return body, wheels, nose, title


fig.tight_layout()
animation = FuncAnimation(fig, update, frames=frames, interval=90, blit=False)
animation.save(ROOT / 'Docs/smooth_wheel_pivot_front_left_20261006.gif', writer=PillowWriter(fps=11))
print('Saved smooth_wheel_pivot_front_left_20261006.gif')
