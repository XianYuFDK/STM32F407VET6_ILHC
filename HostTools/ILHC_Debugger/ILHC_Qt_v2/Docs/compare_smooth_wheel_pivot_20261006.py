"""Compare saved v2.1.8 and current v2.1.9 integer-wheel response curves."""
import json
import math
from pathlib import Path
import sys

qt = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(qt))
import core
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
docs = qt / 'Docs'
old = json.loads((docs/'wheel_pivot_four_corners_20261006.json').read_text(encoding='utf-8'))
new = json.loads((docs/'smooth_wheel_pivot_four_corners_20261006.json').read_text(encoding='utf-8'))


def response(row):
    first = row['program']['waypoints'][0]
    previous = (*core.layout_to_field(first['x_mm'], first['y_mm']), -90-first['layout_yaw_deg'])
    times, speeds, rates, indices = [], [], [], []
    boundary_idle = 0
    for index, point in enumerate(row['trajectory']):
        current = point['x_mm'], point['y_mm'], point['field_yaw_deg']
        speed = math.dist(current[:2], previous[:2])/.02
        omega = ((current[2]-previous[2]+180)%360-180)/.02
        times.append((index+1)*.02); speeds.append(speed); rates.append(abs(omega))
        if point.get('pivot_wheel'):
            indices.append(index)
        layout = core.field_to_layout(*current[:2])
        near_corner = min(math.dist(layout, row['pivot'][key]) for key in ('entry_mm','exit_mm')) < 3
        if near_corner and speed<=1 and abs(omega)<=1:
            boundary_idle += 1
        previous = current
    return dict(times=times, speed_mm_s=speeds, omega_deg_s=rates,
                pivot_interval_s=[times[indices[0]],times[indices[-1]]],
                boundary_idle_s=round(boundary_idle*.02, 3), total_s=round(len(times)*.02, 3))


results = []
fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
for index, (before, after) in enumerate(zip(old,new)):
    a, b = response(before), response(after)
    results.append(dict(corner_mm=after['corner_mm'], reverse=after['reverse'],
                        old_total_s=a['total_s'], new_total_s=b['total_s'],
                        old_corner_idle_s=a['boundary_idle_s'], new_corner_idle_s=b['boundary_idle_s'],
                        old_pivot_interval_s=a['pivot_interval_s'], new_pivot_interval_s=b['pivot_interval_s']))
    if index:
        continue
    for data, color, label in ((a,'#929ba6','v2.1.8：进出弯停稳等待'),(b,'#087f5b','v2.1.9：减速衔接，过弯不等待')):
        axes[0].plot(data['times'],data['speed_mm_s'],color=color,label=label)
        axes[1].plot(data['times'],data['omega_deg_s'],color=color)
        axes[1].axvspan(*data['pivot_interval_s'],color=color,alpha=.08)
axes[0].set_ylabel('车心平移速度 / mm/s');axes[1].set_ylabel('实际角速度 / °/s')
axes[1].set_xlabel('离线模型时间 / s');axes[0].legend(fontsize=9)
for ax in axes:
    ax.grid(alpha=.2)
fig.suptitle('相同左前轮支点与整条路线：8.86 s → 6.62 s\n灰/绿浅色区表示对应绕轮阶段；最终STOP仍检查200 ms停稳')
fig.tight_layout()
fig.savefig(docs/'smooth_wheel_pivot_speed_comparison_20261006.png',dpi=140)
(docs/'smooth_wheel_pivot_comparison_20261006.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False))
