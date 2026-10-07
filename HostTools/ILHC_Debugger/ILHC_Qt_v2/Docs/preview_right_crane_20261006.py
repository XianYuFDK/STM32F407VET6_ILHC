"""用实际已验证路线绘制目标停靠姿态和右侧塔吊方向，无串口。"""
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, Circle
from matplotlib import font_manager
import competition_simulation as competition
import core
from work_orientation import work_heading


def draw():
    font = Path('C:/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    data = competition.load_profile()
    match = competition.compile_match(data, coordinate_mode=True)
    project = lambda x, y: (y, 2400-x)
    fig, ax = plt.subplots(figsize=(9, 9), constrained_layout=True)
    ax.set_facecolor('#f0f3f7')
    for x0, y0, x1, y1, name in data['rects']:
        ax.add_patch(Rectangle((y0, 2400-x1), y1-y0, x1-x0,
                              color='#b9a797', alpha=.85))
    for x, y, radius, _ in data['circles']:
        ax.add_patch(Circle(project(x, y), radius, color='#b9a797'))
    for zone in ('1', '2'):
        x, y = project(*data['competition']['start_zones'][zone])
        ax.add_patch(Rectangle((x-150, y-150), 300, 300, color='#2874cf', alpha=.65))
        ax.text(x, y, '启停区'+zone, ha='center', va='center', color='white', fontsize=10)
    for leg in match['legs']:
        points = [project(*core.field_to_layout(p['x_mm'], p['y_mm'])) for p in leg['route']['trajectory'][::4]]
        ax.plot(*zip(*points), color='#389d79', alpha=.6, linewidth=1.7)
    for station, label in (('raw', '原料区：车头0°'), ('rough', '加工区：车头180°'), ('storage', '暂存区：车头90°')):
        x, y = data['competition']['stations'][station]
        yaw = work_heading(data['competition'], station)
        theta = math.radians(yaw)
        c, s = math.cos(theta), math.sin(theta)
        vertices = [project(x+c*dx-s*dy, y+s*dx+c*dy)
                    for dx, dy in ((-140, -130), (140, -130), (140, 130), (-140, 130))]
        ax.add_patch(Polygon(vertices, facecolor='#ffa534', edgecolor='#a85b0a', zorder=4))
        px, py = project(x, y)
        ax.annotate('', (px+110*s, py-110*c), (px, py),
                    arrowprops=dict(arrowstyle='->', color='#1257ab', lw=2.5), zorder=5)
        right = math.radians(yaw-90)
        tx, ty = project(x+105*math.cos(right), y+105*math.sin(right))
        ax.add_patch(Circle((tx, ty), 22, color='#9b38a6', zorder=5))
        ax.annotate('', project(x+280*math.cos(right), y+280*math.sin(right)), (tx, ty),
                    arrowprops=dict(arrowstyle='->', color='#9b38a6', lw=2.5), zorder=5)
        offset = (-480, -55) if station == 'raw' else (190, -55) if station == 'rough' else (170, -20)
        ax.text(px+offset[0], py+offset[1], label, fontsize=11, weight='bold', zorder=6)
    ax.set(xlim=(0, 2400), ylim=(0, 2400), aspect='equal')
    ax.set_xticks(range(0, 2401, 300)); ax.set_yticks(range(0, 2401, 300))
    ax.grid(alpha=.22)
    ax.set_title('右侧塔吊作业停靠姿态\n蓝箭头：车头；紫箭头：车右塔吊朝向；绿线：验证后的路线', fontsize=14)
    ax.set_xlabel('目标姿态示意，尺寸mm；不代表塔吊安装点实测坐标')
    path = Path(__file__).with_name('right_crane_station_headings_20261006.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


if __name__ == '__main__':
    print(draw())
