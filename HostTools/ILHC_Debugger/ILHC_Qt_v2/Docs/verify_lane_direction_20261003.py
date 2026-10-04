"""两出发区×四场景，复现路形/方向修改后的整轮计时与逐帧扫掠检查。"""
from verify_mecanum_20261003 import main
import competition_simulation as competition


if __name__ == '__main__':
    main('lane_direction_results_20261003.json', [
        ('无障碍', (), {1: 78.20, 2: 78.22}),
        ('截图单障碍', ((700, 1200),), {}),
        ('四障碍预设', competition.DEMO_OBSTACLES, {1: 93.94, 2: 94.12}),
        ('截图两障碍', ((1200, 1200), (297, 294)), {1: 119.76, 2: 126.46}),
    ])
