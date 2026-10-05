"""离线编译位置核心的实际函数，不引入GPIO、串口或驱动桩替代闭环。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def position_source():
    source = (ROOT / 'Hardware/mecanum_control.c').read_text(encoding='utf-8')
    start = source.index('uint8_t chassis_move_reference(')
    brace = source.index('{', start)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def write_position_kernel(folder):
    path = Path(folder) / 'chassis_position.c'
    path.write_text('#include "chassis_position.h"\n#include <math.h>\n' +
                    position_source(), encoding='utf-8')
    return path
