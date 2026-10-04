"""DM 回转调试：单位、上位机参考曲线、连续到位判定及示教文件。"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

POSITIONS = {"pickup": "待抓取位", "slot1": "载盘 1 号位",
             "slot2": "载盘 2 号位", "slot3": "载盘 3 号位"}


def motor_rad(angle_deg, ratio=1.0):
    """ratio 是额外传动的 DM 输出轴转数 / 回转轴转数。"""
    if not math.isfinite(ratio) or ratio <= 0:
        raise ValueError("额外传动比必须大于零")
    value = math.radians(angle_deg) * ratio
    if not math.isfinite(value) or not -12.5 <= value <= 12.5:
        raise ValueError("目标超过现有 STM32 的 ±12.5 rad 范围")
    return value


def validate_target(angle, ratio, lower, upper):
    if not all(math.isfinite(x) for x in (angle, lower, upper)) or lower >= upper:
        raise ValueError("软限位必须有限，且下限小于上限")
    motor_rad(lower, ratio)
    motor_rad(upper, ratio)
    if not lower <= angle <= upper:
        raise ValueError("目标超过回转软限位")
    return motor_rad(angle, ratio)


@dataclass
class ReferenceMove:
    start: float
    target: float
    speed: float
    acceleration: float
    smooth: bool = True

    def __post_init__(self):
        if (not all(math.isfinite(x) for x in
                    (self.start, self.target, self.speed, self.acceleration))
                or self.speed <= 0 or self.acceleration <= 0):
            raise ValueError("速度和参考加速度必须大于零")
        distance = abs(self.target - self.start)
        # 五次曲线最大一阶导数为 1.875，最大二阶导数为 10/√3。
        self.duration = max(0.2, 1.875 * distance / self.speed,
                            math.sqrt((10 / math.sqrt(3)) * distance / self.acceleration))
        if not self.smooth:
            self.duration = distance / self.speed

    def position(self, elapsed):
        if not self.smooth:
            return self.target
        u = min(1.0, max(0.0, elapsed / self.duration))
        return self.start + (self.target-self.start) * (10*u**3-15*u**4+6*u**5)


class Arrival:
    """仅新遥测采样能推进停稳窗口；窗口内每个采样均需满足角度和速度。"""
    def __init__(self, tolerance, velocity, dwell):
        self.tolerance, self.velocity, self.dwell = tolerance, velocity, dwell
        self.since = None
        self.last_stamp = None

    def update(self, stamp, error, velocity, eligible=True):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        gap = self.last_stamp is not None and stamp-self.last_stamp > 0.35
        self.last_stamp = stamp
        good = (eligible and not gap and math.isfinite(error) and math.isfinite(velocity)
                and abs(error) <= self.tolerance and abs(velocity) <= self.velocity)
        if not good:
            self.since = None
            return False
        if self.since is None:
            self.since = stamp
        return stamp-self.since >= self.dwell


def save_profile(path, profile):
    """原子替换：读取失败或写入失败时，不丢失原来的示教文件。"""
    validate_profile(profile)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False),
                    encoding="utf-8")
    temp.replace(path)


def validate_profile(data):
    if data.get("version") != 1:
        raise ValueError("示教文件版本不支持")
    if type(data.get("motor_id")) is not int or not 1 <= data["motor_id"] <= 1791:
        raise ValueError("示教电机 ID 无效")
    settings = data["settings"]
    ratio, lower, upper = (float(settings[k]) for k in ("ratio", "lower", "upper"))
    if not math.isfinite(ratio) or not 0.01 <= ratio <= 100:
        raise ValueError("示教传动比超范围")
    if not -716 <= lower < upper <= 716:
        raise ValueError("示教软限位超出界面范围")
    validate_target(lower, ratio, lower, upper)
    for key, value in data["positions"].items():
        if key not in POSITIONS:
            raise ValueError("未知示教位置")
        validate_target(float(value), ratio, lower, upper)
    for key, lo, hi in (("speed", 0.01, 360), ("acceleration", 0.01, 720),
                        ("step", 0.01, 30), ("tolerance", 0.01, 10),
                        ("velocity", 0.01, 30), ("dwell", 0.1, 5), ("timeout", 1, 120)):
        x = float(settings[key])
        if not math.isfinite(x) or not lo <= x <= hi:
            raise ValueError("示教设置超范围：" + key)
    if type(settings["smooth"]) is not bool:
        raise ValueError("曲线设置无效")
    if math.radians(float(settings["speed"])) * ratio > 30:
        raise ValueError("速度超过 STM32 的 30 rad/s 范围")
    return data


def load_profile(path):
    return validate_profile(json.loads(Path(path).read_text(encoding="utf-8")))
