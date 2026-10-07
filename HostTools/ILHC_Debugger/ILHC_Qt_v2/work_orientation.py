"""右侧塔吊作业航向：布局坐标中的车右方向为车头角减90度。"""
import math

from navigation_planner import EPS, point2

DEFAULT_WORK_AREAS = {'raw': (1200, 2400), 'rough': (1200, 75), 'storage': (75, 1200)}


def work_heading(config, station, point=None):
    """优先使用设备边缘的固定作业航向；旧地图保留指向中心的算法。"""
    if station not in DEFAULT_WORK_AREAS:
        return None
    origin = point2(config['stations'][station] if point is None else point, '作业停靠点')
    areas = config.get('work_areas', DEFAULT_WORK_AREAS)
    if not isinstance(areas, dict) or station not in areas:
        raise ValueError('competition.work_areas缺少作业区：' + station)
    target = point2(areas[station], '作业区中心')
    if math.dist(origin, target) <= EPS:
        raise ValueError('停靠点不能与作业区中心重合：' + station)
    headings = config.get('work_heading_field_deg', {})
    if not isinstance(headings, dict):
        raise ValueError('competition.work_heading_field_deg须为对象')
    if station in headings:
        heading = float(headings[station])
        if not math.isfinite(heading):
            raise ValueError('作业航向必须为有限数值：' + station)
        # 场地航向0°向前、90°向左；转换为规划器LAYOUT角。
        return (heading-180) % 360
    return (math.degrees(math.atan2(target[1]-origin[1], target[0]-origin[0]))+90) % 360


def station_at(config, point, tolerance_mm=1.0):
    """仅精确站点点击强制作业朝向，普通地图目标保持用户指定航向。"""
    stations = config.get('stations', {})
    return next((name for name in DEFAULT_WORK_AREAS if name in stations
                 and math.dist(point, point2(stations[name])) <= tolerance_mm), None)
