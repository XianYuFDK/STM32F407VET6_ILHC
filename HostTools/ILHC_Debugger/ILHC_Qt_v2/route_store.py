"""无额外障碍的已验证路线库：按地图/参数/几何/算法内容自动失效。"""
from collections import OrderedDict
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading

ROOT = Path(__file__).with_name('precomputed_routes')
FORMAT = 1
MAX_BYTES = 8_000_000
MAX_MEMORY_ROUTES = 16
_memory = OrderedDict()
_lock = threading.RLock()
SOURCES = ('competition_simulation.py', 'coordinate_navigation.py', 'coordinate_search.py',
           'pivot_turns.py', 'mecanum_geometry.py', 'mecanum_planner.py', 'navigation_planner.py',
           'trajectory.py', 'trajectory_tracking.py', 'arc_smoothing.py', 'core.py',
           'work_orientation.py', 'route_store.py', 'approach_heading.py')


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode('utf-8')).hexdigest()


def algorithm_revision():
    """读取实际源文件内容，部署复制源码后无需手工同步版本号。"""
    result = hashlib.sha256()
    for name in SOURCES:
        result.update(name.encode('ascii'))
        result.update(Path(__file__).with_name(name).read_text(encoding='utf-8').encode('utf-8'))
    return result.hexdigest()


class RouteStore:
    def __init__(self, data, margin, control, footprint, *, root=None):
        self.root = Path(root or os.environ.get('ILHC_ROUTE_CACHE_DIR') or ROOT)
        # CLI整数与Qt/串口浮点回读的同一数值须命中同一缓存。
        self.context = digest(dict(format=FORMAT, algorithm=algorithm_revision(), map=data,
                                   margin=float(margin), control={k: float(v) for k, v in control.items()},
                                   footprint=[float(v) for v in footprint]))
        self.warning = None

    def identity(self, start, goal, heading, goal_heading):
        return digest(dict(context=self.context, start=[float(v) for v in start], goal=[float(v) for v in goal],
                           heading=float(heading % 360),
                           goal_heading=None if goal_heading is None else float(goal_heading % 360)))

    def path(self, identity):
        return self.root / (identity + '.json.gz')

    def get(self, start, goal, heading, goal_heading, cancelled=lambda: False):
        if cancelled():
            raise ValueError('固定路线读取已取消')
        identity = self.identity(start, goal, heading, goal_heading)
        # 文件夹包含在内存键中，独立部署/测试目录不会相互复用。
        key = str(self.root.resolve()), identity
        with _lock:
            route = _memory.get(key)
            source = 'MEMORY'
            if route is None:
                source = 'DISK'
                try:
                    path = self.path(identity)
                    if path.stat().st_size > MAX_BYTES:
                        return None
                    with gzip.open(path, 'rt', encoding='utf-8') as stream:
                        text = stream.read(MAX_BYTES+1)
                    if len(text) > MAX_BYTES:
                        return None
                    envelope = json.loads(text)
                    route = envelope['route']
                    if (envelope['format'] != FORMAT or envelope['identity'] != identity or
                            envelope['sha256'] != digest(route)):
                        return None
                    from coordinate_navigation import CoordinateTracker
                    tracker = CoordinateTracker(route['waypoint_program'])
                    if not route.get('execution_safe') or not route.get('trajectory_safe') or not route.get('trajectory'):
                        return None
                    rows = route['waypoint_program']['waypoints']
                    if (list(start) != [rows[0]['x_mm'], rows[0]['y_mm']] or
                            list(goal) != [rows[-1]['x_mm'], rows[-1]['y_mm']]):
                        return None
                    if goal_heading is not None and abs((rows[-1]['layout_yaw_deg']-goal_heading+180)%360-180) > 1e-7:
                        return None
                    self._remember(key, route)
                except (OSError, ValueError, TypeError, KeyError, EOFError, OverflowError):
                    return None
            else:
                _memory.move_to_end(key)
            result = copy.deepcopy(route)
        if cancelled():
            raise ValueError('固定路线读取已取消')
        result['route_reuse'] = dict(source=source, identity=identity, context=self.context,
                                     search_skipped=True, extra_obstacles=False)
        return result

    @staticmethod
    def _remember(key, route):
        _memory[key] = copy.deepcopy(route)
        _memory.move_to_end(key)
        while len(_memory) > MAX_MEMORY_ROUTES:
            _memory.popitem(last=False)

    def put(self, start, goal, heading, goal_heading, route, cancelled=lambda: False):
        if cancelled():
            raise ValueError('固定路线保存已取消')
        identity = self.identity(start, goal, heading, goal_heading)
        saved = copy.deepcopy(route)
        saved.pop('route_reuse', None)
        envelope = dict(format=FORMAT, identity=identity, sha256=digest(saved), route=saved)
        encoded = canonical(envelope).encode('utf-8')
        if len(encoded) > MAX_BYTES:
            return
        temporary = None
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=self.root, suffix='.tmp', delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(gzip.compress(encoded, compresslevel=3, mtime=0))
            if cancelled():
                raise ValueError('固定路线保存已取消')
            os.replace(temporary, self.path(identity))
            with _lock:
                self._remember((str(self.root.resolve()), identity), saved)
        except OSError as exc:
            # 只读部署仍能规划和本进程复用；不因保存失败拦截安全路线。
            self.warning = str(exc)
            with _lock:
                self._remember((str(self.root.resolve()), identity), saved)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)


def clear_memory():
    with _lock:
        _memory.clear()
