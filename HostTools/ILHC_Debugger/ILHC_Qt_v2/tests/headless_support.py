"""Run ACTUAL MainWindow methods without Qt (logic test, not rendering/integration).

Methods are compiled from source unchanged. Fake widgets are only input/output
sinks. Simulator and collision model are the production implementations.
"""
from __future__ import annotations
import ast
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import queue
import threading
import time
import core
import navigation_planner as nav

ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT/'main.py').read_text(encoding='utf-8-sig'))
source = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MainWindow')
klass = ast.ClassDef(name='WindowMethods', bases=[], keywords=[],
                    body=[n for n in source.body if isinstance(n, ast.FunctionDef)], decorator_list=[])
# Execute only imports that really exist at main.py module scope.  The v2.1.1
# helper injected threading from this test module even when main.py had NOT
# imported it, hiding the exact NameError seen after a real map click.
SAFE_IMPORT_ROOTS = {
    '__future__', 'argparse', 'copy', 'concurrent', 'csv', 'faulthandler',
    'html', 'json', 'logging', 'math', 'os', 'queue', 'sys', 'threading',
    'time', 'traceback', 'pathlib', 'numpy', 'core', 'navigation_planner',
}

def _production_imports(statements):
    result = []
    for node in statements:
        if isinstance(node, ast.Import):
            if all(alias.name.split('.')[0] in SAFE_IMPORT_ROOTS for alias in node.names):
                result.append(node)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module and node.module.split('.')[0] in SAFE_IMPORT_ROOTS:
                result.append(node)
        elif isinstance(node, ast.Try):
            # navigation_planner is guarded by an ImportError handler in production.
            # Do not scan classes/functions: their local imports are NOT globals.
            result.extend(_production_imports(node.body))
    return result

module = ast.Module(body=_production_imports(tree.body) + [klass], type_ignores=[])
namespace = {'BASE_DIR': ROOT}
exec(compile(ast.fix_missing_locations(module), str(ROOT/'main.py'), 'exec'), namespace)
WindowMethods = namespace['WindowMethods']

class Widget:
    def __init__(self, value=None):
        self.v=value; self.text=''; self.points=None; self.props={}; self.active=False
    def value(self): return self.v
    def currentData(self): return self.v
    def isChecked(self): return bool(self.v)
    def setValue(self,v): self.v=v
    def setChecked(self,v): self.v=bool(v)
    def setEnabled(self,v): self.enabled=bool(v)
    def setText(self,v): self.text=v
    def setPlainText(self,v): self.text=v
    def setProperty(self,k,v): self.props[k]=v
    def style(self): return self
    def polish(self,_): pass
    def unpolish(self,_): pass
    def set_path(self,v,waypoint_points=None):
        self.points=v; self.waypoint_points=waypoint_points; self.trajectory=None
        if not v: self.skeleton=None; self.reference=None
    def set_navigation_map(self,v): self.navigation_map=v
    def set_trajectory(self,v): self.trajectory=v
    def set_skeleton(self,v): self.skeleton=v
    def set_reference(self,v): self.reference=v
    def set_target(self,*v): self.points=v
    def set_sim_obstacles(self,v): self.obstacles=v
    def start(self): self.active=True
    def stop(self): self.active=False


def make_window():
    w=WindowMethods.__new__(WindowMethods)
    w.line_q=core.CommandQueue(); w.urgent_q=queue.Queue(); w.frame_q=queue.Queue()
    w.worker=None; w.sim=core.Simulator(w.frame_q,w.line_q,w.urgent_q)
    w.sim.handle_line('ZERO'); w.sim.make_frame(0.0)
    w.latest=(0.0,)*24; w.latest_t=0.0; w.latest_received_monotonic=time.monotonic()
    w.map_ox=w.map_oy=w.map_theta=0.0
    w.map_target=None; w.planned_points=[]; w.planned_result=None; w._planned_context=None
    w.sim_obstacles=[]          # 模拟障碍：运行时演示物体，不写进地图数据
    w.follow=None; w.wheel_state=True; w.send_count=0
    w._plan_request_id=0; w._plan_cancel=None; w._plan_future=None; w._plan_context_pending=None
    w.nav_map=nav.load_map(ROOT/'navigation_map.json')
    w._planner_pool=ThreadPoolExecutor(max_workers=1)
    for name in ('map_status','plan_info','plan_text','follow_btn','follow_timer','map_view',
                 'obstacle_info','obstacle_mode_check','strafe_limit_check'):
        setattr(w,name,Widget())
    for name,value in (('map_ox_spin',0),('map_oy_spin',0),('map_theta_spin',0),
                       ('plan_grid_spin',10),('plan_pad_spin',1),('map_yaw_combo',None),
                       ('zone_combo',1),('plan_click_check',True),
                       ('turn_penalty_spin',nav.DEFAULT_TURN_PENALTY_MM),
                       ('strafe_limit_spin',core.STRAFE_RUN_LIMIT_MM/core.OPS_CM_TO_MM)):
        setattr(w,name,Widget(value))
    w.logs=[]; w.log=lambda msg,*rest: w.logs.append(msg)
    w._manual_stop=lambda:None
    w._update_wheel_status=lambda:None
    return w


def close_window(w):
    w._clear_path()
    w._planner_pool.shutdown(wait=True,cancel_futures=True)


def refresh_frame(w):
    """重新喂一帧：规划变慢后，手动喂帧的陈旧度会超过 350ms 的定位闸门。"""
    w.sim.make_frame(0.0)


def advance(w, steps=1):
    for _ in range(steps):
        snap=w.sim.navigation_snapshot()
        w.latest=w.sim.make_frame((snap['frame_seq']+1)/50.0)
        w.latest_received_monotonic=time.monotonic()
        w._follow_step()
