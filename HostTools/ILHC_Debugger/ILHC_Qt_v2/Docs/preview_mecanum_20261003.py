"""离屏展示麦轮比赛实际运行，无硬件连接。"""
import argparse
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch

os.environ['QT_QPA_PLATFORM']='offscreen'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PySide6.QtGui import QFontDatabase, QFont
import core
import main


def main_preview(sim_obstacles=None, output_name='mecanum_preview_20261003.png',
                 preview_stage='第1批：前往原料区'):
    app=main.QApplication.instance() or main.QApplication([])
    for font in ('msyh.ttc','msyhbd.ttc','consola.ttf'):
        QFontDatabase.addApplicationFont('C:/Windows/Fonts/'+font)
    app.setFont(QFont('Microsoft YaHei',10))
    w=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
    w.resize(1860,1200); w._select_page(2); w.show(); app.processEvents()
    w.sim=core.Simulator(w.frame_q,w.line_q,w.urgent_q)
    w.sim.handle_line('ZERO'); w.sim.make_frame(0)
    w.sim_obstacles=[(1200,1200),(297,294)] if sim_obstacles is None else list(sim_obstacles)
    w._start_competition()
    deadline=time.monotonic()+30
    while w._competition_future is not None and time.monotonic()<deadline:
        app.processEvents(); time.sleep(.01)
    assert w.competition and w.competition.active, w.competition_status.text()
    runner=w.competition
    for i in range(9200):
        with patch.object(core.random,'gauss',return_value=0):
            frame=w.sim.make_frame(i*.02)
        w.frame_q.put((w.t0_monotonic+i*.02,frame)); w._process_frames(); w._poll_competition()
        if runner.stage and runner.stage['label']==preview_stage and runner.sim._nav_tracker.progress>650:
            break
    w._render_ui(); w._update_map_trail(); app.processEvents()
    w.grab().save(str(Path(__file__).with_name(output_name)))
    print(runner.stage['label'],w._field_heading(w.sim.navigation_snapshot()['yaw']),runner.elapsed_s)
    w.close(); w._planner_pool.shutdown(wait=True,cancel_futures=True)


if __name__=='__main__':
    main_preview()
