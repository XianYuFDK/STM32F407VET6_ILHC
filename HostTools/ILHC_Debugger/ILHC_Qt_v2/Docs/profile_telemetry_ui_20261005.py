"""Offline event-loop / map render profile. Never opens a port."""
import argparse
import cProfile
import json
import os
import pstats
import sys
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import main
from PySide6.QtGui import QFontDatabase

app=main.QApplication.instance() or main.QApplication([])
# The offscreen Windows platform does not populate the system font database.
# Load installed fonts for a representative paint profile and readable preview.
for filename in ('msyh.ttc','msyhbd.ttc','segoeui.ttf'):
    font=Path('C:/Windows/Fonts')/filename
    if font.exists():QFontDatabase.addApplicationFont(str(font))
app.setFont(main.QFont('Microsoft YaHei UI',9))
w=main.MainWindow(argparse.Namespace(port=None,baud=115200,simulate=False))
w.resize(1400,900);w.show();w.stack.setCurrentIndex(2)
app.processEvents()
profile=cProfile.Profile();profile.enable()
durations=[];rects=set();renders=0
started=time.monotonic();next_frame=started
while time.monotonic()-started<3:
    now=time.monotonic()
    if now>=next_frame:
        values=(39.2+2*(now-started),107.,2.8)+(0.,)*21
        w.frame_q.put((now,values))
        next_frame+=.02
    begin=time.perf_counter();app.processEvents();durations.append(time.perf_counter()-begin)
    r=w.map_view.viewport().geometry();rects.add((r.x(),r.y(),r.width(),r.height()))
    time.sleep(.001)
profile.disable()
print(json.dumps(dict(receive_hz=w.fps,render_interval_ms=w.render_timer.interval(),
                     ui_refresh_hz=w.ui_refresh_hz,
                     event_loop_max_ms=max(durations)*1000,map_rects=sorted(rects)),ensure_ascii=False))
pstats.Stats(profile).strip_dirs().sort_stats('cumtime').print_stats(18)
w.grab().save(str(Path(__file__).with_name('telemetry_ui_preview_20261005.png')))
w.close()
