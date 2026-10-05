"""离屏真实Qt坐标闭环过弯截图；不连接串口、不启动实机。"""
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_map_click_qt import MapClickQtTests
import core
import competition_simulation as competition
from PySide6.QtGui import QFont, QFontDatabase


def main():
    MapClickQtTests.setUpClass()
    for path in ('C:/Windows/Fonts/msyh.ttc','C:/Windows/Fonts/msyhbd.ttc'):
        if Path(path).exists():
            assert QFontDatabase.addApplicationFont(path)>=0
    MapClickQtTests.app.setFont(QFont('Microsoft YaHei',10))
    test=MapClickQtTests()
    test.setUp()
    try:
        w=test.w
        w.resize(1500,1000)
        test.app.processEvents()
        # 比赛已经安全出库后的二维码→原料段，显示真正的同步转头。
        match=competition.compile_match(competition.load_profile(),coordinate_mode=True)
        route=match['legs'][1]['route']
        first=route['waypoint_program']['start']
        w.sim.hold=first['x_mm'],first['y_mm']
        w.sim.zval=90-first['field_yaw_deg']
        w.sim.make_frame(0)
        context=w._prepare_plan(*route['points'][-1])
        w._finish_plan(route,context)
        assert w.planned_result['waypoint_program']
        w.sim.make_frame(0)
        w._toggle_follow()
        assert w.follow is not None,w.map_status.text()
        captured=False
        for i in range(2000):
            frame=w.sim.make_frame(i/core.SEND_HZ)
            w.frame_q.put((time.monotonic(),frame))
            w._process_frames()
            w._follow_step()
            snap=w.sim.navigation_snapshot()
            if (snap['active'] and 20<abs(snap['yaw'])<70 and
                    snap['speed_mm_s']>0 and abs(snap['yaw_rate_deg_s'])>0):
                w._update_map_trail()
                w._render_ui()
                test.app.processEvents()
                assert w.grab().save(str(Path(__file__).parent/'coordinate_navigation_qt_preview_20261004.png'))
                print('已保存真实Qt过弯截图：speed=%.1fmm/s yaw=%.1fdeg omega=%.1fdeg/s'%
                      (snap['speed_mm_s'],snap['yaw'],snap['yaw_rate_deg_s']))
                captured=True
                break
        assert captured,'未找到同时平移和转头的运行帧'
        test.assert_no_command_sent()
    finally:
        test.tearDown()


if __name__=='__main__':
    main()
