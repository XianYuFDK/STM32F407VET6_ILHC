"""实际微小航向偏差下，正常回启停区保持整车边界检查。"""
import copy
import threading
import unittest
import core
import navigation_planner as nav
from hardware_trajectory import make_path_batch, ROTATE, STOP
from tests.headless_support import make_window, close_window


class HomeReturnTests(unittest.TestCase):
    def setUp(self):
        self.w=make_window()
        self.w.sim.hold=(1045,1031)

    def tearDown(self):
        close_window(self.w)

    def context(self, yaw, zone=1):
        self.w.sim.zval=yaw
        self.w.sim.make_frame(0)
        ctx=self.w._prepare_plan(*core.ZONE_CENTER[zone])
        self.assertIsNotNone(ctx)
        return ctx

    def test_screenshot_negative_zero_reproduces_old_target_failure(self):
        ctx=self.context(-.005)
        result=core.plan_path(ctx['start'],ctx['goal'],**ctx['kwargs'])
        self.assertEqual(result['code'],'INVALID_GOAL')
        self.assertIn('场地边界',result['reason'])
        self.assertEqual(format(-.005,'.1f'),'-0.0')

    def test_both_zones_home_restores_zero_and_preserves_actual_alignment(self):
        for zone in (1,2):
            for yaw in (-.005,-.02,.02,45,90):
                with self.subTest(zone=zone,yaw=yaw):
                    ctx=self.context(yaw,zone)
                    ctx['kwargs']['goal_heading_deg']=-180
                    result=core.plan_home_path(ctx['start'],ctx['goal'],**ctx['kwargs'])
                    self.assertTrue(result['ok'],result.get('reason'))
                    self.assertAlmostEqual(result['start_heading_deg'],ctx['kwargs']['start_heading_deg'])
                    self.assertEqual(result['goal_heading_deg'],180)
                    self.assertIsNone(self.w._validate_quantized(result,ctx))
                    batch=make_path_batch(result,ctx['start'],result['start_heading_deg'],-180,
                                          (0,0,0),self.w._scene_for_context(ctx),self.w.nav_map)
                    self.assertEqual(batch['points'][0][4] & ~STOP,0,'首点不能携带ROTATE')
                    target=self.w._field_to_ops(*ctx['goal'])
                    self.assertEqual(batch['points'][-1][:2],tuple(round(v*10) for v in target))
                    self.assertEqual(batch['points'][-1][3],0)
                    self.assertTrue(batch['points'][-1][4]&STOP)
                    self.assertFalse(self.w._home_after_stop)
                    if abs(yaw)<.1 and abs(yaw)>=.02:
                        self.assertEqual(batch['points'][0][3],round(yaw*100))
                        self.assertTrue(batch['points'][1][4]&ROTATE)

    def test_unsafe_actual_alignment_is_rejected_without_rounding_pose(self):
        ctx=self.context(45)
        k=copy.deepcopy(ctx['kwargs']);k.update(goal_heading_deg=180,
            bounds=(0,0,2400,2400),rects=[],circles=[],drivable_polygons=None)
        # 此斜车体可置于方形停车位内，但完整旋转扫掠无法包含。
        actual=k['start_heading_deg']
        body=nav.CollisionScene([],[],k['bounds'],10,(280,260,actual))
        pose=(1200,1200)
        from shapely.geometry import box
        width=body.ex*2; height=body.ey*2
        k['drivable_polygons']=[list(box(1200-width/2,1200-height/2,1200+width/2,1200+height/2).exterior.coords)]
        r=core.plan_home_path(pose,core.ZONE_CENTER[1],**k)
        self.assertFalse(r['ok']);self.assertIn('对齐不安全',r['reason'])

    def test_home_does_not_reduce_margin_or_ignore_obstacles(self):
        ctx=self.context(-.02);k=copy.deepcopy(ctx['kwargs']);k['goal_heading_deg']=-180
        k['pad']=20
        result=core.plan_home_path(ctx['start'],ctx['goal'],**k)
        self.assertFalse(result['ok']);self.assertIn('场地边界',result['reason'])
        k['pad']=10;k['circles'].append((*core.ZONE_CENTER[1],25,'回库障碍'))
        result=core.plan_home_path(ctx['start'],ctx['goal'],**k)
        self.assertFalse(result['ok']);self.assertIn('回库障碍',result['reason'])

    def test_cancelled_home_cannot_produce_executable_path(self):
        ctx=self.context(-.02);k=ctx['kwargs'];k['goal_heading_deg']=-180
        cancel=threading.Event();cancel.set()
        result=core.plan_home_path(ctx['start'],ctx['goal'],cancel=cancel,**k)
        self.assertFalse(result['ok']);self.assertEqual(result['code'],'CANCELLED')


if __name__=='__main__':unittest.main()
