"""整批上传协议、作业继续、会话失效、坐标转换和整数CRC专项。"""
import copy
import math
import unittest
import zlib
import core
import competition_simulation as competition
from hardware_trajectory import *


def tiny_batch():
    points=[(0,0,0,0,0),(0,200,200,0,STOP|WAIT),(0,400,400,0,STOP)]
    return dict(id=17,points=points,waits={1:['取料']},map_version=1,margin_mm=10,
                crc=zlib.crc32(b''.join(POINT.pack(*p) for p in points)),length_mm=40)


class UploaderTests(unittest.TestCase):
    def setUp(self):
        self.lines=[];self.now=0;self.valid=True
        self.job=BatchUploader(tiny_batch(),self.lines.append,clock=lambda:self.now,valid=lambda:self.valid)
        self.job.start()

    def ready(self):
        self.job.handle_reply('TCAPS 1 4096 3')
        self.job.handle_reply('TSTAT 17 1 0 0 0 0')
        self.job.handle_reply('TSTAT 17 1 3 0 0 0')
        self.job.handle_reply('TSTAT 17 3 3 0 0 0')

    def test_upload_all_before_run_then_resume_does_not_resend(self):
        self.ready()
        self.assertEqual(self.lines,['TCAPS','TBEGIN=17,3,1,%08X,10'%self.job.batch['crc'],
            'TPOINT=17,0,0,0,0,0,0','TPOINT=17,1,0,200,0,200,3',
            'TPOINT=17,2,0,400,0,400,1','TCOMMIT=17','TRUN=17'])
        self.job.handle_reply('TSTAT 17 4 3 0 0 0')
        self.job.handle_reply('TSTAT 17 5 3 1 200 0');self.job.resume()
        self.assertEqual(self.lines[-1],'TRESUME=17,1')
        self.job.handle_reply('TSTAT 17 4 3 1 200 0')
        self.job.handle_reply('TSTAT 17 6 3 2 400 0')
        self.assertEqual(self.job.state,'DONE');self.assertFalse(self.job.active)

    def test_incomplete_ready_and_old_firmware_capacity_fail_closed(self):
        self.job.handle_reply('TCAPS 0 4096 3');self.assertEqual(self.lines[-1],'STOP')
        self.setUp();self.job.handle_reply('TCAPS 1 4096 3')
        self.job.handle_reply('TSTAT 17 3 0 0 0 0')
        self.assertEqual(self.job.state,'CANCELLED');self.assertNotIn('TRUN=17',self.lines)
        self.setUp();self.job.handle_reply('TCAPS 1 4096 3')
        self.job.handle_reply('TSTAT 17 1 0 0 0 0')
        self.job.handle_reply('TSTAT 17 4 3 0 0 0')
        self.assertEqual(self.job.state,'CANCELLED');self.assertNotIn('TRUN=17',self.lines)

    def test_rotation_center_fault_names_phase_and_keeps_stop(self):
        self.job.batch['points'][2]=(0,200,200,9000,STOP|ROTATE)
        self.ready()
        self.job.handle_reply('TSTAT 17 8 3 1 200 13')
        self.assertEqual(self.job.state,'CANCELLED')
        self.assertIn('原地转头中心偏差',self.job.reason)
        self.assertIn('8.0mm',self.job.reason)
        self.assertEqual(self.lines[-1],'STOP')

    def test_invalid_session_and_timeout_cancel(self):
        self.valid=False;self.job.tick();self.assertEqual(self.lines[-2:],['TABORT=17','STOP'])
        self.setUp();self.now=3.01;self.job.tick();self.assertEqual(self.job.state,'CANCELLED')
        self.assertNotIn('TRUN=17',self.lines)

    def test_invalid_context_preserves_specific_reason_and_stop_before_run(self):
        lines=[]
        job=BatchUploader(tiny_batch(),lines.append,valid=lambda:False,
            invalid_reason=lambda:'OPS遥测过期：400ms未收到有效帧（上限350ms）')
        job.start()
        self.assertEqual(job.state,'CANCELLED')
        self.assertIn('OPS遥测过期',job.reason)
        self.assertEqual(lines,['TABORT=17','STOP'])
        self.job.invalid_reason=lambda:'导航配置已变化：安全裕量'
        self.valid=False;self.job.tick()
        self.assertEqual(self.job.reason,'导航配置已变化：安全裕量')
        self.assertEqual(self.lines[-2:],['TABORT=17','STOP'])

    def test_old_batch_and_stale_upload_reply_cannot_rewind(self):
        self.ready();self.job.handle_reply('TSTAT 99 8 3 0 0 7')
        self.assertEqual(self.job.state,'STARTING')
        self.job.handle_reply('TSTAT 17 4 3 0 0 0')
        self.job.handle_reply('TSTAT 17 1 3 0 0 0')
        self.assertEqual(self.job.state,'RUNNING')
        self.job.handle_reply('TSTAT 17 5 3 2 400 0')
        self.assertEqual(self.job.state,'CANCELLED')

    def test_status_parser_preserves_identical_progress_replies(self):
        p=core.FrameParser()
        p.feed(b'TSTAT 17 5 3 1 200 0\r\n'*2)
        self.assertEqual(p.take_text(),['TSTAT 17 5 3 1 200 0']*2)


class MatchPackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data=competition.load_profile()
        cls.match=competition.compile_match(cls.data)
        cls.scene=competition.collision_scene(cls.data,10)

    def test_click_path_initial_and_final_rotation_and_quantized_crc(self):
        start=(2100,2100);goal=(2100,1500)
        route=core.plan_path(start,goal,grid=100,pad=10,footprint=(280,260,180),
            start_heading_deg=180,goal_heading_deg=180,rects=self.data['rects'],circles=self.data['circles'],
            bounds=self.data['bounds'],drivable_polygons=self.data['drivable_polygons'],geometry_verified=True)
        self.assertTrue(route['ok'],route['reason'])
        primitives=[dict(kind='LINE',start=start,end=goal)]
        route.update(core.generate_trajectory(primitives,self.scene),smoothed_primitives=primitives)
        batch=make_path_batch(route,start,180,180,(0,0,0),self.scene,self.data,token=17)
        self.assertEqual(batch['kind'],'STM32_POINT_PATH')
        self.assertEqual(batch['execution_mode'],'CONTINUOUS')
        self.assertEqual(batch['points'][0],(1500,1500,0,0,STOP))
        self.assertEqual(batch['points'][1][4],STOP|ROTATE)
        self.assertEqual(batch['points'][-1][4],STOP|ROTATE)
        self.assertEqual(batch['points'][-1][3],0)
        self.assertEqual(batch['stations'],{});self.assertEqual(batch['waits'],{})
        self.assertEqual(batch['collision_snapshot']['circles'],list(self.scene.circles))
        self.assertEqual(batch['crc'],zlib.crc32(b''.join(POINT.pack(*p) for p in batch['points'])))
        self.assertFalse(batch['match']['map_snapshot']['geometry_verified'])
        verified=copy.deepcopy(self.data);verified['geometry_verified']=True
        checked=make_path_batch(route,start,180,180,(0,0,0),self.scene,verified,token=17)
        self.assertEqual(checked['points'],batch['points'])
        self.assertEqual(checked['crc'],batch['crc'])

    def test_click_fallback_stops_at_turns_and_never_forces_smoothing(self):
        route=dict(ok=True,axis_matched=True,execution_safe=True,trajectory_safe=False,
            trajectory_reason='所有半径失败',steps=[
                dict(kind='TURN',to_x=2100,to_y=2100,heading_deg=270,action='TURN'),
                dict(kind='MOVE',to_x=2100,to_y=1500,heading_deg=270,action='FORWARD')])
        batch=make_path_batch(route,(2100,2100),180,180,(0,0,0),self.scene,self.data)
        self.assertEqual(batch['execution_mode'],'STOP_TURN_FALLBACK')
        self.assertIn('所有半径失败',batch['fallback_reason'])
        self.assertTrue(batch['points'][0][4]&STOP)
        self.assertEqual(batch['points'][1][4],STOP|ROTATE)
        self.assertFalse(any(p[4]&ARC for p in batch['points']))
        self.assertFalse(any(p[4]&STOP for p in batch['points'][2:-2]))

    def test_full_match_quantized_crc_body_tangent_and_automatic_stations(self):
        b=make_match_batch(self.match,(0,0,0),self.scene,token=17)
        self.assertEqual(POINT.size,16);self.assertLessEqual(b['point_count'],4096)
        self.assertEqual(b['crc'],zlib.crc32(b''.join(POINT.pack(*p) for p in b['points'])))
        self.assertEqual(len(b['stations']),7);self.assertEqual(b['waits'],{})
        self.assertEqual(b['station_mode'],'AUTO_ROUTE')
        self.assertFalse(any(q[4]&WAIT for q in b['points']))
        self.assertTrue(all(b['points'][i][4]&STOP for i in b['stations']))
        self.assertEqual(b['points'][0],(0,0,0,0,0))
        self.assertEqual(b['points'][-1][4]&3,STOP)
        for a,q in zip(b['points'],b['points'][1:]):
            if q[4]&ROTATE:self.assertEqual(a[:3],q[:3])
            else:
                self.assertGreater(q[2],a[2])
                self.assertLessEqual(abs(wrap((q[3]-a[3])/100)),3.1)
        self.assertTrue(any(abs(wrap(p['tangent_yaw_deg']-p['field_yaw_deg']))>170 for p in b['field_points']))

    def test_future_real_action_wait_is_explicit_and_unknown_mode_rejected(self):
        b=make_match_batch(self.match,(0,0,0),self.scene,station_mode='WAIT_FOR_ACTION')
        self.assertEqual(b['waits'],b['stations']);self.assertEqual(len(b['waits']),7)
        self.assertTrue(all(b['points'][i][4]&(STOP|WAIT)==STOP|WAIT for i in b['waits']))
        with self.assertRaises(ValueError):
            make_match_batch(self.match,(0,0,0),self.scene,station_mode='UNKNOWN')

    def test_map_verification_flag_does_not_block_batch_but_cancel_still_does(self):
        self.assertFalse(self.match['map_snapshot']['geometry_verified'])
        batch=make_match_batch(self.match,(0,0,0),self.scene,token=17)
        verified=copy.deepcopy(self.match);verified['map_snapshot']['geometry_verified']=True
        checked=make_match_batch(verified,(0,0,0),self.scene,token=17)
        self.assertEqual(checked['points'],batch['points'])
        self.assertEqual(checked['crc'],batch['crc'])
        with self.assertRaises(ValueError):make_match_batch(self.match,(0,0,0),self.scene,cancelled=lambda:True)

    def test_calibration_inverse_applies_rotation_translation_to_ops(self):
        b=make_match_batch(self.match,(150,200,90),self.scene,token=17)
        self.assertEqual(b['points'][0],(2000,-1500,0,-9000,0))
        c,s=0,1
        for q,p in zip(b['points'],b['field_points']):
            self.assertLessEqual(math.dist((150+c*q[0]/10+s*q[1]/10,200-s*q[0]/10+c*q[1]/10),
                                          (p['x_mm'],p['y_mm'])),.08)


if __name__=='__main__':unittest.main()
