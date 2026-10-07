"""DL-20带宽、独立CRC2帧和慢通道新鲜度；不打开真实串口。"""
import math
import queue
import struct
import unittest
from unittest.mock import Mock, patch
import zlib
import core
from dm_motion import Arrival


def packet(seq,tick,values,version=2):
    raw=bytes((0xa5,0x5a,version,len(values)))+struct.pack('<II',seq,tick)+struct.pack('<%df'%len(values),*values)
    return raw+struct.pack('<I',zlib.crc32(raw))


class WirelessTelemetryTests(unittest.TestCase):
    def test_split_mixed_full_pose_text_and_crc1(self):
        full=tuple(float(i) for i in range(24))
        stream=packet(0,0,full)+b'XVMIN=0.500\r\n'+packet(1,20,(8.,9.,10.))+packet(2,40,full,1)
        for size in (1,3,7,28,112,len(stream)):
            p=core.FrameParser();frames=[];counts=[];ticks=[]
            for i in range(0,len(stream),size):
                frames.extend(p.feed(stream[i:i+size]));counts.extend(p.frame_channels);ticks.extend(p.frame_full_ticks)
            self.assertEqual(frames,[full,(8.,9.,10.)+full[3:],full])
            self.assertEqual(counts,[24,3,24]);self.assertEqual(ticks,[0,0,40])
            self.assertEqual(p.take_params(),[('XVMIN',.5)])
            self.assertEqual(p.crc_errors+p.lost_packets,0)

    def test_corruption_noise_and_loss_recover_without_delta_dependency(self):
        p=core.FrameParser();full=tuple(float(i) for i in range(24))
        self.assertEqual(len(p.feed(packet(0,0,full))),1)
        bad=bytearray(packet(1,20,(10.,20.,30.)));bad[-1]^=1
        good=packet(3,60,(4.,5.,6.))
        result=p.feed(b'\xff'+bad+b'\x00\x00\x80\x7f'+good)
        self.assertEqual(result,[(4.,5.,6.)+full[3:]])
        self.assertEqual(p.crc_errors,1);self.assertEqual(p.lost_packets,2)
        self.assertEqual(p.take_params(),[])

    def test_reboot_and_missing_full_do_not_invent_slow_channels(self):
        p=core.FrameParser();full=tuple(float(i) for i in range(24))
        self.assertEqual(p.feed(packet(1,20,(1.,2.,3.))),[])
        p.feed(packet(30,1000,full))
        self.assertEqual(p.feed(packet(0,0,(1.,2.,3.))),[])
        self.assertEqual(p.device_restarts,1)
        self.assertEqual(p.feed(packet(1,20,full)),[full])

    def test_invalid_pose_and_duplicates_never_refresh(self):
        p=core.FrameParser();full=tuple(float(i) for i in range(24))
        p.feed(packet(0,0,full))
        self.assertEqual(p.feed(packet(1,20,(math.nan,1.,2.))),[])
        p.feed(packet(2,40,(1.,2.,3.)))
        self.assertEqual(p.feed(packet(2,40,(1.,2.,3.))),[])
        self.assertEqual(p.invalid_pose_frames,1);self.assertEqual(p.duplicate_packets,1)

    def test_worker_records_present_channels_and_full_feedback_age(self):
        events=[];frames=queue.Queue()
        worker=core.SerialWorker('TEST',115200,frames,queue.Queue(),link_mode='dl20',
            event_cb=lambda event,**data:events.append((event,data)))
        full=tuple(float(i) for i in range(24))
        worker._receive(packet(0,0,full)+packet(1,20,(1.,2.,3.)),100.)
        old=frames.get();new=frames.get()
        self.assertAlmostEqual(old[1].full_state_monotonic,99.98)
        self.assertAlmostEqual(new[1].full_state_monotonic,99.98)
        rows=[d for e,d in events if e=='FRAME']
        self.assertEqual([r['present_channels'] for r in rows],[24,3])
        self.assertEqual(rows[-1]['full_state_device_tick_ms'],0)
        arrival=Arrival(1,1,.2)
        self.assertFalse(arrival.update(new[1].full_state_monotonic,0,0))
        self.assertFalse(arrival.update(new[1].full_state_monotonic,0,0))

    def test_normal_upload_is_paced_and_clear_cannot_leave_a_deferred_command(self):
        q=queue.Queue();w=core.SerialWorker('TEST',115200,queue.Queue(),q,link_mode='dl20')
        w.ser=Mock(is_open=True,out_waiting=0)
        w.ser.write.side_effect=lambda data:len(data)
        for _ in range(100):q.put('CPOINT='+('1,'*40)+'1')
        total=0
        for i in range(1001):
            with patch.object(core.time,'monotonic',return_value=100+i*.01):
                if w._ordinary_ready():
                    cmd=w._drain_one(q);w._write_line(cmd);total+=len(cmd)+1
        self.assertLessEqual(total,450*10+128)
        self.assertGreater(total,4000)
        while not q.empty():q.get_nowait()
        self.assertFalse(w._ordinary_ready())
        w._tx_credit=-128
        self.assertTrue(w._write_line('STOP'),'急停不等待普通命令的带宽信用')

    def test_backpressure_and_recovery_keep_dl20_mode(self):
        w=core.SerialWorker('TEST',115200,queue.Queue(),queue.Queue(),link_mode='dl20')
        w.line_q.put('PING');w.ser=Mock(is_open=True,out_waiting=200)
        self.assertFalse(w._ordinary_ready())
        w.opened_monotonic=1
        with patch.object(core.time,'monotonic',return_value=10),patch.object(w,'_write_line',return_value=True) as send:
            w._maybe_resend_vofa();self.assertEqual(send.call_args.args,('TELEM=2',))
            w._maybe_negotiate_telemetry();self.assertEqual(send.call_args.args,('TELEM=2',))


if __name__=='__main__':unittest.main()
