"""Transport corruption must be rejected before GUI / runtime / raw recorder."""
import math
import queue
import struct
import unittest
from unittest.mock import patch
import zlib
import core


def packet(sequence, tick=None, values=None):
    values = values or (39.2+sequence,107.,-2.8)+(0.,)*21
    body = core.CRC_FRAME_MAGIC+struct.pack('<II24f',sequence,
                sequence*20 if tick is None else tick,*values)
    return body+struct.pack('<I',zlib.crc32(body))


class CrcTelemetryTests(unittest.TestCase):
    def test_each_byte_loss_insertion_bit_damage_only_delivers_intact_positions(self):
        good = packet(0)
        bad = packet(1)
        successor = packet(2)
        for index in range(len(bad)):
            corruptions = [bad[:index]+bad[index+1:]]
            if index:
                corruptions.append(bad[:index]+b'\xab'+bad[index:])
            corruptions.extend(bad[:index]+bytes([bad[index]^(1<<bit)])+bad[index+1:]
                               for bit in range(8))
            for corrupted in corruptions:
                for chunk in (1,17,2048):
                    parser = core.FrameParser();stream = good+corrupted+successor
                    frames = []
                    for start in range(0,len(stream),chunk):
                        frames.extend(parser.feed(stream[start:start+chunk]))
                    self.assertEqual([round(v[0],1) for v in frames],[39.2,41.2],(index,chunk))
                    self.assertEqual(parser.lost_packets,1)
                    self.assertFalse(parser.take_params())

    def test_bad_initial_header_never_locks_onto_payload_inf(self):
        values = (1.,2.,3.)+(float('inf'),)*21
        bad = bytearray(packet(0,values=values));bad[0]^=1
        parser=core.FrameParser()
        frames=parser.feed(bad+packet(1))
        self.assertEqual(len(frames),1)
        self.assertAlmostEqual(frames[0][0],40.2,places=4)

    def test_printable_payload_does_not_inject_parameters_or_status(self):
        body=bytearray(packet(0)[:-4])
        body[24:24+len(b'XVMIN=99.000\r\n')]=b'XVMIN=99.000\r\n'
        body+=struct.pack('<I',zlib.crc32(body))
        parser=core.FrameParser()
        self.assertEqual(len(parser.feed(body)),1)
        self.assertFalse(parser.take_params());self.assertFalse(parser.take_text())
        self.assertEqual(len(parser.feed(b'XVMIN=5.000\r\n'+packet(1))),1)
        self.assertEqual(parser.take_params(),[('XVMIN',5.)])
        self.assertEqual(parser.err_bytes,0)

    def test_legacy_then_crc_text_chunking_and_no_fallback_after_corruption(self):
        values=(1.,2.,3.)+(float('inf'),)*21
        legacy=struct.pack('<24f',*values)+core.FRAME_TAIL
        for chunk in (1,3,17,1000):
            parser=core.FrameParser();frames=[]
            stream=legacy+b'CCAPS 1 2048 3\r\n'+packet(0)+b'XVMIN=5.000\r\n'+legacy+packet(1)
            for start in range(0,len(stream),chunk):frames.extend(parser.feed(stream[start:start+chunk]))
            # At mode switch a buffered legacy frame can be discarded in favor
            # of the authenticated packet. Never accept a legacy frame AFTER it.
            self.assertIn(len(frames),(2,3))
            self.assertEqual([round(v[0],1) for v in frames[-2:]],[39.2,40.2])
            self.assertEqual(parser.protocol,'CRC1')
            self.assertEqual(parser.take_text(),['CCAPS 1 2048 3'])
            self.assertEqual(parser.take_params(),[('XVMIN',5.)])

    def test_sequence_wrap_duplicates_reboot_and_invalid_position(self):
        parser=core.FrameParser()
        frames=parser.feed(packet(0xfffffffe,10000)+packet(0xffffffff,10020)+packet(0,10040)+
                           packet(0,10040)+packet(2,10080))
        self.assertEqual(len(frames),4)
        self.assertEqual(parser.lost_packets,1);self.assertEqual(parser.duplicate_packets,1)
        self.assertEqual(len(parser.feed(packet(0,1500))),1)
        self.assertEqual(parser.device_restarts,1)
        bad_values=(math.nan,2.,3.)+(0.,)*21
        self.assertFalse(parser.feed(packet(1,1520,bad_values)))
        self.assertEqual(parser.invalid_pose_frames,1)
        self.assertEqual(len(parser.feed(packet(2,1540))),1)

    def test_batch_read_preserves_device_spacing_and_rejects_bad_frame_before_queue(self):
        worker=core.SerialWorker('TEST',115200,queue.Queue(),queue.Queue())
        broken=bytearray(packet(1));broken[16]^=1
        worker._receive(packet(0)+broken+packet(2)+packet(3),123.0)
        frames=list(worker.frame_q.queue)
        self.assertEqual(len(frames),3)
        self.assertAlmostEqual(frames[0][0],122.94)
        self.assertAlmostEqual(frames[1][0],122.98)
        self.assertEqual(frames[2][0],123.0)
        self.assertEqual(worker.last_frame_monotonic,123.0)

    def test_no_corrupt_data_refreshes_link_freshness(self):
        worker=core.SerialWorker('TEST',115200,queue.Queue(),queue.Queue())
        worker._receive(packet(0),100.0)
        bad=bytearray(packet(1));bad[-1]^=1
        worker._receive(bad,200.0)
        self.assertEqual(worker.last_frame_monotonic,100.0)
        self.assertEqual(worker.frame_q.qsize(),1)

    def test_batched_mode_switch_does_not_enqueue_legacy_after_crc_is_observed(self):
        worker=core.SerialWorker('TEST',115200,queue.Queue(),queue.Queue())
        legacy=struct.pack('<24f',*(1.,2.,3.,)+(0.,)*21)+core.FRAME_TAIL
        worker._receive(legacy,122.9)
        worker.frame_q.get_nowait()
        worker._receive(legacy+packet(0)+packet(1),123.)
        frames=list(worker.frame_q.queue)
        self.assertEqual(len(frames),2)
        self.assertEqual([round(f[1][0],1) for f in frames],[39.2,40.2])
        self.assertLess(frames[0][0],frames[1][0])

    def test_renegotiate_crc_after_firmware_reset_without_replaying_motion(self):
        worker=core.SerialWorker('TEST',115200,queue.Queue(),queue.Queue())
        worker.opened_monotonic=10;worker._vofa_last=10
        worker._receive(packet(30,20000),10.1)
        writes=[]
        with patch.object(worker,'_write_line',side_effect=lambda s:writes.append(s) or True), \
             patch.object(core.time,'monotonic',return_value=12):
            worker._maybe_resend_vofa();worker._maybe_negotiate_telemetry()
        self.assertEqual(writes,['VOFA','TELEM=1'])
        worker._receive(packet(0,1500),12.1)
        self.assertEqual(worker.parser.device_restarts,1)
        with patch.object(worker,'_write_line',side_effect=lambda s:writes.append(s) or True), \
             patch.object(core.time,'monotonic',return_value=14):
            worker._maybe_negotiate_telemetry()
        self.assertEqual(writes,['VOFA','TELEM=1'])


if __name__=='__main__':unittest.main()
