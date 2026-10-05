"""帧间文字不得把payload中的+inf伪帧尾转为假OPS坐标。"""
import math
import struct
import unittest
import core


class MixedTelemetryTests(unittest.TestCase):
    def frame(self):
        values=[0.,2.,.5]+[0.]*21
        values[18]=float('inf')
        return struct.pack('<24f',*values)+core.FRAME_TAIL

    def test_status_and_embedded_tail_preserve_every_pose_and_text(self):
        frame=self.frame()
        for text in (b'TSTAT 17 4 4 1 10760 0\r\n',b'XVMIN=5.000\r\n',
                     b'ACK S35 CAN_SUBMITTED CMD=3A (NO MOTOR ACK)\r\n'):
            for chunk in (1,3,17,1000):
                parser=core.FrameParser();stream=frame+text+frame+text+frame
                result=[]
                for start in range(0,len(stream),chunk):
                    result.extend(parser.feed(stream[start:start+chunk]))
                self.assertEqual(len(result),3,(text,chunk))
                self.assertEqual([v[:3] for v in result],[(0.,2.,.5)]*3)
                self.assertTrue(all(math.isinf(v[18]) for v in result))
                self.assertTrue(parser.synced)
                if text.startswith(b'TSTAT'):
                    self.assertEqual(parser.take_text(),[text.decode().strip()]*2)
                if text.startswith(b'XVMIN'):
                    self.assertEqual(parser.take_params(),[('XVMIN',5.)]*2)

    def test_reply_on_startup_and_repeated_partial_rows_preserve_frame(self):
        parser=core.FrameParser();frame=self.frame()
        self.assertEqual(parser.feed(b'CCAP'),[])
        self.assertEqual(parser.feed(b'S 1 2048 3\r'),[])
        self.assertEqual(parser.feed(b'\n'+frame)[0][:3],(0.,2.,.5))
        self.assertEqual(parser.take_text(),['CCAPS 1 2048 3'])


if __name__=='__main__':unittest.main()
