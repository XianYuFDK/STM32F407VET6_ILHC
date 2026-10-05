"""真实公共坐标闭环：浮点坐标、世界/车体轴、最短转角与异常拒绝。"""
import ctypes
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

from position_test_support import ROOT, write_position_kernel


class ChassisPositionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='ilhc-position-')
        folder = Path(cls.temp.name)
        binary = folder / 'position.dll'
        subprocess.run(['gcc', '-shared', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I', str(ROOT/'Hardware'), str(write_position_kernel(folder)),
                        '-o', str(binary), '-lm'], check=True)
        cls.dll = ctypes.CDLL(str(binary))
        p = ctypes.POINTER(ctypes.c_float)
        cls.dll.chassis_move_reference.argtypes = [p, p, p, p, ctypes.c_uint8, p, p]
        cls.dll.chassis_move_reference.restype = ctypes.c_uint8

    @classmethod
    def tearDownClass(cls):
        # Windows先卸载真实C核心，避免临时DLL仍被占用。
        handle = cls.dll._handle
        cls.dll = None
        ctypes.windll.kernel32.FreeLibrary.argtypes = [ctypes.c_void_p]
        ctypes.windll.kernel32.FreeLibrary(handle)
        cls.temp.cleanup()

    def command(self, actual, target, gains=(6, 6, 6), feedforward=None, body=0):
        f = ctypes.c_float * 3
        error, command = f(999, 999, 999), f(999, 999, 999)
        ok = self.dll.chassis_move_reference(f(*actual), f(*target), f(*gains),
                None if feedforward is None else f(*feedforward), body, error, command)
        return ok, tuple(error), tuple(command)

    def test_fractional_target_coordinates_are_not_truncated(self):
        ok, error, command = self.command((10.1, 20.2, 30.3), (10.35, 20.7, 30.425))
        self.assertEqual(ok, 1)
        for value, expected in zip(error, (.25, .5, .125)):
            self.assertAlmostEqual(value, expected, places=4)
        for value, expected in zip(command, (1.5, 3, .75)):
            self.assertAlmostEqual(value, expected, places=4)

    def test_world_feedback_independent_of_car_heading(self):
        for yaw in (0, 90, 180, -90, 37.5):
            with self.subTest(yaw=yaw):
                ok, error, command = self.command((100, -100, yaw), (104, -103, yaw), (2, 3, 4))
                self.assertEqual((ok, error, command), (1, (4, -3, 0), (8, -9, 0)))

    def test_body_gains_apply_after_coordinate_rotation(self):
        for yaw in (0, 90, 180, -90, 37.5):
            ok, error, command = self.command((0, 0, yaw), (20, 40, yaw+1), (2, 3, 4), body=1)
            self.assertEqual(ok, 1)
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            self.assertAlmostEqual(command[0], 2*(c*20-s*40), places=4)
            self.assertAlmostEqual(command[1], 3*(s*20+c*40), places=4)
            self.assertEqual(error, (20, 40, 1))

    def test_heading_wrap_uses_shortest_turn(self):
        for actual, target, expected in ((179, -179, 2), (-179, 179, -2),
                (359, 1, 2), (1, 359, -2), (1080, .25, .25)):
            ok, error, command = self.command((0, 0, actual), (0, 0, target))
            self.assertEqual(ok, 1)
            self.assertAlmostEqual(error[2], expected)
            self.assertAlmostEqual(command[2], 6*expected)

    def test_feedforward_preserves_cruise_and_feedback_corrects_actual_pose(self):
        self.assertEqual(self.command((0, 0, 0), (0, 0, 0), feedforward=(0, 500, 0))[2], (0, 500, 0))
        ok, error, command = self.command((2, 10, 1), (0, 10, 0), feedforward=(0, 500, 0))
        self.assertEqual((ok, error, command), (1, (-2, 0, -1), (-12, 500, -6)))
        # 倒退及横移使用同一坐标闭环，不以车头方向限制速度符号。
        self.assertEqual(self.command((0, 0, 0), (0, 0, 0), feedforward=(100, -500, 0))[2], (100, -500, 0))

    def test_nonfinite_inputs_clear_previous_output(self):
        for field in range(4):
            for axis in range(3):
                for bad in (math.nan, math.inf, -math.inf):
                    vectors = [[0, 0, 0], [1, 1, 1], [6, 6, 6], [0, 500, 0]]
                    vectors[field][axis] = bad
                    self.assertEqual(self.command(*vectors[:3], feedforward=vectors[3]),
                                     (0, (0, 0, 0), (0, 0, 0)))
        self.assertEqual(self.command((-3e38, 0, 0), (3e38, 0, 0)),
                         (0, (0, 0, 0), (0, 0, 0)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
