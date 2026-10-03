"""审查反例已修复；此入口运行对应的修复回归测试，不连接硬件。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromName('tests.test_navigation_audit_fixes')
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
