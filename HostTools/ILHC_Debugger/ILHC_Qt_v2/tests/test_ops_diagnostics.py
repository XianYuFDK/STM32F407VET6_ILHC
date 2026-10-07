"""OPS诊断字段精度、分片解析与运行结束前的原始/结构化记录及离线分析。"""
import json
from pathlib import Path
import tempfile
import unittest

import core
from analyze_run import analyze
from ops_diagnostics import parse_ops_text,summarize
from run_journal import RunJournal
from tests.test_crc_telemetry import packet


class OpsDiagnosticTests(unittest.TestCase):
    def test_combined_causes_and_uint32_are_exact_and_invalid_lines_rejected(self):
        e=parse_ops_text('OPSE 4294967295 514 4294967295 4294967295 8 4294967295 0 65535')
        self.assertEqual(e['causes'],['OPS时间戳回退','USART2硬件错误'])
        self.assertEqual(e['previous_session_id'],4294967295)
        self.assertEqual(parse_ops_text('OPSE 20 2048 209976 4 4 209250 209270 41853')['causes'],
                         ['OPS接收已恢复（续行）'])
        self.assertEqual(parse_ops_text('OPSE 21 1024 210157 4 4 209250 209250 41849')['causes'],
                         ['OPS接收恢复超过200ms'])
        self.assertEqual(parse_ops_text('OPSE 1 2147483648 1 1 2 3 4 5')['unknown_cause_bits'],2147483648)
        for line in ['OPSE 1 2','OPSE -1 1 2 3 4 5 6 7','OPSE 4294967296 1 2 3 4 5 6 7',
                     'OPSD 1 2 3 4 256 1 6 7 8','OPSD 1 2 3 4 11 2 6 7 8',
                     'OPSX 1 1 nan 2 3 4 5 6','OPSE 1 1 2 3 4 5 6 65536']:
            self.assertIsNone(parse_ops_text(line),line)

    def test_fragmented_text_and_telemetry_keep_event_identity_and_flag_bits(self):
        text=b'OPSE 7 16 100 3 3 80 100 9\r\nOPSX 7 15 20 20 0 0 0 0\r\n'
        for size in [1,3,17,112]:
            parser=core.FrameParser();frames=[];data=text+packet(0)+b'TSTAT 17 7 4 1 2193 17\r\n'
            for start in range(0,len(data),size):frames.extend(parser.feed(data[start:start+size]))
            parsed=[parse_ops_text(t) for t in parser.take_text()];parsed=[d for d in parsed if d]
            report=summarize(parsed)
            self.assertEqual(len(frames),1)
            self.assertTrue(report['faults'][0]['complete'])
            self.assertTrue(report['faults'][0]['OPSX']['imu_rebased'])

    def test_lost_context_is_not_filled_from_unrelated_snapshot_or_event(self):
        records=[parse_ops_text('OPSE 7 16 100 3 3 80 100 9'),parse_ops_text('OPSX 8 11 0 0 0 8 0 0'),
                 parse_ops_text('OPSD 101 3 10 101 11 1 0 0 0')]
        report=summarize(records)
        self.assertFalse(report['faults'][0]['complete'])
        self.assertNotIn('OPSX',report['faults'][0])
        self.assertFalse(report['faults'][1]['complete'])

    def test_native_diagnostic_text_is_analyzed_and_retained_after_run_ends(self):
        with tempfile.TemporaryDirectory() as directory:
            log=RunJournal(directory,core.CHANNELS);path=log.start('REAL',{})
            rid=log.begin_run('COORDINATE_BATCH',{},dict(id=17,points=[]))
            try:
                for text in ['OPSE 7 512 100 3 3 80 80 9','OPSX 7 11 0 20 0 8 2 1',
                             'OPSD 100 3 9 80 11 0 20 2 0','OPSR 100 1 8 1 0 0 0',
                             'TSTAT 17 7 4 1 2193 17']:
                    log.emit('RX_TEXT',text=text)
                    diagnostic=parse_ops_text(text)
                    if diagnostic:log.emit('OPS_DIAGNOSTIC',**diagnostic)
                log.end_run('CANCELLED','STM32 CANCELLED：OPS会话变化')
                self.assertFalse(log.emit('RX_TEXT',text='OPSD 200 3 9 80 11 0 120 2 0'))
            finally:log.close()
            report,_=analyze(path)
            diagnostic=report['runs'][0]['ops_diagnostics']
            self.assertEqual(diagnostic['faults'][0]['causes'],['USART2硬件错误'])
            self.assertEqual(diagnostic['faults'][0]['OPSX']['uart_error'],8)
            self.assertEqual(len(diagnostic['snapshots']),2)
            events=[json.loads(x) for x in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual(sum(e['event']=='OPS_DIAGNOSTIC' for e in events),4)
            self.assertTrue((path/'runs'/rid/'result.json').exists())


if __name__=='__main__':unittest.main(verbosity=2)
