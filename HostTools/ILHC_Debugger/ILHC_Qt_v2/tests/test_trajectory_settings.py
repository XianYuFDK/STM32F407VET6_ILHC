"""轨迹参数配置与固件接口一致性校验。"""
import copy
import math
from pathlib import Path
import re
import unittest
import core

class SettingsTests(unittest.TestCase):
    def document(self):
        return dict(schema_version=1,kind='ILHC_TRAJECTORY_SETTINGS',
                    parameters={n:d for n,_,_,_,d,_ in core.TRAJECTORY_PARAMS})

    def test_valid_profile_and_default_units(self):
        values=core.validate_trajectory_settings(self.document())
        self.assertEqual(values['TVMAX'],500)
        self.assertEqual(values['TLOOK'],100)
        self.assertEqual(len(values),12)

    def test_missing_unknown_boolean_nonfinite_and_range_rejected(self):
        for key,value in (('TVMAX',True),('TVMAX',float('nan')),('TVMAX',float('inf')),
                          ('TVMAX',0),('TKPX',31),('TLOOK',301)):
            data=self.document();data['parameters'][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                core.validate_trajectory_settings(data)
        for data in (None,{},dict(schema_version=2,kind='ILHC_TRAJECTORY_SETTINGS',parameters={})):
            with self.assertRaises(ValueError):core.validate_trajectory_settings(data)
        for modify in (lambda v:v.pop('TKPX'),lambda v:v.update(OTHER=1)):
            data=self.document();modify(data['parameters'])
            with self.assertRaises(ValueError):core.validate_trajectory_settings(data)

    def test_old_nine_parameter_json_gets_new_speed_defaults(self):
        data=self.document()
        for key in ('TACC','TDEC','TVARC'):data['parameters'].pop(key)
        values=core.validate_trajectory_settings(data)
        self.assertEqual(values['TACC'],600)
        self.assertEqual(values['TDEC'],1000)
        self.assertEqual(values['TVARC'],150)

    def test_ui_ranges_match_actual_firmware_table(self):
        root=Path(__file__).resolve().parents[4]
        source=(root/'Hardware/debug_usart.c').read_text(encoding='utf-8')
        rows=dict((name,(float(lo),float(hi))) for name,lo,hi in re.findall(
            r'\{"(T[A-Z]+)",\s*&traj_control_params\.\w+,\s*([\d.]+)f,\s*([\d.]+)f\}',source))
        self.assertEqual(set(rows),core.TRAJECTORY_NAMES)
        for name,_,lo,hi,_,_ in core.TRAJECTORY_PARAMS:self.assertEqual(rows[name],(lo,hi))

if __name__=='__main__':unittest.main(verbosity=2)
