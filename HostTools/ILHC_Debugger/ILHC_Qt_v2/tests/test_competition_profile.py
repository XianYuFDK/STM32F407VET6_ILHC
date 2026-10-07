import copy
import unittest
from unittest.mock import patch
import competition_simulation as competition
import navigation_planner as nav
from pathlib import Path


class CompetitionProfileTests(unittest.TestCase):
    def test_add_defaults_without_replacing_map_or_modifying_input(self):
        data=nav.load_map(Path(competition.__file__).with_name('navigation_map.json'))
        data.pop('competition', None)  # 已标定默认地图包含配置；这里仍专测缺配置时的补齐。
        data['dynamic_circles']=[[2100,1200,25,'实时障碍']]
        original=copy.deepcopy(data)
        configured,added=competition.with_competition_defaults(data)
        self.assertTrue(added)
        self.assertEqual(data,original)
        for key,value in original.items():self.assertEqual(configured[key],value)
        self.assertEqual(configured['competition']['start_zones']['1'],[2250,2250])
        self.assertIn('实时障碍',competition.collision_scene(configured).pose_reason(2100,1200,180))

    def test_partial_config_preserves_existing_station_and_fills_missing_fields(self):
        data=competition.load_profile()
        data['competition']={'stations':{'qr':[2100,1250]},'custom_note':'keep'}
        configured,added=competition.with_competition_defaults(data)
        self.assertEqual(configured['competition']['stations']['qr'],[2100,1250])
        self.assertEqual(configured['competition']['custom_note'],'keep')
        self.assertNotIn('stations.qr',added)
        self.assertIn('stations.raw',added)

    def test_complete_user_profile_needs_no_default_file_and_is_idempotent(self):
        data=competition.load_profile()
        data['competition']['stations']['qr']=[2100,1250]
        with patch.object(competition,'load_profile',side_effect=AssertionError('must not replace custom config')):
            configured,added=competition.with_competition_defaults(data)
        self.assertEqual(configured,data);self.assertFalse(added)

    def test_invalid_explicit_coordinates_and_layout_are_not_overwritten(self):
        for key,value in (('start_zones',{'1':[float('nan'),0]}),('stations',{'qr':[2401,1200]}),
                          ('staging',None),('lane_nodes',[[1]])):
            data=competition.load_profile();data['competition'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):competition.with_competition_defaults(data)
        data=competition.load_profile();data['frame_id']='FIELD_MM'
        with self.assertRaises(ValueError):competition.with_competition_defaults(data)

    def test_damaged_default_template_reports_a_configuration_error(self):
        data=competition.load_profile();data.pop('competition')
        with patch.object(competition,'load_profile',return_value={'competition':None}),self.assertRaisesRegex(ValueError,'默认模板'):
            competition.with_competition_defaults(data)


if __name__=='__main__':unittest.main()
