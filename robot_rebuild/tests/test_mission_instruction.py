"""Fixed semantic tasks resolve named locations without executing instruction text."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import contextlib
import io
import runpy
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
from mission import MissionSession, load_mission, parse_instruction,parse_semantic_instruction


class MissionInstructionTests(unittest.TestCase):
    LOCATIONS = {'门口': [1000, 0], '桌边': [1000, 1000], '窗边': [0, 1000]}

    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.json'
            path.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
            return load_mission(path)

    def test_single_person_instruction_uses_configured_location(self):
        self.assertEqual(parse_instruction('去门口找人', self.LOCATIONS), [(1000, 0)])
        self.assertEqual(parse_instruction(' 去桌边找人 ', self.LOCATIONS), [(1000, 1000)])

    def test_navigation_only_and_polite_search_phrasing_have_explicit_actions(self):
        for text in ('去门口','前往门口','导航到门口停车','请移动到门口。'):
            plan=parse_semantic_instruction(text,self.LOCATIONS)
            self.assertEqual(plan['action'],'navigate')
            self.assertEqual(plan['points'],[(1000,0)])
        for text in ('请前往门口寻找人员。','到门口找人','去 门口 寻找人'):
            self.assertEqual(parse_semantic_instruction(text,self.LOCATIONS)['action'],'find_person')

    def test_offline_cli_previews_without_constructing_any_serial_link(self):
        script=Path(__file__).resolve().parents[1]/'rk/mission.py'
        output=io.StringIO()
        with mock.patch.object(sys,'argv',[str(script),'--instruction','依次去前方、左边、右边、后边、左前方、右前方停车']), \
             mock.patch('protocol.Link',side_effect=AssertionError('must not connect serial')), \
             mock.patch('protocol.PosixControlLink',side_effect=AssertionError('must not connect serial')), \
             contextlib.redirect_stdout(output):
            runpy.run_path(str(script),run_name='__main__')
        plan=json.loads(output.getvalue())
        self.assertFalse(plan['starts_motion'])
        self.assertEqual(plan['task']['action'],'navigate')
        self.assertEqual(plan['task']['location_names'],['前方','左边','右边','后边','左前方','右前方'])
        self.assertEqual(plan['task']['points'],[[1000,0],[0,1000],[0,-1000],[-1000,0],[1000,1000],[1000,-1000]])

    def test_direction_examples_resolve_actions_and_fixed_task_coordinates(self):
        rk=Path(__file__).resolve().parents[1]/'rk'
        expected={'前方':(1000,0),'左边':(0,1000),'右边':(0,-1000),
                  '后边':(-1000,0),'左前方':(1000,1000),'右前方':(1000,-1000)}
        for filename,default_action in (('mission_semantic_example.json','navigate'),
                                        ('mission_bottle_example.json','find_bottle')):
            path=rk/filename
            config=load_mission(path,include_semantics=True)
            self.assertEqual(config['action'],default_action)
            self.assertEqual(config['points'],[(1000,0)])
            for name,point in expected.items():
                for suffix,action in (('停车','navigate'),('找人','find_person'),('找水瓶','find_bottle')):
                    with self.subTest(filename=filename,name=name,action=action):
                        plan=load_mission(path,instruction='去'+name+suffix,include_semantics=True)
                        self.assertEqual(MissionSession(**plan).action,action)
                        self.assertEqual(plan['points'],[point])
            sequence=load_mission(path,instruction='依次去左边、前方、后边、右前方停车',include_semantics=True)
            session=MissionSession(**sequence)
            self.assertEqual(session.points,[(0,1000),(1000,0),(-1000,0),(1000,-1000)])
            self.assertTrue(session.initialize_pose)
            self.assertFalse(session.enabled)

    def test_sequential_navigation_and_comma_separators_preserve_order(self):
        plan=parse_semantic_instruction('请依次前往窗边，门口，桌边停车',self.LOCATIONS)
        self.assertEqual(plan['action'],'navigate')
        self.assertEqual(plan['location_names'],['窗边','门口','桌边'])
        self.assertEqual(plan['points'],[(0,1000),(1000,0),(1000,1000)])

    def test_bottle_search_is_an_explicit_task_and_never_a_color_request(self):
        for text in ('去门口找水瓶','请前往桌边寻找瓶子。','依次去门口、窗边找瓶子'):
            plan=parse_semantic_instruction(text,self.LOCATIONS)
            self.assertEqual(plan['action'],'find_bottle')
        config=self.load(dict(units='mm',locations=self.LOCATIONS,instruction='去门口找水瓶'))
        session=MissionSession(**config)
        self.assertEqual(session.target_category,39);self.assertFalse(session.enabled)

    def test_runtime_instruction_overrides_file_without_mutating_or_starting_it(self):
        data=dict(units='mm',locations=self.LOCATIONS,instruction='去门口找人')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'mission.json'
            path.write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8-sig')
            config=load_mission(path,instruction='去桌边',include_semantics=True)
            self.assertEqual(config['action'],'navigate')
            self.assertEqual(config['location_names'],['桌边'])
            self.assertEqual(config['instruction'],'去桌边')
            self.assertFalse(MissionSession(**config).enabled)
            self.assertEqual(json.loads(path.read_text(encoding='utf-8-sig')),data)

    def test_conflicting_action_and_invalid_visual_confirmation_limits_rejected(self):
        with self.assertRaises(ValueError):
            self.load(dict(units='mm',locations=self.LOCATIONS,instruction='去门口',action='find_person'))
        for value in (True,float('nan'),.1,.8):
            with self.subTest(value=value),self.assertRaises(ValueError):
                MissionSession([(0,0)],visual_stop_width=value)
        self.assertEqual(MissionSession([(0,0)],min_person_width=.6).visual_stop_width,.6)
        self.assertEqual(MissionSession([(0,0)],action='navigate',total_timeout_s=10).total_timeout_s,10)

    def test_sequential_instruction_preserves_requested_search_order(self):
        self.assertEqual(parse_instruction('依次去窗边、门口、桌边找人', self.LOCATIONS),
                         [(0, 1000), (1000, 0), (1000, 1000)])

    def test_named_instruction_loads_into_existing_session_parameters(self):
        config = self.load(dict(units='mm', locations=self.LOCATIONS,
                                instruction='依次去门口、桌边找人', dwell_s=4.))
        self.assertEqual(config, dict(points=[(1000, 0), (1000, 1000)], dwell_s=4.,initialize_pose=True))
        session = MissionSession(**config)
        self.assertEqual(session.state, 'preparing')
        self.assertFalse(session.enabled)

    def test_explicit_coordinate_mission_remains_compatible(self):
        config = self.load(dict(units='mm', points=[[1000, 0]]))
        self.assertEqual(config, dict(points=[[1000, 0]]))

    def test_unknown_location_is_rejected(self):
        for text in ('去走廊找人', '依次去门口、走廊找人'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_instruction(text, self.LOCATIONS)

    def test_incomplete_or_ambiguous_templates_are_rejected(self):
        for text in ('去门口、桌边找人', '依次去门口找人', '依次去门口、找人',
                     '依次去、门口找人', '依次去门口、门口找人',
                     '去门口或桌边找人',
                     '去门口找人然后回来', '先去门口找人', '去门口找人找人'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_instruction(text, self.LOCATIONS)

    def test_unknown_targets_and_colors_are_not_task_grammar(self):
        for text in ('去门口找椅子', '去门口找红色水瓶', '去门口找红色的人',
                     '去门口找穿红衣服的人', '去门口跟随人'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_instruction(text, self.LOCATIONS)

    def test_code_and_extra_commands_are_only_rejected_strings(self):
        for text in ("__import__('os').system('echo test')", '去门口找人;前进',
                     '去门口找人\nimport os', '去门口找人 && reboot',
                     '{"points":[[1000,0]]}', '<script>去门口找人</script>'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_instruction(text, self.LOCATIONS)

    def test_all_location_coordinates_are_validated_including_unused_names(self):
        for point in ([10001, 0], [-10001, 0], [0, 10001], [True, 0],
                      [1.5, 0], [1000], [1000, 0, 0], '1000,0'):
            with self.subTest(point=point), self.assertRaises(ValueError):
                parse_instruction('去门口找人', {'门口': [1000, 0], '桌边': point})
        self.assertEqual(parse_instruction('去门口找人', {'门口': [-10000, 10000]}),
                         [(-10000, 10000)])

    def test_location_names_and_count_are_bounded(self):
        for locations in ({}, [], {'门口、桌边': [1000, 0]}, {'门口找人': [1000, 0]},
                          {'门口停车':[1000,0]},{'门口停下':[1000,0]},
                          {'门口\n': [1000, 0]}, {1: [1000, 0]},
                          {f'点{i}': [i, 0] for i in range(9)}):
            with self.subTest(locations=locations), self.assertRaises(ValueError):
                parse_instruction('去门口找人', locations)
        locations = {f'点{i}': [i*100, 0] for i in range(8)}
        text = '依次去'+'、'.join(locations)+'找人'
        self.assertEqual(len(parse_instruction(text, locations)), 8)

    def test_configuration_cannot_mix_instructions_with_points_or_omit_mapping(self):
        for data in (dict(units='mm', points=[[1000, 0]], locations=self.LOCATIONS,
                          instruction='去门口找人'),
                     dict(units='mm', points=[[1000, 0]], locations=self.LOCATIONS),
                     dict(units='mm', instruction='去门口找人'),
                     dict(units='mm', locations=self.LOCATIONS),
                     dict(units='m', locations=self.LOCATIONS, instruction='去门口找人'),
                     dict(units='mm', locations=self.LOCATIONS, instruction='去门口找人', color='red')):
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.load(data)

    def test_duplicate_json_location_names_are_ambiguous_and_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.json'
            path.write_text('{"units":"mm","instruction":"去门口找人",'
                            '"locations":{"门口":[1000,0],"门口":[0,1000]}}', encoding='utf-8')
            with self.assertRaises(ValueError):
                load_mission(path)


if __name__ == '__main__':
    unittest.main(verbosity=2)
