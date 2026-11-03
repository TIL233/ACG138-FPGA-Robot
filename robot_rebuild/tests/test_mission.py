"""Task 6 dispatch tests: real navigation handshakes, no serial/camera hardware."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rk'))
from mission import MissionSession
from protocol import (Telemetry, NavTelemetry, NAV_CLEAR, NAV_WRITE,
                      NAV_COMMIT, NAV_POSE_RESET, command_packet,
                      navigation_packet)


class SimulatedLink:
    """Acknowledge configuration, but only publish fresh samples on a clock tick.

    Navigation travel is deliberately not simulated: tests explicitly signal a
    waypoint arrival. This exercises the real NavigationSession ACK and ARM
    lifecycle without pretending to validate odometry or motor mechanics.
    """
    def __init__(self):
        self.now = 0.
        self.seq = 0
        self.last_rx = self.last_nav_rx = 0.
        self.connected = self.nav_connected = True
        self.telemetry = Telemetry(0, 0, 0x3f, 0, 0, 1000, 1000,
                                   0, 0, 0, 0, 0, 0)
        self.nav_telemetry = NavTelemetry(0, 0, 0, 0, 128 | 64,
                                          0, 0, 0, 0, 255)
        self.commands = []
        self.configs = []
        self.fail_send = False

    def fresh(self, now):
        self.now = now
        if self.connected:
            self.last_rx = now
            self.telemetry = replace(self.telemetry, seq=(self.telemetry.seq + 1) & 255)
        if self.nav_connected:
            self.last_nav_rx = now
            self.nav_telemetry = replace(self.nav_telemetry,
                                         seq=(self.nav_telemetry.seq + 1) & 255)

    def live(self, now=None):
        return self.connected and 0 <= (self.now if now is None else now) - self.last_rx < .4

    def nav_live(self, now=None):
        return self.nav_connected and 0 <= (self.now if now is None else now) - self.last_nav_rx < .4

    def send(self, armed=False, command=0, speed=0, *, nav=False, follow=False):
        if self.fail_send:
            raise OSError('simulated UART write failure')
        command_packet(self.seq, armed, command, speed, nav=nav, follow=follow)
        self.seq = (self.seq + 1) & 255
        self.commands.append(dict(time=self.now, armed=armed, command=command,
                                  speed=speed, nav=nav,follow=follow))
        health = self.telemetry.health | 64 if armed else self.telemetry.health & ~64
        self.telemetry = replace(self.telemetry, health=health,
                                  command=command if armed else 0,
                                  speed=speed if armed else 0)
        flags = self.nav_telemetry.flags
        flags = (flags | 16 | 1) if armed and nav else flags & ~(16 | 1)
        self.nav_telemetry = replace(self.nav_telemetry, flags=flags)

    def configure_navigation(self, opcode, **fields):
        seq = self.seq
        navigation_packet(seq, opcode, **fields)
        self.seq = (self.seq + 1) & 255
        self.configs.append((opcode, dict(fields)))
        n = self.nav_telemetry
        flags = n.flags
        if opcode == NAV_CLEAR:
            flags &= ~(8 | 4 | 2 | 1)
        elif opcode == NAV_COMMIT:
            flags |= 8
            n = replace(n, count=fields['count'], index=0)
        elif opcode == NAV_POSE_RESET:
            flags = (flags | 32) & ~(4 | 2)
            n = replace(n, x_mm=0, y_mm=0, heading=0)
        self.nav_telemetry = replace(n, flags=flags, ack_seq=seq, ack_status=0)
        return seq

    def arrived(self):
        self.nav_telemetry = replace(self.nav_telemetry,
                                     flags=(self.nav_telemetry.flags | 2) & ~1)
        self.telemetry = replace(self.telemetry, velocity_l=0, velocity_r=0)


class MissionTests(unittest.TestCase):
    PERSON = [(0, .93, (230, 80, 410, 460))]
    OTHER = [(0, .90, (450, 80, 600, 460))]
    BOTTLE = [(39,.94,(290,140,350,380))]

    def setUp(self):
        self.link = SimulatedLink()
        self.now = 0.
        self.session = MissionSession([(1000, 0)], dwell_s=1,
                                       confirm_frames=3, confirm_s=.2)

    def tick(self, objects=(), dt=.1, captured=None, camera_live=True,
             fresh=True,image_height=480):
        self.now = round(self.now + dt, 6)
        if fresh:
            self.link.fresh(self.now)
        else:
            self.link.now = self.now
        self.session.step(self.link, self.now, objects,
                          self.now if captured is None else captured,
                          640, camera_live,height=image_height)
        return self.session.state

    def test_button_request_preparing_refreshes_idle_command_health_and_runs_once(self):
        self.link.telemetry=replace(self.link.telemetry,health=0x12)
        send=self.link.send
        def refreshed(*args,**kwargs):
            send(*args,**kwargs)
            self.link.telemetry=replace(self.link.telemetry,health=self.link.telemetry.health|1)
        self.link.send=refreshed
        self.assertTrue(self.session.request_run(self.link,self.now,source='button'))
        deadline=self.session.run_request[0]
        self.assertTrue(self.session.request_run(self.link,self.now+.01,source='button'))
        self.assertEqual(self.session.run_request[0],deadline)
        self.until('navigating')
        for _ in range(8):self.tick()
        self.assertTrue(self.session.navigator.enabled,self.session.text)

    def test_button_ready_waits_for_vision_and_zero_velocity_without_reclick(self):
        self.until('ready')
        self.link.telemetry=replace(self.link.telemetry,velocity_l=1)
        self.assertTrue(self.session.request_run(self.link,self.now,source='button'))
        for _ in range(4):self.tick(camera_live=False)
        self.assertEqual(self.arm_count(),0)
        self.link.telemetry=replace(self.link.telemetry,velocity_l=0)
        self.until('navigating')
        # A single encoder pulse during final confirmation must restart the
        # existing zero-speed/new-feedback boundary, never widen its zero gate.
        self.link.telemetry=replace(self.link.telemetry,velocity_r=1)
        self.tick()
        self.assertEqual(self.arm_count(),0)
        self.link.telemetry=replace(self.link.telemetry,velocity_r=0)
        for _ in range(10):self.tick()
        self.assertTrue(self.session.navigator.enabled,self.session.text)

    def test_button_stop_disconnect_fault_timeout_and_clock_rollback_never_resume(self):
        for cause in ('stop','disconnect','fault','timeout','clock','permission'):
            with self.subTest(cause=cause):
                self.setUp()
                self.assertTrue(self.session.request_run(self.link,0.,source='button'))
                self.tick(camera_live=False)
                before=len(self.link.commands)
                if cause=='stop':
                    self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
                elif cause=='disconnect':self.link.connected=False
                elif cause=='fault':self.link.telemetry=replace(self.link.telemetry,fault=32)
                elif cause=='permission':self.link.telemetry=replace(self.link.telemetry,health=0x11)
                if cause=='clock':
                    self.session.step(self.link,0.,(),0.,640,True,height=480)
                else:self.tick(dt=12. if cause=='timeout' else .1,camera_live=False)
                self.assertIn(self.session.state,('stopped','failed'))
                if cause=='stop':self.assertEqual(len(self.link.commands),before)
                self.link.connected=True
                self.link.telemetry=replace(self.link.telemetry,health=0x3f,fault=0)
                for _ in range(8):self.tick()
                self.assertEqual(self.arm_count(),0)
                self.assertFalse(self.session.request_run(self.link,self.now,source='button'))
                self.assertIn('重新生成',self.session.text)

    def test_button_navigate_needs_no_camera_ranges_or_lidar(self):
        self.session=MissionSession([(1000,0)],action='navigate')
        self.link.telemetry=replace(self.link.telemetry,health=0x13,distance_l=65535,distance_r=65535)
        self.assertTrue(self.session.request_run(self.link,0.,source='button'))
        for _ in range(30):self.tick(camera_live=False,image_height=None)
        self.assertTrue(self.session.navigator.enabled,self.session.text)

    def test_button_mode_or_manual_motion_cancels_without_disarm_override(self):
        for manual in (True,False):
            self.setUp()
            self.link.nav_telemetry=replace(self.link.nav_telemetry,nav_mode_count=0)
            self.assertTrue(self.session.request_run(self.link,0.,source='button'))
            self.tick(camera_live=False)
            before=len(self.link.commands)
            if manual:self.link.telemetry=replace(self.link.telemetry,command=2,state=1)
            else:self.link.nav_telemetry=replace(self.link.nav_telemetry,nav_mode_count=1)
            self.tick(camera_live=False)
            self.assertEqual(self.session.state,'stopped')
            self.assertEqual(len(self.link.commands),before)

    def test_button_first_tick_disconnect_and_stop_at_deadline_are_consumed(self):
        self.assertTrue(self.session.request_run(self.link,0.,source='button'))
        self.link.connected=False
        self.tick(camera_live=False)
        self.assertEqual(self.session.state,'stopped')
        self.setUp()
        self.assertTrue(self.session.request_run(self.link,0.,source='button'))
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
        self.tick(dt=12.)
        self.assertEqual(self.link.commands,[])

    def test_button_click_accepts_only_brief_late_ordinary_snapshot_with_live_stop_stream(self):
        self.now=.45;self.link.now=.45;self.link.last_nav_rx=.45
        self.assertFalse(self.link.live(self.now))
        self.assertTrue(self.session.request_run(self.link,self.now,source='button'))
        self.tick()
        self.until('navigating')
        for _ in range(8):self.tick()
        self.assertTrue(self.session.navigator.enabled)
        self.setUp();self.now=1.;self.link.now=1.;self.link.last_nav_rx=1.
        self.assertFalse(self.session.request_run(self.link,1.,source='button'))

    def test_button_prearm_camera_gap_waits_inside_original_deadline(self):
        self.until('ready')
        self.assertTrue(self.session.request_run(self.link,self.now,source='button'))
        self.tick()
        self.assertEqual(self.session.state,'navigating')
        for _ in range(25):self.tick(camera_live=False)
        self.assertEqual(self.arm_count(),0)
        self.assertEqual(self.session.state,'ready')
        for _ in range(10):self.tick()
        self.assertTrue(self.session.navigator.enabled,self.session.text)

    def test_button_prearm_done_cannot_be_mistaken_for_arrival(self):
        self.until('ready')
        self.assertTrue(self.session.request_run(self.link,self.now,source='button'))
        self.tick()
        self.link.arrived()
        self.tick()
        self.assertEqual(self.session.state,'stopped')
        self.assertEqual(self.arm_count(),0)

    def test_button_prearm_late_ordinary_frame_restarts_full_stop_boundary(self):
        self.until('ready')
        self.assertTrue(self.session.request_run(self.link,self.now,source='button'))
        self.tick()
        self.now=round(self.now+.45,6);self.link.now=self.now;self.link.last_nav_rx=self.now
        self.session.step(self.link,self.now,(),self.now,640,True,height=480)
        self.assertEqual(self.session.state,'ready')
        self.assertEqual(self.arm_count(),0)
        self.tick()
        self.assertEqual(self.arm_count(),0)
        for _ in range(9):self.tick()
        self.assertTrue(self.session.navigator.enabled,self.session.text)

    def test_motor_failure_keeps_cause_and_snapshot_after_disarm_clears_fault(self):
        self.begin()
        self.link.telemetry=replace(self.link.telemetry,fault=32)
        self.tick()
        self.assertEqual(self.session.state,'failed')
        self.assertIn('电机闭环',self.session.text)
        self.assertIn('0x0020',self.session.text)
        self.assertEqual(self.session.report['failure_snapshot']['telemetry']['fault'],32)
        self.link.telemetry=replace(self.link.telemetry,fault=0)
        self.tick()
        self.assertEqual(self.session.report['failure_snapshot']['telemetry']['fault'],32)
        self.assertFalse(self.session.enabled)
        self.session.reset()
        self.assertIsNone(self.session.report['failure_snapshot'])

    def until(self, states, objects=(), steps=80, **kwargs):
        states = {states} if isinstance(states, str) else set(states)
        for _ in range(steps):
            if self.session.state in states:
                return self.session.state
            self.tick(objects, **kwargs)
        self.fail(f'expected {states}, got {self.session.state}: {self.session.text}')

    def begin(self):
        self.until('ready')
        self.assertTrue(self.session.start(self.link, self.now))
        for _ in range(20):
            self.tick()
            if any(c['armed'] and c['nav'] for c in self.link.commands):
                break
        self.assertEqual(self.session.state, 'navigating', self.session.text)
        self.assertTrue(any(c['armed'] and c['nav'] for c in self.link.commands))

    def observe(self):
        self.begin()
        self.link.arrived()
        self.until('observing')

    def approach(self, objects=None):
        objects = self.PERSON if objects is None else objects
        self.observe()
        self.until('approaching', objects)
        self.tick(objects)
        self.tick(objects)  # One fresh acknowledgement after the first ARM.
        self.assertTrue(any(c['armed'] and not c['nav'] for c in self.link.commands))

    def arm_count(self):
        return sum(c['armed'] for c in self.link.commands)

    def assert_latched_failure(self):
        self.assertEqual(self.session.state, 'failed', self.session.text)
        arms = self.arm_count()
        self.link.connected = self.link.nav_connected = True
        self.link.telemetry = replace(self.link.telemetry, fault=0, state=0,
                                      health=0x3f, distance_l=1000, distance_r=1000)
        self.link.nav_telemetry = replace(self.link.nav_telemetry, flags=128 | 64 | 8 | 32)
        for _ in range(10):
            self.tick(self.PERSON)
        self.assertEqual(self.session.state, 'failed')
        self.assertEqual(self.arm_count(), arms, 'a recovered input must not auto-resume motion')
        self.assertFalse(self.link.commands[-1]['armed'])

    def test_startup_configures_but_cannot_arm_before_explicit_start(self):
        self.until('ready')
        for _ in range(10):
            self.tick(self.PERSON)
        self.assertEqual(self.arm_count(), 0)
        self.assertFalse(self.session.enabled)
        self.assertEqual(sum(op == NAV_POSE_RESET for op, _ in self.link.configs), 1)
        self.assertTrue(any(op == NAV_COMMIT for op, _ in self.link.configs))

    def test_real_navigation_to_person_has_disarm_stillness_then_success(self):
        self.approach()
        nav_arms = [i for i, c in enumerate(self.link.commands) if c['armed'] and c['nav']]
        follow_arm = next(i for i, c in enumerate(self.link.commands)
                          if c['armed'] and not c['nav'])
        between = self.link.commands[nav_arms[-1]+1:follow_arm]
        self.assertTrue(between and all(not c['armed'] for c in between))
        self.assertGreaterEqual(self.link.commands[follow_arm]['time'] - between[0]['time'], .25)
        self.link.telemetry = replace(self.link.telemetry, distance_l=500, distance_r=520)
        self.until('complete', self.PERSON)
        self.assertFalse(self.session.enabled)
        self.assertFalse(self.link.commands[-1]['armed'])
        arms = self.arm_count()
        for _ in range(10):
            self.tick(self.PERSON)
        self.assertEqual(self.arm_count(), arms)

    def test_delayed_fresh_feedback_survives_periodic_disarm_until_complete(self):
        self.begin()
        self.link.arrived()
        self.tick()
        self.assertEqual(self.session.state, 'settling')

        def delayed_tick(objects=()):
            # The viewer ticks every 210 ms, after the latest 100 ms-old
            # ordinary/navigation samples. Both streams remain live.
            self.link.fresh(round(self.now + .11, 6))
            self.tick(objects, dt=.21, fresh=False)

        for state, objects in (('observing', ()), ('approaching', self.PERSON),
                               ('complete', self.PERSON)):
            if state == 'complete':
                self.link.telemetry = replace(self.link.telemetry,
                                              distance_l=500, distance_r=520)
            for _ in range(21):
                if self.session.state == state:
                    break
                delayed_tick(objects)
            self.assertEqual(self.session.state, state, self.session.text)
        self.assertFalse(self.session.enabled)
        self.assertFalse(self.link.commands[-1]['armed'])

    def test_next_search_point_keeps_original_coordinate_origin(self):
        self.session = MissionSession([(1000, 0), (1000, 1000)], dwell_s=1,
                                       confirm_frames=3, confirm_s=.2)
        self.begin()
        self.link.nav_telemetry = replace(self.link.nav_telemetry,
                                          x_mm=1000, y_mm=0, heading=3000)
        self.link.arrived()
        self.until('observing')
        self.until('navigating')
        self.assertEqual(sum(op == NAV_POSE_RESET for op, _ in self.link.configs), 1)
        writes = [fields for op, fields in self.link.configs if op == NAV_WRITE]
        self.assertEqual([(f['x_mm'], f['y_mm']) for f in writes],
                         [(1000, 0), (1000, 1000)])
        self.assertEqual((self.link.nav_telemetry.x_mm, self.link.nav_telemetry.y_mm), (1000, 0))
        self.link.arrived()
        self.until('failed')
        self.assert_latched_failure()

    def test_repeated_frame_cannot_confirm_person(self):
        self.observe()
        captured = self.now + .05
        for _ in range(4):
            self.tick(self.PERSON, dt=.05, captured=captured)
        self.assertNotIn(self.session.state, ('approaching', 'complete'))
        self.assertFalse(any(c['armed'] and not c['nav'] for c in self.link.commands))

    def test_persistent_stale_or_missing_camera_stops_instead_of_confirming(self):
        for missing in (False, True):
            with self.subTest(missing=missing):
                self.setUp()
                self.observe()
                captured=self.now-1
                for _ in range(23):
                    self.tick(self.PERSON,captured=captured,camera_live=not missing)
                    if self.session.state=='failed':break
                self.assert_latched_failure()

    def test_navigation_survives_short_vision_gap_then_uses_fresh_frames(self):
        self.begin()
        for _ in range(5):self.tick()
        captured=self.now
        for _ in range(8):
            self.tick(self.PERSON,captured=captured,camera_live=False)
            self.assertEqual(self.session.state,'navigating')
            self.assertTrue(self.session.enabled)
        self.assertTrue(self.session.report['vision_waiting'])
        self.assertTrue(self.link.commands[-1]['armed'])
        self.assertTrue(self.link.commands[-1]['nav'])
        self.tick()
        self.assertFalse(self.session.report['vision_waiting'])
        self.assertEqual(self.session.state,'navigating')

    def test_navigation_persistent_vision_gap_latches_and_does_not_auto_resume(self):
        self.begin()
        for _ in range(5):self.tick()
        captured=self.now
        for _ in range(23):
            self.tick(captured=captured,camera_live=False)
            if self.session.state=='failed':break
        self.assertIn('持续超时',self.session.text)
        self.assertIn('vision',self.session.report['failure_snapshot'])
        self.assert_latched_failure()

    def test_late_recovery_after_vision_deadline_does_not_bypass_failure(self):
        self.begin()
        for _ in range(5):self.tick()
        self.tick(camera_live=False)
        self.tick(dt=2.05)
        self.assertIn('持续超时',self.session.text)
        self.assert_latched_failure()

    def test_remote_stop_during_vision_gap_wins_without_later_arm(self):
        self.begin()
        for _ in range(5):self.tick()
        self.tick(camera_live=False)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
        before=self.arm_count()
        self.tick(camera_live=False)
        self.assertEqual(self.session.state,'failed')
        self.assertIn('停车事件',self.session.text)
        self.tick()
        self.assertEqual(self.arm_count(),before)

    def test_observation_gap_cannot_confirm_stale_target_and_preserves_dwell(self):
        self.observe()
        self.tick(self.PERSON);self.tick(self.PERSON)
        self.assertEqual(self.session.report['confirmed_frames'],2)
        for _ in range(8):self.tick(self.PERSON,camera_live=False)
        self.assertEqual(self.session.state,'observing')
        self.assertEqual(self.session.report['confirmed_frames'],0)
        self.tick(self.PERSON)
        self.assertEqual(self.session.state,'observing')
        self.assertEqual(self.session.report['confirmed_frames'],1)
        self.until('approach_wait',self.PERSON)

    def test_visual_approach_still_stops_on_first_missing_frame(self):
        self.approach()
        self.tick(self.PERSON,camera_live=False)
        self.assertIn('接近目标时',self.session.text)
        self.assert_latched_failure()

    def test_fresh_but_regressed_detection_timestamp_still_fails(self):
        self.begin()
        for _ in range(5):self.tick()
        self.tick(captured=self.now-.05)
        self.assertIn('时间戳倒退',self.session.text)
        self.assert_latched_failure()

    def test_vision_gap_before_initial_arm_waits_for_fresh_frame(self):
        self.until('ready')
        self.assertTrue(self.session.start(self.link,self.now))
        for _ in range(8):self.tick(camera_live=False)
        self.assertEqual(self.arm_count(),0)
        self.assertEqual(self.session.state,'navigating')
        for _ in range(8):
            self.tick()
            if self.arm_count():break
        self.assertTrue(self.arm_count())

    def test_next_waypoint_prepares_but_cannot_start_during_vision_gap(self):
        self.session=MissionSession([(1000,0),(1000,1000)],dwell_s=1,
                                    confirm_frames=3,confirm_s=.2)
        self.observe();self.until('next_preparing')
        before=self.arm_count()
        for _ in range(15):self.tick(camera_live=False)
        self.assertTrue(self.session.navigator.prepared)
        self.assertEqual(self.session.state,'next_preparing')
        self.assertEqual(self.arm_count(),before)
        self.tick()
        self.assertEqual(self.session.state,'navigating')

    def test_motor_fault_during_vision_grace_still_stops_immediately(self):
        self.begin()
        for _ in range(5):self.tick()
        self.tick(camera_live=False)
        self.link.telemetry=replace(self.link.telemetry,fault=32)
        self.tick(camera_live=False)
        self.assertIn('0x0020',self.session.text)
        self.assert_latched_failure()

    def test_multiple_people_during_search_fail_without_selecting_arbitrarily(self):
        self.observe()
        self.tick(self.PERSON + self.OTHER)
        self.assert_latched_failure()
        self.assertFalse(any(c['armed'] and not c['nav'] for c in self.link.commands))

    def test_bottle_only_does_not_complete_or_start_approach(self):
        self.observe()
        self.until('failed', [(39, .99, (230, 80, 410, 460))])
        self.assertFalse(any(c['armed'] and not c['nav'] for c in self.link.commands))

    def test_lost_or_ambiguous_person_during_approach_latches_failure(self):
        for objects in ([], self.PERSON + self.OTHER):
            with self.subTest(objects=objects):
                self.setUp()
                self.approach()
                self.tick(objects)
                self.assert_latched_failure()

    def test_approach_invalid_range_sensor_fault_control_and_comms_failures(self):
        for failure in ('close', 'sensor', 'fault', 'detour', 'remote5', 'offline'):
            with self.subTest(failure=failure):
                self.setUp()
                self.approach()
                changes = {'close': dict(distance_l=200),
                           'invalid_range': dict(distance_r=65535),
                           'sensor': dict(health=0x7d),
                           'fault': dict(fault=32),
                           'detour': dict(state=2),
                           'remote5': dict(health=0x3f)}
                if failure == 'offline':
                    self.link.connected = False
                else:
                    self.link.telemetry = replace(self.link.telemetry, **changes[failure])
                self.tick(self.PERSON)
                self.assert_latched_failure()

    def test_missing_sonar_permits_start_navigation_and_mode2_approach(self):
        self.link.telemetry=replace(self.link.telemetry,health=0x33,distance_l=65535,distance_r=65535)
        self.approach()
        self.assertTrue(self.session.enabled)
        arms=[c for c in self.link.commands if c['armed'] and not c['nav']]
        self.assertTrue(arms)
        self.assertTrue(all(c['follow'] for c in arms))
        self.assertIsNone(self.session.measured_distance_mm)

    def test_bottle_search_ignores_people_and_completes_using_target_height(self):
        self.session=MissionSession([(1000,0)],action='find_bottle',dwell_s=1,confirm_frames=3,confirm_s=.2)
        objects=self.BOTTLE+self.PERSON
        self.approach(objects)
        self.assertEqual(self.session.follower.category,39)
        self.assertTrue(all(c['follow'] for c in self.link.commands if c['armed'] and not c['nav']))
        self.link.telemetry=replace(self.link.telemetry,distance_l=500,distance_r=520)
        self.until('complete',objects)
        self.assertEqual(self.session.report['target_name'],'水瓶')
        self.assertEqual(self.session.report['target_size_metric'],'height')
        self.assertEqual(self.session.completion_basis,'range_and_vision')
        self.assertFalse(self.link.commands[-1]['armed'])

    def test_bottle_search_without_depth_has_explicit_visual_only_completion(self):
        self.session=MissionSession([(1000,0)],action='find_bottle',dwell_s=1,confirm_frames=3,confirm_s=.2)
        self.link.telemetry=replace(self.link.telemetry,health=0x33,distance_l=65535,distance_r=65535)
        self.approach(self.BOTTLE)
        larger=[(39,.94,(290,100,350,400))]
        self.until('complete',larger)
        self.assertEqual(self.session.completion_basis,'vision_only')
        self.assertIsNone(self.session.measured_distance_mm)

    def test_person_only_cannot_satisfy_bottle_search(self):
        self.session=MissionSession([(1000,0)],action='find_bottle',dwell_s=1,confirm_frames=3,confirm_s=.2)
        self.observe();self.until('failed',self.PERSON)
        self.assertFalse(any(c['armed'] and c['follow'] for c in self.link.commands))

    def test_bottle_task_needs_known_frame_height_and_stops_on_lost_bottle(self):
        self.session=MissionSession([(1000,0)],action='find_bottle',dwell_s=1,confirm_frames=3,confirm_s=.2)
        self.until('ready',self.BOTTLE,image_height=None)
        self.assertFalse(self.session.start(self.link,self.now))
        self.tick(self.BOTTLE)  # A fresh frame with dimensions restores readiness.
        self.approach(self.BOTTLE)
        self.tick(self.PERSON)
        self.assert_latched_failure()

    def test_single_valid_near_side_can_complete_with_visual_confirmation(self):
        self.approach()
        self.link.telemetry=replace(self.link.telemetry,health=self.link.telemetry.health&~8,
                                    distance_l=500,distance_r=65535)
        self.until('complete',self.PERSON)
        self.assertEqual(self.session.completion_basis,'range_and_vision')

    def test_invalid_stale_near_does_not_stop_approach(self):
        self.approach()
        self.link.telemetry=replace(self.link.telemetry,health=self.link.telemetry.health&~12,
                                    distance_l=150,distance_r=170)
        self.tick(self.PERSON)
        self.assertEqual(self.session.state,'approaching',self.session.text)
        self.assertTrue(self.link.commands[-1]['follow'])

    def test_no_measured_depth_completes_only_on_large_centered_person_and_stillness(self):
        for health,distance in ((0x33,65535),(0x3f,65534)):
            with self.subTest(health=health,distance=distance):
                self.setUp()
                self.link.telemetry=replace(self.link.telemetry,health=health,distance_l=distance,distance_r=distance)
                self.approach()
                # Small boxes remain approach targets, never fabricated range arrivals.
                for _ in range(3):self.tick(self.PERSON)
                self.assertEqual(self.session.state,'approaching')
                # Grow the same person's box gradually to preserve target association.
                for half in (110,130,150):
                    person=[(0,.93,(320-half,80,320+half,460))]
                    self.tick(person)
                self.until('complete',person)
                self.assertEqual(self.session.completion_basis,'vision_only')
                self.assertIsNone(self.session.report['measured_distance_mm'])
                self.assertFalse(self.link.commands[-1]['armed'])

    def test_missing_depth_at_completing_reenters_stopped_visual_handoff(self):
        self.approach()
        self.link.telemetry=replace(self.link.telemetry,distance_l=500,distance_r=520)
        self.until('completing',self.PERSON)
        self.link.telemetry=replace(self.link.telemetry,health=self.link.telemetry.health&~12,
                                    distance_l=65535,distance_r=65535)
        before=self.arm_count();self.tick(self.PERSON)
        self.assertEqual(self.session.state,'approach_wait')
        self.assertEqual(self.arm_count(),before)
        self.until('approaching',self.PERSON)

    def test_navigation_only_visits_named_points_without_camera_or_sonar(self):
        self.session=MissionSession([(1000,0),(1000,1000),(0,1000),(0,0)],action='navigate')
        self.link.telemetry=replace(self.link.telemetry,health=0x33,distance_l=65535,distance_r=65535)
        self.until('ready',camera_live=False)
        self.assertTrue(self.session.start(self.link,self.now))
        for index in range(4):
            for _ in range(40):
                self.tick(camera_live=False)
                if self.session.navigator.enabled:break
            self.assertTrue(self.session.navigator.enabled,self.session.text)
            self.link.arrived()
            self.until('next_preparing' if index<3 else 'complete',camera_live=False)
        self.assertEqual(self.session.completion_basis,'waypoint_and_stopped')
        self.assertEqual(sum(op==NAV_POSE_RESET for op,_ in self.link.configs),1)
        self.assertTrue(all(c['nav'] for c in self.link.commands if c['armed']))

    def test_default_named_task_and_restart_use_current_placement_without_manual_origin(self):
        from mission import load_mission
        config=load_mission(Path(__file__).resolve().parents[1]/'rk/mission_semantic_example.json')
        self.link.nav_telemetry=replace(self.link.nav_telemetry,
                                        flags=self.link.nav_telemetry.flags|32,
                                        x_mm=750,y_mm=-300,heading=9000)
        self.session=MissionSession(**config)
        self.until('ready',camera_live=False)
        self.assertEqual((self.link.nav_telemetry.x_mm,self.link.nav_telemetry.y_mm,
                          self.link.nav_telemetry.heading),(0,0,0))
        self.assertEqual(sum(op==NAV_POSE_RESET for op,_ in self.link.configs),1)
        self.assertFalse(any(c['armed'] for c in self.link.commands))
        self.link.nav_telemetry=replace(self.link.nav_telemetry,x_mm=-400,y_mm=900,heading=-4500)
        self.session.reset()
        self.until('ready',camera_live=False)
        self.assertEqual((self.link.nav_telemetry.x_mm,self.link.nav_telemetry.y_mm,
                          self.link.nav_telemetry.heading),(0,0,0))
        self.assertEqual(sum(op==NAV_POSE_RESET for op,_ in self.link.configs),2)
        self.assertFalse(any(c['armed'] for c in self.link.commands))

    def test_explicit_auto_policy_keeps_existing_coordinates(self):
        self.session=MissionSession([(1000,0)],action='navigate',initialize_pose='auto')
        self.until('ready')
        self.assertTrue(self.session.pose_reset_requested)
        self.assertEqual(sum(op==NAV_POSE_RESET for op,_ in self.link.configs),1)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,x_mm=1000,y_mm=200)
        self.session=MissionSession([(1000,1000)],action='navigate',initialize_pose='auto')
        self.until('ready')
        self.assertFalse(self.session.pose_reset_requested)
        self.assertEqual(sum(op==NAV_POSE_RESET for op,_ in self.link.configs),1)
        self.assertEqual((self.link.nav_telemetry.x_mm,self.link.nav_telemetry.y_mm),(1000,200))
        self.session.reset(reset_origin=True)
        self.until('ready')
        self.assertEqual(sum(op==NAV_POSE_RESET for op,_ in self.link.configs),2)

    def test_origin_selection_waits_for_live_navigation_before_configuration(self):
        self.session=MissionSession([(1000,0)],initialize_pose='auto')
        self.link.nav_connected=False
        for _ in range(5):self.tick(self.PERSON)
        self.assertFalse(self.link.configs)
        self.assertFalse(any(c['armed'] for c in self.link.commands))
        self.link.nav_connected=True
        self.until('ready')

    def test_physical_stop_does_not_cancel_manual_ir_preparation_with_uart_stop(self):
        self.approach()
        self.link.telemetry=replace(self.link.telemetry,health=0x3f,command=0,speed=16)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1,
                                       flags=self.link.nav_telemetry.flags&~16)
        before=len(self.link.commands);self.tick(self.PERSON)
        self.assertEqual(self.session.state,'failed')
        for _ in range(5):self.tick(self.PERSON)
        self.assertEqual(len(self.link.commands),before)

    def test_navigation_failure_or_remote_stop_never_rearms(self):
        for failure in ('nav_failure', 'nav_stale', 'remote_stop'):
            with self.subTest(failure=failure):
                self.setUp()
                self.begin()
                # Expire the navigation ARM acknowledgement grace period first.
                for _ in range(5):
                    self.tick()
                if failure == 'nav_failure':
                    self.link.nav_telemetry = replace(self.link.nav_telemetry,
                                                     flags=self.link.nav_telemetry.flags | 4)
                elif failure == 'nav_stale':
                    self.link.nav_connected = False
                else:
                    self.link.telemetry = replace(self.link.telemetry, health=0x3f)
                    self.link.nav_telemetry = replace(self.link.nav_telemetry,
                                                     flags=self.link.nav_telemetry.flags & ~16)
                self.tick()
                self.assert_latched_failure()

    def test_legacy_navigation_firmware_cannot_run_mission(self):
        self.link.nav_telemetry = replace(self.link.nav_telemetry,
                                          flags=self.link.nav_telemetry.flags & ~64)
        for _ in range(40):
            self.tick(self.PERSON)
        self.assertFalse(self.session.start(self.link, self.now))
        self.assertEqual(self.arm_count(), 0)

    def test_remote_stop_epoch_cancels_even_while_disarmed_switching_modes(self):
        for epoch, next_epoch in ((0, 1), (15, 0)):
            for stage in ('observing', 'approach_wait', 'completing'):
                with self.subTest(epoch=epoch, next_epoch=next_epoch, stage=stage):
                    self.setUp()
                    self.link.nav_telemetry = replace(self.link.nav_telemetry, stop_epoch=epoch)
                    self.observe()
                    if stage == 'approach_wait':
                        self.until('approach_wait', self.PERSON)
                    elif stage == 'completing':
                        self.until('approaching', self.PERSON)
                        self.link.telemetry = replace(self.link.telemetry,
                                                      distance_l=500, distance_r=520)
                        self.until('completing', self.PERSON)
                    # DISARM already cleared owner flags; epoch is the only
                    # reliable evidence of an intervening physical remote STOP.
                    self.assertFalse(self.link.telemetry.health & 64)
                    self.link.nav_telemetry = replace(self.link.nav_telemetry,
                                                     stop_epoch=next_epoch)
                    self.tick(self.PERSON)
                    self.assert_latched_failure()

    def test_ultrasound_wall_without_large_centered_person_is_not_success(self):
        for objects in ([(0, .93, (280, 80, 360, 460))],
                        [(0, .93, (50, 80, 230, 460))]):
            with self.subTest(objects=objects):
                self.setUp()
                self.approach(objects)
                self.link.telemetry = replace(self.link.telemetry,
                                              distance_l=500, distance_r=520)
                self.until('failed', objects)
                self.assert_latched_failure()

    def test_completion_needs_zero_wheels_and_a_fresh_visible_target(self):
        self.approach()
        self.link.telemetry = replace(self.link.telemetry, distance_l=500,
                                      distance_r=520, velocity_l=2, velocity_r=2)
        for _ in range(5):
            self.tick(self.PERSON)
        self.assertNotEqual(self.session.state, 'complete')
        self.link.telemetry = replace(self.link.telemetry, velocity_l=0, velocity_r=0)
        self.tick([])
        self.assert_latched_failure()

    def test_old_stopped_telemetry_cannot_arm_person_approach(self):
        self.observe()
        self.until('approach_wait', self.PERSON)
        for _ in range(5):
            self.tick(self.PERSON, dt=.05, fresh=False)
        self.assertFalse(any(c['armed'] and not c['nav'] for c in self.link.commands))

    def test_explicit_stop_is_latched_and_reset_does_not_restart(self):
        self.approach()
        self.session.stop('test operator stop')
        self.tick(self.PERSON)
        self.assertEqual(self.session.state, 'stopped')
        arms = self.arm_count()
        for _ in range(5):
            self.tick(self.PERSON)
        self.assertEqual(self.arm_count(), arms)
        self.session.reset()
        self.until('ready')
        self.assertEqual(self.arm_count(), arms)

    def test_delayed_start_tick_sends_disarm_alone_before_any_navigation_arm(self):
        self.until('ready')
        self.assertTrue(self.session.start(self.link, self.now))
        commands_before = len(self.link.commands)
        self.tick(dt=.5)  # Already past NavigationSession's .25 s start delay.
        emitted = self.link.commands[commands_before:]
        self.assertTrue(emitted)
        self.assertTrue(all(not c['armed'] for c in emitted))
        self.assertEqual(self.arm_count(), 0)
        # A just-finished frame may have been generated before DISARM. Wait
        # a complete snapshot period plus the UART margin before authorizing.
        self.tick(dt=.05)
        self.assertEqual(self.arm_count(), 0)
        self.tick(dt=.1)
        self.assertTrue(any(c['armed'] and c['nav'] for c in self.link.commands))

    def test_navigation_start_waits_for_post_disarm_stop_epoch_snapshot(self):
        self.until('ready')
        self.assertTrue(self.session.start(self.link, self.now))
        self.tick(dt=.5)
        sent_at = self.session.navigator.last_send
        # IR5 has occurred, but its incremented epoch is still waiting for the
        # 10 Hz navigation snapshot. Legacy telemetry alone cannot unlock ARM.
        self.now = round(self.now+.05, 6)
        self.link.fresh(self.now)
        self.link.last_nav_rx = sent_at
        self.session.step(self.link, self.now, [], self.now, 640, True)
        self.assertEqual(self.arm_count(), 0)
        self.link.nav_telemetry = replace(self.link.nav_telemetry, stop_epoch=1)
        self.tick(dt=.05)
        self.assert_latched_failure()

    def test_approach_cannot_disarm_and_arm_from_pre_disarm_snapshots(self):
        for held_stream in ('last_rx', 'last_nav_rx'):
            with self.subTest(held_stream=held_stream):
                self.setUp()
                self.observe()
                self.until('approach_wait', self.PERSON)
                baseline_arms = self.arm_count()
                old_snapshot = getattr(self.link, held_stream)
                # Keep one stream on its pre-DISARM snapshot while the other
                # stream and detector remain fresh. At .2 s a DISARM heartbeat
                # may release the FPGA's IR owner, but must not be followed by
                # an ARM until a later navigation snapshot reports its epoch.
                for _ in range(6):
                    self.now = round(self.now+.05, 6)
                    self.link.fresh(self.now)
                    setattr(self.link, held_stream, old_snapshot)
                    before = len(self.link.commands)
                    self.session.step(self.link, self.now, self.PERSON, self.now, 640, True)
                    self.assertTrue(all(not c['armed'] for c in self.link.commands[before:]))
                self.assertEqual(self.session.state, 'approach_wait')
                self.assertEqual(self.arm_count(), baseline_arms)
                self.link.nav_telemetry = replace(self.link.nav_telemetry, stop_epoch=1)
                self.tick(self.PERSON, dt=.05)
                self.assert_latched_failure()

    def test_old_epoch_frame_received_just_after_disarm_cannot_arm_approach(self):
        self.observe()
        self.until('approach_wait', self.PERSON)
        baseline_arms = self.arm_count()
        # Stop time itself has elapsed, but both telemetry samples are from a
        # frame received only 50 ms after the last DISARM. Its epoch can still
        # describe a snapshot taken before the IR owner was released.
        self.session._still_since = self.now-.3
        self.session._first_still_seq = (self.link.telemetry.seq-1) & 255
        self.tick(self.PERSON, dt=.05)
        self.assertEqual(self.session.state, 'approach_wait')
        self.assertEqual(self.arm_count(), baseline_arms)
        self.link.nav_telemetry = replace(self.link.nav_telemetry, stop_epoch=1)
        self.tick(self.PERSON, dt=.1)
        self.assert_latched_failure()

    def test_two_hundred_millisecond_gui_ticks_do_not_starve_mode_switch(self):
        self.until('ready', dt=.2)
        self.assertTrue(self.session.start(self.link, self.now))
        for _ in range(20):
            self.tick(dt=.2)
            if any(c['armed'] and c['nav'] for c in self.link.commands):
                break
        self.assertTrue(any(c['armed'] and c['nav'] for c in self.link.commands))
        self.link.arrived()
        self.until('observing', dt=.2)
        self.until('approaching', self.PERSON, dt=.2)
        self.assertTrue(any(c['armed'] and not c['nav'] for c in self.link.commands))
        self.link.telemetry = replace(self.link.telemetry, distance_l=500, distance_r=520)
        self.until('complete', self.PERSON, dt=.2)
        self.assertFalse(self.session.enabled)
        self.assertFalse(self.link.commands[-1]['armed'])


    def remote_bottle_start(self,points=((1000,0),)):
        self.session=MissionSession(points,action='find_bottle',dwell_s=1.,confirm_frames=3,confirm_s=.2)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,nav_mode_count=1)
        self.assertTrue(self.session.request_run(self.link,self.now))
        self.assertEqual(self.arm_count(),0)
        for _ in range(80):
            self.tick()
            if any(c['armed'] and c['nav'] for c in self.link.commands):break
        self.assertTrue(self.session.enabled,self.session.text)
        self.assertTrue(any(c['armed'] and c['nav'] for c in self.link.commands))
        self.assertIsNone(self.session.run_request)

    def test_key7_search_starts_without_e_and_visits_next_point_if_only_people_seen(self):
        self.remote_bottle_start(((1000,0),(1000,1000)))
        self.link.arrived()
        self.until('observing')
        self.until('next_preparing',self.PERSON)
        self.assertEqual(self.session.point_index,1)
        self.until('navigating',self.PERSON)
        self.assertEqual(self.session.navigator.points,[(1000,1000)])
        self.assertFalse(self.session.navigator.initialize_pose)
        self.assertFalse(any(c['armed'] and c['follow'] for c in self.link.commands))

    def test_key7_search_confirms_bottle_approaches_and_completes(self):
        self.remote_bottle_start()
        self.link.arrived();self.until('observing')
        self.until('approaching',self.BOTTLE)
        self.tick(self.BOTTLE);self.tick(self.BOTTLE)
        self.assertTrue(any(c['armed'] and c['follow'] for c in self.link.commands))
        self.link.telemetry=replace(self.link.telemetry,distance_l=500,distance_r=520)
        self.until('complete',self.BOTTLE)
        self.assertEqual(self.session.completion_basis,'range_and_vision')
        self.assertFalse(self.link.commands[-1]['armed'])

    def test_key7_stop_during_preparation_cancels_without_undoing_manual_stop(self):
        self.session=MissionSession([(500,0)],action='find_bottle')
        self.assertTrue(self.session.request_run(self.link,0.))
        self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
        self.tick()
        self.assertEqual(self.session.state,'stopped')
        self.assertIsNone(self.session.run_request)
        self.assertEqual(self.link.commands,[])
        for _ in range(10):self.tick(self.BOTTLE)
        self.assertEqual(self.arm_count(),0)

    def test_key7_startup_waits_for_fresh_vision_then_runs_with_original_request(self):
        self.session=MissionSession([(500,0)],action='find_bottle')
        self.assertTrue(self.session.request_run(self.link,0.))
        deadline=self.session.run_request[0]
        # Camera preview can still be live at 0.45s, but control requires .4s.
        # Exercise the initial stale frame and absence after route readiness.
        self.tick(captured=-.35)
        for _ in range(25):self.tick(camera_live=False)
        self.assertEqual(self.session.state,'ready',self.session.text)
        self.assertIsNotNone(self.session.run_request)
        self.assertIn('等待',self.session.text)
        self.assertEqual(self.session.run_request[0],deadline)
        self.assertEqual(self.arm_count(),0)
        for _ in range(20):
            self.tick(self.BOTTLE)
            if self.arm_count():break
        self.assertTrue(self.session.enabled,self.session.text)
        self.assertTrue(self.session.navigator.enabled)
        self.assertGreater(self.arm_count(),0)
        self.assertIsNone(self.session.run_request)

    def test_key7_vision_wait_does_not_extend_deadline_or_arm_on_expiry_frame(self):
        self.session=MissionSession([(500,0)],action='find_bottle')
        self.assertTrue(self.session.request_run(self.link,0.))
        for _ in range(119):self.tick(camera_live=False)
        self.assertIsNotNone(self.session.run_request)
        self.assertTrue(self.session.request_run(self.link,self.now))
        self.assertEqual(self.session.run_request[0],12.)
        self.assertEqual(self.arm_count(),0)
        self.tick(self.BOTTLE)  # Fresh image exactly at deadline cannot revive it.
        self.assertEqual(self.session.state,'stopped')
        self.assertIsNone(self.session.run_request)
        for _ in range(20):self.tick(self.BOTTLE)
        self.assertEqual(self.arm_count(),0)

    def test_key7_vision_wait_stop_and_sensor_fault_beat_recovery(self):
        for problem in ('stop','fault','imu','nav_support'):
            with self.subTest(problem=problem):
                self.setUp()
                self.session=MissionSession([(500,0)],action='find_bottle')
                self.assertTrue(self.session.request_run(self.link,0.))
                for _ in range(25):self.tick(camera_live=False)
                sent=len(self.link.commands)
                if problem=='stop':self.link.nav_telemetry=replace(self.link.nav_telemetry,stop_epoch=1)
                elif problem=='fault':self.link.telemetry=replace(self.link.telemetry,fault=32)
                elif problem=='imu':self.link.telemetry=replace(self.link.telemetry,health=0x3d)
                else:self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=0)
                self.tick(self.BOTTLE)
                self.assertEqual(self.session.state,'stopped')
                self.assertIsNone(self.session.run_request)
                if problem=='stop':self.assertEqual(len(self.link.commands),sent)
                self.link.telemetry=replace(self.link.telemetry,health=0x3f,fault=0)
                self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=128|64|32|8)
                for _ in range(20):self.tick(self.BOTTLE)
                self.assertEqual(self.arm_count(),0)

    def test_key7_ready_vision_wait_still_rejects_failed_navigation(self):
        self.session=MissionSession([(500,0)],action='find_bottle')
        self.assertTrue(self.session.request_run(self.link,0.))
        for _ in range(25):self.tick(camera_live=False)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=self.link.nav_telemetry.flags|4)
        self.tick(self.BOTTLE)
        self.assertEqual(self.session.state,'stopped')
        self.assertIsNone(self.session.run_request)
        self.link.nav_telemetry=replace(self.link.nav_telemetry,flags=self.link.nav_telemetry.flags&~4)
        for _ in range(20):self.tick(self.BOTTLE)
        self.assertFalse(self.session.enabled)
        self.assertFalse(self.session.navigator.enabled)
        self.assertEqual(self.arm_count(),0)

    def test_key7_lost_link_and_generation_reset_cancel_preparation(self):
        for reset in (False,True):
            with self.subTest(reset=reset):
                self.setUp()
                self.session=MissionSession([(500,0)],action='find_bottle')
                self.link.nav_telemetry=replace(self.link.nav_telemetry,nav_mode_count=2)
                self.assertTrue(self.session.request_run(self.link,0.))
                self.tick()
                if reset:self.link.nav_telemetry=replace(self.link.nav_telemetry,nav_mode_count=0)
                else:self.link.connected=self.link.nav_connected=False
                self.tick()
                self.assertEqual(self.session.state,'stopped')
                self.link.connected=self.link.nav_connected=True
                for _ in range(15):self.tick(self.BOTTLE)
                self.assertEqual(self.arm_count(),0)

    def test_key7_preparation_timeout_and_dialog_cancel_do_not_arm(self):
        for allowed,now in ((True,12.),(False,.1)):
            with self.subTest(allowed=allowed):
                self.setUp()
                self.session=MissionSession([(500,0)],action='find_bottle')
                self.assertTrue(self.session.request_run(self.link,0.))
                self.link.fresh(now)
                self.session.step(self.link,now,[],now,640,True,height=480,remote_allowed=allowed)
                self.assertEqual(self.session.state,'stopped')
                self.assertIsNone(self.session.run_request)
                self.assertEqual(self.arm_count(),0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
