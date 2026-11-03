"""Following safety and identity tests, without camera/motor hardware."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from follow import TargetFollower


class FollowTests(unittest.TestCase):
    def test_target_profiles_keep_remote0_person_default_and_validate_integer_speed(self):
        self.assertEqual(TargetFollower().category,0)
        self.assertEqual(TargetFollower(category=39).category,39)
        for fields in (dict(speed=8.5),dict(category=1),dict(category=True)):
            with self.subTest(fields=fields),self.assertRaises(ValueError):TargetFollower(**fields)

    def test_speed_preset_does_not_start_or_release_distance_hold(self):
        f=TargetFollower()
        for speed in (4,6,8,10,12):
            f.set_speed(speed);self.assertEqual(f.speed,speed);self.assertFalse(f.enabled)
        for speed in (True,3,13,8.5,None):
            with self.subTest(speed=speed),self.assertRaises(ValueError):f.set_speed(speed)
        self.select();self.t.distance_l=500
        self.assertEqual(self.update().command,0)
        self.f.set_speed(12)
        self.assertEqual(self.update().command,0)
        self.t.distance_l=1000
        self.assertEqual(self.update().speed,12)

    def setUp(self):
        self.f=TargetFollower()
        self.objects=[(0,.9,(250,100,390,450)),(39,.9,(20,100,80,220))]
        self.t=SimpleNamespace(fault=0,sensors_ready=True,health=0x1f,state=0,distance_l=1000,distance_r=1000)

    def select(self):
        self.assertTrue(self.f.select(self.objects,300,200,10,10))
        self.assertTrue(self.f.start(10))

    def test_checked_far_allows_follow_but_near_or_lost_sensor_stops(self):
        self.select();self.t.distance_l=65534;self.t.distance_r=65534
        cmd=self.update()
        self.assertTrue(cmd.armed);self.assertEqual(cmd.command,1)
        self.t.distance_r=150
        cmd=self.update();self.assertEqual(cmd.command,0)
        self.t.health=0x13;self.t.distance_l=self.t.distance_r=65535
        cmd=self.update();self.assertTrue(cmd.armed);self.assertEqual(cmd.command,1)
        self.t.sensors_ready=False
        cmd=self.update();self.assertFalse(cmd.armed)

    def test_only_valid_near_sonar_holds_and_missing_sonar_releases_hold(self):
        self.select();self.t.health=0x17;self.t.distance_l=300;self.t.distance_r=65535
        self.assertEqual(self.update().command,0)
        self.t.health=0x13  # A stale 300mm value cannot retain distance holding.
        self.assertEqual(self.update().command,1)
        self.t.health=0x1b;self.t.distance_l=65535;self.t.distance_r=600
        self.assertEqual(self.update().command,0)
        self.t.distance_r=601
        self.assertEqual(self.update().command,1)

    def update(self,objects=None,stamp=10,now=10):
        return self.f.update(self.objects if objects is None else objects,stamp,now,640,self.t,True)

    def test_explicit_selection_and_start_required(self):
        self.assertFalse(self.update().armed)
        self.assertFalse(self.f.select(self.objects,40,150,10,10))
        self.assertTrue(self.f.select(self.objects,300,200,10,10))
        self.assertFalse(self.update().armed)
        self.assertTrue(self.f.start(10))
        self.assertEqual(self.update().command,1)

    def test_sixty_cm_threshold_on_both_sides_and_no_reverse(self):
        self.select()
        self.t.distance_l=600
        self.assertEqual(self.update().command,0)
        self.t.distance_l=601
        self.assertEqual(self.update().command,1)
        self.t.distance_r=600
        self.assertEqual(self.update().command,0)
        self.t.distance_r=601
        self.assertEqual(self.update().command,1)
        self.t.distance_l=599
        self.assertEqual(self.update().command,0)
        self.t.distance_l=650
        self.assertEqual(self.update().command,1)
        self.t.distance_r=100
        self.assertEqual(self.update().command,0)

    def test_left_right_forward_arcs(self):
        for box,command in [((40,100,180,450),3),((460,100,600,450),4)]:
            self.f=TargetFollower()
            objects=[(0,.9,box)]
            self.f.select(objects,(box[0]+box[2])/2,200,10,10);self.f.start(10)
            self.assertEqual(self.update(objects).command,command)

    def test_lost_target_does_not_switch_to_other_person_or_resume(self):
        self.select()
        other=[(0,.99,(450,100,600,450))]
        self.assertFalse(self.update(other,10.1,10.1).armed)
        self.assertIsNone(self.f.box)
        self.assertFalse(self.update(self.objects,10.2,10.2).armed)

    def test_ambiguous_crossing_stops(self):
        self.select()
        objects=[(0,.9,(251,100,391,450)),(0,.95,(249,100,389,450))]
        self.assertFalse(self.update(objects,10.1,10.1).armed)

    def test_stale_future_nan_image_stops(self):
        for stamp in (9,11,float('nan')):
            self.setUp();self.select()
            self.assertFalse(self.update(stamp=stamp).armed)

    def test_sensor_fault_and_detour_cannot_auto_resume(self):
        for attribute,value in [('fault',32),('sensors_ready',False),('state',2)]:
            self.setUp();self.select();setattr(self.t,attribute,value)
            self.assertFalse(self.update().armed)
            self.t=SimpleNamespace(fault=0,sensors_ready=True,health=0x1f,state=0,distance_l=1000,distance_r=1000)
            self.assertFalse(self.update().armed)

    def test_telemetry_loss_stops(self):
        self.select()
        self.assertFalse(self.f.update(self.objects,10,10,640,self.t,False).armed)

    def test_bottle_cannot_be_selected_or_followed(self):
        self.assertFalse(self.f.select(self.objects,40,150,10,10))
        self.assertFalse(self.f.start(10))
        self.assertFalse(self.update().armed)

    def test_close_target_image_holds(self):
        objects=[(0,.9,(50,20,600,470))]
        self.f.select(objects,300,200,10,10);self.f.start(10)
        self.assertEqual(self.update(objects).command,0)

    def test_automatic_selection_uses_person_confidence_not_size_or_bottle(self):
        objects=[(39,.999,(0,0,600,450)),(0,.7,(0,0,600,450)),
                 (0,.94,(250,100,390,450))]
        self.assertTrue(self.f.select_best(objects,10,10))
        self.assertEqual(self.f.box,(250,100,390,450))
        self.assertFalse(self.f.enabled)
        self.assertFalse(self.update(objects).armed)

    def test_automatic_selection_rejects_stale_and_invalid_candidates(self):
        for stamp,objects in [(9,self.objects),(11,self.objects),
                              (float('nan'),self.objects),
                              (10,[(39,.99,(0,0,20,20)),(0,.4,(0,0,20,20)),
                                   (0,.99,(1,1,1,5)),(0,float('nan'),(0,0,20,20))])]:
            self.assertFalse(self.f.select_best(objects,stamp,10))
            self.assertIsNone(self.f.box)

    def test_active_target_does_not_switch_to_higher_confidence_person(self):
        self.f.select_best(self.objects,10,10)
        self.f.start(10)
        other=[(0,.99,(450,100,600,450))]
        self.assertFalse(self.f.select_best(other,10.1,10.1))
        self.assertEqual(self.f.box,(250,100,390,450))
        self.assertTrue(self.update(self.objects+other,10.1,10.1).armed)
        self.assertEqual(self.f.box,(250,100,390,450))

    def test_auto_reselection_after_stop_cannot_restart_motion(self):
        self.f.select_best(self.objects,10,10);self.f.start(10)
        self.f.stop('remote STOP')
        self.assertTrue(self.f.select_best(self.objects,10.1,10.1))
        self.assertFalse(self.update(self.objects,10.1,10.1).armed)


if __name__=='__main__':unittest.main()
