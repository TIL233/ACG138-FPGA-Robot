from pathlib import Path
import sys, struct, unittest
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from follow import RemoteFollowTrigger
from protocol import NavTelemetryParser,crc16

def nav(count,stop=0):return SimpleNamespace(follow_count=count,stop_epoch=stop)

class RemoteFollowTests(unittest.TestCase):
    def test_navigation_mode_counter_is_independent_with_stop_priority(self):
        trigger=RemoteFollowTrigger('nav_mode_count')
        snapshot=SimpleNamespace(nav_mode_count=5,follow_count=10,stop_epoch=0)
        self.assertIsNone(trigger.observe(snapshot,True))
        snapshot.follow_count+=1
        self.assertIsNone(trigger.observe(snapshot,True))
        snapshot.nav_mode_count+=1
        self.assertEqual(trigger.observe(snapshot,True),'start')
        snapshot.nav_mode_count+=1;snapshot.stop_epoch+=1
        self.assertEqual(trigger.observe(snapshot,True),'stop')
        self.assertTrue(trigger.remote_stop)
        self.assertIsNone(trigger.observe(snapshot,True))
        self.assertEqual(trigger.observe(snapshot,False),'stop')
        snapshot.nav_mode_count+=1
        self.assertIsNone(trigger.observe(snapshot,True))

    def test_baseline_repeat_stop_reset_and_reconnect(self):
        trigger=RemoteFollowTrigger()
        self.assertIsNone(trigger.observe(nav(10),True))
        self.assertEqual(trigger.observe(nav(11),True),'start')
        self.assertIsNone(trigger.observe(nav(11),True))
        self.assertEqual(trigger.observe(nav(12,1),True),'stop')
        self.assertTrue(trigger.remote_stop)
        self.assertIsNone(trigger.observe(nav(12,1),True))
        self.assertFalse(trigger.remote_stop)
        self.assertEqual(trigger.observe(nav(13,1),True),'start')
        self.assertEqual(trigger.observe(nav(0),True),'stop')
        self.assertEqual(trigger.observe(nav(1),True),'start')
        self.assertEqual(trigger.observe(nav(1),False),'stop')
        self.assertFalse(trigger.remote_stop)
        self.assertIsNone(trigger.observe(nav(8),True))
        self.assertEqual(trigger.observe(nav(9),True),'start')

    def test_missed_burst_and_saturation_cannot_replay_start(self):
        t=RemoteFollowTrigger();t.observe(nav(1),True)
        self.assertEqual(t.observe(nav(3),True),'stop')
        t=RemoteFollowTrigger();t.observe(nav(65535),True)
        self.assertIsNone(t.observe(nav(65535),True))
        self.assertEqual(t.observe(nav(0),True),'stop')
        self.assertEqual(t.observe(nav(None),True),'stop')

    def test_nav_versions_split_crc_and_generation_endian(self):
        def packet(version):
            b=struct.pack('<BBBBBBiihBB',version,7,0x32,1,2,0xc0,-100,200,-300,9,0)
            if version==2:b+=struct.pack('<H',0x1234)
            return b'\x5a\xc3'+b+struct.pack('<H',crc16(b))+b'\r\n'
        p=NavTelemetryParser();v2=packet(2)
        self.assertEqual(p.feed(v2[:23]),[])
        frames=p.feed(v2[23:]+packet(1))
        self.assertEqual((frames[0].follow_count,frames[0].stop_epoch,frames[0].state),(0x1234,3,2))
        self.assertIsNone(frames[1].follow_count)
        bad=bytearray(v2);bad[20]^=1
        self.assertEqual(len(p.feed(bad+v2)),1)
        self.assertGreater(p.bad_frames,0)

    def test_navigation_v3_key7_split_crc_and_legacy_compatibility(self):
        def packet(version):
            body=struct.pack('<BBBBBBiihBB',version,7,0x32,1,2,0xc0,-100,200,-300,9,0)
            if version>=2:body+=struct.pack('<H',0x1234)
            if version==3:body+=struct.pack('<H',0x5678)
            return b'\x5a\xc3'+body+struct.pack('<H',crc16(body))+b'\r\n'
        parser=NavTelemetryParser();v3=packet(3)
        self.assertEqual(len(v3),28)
        self.assertEqual(parser.feed(v3[:25]),[])
        frames=parser.feed(v3[25:]+packet(2)+packet(1))
        self.assertEqual((frames[0].follow_count,frames[0].nav_mode_count,frames[0].stop_epoch),(0x1234,0x5678,3))
        self.assertEqual(frames[1].follow_count,0x1234)
        self.assertIsNone(frames[1].nav_mode_count)
        self.assertIsNone(frames[2].follow_count)
        broken=bytearray(v3);broken[22]^=1
        self.assertEqual(parser.feed(broken),[])
        self.assertEqual(parser.feed(v3)[0].nav_mode_count,0x5678)

if __name__=='__main__':unittest.main()
