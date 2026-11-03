from pathlib import Path
import struct
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'rk'),str(ROOT/'tools')]
from motion_model import fit_model
from protocol import PredictiveTelemetryParser,TelemetryParser,NavTelemetryParser,MotionTelemetryParser,crc16


def samples(n=1200):
    rows=[];vl=0.;vr=0.;yaw=17900.
    left=(4,20,8,-12,-4,16)
    right=(16,4,-8,-20,12,4)
    for i in range(n):
        ul=left[(i//30)%len(left)];ur=right[(i//30)%len(right)]
        vl+=(ul-vl)/4;vr+=(ur-vr)/8
        yaw=(yaw+(round(vr)-round(vl))*4+18000)%36000-18000
        rows.append(dict(timestamp_ms=(0xfffff000+i*20)&0xffffffff,flags=11,
                         velocity_l=round(vl),velocity_r=round(vr),target_l=ul,target_r=ur,imu_heading=round(yaw)))
    return rows


def packet(seq=7):
    body=struct.pack('<BBHIHHBBhhhhIhHBB',1,seq,0x73,1000,42,181,6,8,-12,12,-8,8,
                     0x12345678,-90,1024,0x22,2)
    return b'\x5a\xf6'+body+struct.pack('<H',crc16(body))+b'\r\n'


class PredictiveModelTests(unittest.TestCase):
    def test_identifies_different_wheel_lag_and_turn_gain_across_heading_and_clock_wrap(self):
        result=fit_model(samples())
        self.assertEqual(result['wheels']['l']['alpha_shift'],2)
        self.assertEqual(result['wheels']['r']['alpha_shift'],3)
        self.assertAlmostEqual(result['yaw_gain_q8'],1024,delta=4)
        self.assertTrue(result['validation_required'])
        self.assertEqual(result['parameters']['PRED_MODEL_VALIDATED'],0)
        self.assertLess(result['wheels']['l']['validation_rmse_counts'],1)

    def test_does_not_fit_idle_braked_stale_or_old_10hz_data(self):
        rows=samples()
        variants=[[],[dict(r,flags=523) for r in rows],[dict(r,flags=3) for r in rows],
                  [dict(r,timestamp_ms=i*100) for i,r in enumerate(rows)],
                  [dict(r,velocity_l=8,velocity_r=8,target_l=8,target_r=8) for r in rows]]
        for variant in variants:
            with self.assertRaises(ValueError):fit_model(variant)

    def test_wrong_yaw_sign_is_rejected_and_straight_data_does_not_invent_gain(self):
        with self.assertRaises(ValueError):fit_model(samples(),yaw_sign=-1)
        straight=[dict(r,imu_heading=0) for r in samples()]
        result=fit_model(straight)
        self.assertEqual(result['turn_windows'],0)
        self.assertEqual(result['yaw_gain_q8'],0)

    def test_prediction_frames_fragmentation_crc_signed_fields_and_old_parser_coexistence(self):
        wire=packet();self.assertEqual(len(wire),38)
        for split in range(len(wire)+1):
            p=PredictiveTelemetryParser();frames=p.feed(wire[:split])+p.feed(wire[split:])
            self.assertEqual(len(frames),1);t=frames[0]
            self.assertEqual((t.timestamp_ms,t.generation,t.solve_cycles,t.candidate,t.horizon),(1000,42,181,6,8))
            self.assertEqual((t.reference_l,t.reference_r,t.output_l,t.output_r),(-12,12,-8,8))
            self.assertEqual((t.cost,t.predicted_error,t.yaw_gain_q8,t.alpha_shifts),(0x12345678,-90,1024,0x22))
            self.assertFalse(t.validated or t.fault);self.assertTrue(t.angular_model)
        broken=bytearray(wire);broken[12]^=1;p=PredictiveTelemetryParser()
        self.assertEqual([t.seq for t in p.feed(broken+packet(8))],[8])
        self.assertGreater(p.bad_frames,0)
        for decoder in (TelemetryParser(),NavTelemetryParser(),MotionTelemetryParser()):
            self.assertEqual(decoder.feed(packet()),[])
        p.feed(bytes(100000));self.assertLess(len(p.buffer),2)


if __name__=='__main__':unittest.main()
