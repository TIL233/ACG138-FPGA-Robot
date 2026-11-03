"""Independent geometry oracle: catches ray/angle sign, missing start and underbounds."""
import json
import math
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class TestLatticeRom(unittest.TestCase):
    def test_generated_bounds(self):
        path = ROOT / 'fpga/rtl/lidar_lattice_rom.json'
        self.assertTrue(path.exists(), 'conservative ROM metadata not implemented')
        data = json.loads(path.read_text())
        tables = data['thresholds_mm']
        self.assertEqual(len(tables), 9)
        # Independent continuous-centerline and dense ray oracle, no generator imports.
        for cid, row in enumerate(tables):
            speed = (2, 6, 12)[cid // 3]
            v = math.ceil(speed * 50000 / 7640)
            radius = 160 + math.ceil(v * .670) + math.ceil(v*v/400)
            for sector, bound in enumerate(row):
                largest = 0.0
                for t in range(321):
                    arc = 300*t/320
                    if cid % 3 == 1:
                        x,y=arc,0
                    else:
                        x=150*math.sin(arc/150)
                        y=(1 if cid%3==0 else -1)*150*(1-math.cos(arc/150))
                    # Continuous sweep excludes segment margin already in table circles.
                    physical_radius=radius-20
                    for a in range(21):
                        angle=-math.radians(sector*10+a*.5)
                        dot=x*math.cos(angle)+y*math.sin(angle)
                        disc=physical_radius**2-x*x-y*y+dot*dot
                        if disc>=0:
                            hit=dot+math.sqrt(disc)
                            if hit>=0:largest=max(largest,hit)
                self.assertGreaterEqual(bound+1e-7, largest, (cid,sector,bound,largest))
                # Also check every expanded construction disc, independently
                # sampling its radial intersections over the closed sector.
                for t in range(9):
                    arc=300*t/8
                    if cid%3==1:x,y=arc,0
                    else:x,y=150*math.sin(arc/150),(1 if cid%3==0 else -1)*150*(1-math.cos(arc/150))
                    for a in range(101):
                        angle=-math.radians(sector*10+a/10)
                        dot=x*math.cos(angle)+y*math.sin(angle)
                        disc=radius**2-x*x-y*y+dot*dot
                        if disc>=0:
                            self.assertGreaterEqual(bound+1e-7,dot+math.sqrt(disc)+2,(cid,sector,t,a))
            # Starting body must be checked behind the robot too.
            self.assertTrue(all(v >= radius+2 for v in row))
            if cid%3==1:
                self.assertEqual(row[0],300+radius+2)
                self.assertEqual(row[35],300+radius+2)
        for group in range(3):
            self.assertEqual(tables[group*3],list(reversed(tables[group*3+2])))

if __name__=='__main__':unittest.main()
