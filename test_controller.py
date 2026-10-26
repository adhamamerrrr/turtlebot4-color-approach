from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from controller import Target, decide, detect_red, read_ppm


class ControllerTests(unittest.TestCase):
    def test_largest_component(self):
        image = [[(0.,0.,0.) for _ in range(10)] for _ in range(10)]
        for y in (4,5):
            for x in (4,5):
                image[y][x] = (1.,0.,0.)
        image[0][0] = (1.,0.,0.)
        self.assertEqual(detect_red(image), Target(0.5, 0.04))

    def test_missing_target(self):
        self.assertIsNone(detect_red([[(0.,1.,0.)]*4]*4))
        self.assertEqual(decide(None,0,1).reason, 'no_target')

    def test_alignment(self):
        left, right = decide(Target(.2,.03),0,1), decide(Target(.8,.03),0,1)
        self.assertGreater(left.angular,0)
        self.assertLess(right.angular,0)
        self.assertEqual(left.linear,0)

    def test_approach(self):
        command=decide(Target(.5,.04),0,1)
        self.assertGreater(command.linear,0)
        self.assertLessEqual(command.linear,.12)
        self.assertEqual(command.angular,0)

    def test_stop_conditions(self):
        t=Target(.5,.04)
        self.assertEqual(decide(t,.6,1).reason,'stale_camera')
        self.assertEqual(decide(t,0,.45).reason,'obstacle')
        self.assertEqual(decide(Target(.5,.2),0,1).reason,'target_large_enough')
        for bad in (float('nan'),float('inf'),-1):
            self.assertEqual(decide(t,0,bad).linear,0)
        self.assertEqual(decide(Target(2,.1),0,1).reason,'invalid_target')

    def test_ppm_and_cli(self):
        p=Path(__file__).with_name('sample.ppm')
        self.assertEqual(len(read_ppm(p)),10)
        run=subprocess.run([sys.executable,str(p.with_name('controller.py')),str(p),
                            '--frame-age','0','--clearance','1'],capture_output=True,text=True,check=True)
        self.assertIn('approach',run.stdout)

    def test_invalid_image(self):
        with self.assertRaises(ValueError):
            detect_red([[(float('nan'),0,0)]])
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.ppm';p.write_text('P3 2 2 255 0')
            with self.assertRaises(ValueError):
                read_ppm(p)


if __name__ == '__main__':
    unittest.main()
