import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('validate_plane', Path(__file__).parents[1]/'calibration/validate_plane.py')
plane = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plane)


def measurements():
    return dict(fit_pixels=[[0,0],[100,0],[100,100],[0,100]],
                fit_world_m=[[0,0],[.1,0],[.1,.1],[0,.1]],
                check_pixels=[[20,20],[80,20],[80,80],[20,80]],
                check_world_m=[[.02,.02],[.08,.02],[.08,.08],[.02,.08]],
                hole_width_m=.028, cap_diameter_m=.020, cup_diameter_m=.018,
                mechanical_error_m=.0002, margin_m=.0003)


class PlaneTests(unittest.TestCase):
    def test_independent_synthetic_geometry_only(self):
        report = plane.evaluate(measurements())
        self.assertEqual(report['status'],'PASS')
        self.assertLess(report['workspace_error_m'],1e-8)

    def test_realistic_displacement_exceeds_budget(self):
        data = measurements()
        data['check_world_m'][0][0] += .002
        self.assertEqual(plane.evaluate(data)['status'],'FAIL')

    def test_reused_or_collinear_points_rejected(self):
        data = measurements()
        data['check_pixels'] = data['fit_pixels']
        with self.assertRaises(ValueError):
            plane.evaluate(data)
        data['check_pixels'] = [[0,0],[1,1],[2,2],[3,3]]
        with self.assertRaises(ValueError):
            plane.evaluate(data)


if __name__ == '__main__':
    unittest.main()
