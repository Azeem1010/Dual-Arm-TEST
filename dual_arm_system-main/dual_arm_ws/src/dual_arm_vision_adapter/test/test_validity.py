import copy
import unittest
from dual_arm_vision_adapter.validity import validate, position_budget


def record():
    # Unit-test fixture only; never saved as a usable calibration.
    return dict(mode='sim', camera_id='arm1', frame_id='arm1_camera_optical_frame',
                reference_frame='arm1_base_link', image_size=[640, 480],
                status='PASS', calibrated_at=100.0, expires_at=200.0,
                mount_revision='mount-v1', pose_signature='home-a-v1',
                evidence_verified=True, evidence_kind='gazebo',
                camera_matrix=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
                distortion=[0., 0., 0., 0., 0.], homography=[.001, 0., 0., 0., .001, 0., 0., 0., 1.],
                workspace_error_m=.0002, allowed_position_error_m=.0005,
                reprojection_error_px=.2, allowed_reprojection_error_px=.5,
                budget_verified=True, slot_order_verified=True)


class ValidityTests(unittest.TestCase):
    def check(self, data, **overrides):
        args = dict(mode='sim', camera_id='arm1', frame_id='arm1_camera_optical_frame',
                    reference_frame='arm1_base_link', image_size=(640, 480), now=150.,
                    image_stamp=149.9, max_age_s=.5, mount_revision='mount-v1', pose_signature='home-a-v1')
        args.update(overrides)
        return validate(data, **args)

    def test_valid_fixture(self):
        self.assertEqual(self.check(record()), [])

    def test_missing_record_fails_closed(self):
        self.assertTrue(self.check({}))

    def test_simulation_never_authorizes_hardware(self):
        self.assertTrue(self.check(record(), mode='hardware'))

    def test_stale_future_expired_and_changed_pose(self):
        for args in [dict(image_stamp=149.), dict(image_stamp=151.), dict(now=201.),
                     dict(pose_signature='other'), dict(mount_revision='new'), dict(max_age_s=0.)]:
            with self.subTest(args=args):
                self.assertTrue(self.check(record(), **args))

    def test_unverified_errors_and_nan_rejected(self):
        for key, value in [('status', 'NOT_RUN'), ('evidence_verified', False),
                           ('budget_verified', False), ('workspace_error_m', float('nan')),
                           ('allowed_position_error_m', 0.), ('workspace_error_m', .001),
                           ('homography', [0.]*9), ('camera_matrix', [0.]*9),
                           ('evidence_kind', 'synthetic')]:
            data = record()
            data[key] = value
            with self.subTest(key=key):
                self.assertTrue(self.check(data))

    def test_arm2_requires_slot_number_verification(self):
        data = record()
        data['camera_id'] = 'arm2'
        data['slot_order_verified'] = False
        self.assertTrue(self.check(data, camera_id='arm2'))

    def test_budget_uses_clearance_and_uncertainty(self):
        self.assertAlmostEqual(position_budget(.028, .020, .018, .0002, .0003), .0005)
        with self.assertRaises(ValueError):
            position_budget(.019, .020, .018, 0., 0.)


if __name__ == '__main__':
    unittest.main()
