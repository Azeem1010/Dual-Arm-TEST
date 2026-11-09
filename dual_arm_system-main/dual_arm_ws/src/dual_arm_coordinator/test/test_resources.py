import unittest
from dual_arm_coordinator.resources import ZoneManager, SlotManager


class ResourceTests(unittest.TestCase):
    def test_b_zone_stays_locked_after_fault(self):
        zone = ZoneManager()
        token = zone.acquire('arm1', observation_clear=True)
        zone.fault(token)
        with self.assertRaises(RuntimeError):
            zone.acquire('arm2', observation_clear=True)
        with self.assertRaises(RuntimeError):
            zone.release(token, withdrawn=False, observation_clear=True)
        self.assertEqual(zone.owner, 'arm1')

    def test_only_owner_with_withdrawal_can_release(self):
        zone = ZoneManager()
        token = zone.acquire('arm1', observation_clear=True)
        with self.assertRaises(RuntimeError):
            zone.release('wrong', withdrawn=True, observation_clear=True)
        zone.release(token, withdrawn=True, observation_clear=True)
        self.assertIsNone(zone.owner)

    def test_slots_commit_only_verified_sequential_result(self):
        slots = SlotManager()
        self.assertEqual(slots.reserve([False]*8), 1)
        with self.assertRaises(RuntimeError):
            slots.commit(1, placed=False, order_ok=True)
        slots.commit(1, placed=True, order_ok=True)
        self.assertEqual(slots.reserve([True]+[False]*7), 2)

    def test_order_error_or_stale_observation_cannot_reserve(self):
        slots = SlotManager()
        with self.assertRaises(RuntimeError):
            slots.reserve([False, True]+[False]*6)
        slots.reserve([False]*8)
        slots.commit(1, placed=True, order_ok=True)
        with self.assertRaises(RuntimeError):
            slots.reserve([False]*8)


if __name__ == '__main__':
    unittest.main()
