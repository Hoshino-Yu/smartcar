import unittest

from control.line_follower import LineFollower


class LineFollowerTests(unittest.TestCase):
    def test_center_keeps_equal_wheel_output(self):
        self.assertEqual(LineFollower().step((0, 0, 1, 0, 0)), (0.35, 0.35))

    def test_turns_toward_line_on_either_side(self):
        left, right = LineFollower().step((0, 1, 0, 0, 0))
        self.assertLess(left, right)
        left, right = LineFollower().step((0, 0, 0, 1, 0))
        self.assertGreater(left, right)

    def test_strong_correction_stays_in_forward_range(self):
        left, right = LineFollower(kp=2.0).step((0, 0, 0, 0, 1))
        self.assertEqual((left, right), (1.0, 0.0))

    def test_line_loss_stops_and_resets_derivative(self):
        follower = LineFollower()
        follower.step((0, 0, 0, 0, 1))
        self.assertEqual(follower.step((0, 0, 0, 0, 0)), (0.0, 0.0))
        self.assertEqual(follower.step((0, 0, 1, 0, 0)), (0.35, 0.35))

    def test_derivative_responds_to_change(self):
        follower = LineFollower(kp=0, kd=0.01)
        self.assertEqual(follower.step((0, 0, 1, 0, 0)), (0.35, 0.35))
        left, right = follower.step((0, 0, 0, 1, 0))
        self.assertGreater(left, right)

    def test_rejects_invalid_input(self):
        follower = LineFollower()
        for readings in ((1, 0), (0, 0, -1, 0, 0), (0, 0, float("nan"), 0, 0)):
            with self.subTest(readings=readings), self.assertRaises(ValueError):
                follower.step(readings)

    def test_rejects_invalid_configuration(self):
        for config in ({"period_s": 0}, {"base_speed": 1.1}, {"kp": -1}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                LineFollower(**config)
