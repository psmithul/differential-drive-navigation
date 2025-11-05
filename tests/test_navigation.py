import math
import unittest

import numpy as np

from navigation.controller import Controller
from navigation.ekf import EKF
from navigation.robot import Robot, Scenario, integrate, wrap
from navigation.simulation import run, summarize
from navigation.world import World


class MotionTests(unittest.TestCase):
    def test_straight_and_circular_motion(self):
        np.testing.assert_allclose(integrate((0, 0, 0), 1, 0, 2), [2, 0, 0])
        np.testing.assert_allclose(integrate((0, 0, 0), 1, 1, math.pi/2), [1, 1, math.pi/2])
        np.testing.assert_allclose(integrate((0, 0, 0), 1, 1, 2*math.pi), [0, 0, 0], atol=1e-14)

    def test_angle_wrap(self):
        self.assertAlmostEqual(wrap(3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap(-3 * math.pi), -math.pi)

    def test_slip_changes_ground_motion_but_not_encoder_rotation(self):
        scenario = Scenario(encoder_scale=0, encoder_noise=0, gyro_noise=0, initial_bias=0, bias_walk=0)
        normal = Robot((1, 1, 0), scenario, 3)
        slipping = Robot((1, 1, 0), scenario, 3)
        slipping.traction[:] = 0.5
        slipping.slip_remaining = 10
        normal_measurement = normal.step(1, 0, 0.1)
        slip_measurement = slipping.step(1, 0, 0.1)
        self.assertAlmostEqual(normal_measurement[0], slip_measurement[0])
        self.assertAlmostEqual(normal.pose[0] - 1, 2 * (slipping.pose[0] - 1))

    def test_disturbance_schedule_is_independent_of_commands(self):
        scenario = Scenario(slip_rate=1, slip_loss=0.6)
        slow, fast = Robot((1, 1, 0), scenario, 9), Robot((1, 1, 0), scenario, 9)
        for _ in range(100):
            slow.step(0.1, 0, 0.1)
            fast.step(0.6, 1, 0.1)
            self.assertEqual(slow.bias, fast.bias)
            np.testing.assert_array_equal(slow.traction, fast.traction)


class PlanningTests(unittest.TestCase):
    def test_route_preserves_endpoints_and_clearance(self):
        world = World()
        path = world.plan(world.start, world.goal)
        self.assertEqual(path[0], world.start[:2])
        self.assertEqual(path[-1], world.goal)
        self.assertTrue(all(world.segment_free(a, b, 0.45) for a, b in zip(path, path[1:])))

    def test_blocked_and_unreachable_goals_raise(self):
        world = World(obstacles=((5, 0, 6, 10),))
        with self.assertRaisesRegex(ValueError, "No route"):
            world.plan(world.start, world.goal)
        with self.assertRaisesRegex(ValueError, "inside"):
            world.plan(world.start, (5.5, 5))

    def test_swept_collision_detects_thin_wall(self):
        world = World(obstacles=((2, 1, 2.05, 4),))
        self.assertTrue(world.free((1, 2), 0.2))
        self.assertTrue(world.free((3, 2), 0.2))
        self.assertFalse(world.segment_free((1, 2), (3, 2), 0.2))

    def test_safe_endpoint_can_connect_outside_its_grid_cell(self):
        world = World()
        for start in ((3.0, 6.1), (7.375, 3.51), (7.9683, 3.525)):
            self.assertTrue(world.free(start, 0.45))
            path = world.plan(start, world.goal)
            self.assertEqual(path[0], start)
            self.assertTrue(all(world.segment_free(a, b, 0.45) for a, b in zip(path, path[1:])))

    def test_sweep_catches_contact_between_sample_points(self):
        world = World(obstacles=((2.015, 2.1998, 2.025, 2.3),))
        self.assertTrue(world.free((2, 2), 0.2))
        self.assertTrue(world.free((2.039, 2), 0.2))
        self.assertFalse(world.segment_free((2, 2), (2.039, 2), 0.2))

    def test_sweep_preserves_round_corner_clearance(self):
        world = World(obstacles=((2, 2, 3, 3),))
        self.assertTrue(world.segment_free((1.8, 1.85), (1.85, 1.8), 0.2))
        self.assertFalse(world.segment_free((1.8, 2), (1.8, 3), 0.2))


class FilterTests(unittest.TestCase):
    def test_transition_covariance_matches_numerical_jacobian(self):
        state = np.array([1., 2., 0.7, 0.5, 0.6, 0.01])
        covariance = np.diag([0.01, 0.02, 0.03, 0.04, 0.05, 0.06])
        jacobian = np.zeros((6, 6))
        for column in range(6):
            predicted = []
            for sign in (-1, 1):
                filter = EKF(state[:3])
                filter.state = state.copy()
                filter.state[column] += sign * 1e-6
                filter.predict(0.1)
                predicted.append(filter.state.copy())
            jacobian[:, column] = (predicted[1] - predicted[0]) / 2e-6
        noisy, zero = EKF(state[:3]), EKF(state[:3])
        noisy.state, zero.state = state.copy(), state.copy()
        noisy.cov, zero.cov = covariance.copy(), np.zeros((6, 6))
        noisy.predict(0.1)
        zero.predict(0.1)
        np.testing.assert_allclose(noisy.cov - zero.cov, jacobian @ covariance @ jacobian.T, atol=1e-9)

    def test_stationary_bias_and_covariance(self):
        ekf = EKF((1, 1, 0))
        for _ in range(150):
            ekf.predict(0.1)
            ekf.update(np.array([0, 0, 0.05]))
            np.testing.assert_allclose(ekf.cov, ekf.cov.T, atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(ekf.cov).min(), -1e-12)
        self.assertAlmostEqual(ekf.state[5], 0.05, delta=0.002)
        np.testing.assert_allclose(ekf.state[:2], [1, 1], atol=1e-12)

    def test_constant_speed_estimate(self):
        ekf = EKF((0, 0, 0))
        for _ in range(100):
            ekf.predict(0.1)
            ekf.update(np.array([0.5, 0, 0.02]))
        self.assertAlmostEqual(ekf.state[0], 5, delta=0.03)
        self.assertAlmostEqual(ekf.state[1], 0, delta=0.03)


class ControllerTests(unittest.TestCase):
    def test_controller_gets_close_to_estimated_goal_before_stopping(self):
        controller = Controller(World(obstacles=(), goal=(11, 1)))
        speed, _ = controller.command(np.array([10.9, 1, 0]), 0.01, 0, 0, 0.1)
        self.assertFalse(controller.finished)
        self.assertGreater(speed, 0)
        self.assertEqual(controller.command(np.array([10.95, 1, 0]), 0.01, 0, 1, 0.1), (0, 0))
        self.assertTrue(controller.finished)

    def test_uncertainty_reduces_speed(self):
        world = World(obstacles=(), goal=(11, 1))
        state = np.array([1, 1, 0, 0, 0, 0])
        low = Controller(world).command(state, 0.01, 0, 0, 0.1)[0]
        high = Controller(world).command(state, 0.5, 0, 0, 0.1)[0]
        self.assertLess(high, low)

    def test_recovery_stops_and_resumes_without_pose_reset(self):
        controller = Controller(World(obstacles=(), goal=(11, 1)))
        state = np.array([1., 1., 0., 0., 0., 0.])
        original = state.copy()
        for time in (5, 5.1, 5.2, 5.3):
            command = controller.command(state, 0.1, 6, time, 0.1)
        self.assertEqual(command, (0, 0))
        self.assertEqual(controller.recoveries, 1)
        self.assertGreater(controller.command(state, 0.1, 0, 7, 0.1)[0], 0)
        np.testing.assert_array_equal(state, original)


class SimulationTests(unittest.TestCase):
    def test_seed_reproduces_results_and_trace(self):
        a, first, _ = run(seed=42, max_time=2, trace=True)
        b, second, _ = run(seed=42, max_time=2, trace=True)
        self.assertEqual(a, b)
        np.testing.assert_array_equal(first, second)

    def test_empty_success_group_has_no_time_to_goal(self):
        result, _, _ = run(max_time=0.1)
        group = summarize([result])[0]
        self.assertFalse(result.success)
        self.assertIsNone(result.time_to_goal_s)
        self.assertIsNone(group["mean_time_to_goal_s"])
        self.assertEqual(group["outcomes"], {"timeout": 1})

    def test_invalid_time_step(self):
        for dt in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                run(dt=dt)

    def test_decimal_time_limit_does_not_drop_a_step(self):
        result, history, _ = run(max_time=0.3, trace=True)
        self.assertEqual(len(history), 3)
        self.assertAlmostEqual(result.elapsed_s, 0.3)

    def test_reported_errors_match_saved_truth(self):
        result, history, _ = run(seed=17, max_time=2, trace=True)
        errors = np.linalg.norm(history[:, 1:3] - history[:, 4:6], axis=1)
        self.assertAlmostEqual(result.position_rmse_m, np.sqrt(np.mean(errors**2)))
        self.assertAlmostEqual(result.final_error_m, errors[-1])
        self.assertAlmostEqual(result.goal_error_m, np.linalg.norm(history[-1, 1:3] - World().goal))

    def test_nominal_reaches_goal(self):
        result, _, _ = run(seed=0, scenario="nominal")
        self.assertTrue(result.success, result)
        self.assertEqual(result.collisions, 0)


if __name__ == "__main__":
    unittest.main()
