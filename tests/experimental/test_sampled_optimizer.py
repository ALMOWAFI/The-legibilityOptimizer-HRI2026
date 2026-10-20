from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from experimental.legibility_optimizer import sampled_optimizer as sampled
from experimental.legibility_optimizer.optimizer_core import OptimizerConfig, joint_motion_cost
from experimental.legibility_optimizer.demo import synthetic_case, identity_fk


class SampledOptimizerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        q, t, cls.goals = synthetic_case()
        cls.t = np.linspace(t[0], t[-1], 41)
        cls.q = np.column_stack([np.interp(cls.t, t, q[:, j]) for j in range(3)])
        cls.kwargs = dict(joint_lower=[-2, -2, -1], joint_upper=[2, 2, 1],
                          max_joint_deviation=[0.25, 0.25, 0.1], support_points=7,
                          config=OptimizerConfig(cost_ratio=1.45, maxiter=200))
        cls.result = sampled.optimize_sampled_path(cls.q, cls.t, cls.goals, 0, identity_fk, **cls.kwargs)

    def test_dense_grid_endpoints_budget_and_deviation(self):
        r = self.result
        self.assertTrue(r.numerical_accepted, r.message)
        self.assertEqual(r.positions.shape, self.q.shape)
        np.testing.assert_array_equal(r.positions[[0, -1]], self.q[[0, -1]])
        np.testing.assert_array_equal(r.times, self.t)
        self.assertLessEqual(joint_motion_cost(r.positions, self.t),
                             1.45 * joint_motion_cost(self.q, self.t) + 1e-7)
        self.assertTrue(np.all(np.abs(r.positions - self.q) <= np.array([0.25, 0.25, 0.1]) + 1e-7))
        self.assertGreater(r.optimized_legibility, r.baseline_legibility)
        self.assertFalse(r.execution_ready)

    def test_no_silent_timestamp_resampling(self):
        for t in (self.t[::-1], np.zeros_like(self.t), self.t - 1):
            with self.assertRaises(ValueError):
                sampled.optimize_sampled_path(self.q, t, self.goals, 0, identity_fk, **self.kwargs)

    def test_dense_baseline_limits_checked_between_supports(self):
        q = self.q.copy()
        q[1, 2] = 2.0
        with self.assertRaisesRegex(ValueError, "dense baseline"):
            sampled.optimize_sampled_path(q, self.t, self.goals, 0, identity_fk, **self.kwargs)

    def test_full_grid_rechecks_solver_flags_and_constraints(self):
        indices = np.linspace(0, len(self.q) - 1, 7).astype(int)
        q = self.q[indices].copy()
        q[1:-1, 2] = [0.9, -0.9, 0.9, -0.9, 0.9]
        fake = replace(self.result, positions=q, numerical_accepted=True)
        with patch.object(sampled, "optimize_joint_path", return_value=fake):
            result = sampled.optimize_sampled_path(self.q, self.t, self.goals, 0, identity_fk, **self.kwargs)
        self.assertFalse(result.numerical_accepted)
        self.assertFalse(result.local_deviation_ok)
        self.assertFalse(result.cost_budget_ok)

    def test_support_failure_remains_rejected(self):
        indices = np.linspace(0, len(self.q) - 1, 7).astype(int)
        fake = replace(self.result, positions=self.q[indices], numerical_accepted=False,
                       solver_success=False)
        with patch.object(sampled, "optimize_joint_path", return_value=fake):
            result = sampled.optimize_sampled_path(self.q, self.t, self.goals, 0, identity_fk, **self.kwargs)
        self.assertFalse(result.numerical_accepted)
        self.assertFalse(result.solver_success)

    def test_invalid_support_counts(self):
        for count in (2, True, 4.5):
            with self.assertRaises(ValueError):
                sampled.optimize_sampled_path(self.q, self.t, self.goals, 0, identity_fk,
                                              **dict(self.kwargs, support_points=count))


if __name__ == "__main__":
    unittest.main()
