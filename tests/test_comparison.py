import csv
from pathlib import Path
import tempfile
import unittest

from scripts.compare_trials import compare, read_trials


class ComparisonTests(unittest.TestCase):
    def test_pairing_uses_seed_not_row_order(self):
        before = {("slip", "adaptive", seed): {"success": "True" if seed == 2 else "False",
                  "collisions": "0", "position_rmse_m": "0.1"} for seed in (1, 2, 3)}
        after = {key: dict(before[key], success="True") for key in reversed(before)}
        row = compare(before, after)[0]
        self.assertEqual(row["gained_successes"], 2)
        self.assertEqual(row["lost_successes"], 0)
        self.assertAlmostEqual(row["success_rate_change"], 2/3)

    def test_missing_seed_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "same"):
            compare({("slip", "fixed", 1): {}}, {})

    def test_duplicate_trial_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trials.csv"
            row = {"seed": 1, "scenario": "slip", "policy": "fixed", "outcome": "collision",
                   "success": "False", "collisions": 1, "position_rmse_m": 0.1, "elapsed_s": 2}
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=row)
                writer.writeheader()
                writer.writerows([row, row])
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                read_trials(path)
