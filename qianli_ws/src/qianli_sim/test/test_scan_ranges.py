from pathlib import Path
import math
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from scan_ranges import normalize_no_returns


class NoReturnTests(unittest.TestCase):
    def test_no_hit_clears_without_changing_obstacle_returns(self):
        values = [.2, 1., 4., math.inf, math.inf]
        output, count = normalize_no_returns(values, .12, 12.)
        self.assertEqual(output[:3], values[:3])
        self.assertEqual(count, 2)
        # Karto accepts readings < physical max; >= mapper threshold has no hit.
        self.assertTrue(11.99 <= output[3] < 12.)
        self.assertEqual(values[3], math.inf)

    def test_renderer_failure_is_not_free_space(self):
        output, count = normalize_no_returns([math.inf]*360, .12, 12.)
        self.assertEqual(count, 0)
        self.assertTrue(all(v == math.inf for v in output))

    def test_sparse_finite_scan_does_not_pass_health_gate(self):
        values = [1., 2., 3.]+[math.inf]*357
        self.assertEqual(normalize_no_returns(values, .12, 12.)[1], 0)

    def test_invalid_returns_remain_invalid(self):
        values = [1., 2., 3., math.nan, -math.inf, .01, math.inf]
        output, count = normalize_no_returns(values, .12, 12.)
        self.assertTrue(math.isnan(output[3]))
        self.assertEqual(output[4:6], values[4:6])
        self.assertEqual(count, 1)

    def test_invalid_metadata_never_creates_clear_rays(self):
        for minimum, maximum in [(.12, math.inf), (12., 1.), (.12, math.nan)]:
            self.assertEqual(normalize_no_returns([1., 2., 3., math.inf], minimum, maximum)[1], 0)


if __name__ == '__main__':
    unittest.main()
