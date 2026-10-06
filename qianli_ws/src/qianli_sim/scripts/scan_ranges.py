"""Conservative Gazebo no-return conversion, independent of ROS."""
import math


def normalize_no_returns(ranges, range_min, range_max, clear_margin=.001):
    values = list(ranges)
    if not (math.isfinite(range_min) and math.isfinite(range_max) and
            0 <= range_min < range_max and 0 < clear_margin < range_max-range_min):
        return values, 0
    returns = sum(math.isfinite(v) and range_min <= v <= range_max for v in values)
    # A failed software renderer can return an entirely infinite scan. Keep it
    # invalid rather than manufacturing a complete circle of free space.
    if returns < max(3, math.ceil(len(values)*.05)):
        return values, 0
    clear_range = range_max-clear_margin
    output = [clear_range if v == math.inf else v for v in values]
    return output, sum(v == math.inf for v in values)
