import math
import sys
import os
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))
from lidar_observer import determine_target_geometry, process_scan


def make_dummy_scan(count=619, angle_min=-math.pi, angle_max=math.pi, range_min=0.1, range_max=10.0, fill_val=5.0):
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    angular_span = angle_max - angle_min
    msg.angle_increment = angular_span / count
    msg.range_min = range_min
    msg.range_max = range_max
    msg.scan_time = 0.1
    msg.ranges = [fill_val] * count
    return msg


def test_geometry_determination_and_locking():
    # 1. 619点入力で target_points=0 の場合、619に固定される
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, target_points_param=0)
    assert geo is not None
    assert geo['target_points'] == 619
    assert geo['full_circle'] is True


def test_variable_input_counts():
    # 2. 619, 636, 672点の入力でも、output count は常に 619
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, 0)

    for count in [619, 636, 672]:
        msg_var = make_dummy_scan(count)
        out = process_scan(msg_var, geo)
        assert out is not None
        assert len(out.ranges) == 619


def test_conservative_downsampling_obstacle_preservation():
    # 3. 近距離障害物を1点挿入した場合、outputの最小有効距離がその障害物距離以下または同値で維持
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, 0)

    msg636 = make_dummy_scan(636, fill_val=5.0)
    msg636.ranges[300] = 0.5  # 近距離障害物

    out = process_scan(msg636, geo)
    assert out is not None
    min_dist = min(out.ranges)
    assert min_dist <= 0.5


def test_invalid_values_filtering():
    # 4. 0.0, NaN, Inf, range外の値が有効障害物として採用されない
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, 0)

    msg636 = make_dummy_scan(636, fill_val=5.0)
    msg636.ranges[10] = 0.0          # range_min 未満
    msg636.ranges[20] = float('nan') # NaN
    msg636.ranges[30] = float('inf') # Inf
    msg636.ranges[40] = 15.0         # range_max 超過

    out = process_scan(msg636, geo)
    assert out is not None
    for r in out.ranges:
        if math.isfinite(r):
            assert r >= msg636.range_min
            assert r <= msg636.range_max


def test_empty_bin_inf():
    # 5. 有効値がない bin は math.inf になる
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, 0)

    msg636 = make_dummy_scan(636, fill_val=float('nan'))
    out = process_scan(msg636, geo)
    assert out is not None
    for r in out.ranges:
        assert math.isinf(r)


def test_input_count_less_than_target():
    # 6. input count < target count の場合は process_scan が None を返す
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, 0)

    msg500 = make_dummy_scan(500)
    out = process_scan(msg500, geo)
    assert out is None


def test_full_circle_angle_increment_relation():
    # 7. full-circle output について、round((angle_max - angle_min) / angle_increment) == len(ranges)
    msg619 = make_dummy_scan(619)
    geo = determine_target_geometry(msg619, 0)

    out = process_scan(msg619, geo)
    assert out is not None
    calculated = round((out.angle_max - out.angle_min) / out.angle_increment)
    assert calculated == len(out.ranges)
