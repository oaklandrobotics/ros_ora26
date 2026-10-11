"""Regression coverage for line protection in the clearing point cloud (#68)."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2

spec = importlib.util.spec_from_file_location(
    'edge_detection',
    Path(__file__).parents[1] / 'ora_perception' / 'edge_detection.py',
)
edge_detection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(edge_detection)


@pytest.mark.parametrize('radius', [0, 1, 2, 4])
def test_line_and_requested_margin_are_never_free(radius):
    lines = np.zeros((21, 21), dtype=np.uint8)
    lines[:, 10] = 255
    valid = np.full_like(lines, 255)
    free = edge_detection.make_free_space_mask(lines, valid, radius)
    assert not free[:, 10-radius:11+radius].any()
    assert (free[:, :10-radius] == 255).all()
    assert (free[:, 11+radius:] == 255).all()


def test_horizon_and_robot_exclusions_remain_excluded():
    lines = np.zeros((20, 20), dtype=np.uint8)
    lines[5:15, 10] = 255
    valid = np.full_like(lines, 255)
    valid[:5] = 0
    valid[15:, 6:14] = 0
    free = edge_detection.make_free_space_mask(lines, valid, 2)
    assert not free[valid == 0].any()
    assert not free[3:17, 8:13].any()
    assert free[10, 2] == 255


def test_border_line_has_protection_without_wraparound():
    lines = np.zeros((10, 10), dtype=np.uint8)
    lines[:, 0] = 255
    free = edge_detection.make_free_space_mask(
        lines, np.full_like(lines, 255), 2,
    )
    assert not free[:, :3].any()
    assert (free[:, 3:] == 255).all()


@pytest.mark.parametrize('all_lines', [False, True])
def test_empty_and_full_line_frames(all_lines):
    lines = np.full((10, 10), 255 if all_lines else 0, dtype=np.uint8)
    valid = np.full_like(lines, 255)
    valid[:2] = 0
    free = edge_detection.make_free_space_mask(lines, valid, 2)
    np.testing.assert_array_equal(free, np.zeros_like(lines) if all_lines else valid)


@pytest.mark.parametrize('sim_mode', [False, True])
def test_callback_publishes_clearing_points_outside_line_margin(sim_mode):
    bridge = CvBridge()
    color = np.zeros((100, 100, 3), dtype=np.uint8)
    color[:, 8:17] = 255
    depth = np.ones((100, 100), dtype=np.float32)
    # Invalid samples must not appear in either point cloud.
    depth[50:55] = np.nan
    image = bridge.cv2_to_imgmsg(color, 'bgr8')
    depth_image = bridge.cv2_to_imgmsg(depth, '32FC1')
    depth_image.header.frame_id = 'camera'
    collected_lines, collected_free = [], []
    node = SimpleNamespace(
        bridge=bridge, use_sim_time=sim_mode, free_space_dilation_radius=2,
        camera_model={'fx': 100., 'fy': 100., 'cx': 0., 'cy': 0.},
        masked_img_pub=SimpleNamespace(publish=lambda msg: None),
        line_publisher=SimpleNamespace(publish=collected_lines.append),
        free_space_publisher=SimpleNamespace(publish=collected_free.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: depth_image.header.stamp)),
    )
    node.image_to_depth = lambda *args, **kwargs: (
        edge_detection.EdgeDetectionNode.image_to_depth(node, *args, **kwargs)
    )
    edge_detection.EdgeDetectionNode.detect_edges(node, image, depth_image)
    marked = point_cloud2.read_points_numpy(collected_lines[0])
    cleared = point_cloud2.read_points_numpy(collected_free[0])
    assert len(marked) > 0 and len(cleared) > 0
    marked_uv = np.rint(marked[:, :2] * 100).astype(int)
    cleared_uv = np.rint(cleared[:, :2] * 100).astype(int)
    assert np.isfinite(marked).all() and np.isfinite(cleared).all()
    for u, v in cleared_uv:
        assert not (((np.abs(marked_uv[:, 0]-u) <= 2)
                    & (np.abs(marked_uv[:, 1]-v) <= 2)).any())
    assert (cleared_uv[:, 1] >= 27).all()
    expected_frame = ('camera_link_optical' if sim_mode
                      else 'zed_left_camera_frame_optical')
    assert collected_free[0].header.frame_id == expected_frame
