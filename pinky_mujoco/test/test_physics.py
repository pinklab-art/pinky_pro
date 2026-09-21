"""Regression tests against actual MuJoCo integration, ray intersections and world loading."""
import math
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from pinky_mujoco.model import Physics, build_model, default_world


@pytest.fixture(scope='module')
def scene(tmp_path_factory):
    share = get_package_share_directory('pinky_mujoco')
    return build_model(share, tmp_path_factory.mktemp('scene'), world=default_world(share))


def settled(scene):
    physics = Physics(scene)
    for _ in range(100):
        physics.step()
    return physics


def test_wheels_move_robot_and_encoders_agree(scene):
    p = settled(scene)
    for _ in range(400):
        p.step(0.15, 0)
    assert 0.55 < p.pose()[0] < 0.65
    assert abs(p.pose()[1]) < 0.01
    assert np.linalg.norm(p.pose()[:2] - p.odom[:2]) < 0.025
    assert 0.025 < p.data.qpos[2] < 0.031
    start = p.pose()[2]
    for _ in range(200):
        p.step(0, 0.7)
    assert 1.1 < p.pose()[2] - start < 1.5
    assert abs(p.pose()[2] - p.odom[2]) < 0.1
    for _ in range(100):
        p.step(0, 0)
    assert np.max(np.abs(p.data.qvel[p.wheel_v])) < 0.01


def test_lidar_uses_urdf_rotated_frame_and_world_collisions(scene):
    p = settled(scene)
    scan = p.scan()
    assert len(scan) == 360
    # URDF lidar frame points backwards (yaw=pi); -pi ray faces world +X.
    assert scan[0] == pytest.approx(2.967, abs=0.02)
    assert scan[180] == pytest.approx(2.933, abs=0.02)
    assert scan[90] == pytest.approx(2.45, abs=0.02)
    assert np.all(np.isfinite(scan))
    # The rigid body must be physically stopped by the east wall.
    for _ in range(1800):
        p.step(0.25, 0)
    assert p.pose()[0] < 2.95
    assert p.pose()[0] > 2.7
    assert np.all(np.isfinite(p.data.qpos))


def gz_world(name):
    return Path(get_package_share_directory('pinky_gz_sim')) / 'worlds' / name


def test_gazebo_pinky_map_world_walls(tmp_path):
    """pinky_map.world: 2 x 1 m room of 0.01 m walls at x=+-1, y=+-0.5; lidar sits 0.017 m behind base."""
    p = settled(build_model(get_package_share_directory('pinky_mujoco'), tmp_path, world=gz_world('pinky_map.world')))
    scan = p.scan()
    assert scan[0] == pytest.approx(0.995 + 0.017, abs=0.02)    # world +X wall
    assert scan[180] == pytest.approx(0.995 - 0.017, abs=0.02)  # world -X wall
    assert scan[90] == pytest.approx(0.495, abs=0.02)           # world +Y wall
    assert scan[270] == pytest.approx(0.495, abs=0.02)          # world -Y wall
    assert np.all(np.isfinite(scan))


def test_gazebo_factory_world_loads_shelves_and_meshes(tmp_path):
    """pinky_factory.world: 4 x 3 m walls, four included shelf models with DAE visual meshes."""
    scene = build_model(get_package_share_directory('pinky_mujoco'), tmp_path, world=gz_world('pinky_factory.world'))
    text = scene.read_text()
    assert text.count('shelf_') >= 4
    assert 'world_mesh_0' in text
    assert '<texture' in text and 'logo' in text  # shelf box texture and the floor logo quad
    p = settled(scene)
    scan = p.scan()
    assert np.all(np.isfinite(scan))
    assert scan[90] == pytest.approx(1.495, abs=0.02)           # y=+1.5 wall, nothing in between
    # shelf_3 collision box (x -1.25..0.-0.53 around y 0.5) is closer than the west wall
    assert 0.3 < scan[180] < 1.995
