#!/usr/bin/env python3
import os
import sys
import numpy as np
import pytest

pytest.importorskip("rclpy")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from rrt_star_planner import RRTStarPlanner, RRTNode

class MockNode(RRTStarPlanner):
    """Mock the node to avoid initializing the full ROS 2 networking during PyTest."""
    def __init__(self):
        # We manually bypass the ROS __init__ and set the internal state
        self._res = 0.05
        self._gw = 120
        self._gh = 120
        self._origin_x = -0.6
        self._origin_y = -3.0
        
        # Create an empty grid
        self._grid = np.zeros((self._gh, self._gw), dtype=np.int8)

def test_world_to_grid():
    planner = MockNode()
    
    # Test the origin
    gx, gy = planner._world_to_grid(-0.6, -3.0)
    assert gx == 0
    assert gy == 0
    
    # Test a point 1 meter ahead
    gx, gy = planner._world_to_grid(0.4, -3.0)
    assert gx == 20
    assert gy == 0

def test_collision_free():
    planner = MockNode()
    
    # Empty grid, path should be valid
    n1 = RRTNode(0.0, 0.0)
    n2 = RRTNode(1.0, 0.0)
    assert planner._check_collision(n1, n2) is True

def test_collision_blocked():
    planner = MockNode()
    
    # Place a block in the grid
    # x=0.5m -> gx = (0.5 - (-0.6))/0.05 = 1.1/0.05 = 22
    # y=0.0m -> gy = (0.0 - (-3.0))/0.05 = 3.0/0.05 = 60
    planner._grid[55:65, 20:25] = 100
    
    n1 = RRTNode(0.0, 0.0)
    n2 = RRTNode(1.0, 0.0)
    assert planner._check_collision(n1, n2) is False

def test_out_of_bounds():
    planner = MockNode()
    # x=10m is outside the 6x6 grid
    assert planner._is_valid(10.0, 0.0) is False
    assert planner._is_valid(0.0, 10.0) is False
