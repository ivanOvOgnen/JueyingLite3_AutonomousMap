"""Goal validation uses real rotated grids and rejects unsafe map geometry."""
import math
from types import SimpleNamespace as NS

import pytest


@pytest.fixture
def goal_grid():
    return NS(info=NS(width=40, height=40, resolution=0.1,
                      origin=NS(position=NS(x=-2.0, y=3.0),
                                orientation=NS(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)))),
              data=[0] * 1600)


def test_goal_uses_rotated_origin_and_rejects_corresponding_occupied_cell(api, api_module, goal_grid):
    # Local centre (2.05, 2.05) becomes (-4.05, 5.05) after +90 degrees.
    api._validate_goal_cell(goal_grid, -4.05, 5.05)
    goal_grid.data[20 * 40 + 20] = 100
    with pytest.raises(api_module.ApiError, match='known-free'):
        api._validate_goal_cell(goal_grid, -4.05, 5.05)


def test_negative_cell_fraction_is_outside_not_truncated_to_zero(api, api_module, goal_grid):
    with pytest.raises(api_module.ApiError, match='outside'):
        api._validate_goal_cell(goal_grid, -2.05, 2.99)


@pytest.mark.parametrize('kind', ['zero_resolution', 'nan_origin', 'quaternion', 'dimensions'])
def test_invalid_grid_is_rejected_before_indexing(api, api_module, goal_grid, kind):
    if kind == 'zero_resolution':
        goal_grid.info.resolution = 0
    elif kind == 'nan_origin':
        goal_grid.info.origin.position.x = float('nan')
    elif kind == 'quaternion':
        goal_grid.info.origin.orientation.w = 0
    else:
        goal_grid.data.pop()
    with pytest.raises(api_module.ApiError, match='invalid planar geometry'):
        api._validate_goal_cell(goal_grid, -4.05, 5.05)
