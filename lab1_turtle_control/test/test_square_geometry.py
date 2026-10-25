import math

from lab1_turtle_control.square_geometry import square_corners


def test_four_equal_sides_with_right_angles():
    pts = square_corners(3.0)
    assert len(pts) == 4
    loop = [(pts[-1].x, pts[-1].y)] + [(p.x, p.y) for p in pts]   # start corner -> 4 corners
    sides = [(loop[i + 1][0] - loop[i][0], loop[i + 1][1] - loop[i][1]) for i in range(4)]
    for dx, dy in sides:
        assert abs(math.hypot(dx, dy) - 3.0) < 1e-9
    for i in range(4):
        (ax, ay), (bx, by) = sides[i], sides[(i + 1) % 4]
        assert abs(ax * bx + ay * by) < 1e-9          # consecutive sides are perpendicular


def test_square_stays_inside_window():
    for p in square_corners(5.0):
        assert 0.0 < p.x < 11.0 and 0.0 < p.y < 11.0
