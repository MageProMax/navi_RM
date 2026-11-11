"""Conservative ground-plane estimation for leveled navigation point clouds."""
import math
import numpy as np


def estimate_ground(points, radius=4., max_slope_deg=12., max_offset=.08,
                    threshold=.035, minimum_points=30):
    """Return z=a*x+b*y+c, or None without spatially supported near-ground data.

    Candidates come from the lowest return in each XY cell near the robot.
    A plane must intersect the expected floor near the robot; walls, ceilings,
    and elevated obstacle tops cannot establish an arbitrary new floor.
    """
    valid = (np.isfinite(points).all(axis=1) &
             (np.hypot(points[:, 0], points[:, 1]) <= radius) &
             (np.abs(points[:, 2]) <= .30))
    seeds = points[valid]
    if len(seeds) < minimum_points:
        return None
    cells = np.floor(seeds[:, :2]/.15).astype(np.int64)
    order = np.argsort(seeds[:, 2], kind='stable')
    _, indices = np.unique(cells[order], axis=0, return_index=True)
    seeds = seeds[order[indices]]
    if len(seeds) < minimum_points:
        return None
    if len(seeds) > 2000:
        seeds = seeds[np.linspace(0, len(seeds)-1, 2000).astype(int)]
    xy1 = np.column_stack((seeds[:, :2], np.ones(len(seeds))))
    slope_limit = math.tan(math.radians(max_slope_deg))
    rng = np.random.default_rng(0)
    best = None
    best_count = 0
    for _ in range(80):
        sample = rng.choice(len(seeds), 3, replace=False)
        try:
            plane = np.linalg.solve(xy1[sample], seeds[sample, 2])
        except np.linalg.LinAlgError:
            continue
        if np.hypot(*plane[:2]) > slope_limit or abs(plane[2]) > max_offset:
            continue
        inliers = np.abs(seeds[:, 2]-xy1@plane) <= threshold
        count = np.count_nonzero(inliers)
        if count > best_count:
            best, best_count = inliers, count
    if best is None or best_count < max(minimum_points, .35*len(seeds)):
        return None
    for _ in range(2):
        plane = np.linalg.lstsq(xy1[best], seeds[best, 2], rcond=None)[0]
        best = np.abs(seeds[:, 2]-xy1@plane) <= threshold
        if np.count_nonzero(best) < max(minimum_points, .35*len(seeds)):
            return None
    support = seeds[best]
    anchors = ((np.hypot(support[:, 0], support[:, 1]) <= 1.5) &
               (np.abs(support[:, 2]) <= .06))
    if (np.hypot(*plane[:2]) > slope_limit or abs(plane[2]) > max_offset or
            np.count_nonzero(anchors) < 8 or
            np.linalg.eigvalsh(np.cov(support[:, :2], rowvar=False))[0] < .02):
        return None
    return plane


def relative_height(points, plane):
    """Preserve XY ray endpoints and express height above the fitted floor."""
    corrected = points.copy()
    corrected[:, 2] -= points[:, :2]@plane[:2]+plane[2]
    return corrected
