import numpy as np
from scipy.spatial import cKDTree

def get_blocking_width(dist, rad):

    return np.arcsin(np.minimum((2*rad)/dist, 1.)) *1.1;

def update_pos(xs, ys):
    p = np.dstack([xs.ravel(), ys.ravel()])[0]
    global kdtree
    kdtree = cKDTree(p)

def get_points_within_radius(p, xs, ys, radius):
    within_r = kdtree.query_ball_point(p, radius)
    x2 = xs[within_r].ravel().transpose()
    y2 = ys[within_r].ravel().transpose()

    return np.array([x2, y2])

def wrapped_diff(a, b, wrapping):
    diff = np.abs(a-b)
    return np.minimum(diff, wrapping-diff)


def get_best_direction(x, y, xs, ys, angle, search_radius, pedestrian_radius):
    p = np.array([x, y])
    points = get_points_within_radius(p.ravel(), xs, ys, search_radius)
    delta = points-p
    delta = delta[:, np.any(delta != 0, axis=0)];
    dist = np.linalg.norm(delta, axis = 0)
    blocking_width = get_blocking_width(dist, pedestrian_radius)
    blocking_angles = np.arctan2(delta[1], delta[0]) % (2 * np.pi)
    num_angles = 100
    angles = np.linspace(0, 2*np.pi, num_angles)
    angle_mask = np.ones(num_angles, dtype=np.bool)
    for i in range(0, blocking_angles.size):
        additional_mask = wrapped_diff(blocking_angles[i], angles, 2*np.pi) > blocking_width[i]
        angle_mask = np.logical_and(angle_mask, additional_mask)

    valid_angles = angles[angle_mask]
    if not np.any(valid_angles):
        search_radius *= 0.5
        if search_radius < pedestrian_radius:
            return 0, 0, angle
        else:
            return get_best_direction(x, y, xs, ys, angle, search_radius, pedestrian_radius)
    best_angle_idx = wrapped_diff(valid_angles, angle, 2*np.pi).argmin()
    new_angle = valid_angles[best_angle_idx]

    if wrapped_diff(new_angle, angle, 2*np.pi) > np.pi:
        search_radius *= 0.5
        if search_radius < pedestrian_radius:
            return 0, 0, angle
        else:
            return get_best_direction(x, y, xs, ys, angle, search_radius, pedestrian_radius)

    return np.cos(new_angle), np.sin(new_angle),angle

