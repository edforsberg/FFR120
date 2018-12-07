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

def collision_resoluton(xs, ys, radius):
    ic = kdtree.query_ball_tree(kdtree, radius*2)

    for i in range(len(ic)):
        for j in ic[i]:
            if i >= j:
                continue
            dx = xs[i]-xs[j]
            dy = ys[i]-ys[j]
            d = np.sqrt(dx*dx + dy*dy)
            xs[i] = xs[j] + dx / d * radius*2
            ys[i] = ys[j] + dy / d * radius*2
            xs[j] = xs[i] - dx / d * radius*2
            ys[j] = ys[i] - dy / d * radius*2

    return xs, ys


def wrapped_diff(a, b, wrapping):
    diff = np.abs(a-b)
    return np.minimum(diff, wrapping-diff)

def lerp_angle(a, b, t):
    diff = b-a
    diff_wrapped = -np.pi*2+diff
    if abs(diff) > abs(diff_wrapped):
        return b + diff_wrapped*t
    else:
        return a + diff*t


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
            #dx, dy = collision_resolution(x, y, pedestrian_radius)
            return 0, 0, (angle + np.random.random()*0.5)%(2*np.pi);
        else:
            return get_best_direction(x, y, xs, ys, angle, search_radius, pedestrian_radius)
    best_angle_idx = wrapped_diff(valid_angles, angle, 2*np.pi).argmin()
    new_angle = valid_angles[best_angle_idx]

    if wrapped_diff(new_angle, angle, 2*np.pi) > np.pi/2.:
        search_radius *= 0.5
        if search_radius < pedestrian_radius:
            return 0, 0, angle
        else:
            return get_best_direction(x, y, xs, ys, angle, search_radius, pedestrian_radius)

    return np.cos(new_angle), np.sin(new_angle), angle

