import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as plt_patches

from PedSim import *
from matplotlib.collections import PatchCollection
from KeepLeft import *
pedestrian_radius = 0.1
search_radius = 10 * pedestrian_radius
global num_agents
w = 10
h = 5
t = 0.01


obstacle_side = 20*2

def angle_from_index(i):
    if i < num_agents * 0.5:
        return 0
    else:
        return np.pi

def init_y():
    return np.random.rand() * h


def init(p_num_agents):
    global x_coordinates, y_coordinates, velocities, direction, num_agents
    num_agents = p_num_agents

    x_coordinates = np.zeros((num_agents + 300, 1))
    y_coordinates = np.zeros((num_agents + 300, 1))
    velocities = np.zeros((num_agents, 1))
    direction = np.zeros((num_agents, 1))

    for i in range(num_agents):
        r = np.random.rand() * w
        x_coordinates[i] = r
        direction[i] = angle_from_index(i)
        y_coordinates[i] = init_y()
        velocities[i] = (0.5 + 0.1 * np.random.rand()) * 3.2

    # Add wall obstacle
    upper_wall_x = np.linspace(0, w, 100)
    upper_wall_y = np.ones(100) * h
    lower_wall_x = np.linspace(0, w, 100)
    lower_wall_y = np.zeros(100)
    for i in range(100):
        x_coordinates[num_agents + i] = upper_wall_x[i]
        x_coordinates[num_agents + 100 + i] = lower_wall_x[i]

        y_coordinates[num_agents + i] = upper_wall_y[i]
        y_coordinates[num_agents + 100 + i] = lower_wall_y[i]

    # Add some obstacle in the walkway.
    for i in range(1):
        for j in range(10):
            x_coordinates[num_agents + 200 + 10 * i + j] = 5 + 0.1 * i + 0.1 * j
            y_coordinates[num_agents + 200 + 10 * i + j] = 2 + 0.1 * j


def try_move(x, y, d, v):
    desired_x = x + v*t*np.cos(d)
    desired_y = y + v*t*np.sin(d)
    return desired_x, desired_y


def check_for_collisions(x, y, d, v):
    x_difference = x_coordinates - x
    y_difference = y_coordinates - y


    if (d == 0):
        valid_x = np.where(x_difference > 0)[0]
        x_difference_tmp = x_difference[valid_x]
        y_difference_tmp = y_difference[valid_x]
    else:
        valid_x = np.where(x_difference < 0)[0]
        x_difference_tmp = x_difference[valid_x]
        y_difference_tmp = y_difference[valid_x]

    r_difference = np.sqrt(x_difference_tmp**2 + y_difference_tmp**2)


    overlapping = np.where(r_difference < search_radius)[0]


    angle = d
    netto_y = 0
    for i in range(len(overlapping)):
        tmp = y_difference_tmp[overlapping[i]]
        r_diff = r_difference[overlapping[i]]
        netto_y += tmp/r_diff**2

    mean_netto_y = np.abs(netto_y / max(1,len(overlapping)))

    angle = np.arctan(mean_netto_y /(v*t)) + d

    if (netto_y > 0):
        if (d == np.pi):
            angle = -d + np.arctan(mean_netto_y /(v*t))
        else:
            angle = -angle

    else:
        if (d == np.pi):
            angle = -d - np.arctan(mean_netto_y /(v*t))


    x = x + v*t*np.cos(angle)
    y = y + v*t*np.sin(angle)
    return x, y

patches = []

def init_plotting():
    plt.ion()
    plt.scatter(x_coordinates[num_agents:], y_coordinates[num_agents:], c = 'r', s = 15)

    for i in range(0, num_agents):
        circle = plt_patches.Circle((i, i), pedestrian_radius, color='g')
        patches.append(circle)


    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(0, w)
    plt.ylim(0, h)

def plot(x, y):

    for i in range(0, num_agents):
        patches[i].center = ([x[i], y[i]])

    c = plt.gcf().gca().add_artist(PatchCollection(patches))
    plt.show()
    plt.pause(0.001)
    c.remove()




new_movement = True

def sim(p_num_agents, num_its, do_plot):
    init(p_num_agents)
    if do_plot:
        init_plotting()
    global x_coordinates, y_coordinates
    kl = 0
    for i in range(num_its):
        update_pos(x_coordinates, y_coordinates)
        x_coordinates,y_coordinates = collision_resoluton(x_coordinates, y_coordinates, pedestrian_radius)
        for j in range(num_agents):

            x = x_coordinates[j]
            y = y_coordinates[j]
            theta = direction[j]
            v = velocities[j]
            if new_movement:
                dx, dy, theta = get_best_direction(x, y, x_coordinates, y_coordinates, theta, search_radius, pedestrian_radius)
                x_coordinates[j] += dx * v * t
                y_coordinates[j] += dy * v * t
                direction[j] = lerp_angle(theta, angle_from_index(j), 0.5)
            else:
                desired_x, desired_y = try_move(x, y, theta, v)
                x, y = check_for_collisions(desired_x, desired_y, theta, v)
                x_coordinates[j] = x
                y_coordinates[j] = y

            if(x_coordinates[j] > w+pedestrian_radius):
                init_y()
                x_coordinates[j] = 0
            elif x_coordinates[j] < 0-pedestrian_radius:
                init_y()
                x_coordinates[j] = w

        if do_plot:
            plot(x_coordinates, y_coordinates)

        kl = kl + keep_left(y_coordinates[0:num_agents], h, direction)
    return kl/num_its

if __name__ == '__main__':
    min_peds = 20
    max_peds = 100
    kl = np.zeros(max_peds-min_peds)

    for i in range(min_peds, max_peds):
        kl[i-min_peds] = sim(i, 1000, False)
        print(i)
    plt.plot(range(min_peds, max_peds), kl)
    plt.show()
    print(kl)

