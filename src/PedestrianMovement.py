import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as plt_patches

from PedSim import *
from matplotlib.collections import PatchCollection
from KeepLeft import *
from AvgSpeed import *
from CalcDensity import *
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


def init(p_num_agents, wall_index):
    global x_coordinates, y_coordinates, velocities, direction, num_agents
    num_agents = p_num_agents
    if wall_index == 1:
        wall_size = h/2
        wall_angle = np.pi/4
    elif wall_index == 2:
        wall_size = h/2
        wall_angle = np.pi/6
    elif wall_index == 3:
        wall_size = h/2
        wall_angle= np.pi/4
    else:
        wall_size = 0
        wall_angle = 0

    # comment out the next ine if we want wall_angle close to horizontal.
    # otherwise div by zero.
    wall_size /= np.arccos(wall_angle)
    num_wall_pieces = int(np.round(wall_size/(pedestrian_radius*2))*2);

    x_coordinates = np.zeros((num_agents + 200 + num_wall_pieces, 1))
    y_coordinates = np.zeros((num_agents + 200 + num_wall_pieces, 1))
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
    for j in range(num_wall_pieces):
        jj = (j-num_wall_pieces/2)/num_wall_pieces*wall_size
        x_coordinates[num_agents + 200 + j] = w/2 + jj*np.cos(wall_angle)
        y_coordinates[num_agents + 200 + j] = h/2 + jj*np.sin(wall_angle)


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

def sim(p_num_agents, num_its, wall_index, do_plot):
    init(p_num_agents, wall_index)
    if do_plot:
        init_plotting()
    global x_coordinates, y_coordinates
    kl = 0
    avg_speed = 0
    density = 0
    teq = num_its/4;
    for i in range(num_its):
        update_pos(x_coordinates, y_coordinates)
        x_coordinates,y_coordinates = collision_resoluton(x_coordinates, y_coordinates, pedestrian_radius, num_agents)
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
                x_coordinates[j] = 0
            elif x_coordinates[j] < 0-pedestrian_radius:
                x_coordinates[j] = w
            if i>teq:
                avg_speed += get_average_speed(dx, v, t, angle_from_index(j))

        if do_plot:
            plot(x_coordinates, y_coordinates)
        if i > teq:
            kl = kl + keep_left(y_coordinates[0:num_agents], h, direction)
            density += calc_density(x_coordinates[0:num_agents], y_coordinates[0:num_agents], w)
    return kl/(num_its-teq), avg_speed/(num_agents*(num_its-teq)), density/(num_its-teq)





if __name__ == '__main__':


    peds = range(20, 100, 10)
    kl = np.zeros(len(peds))
    avg_speed = np.zeros(len(peds))
    density = np.zeros(len(peds))

    for it, i in enumerate(peds):
        kl[it], avg_speed[it], density[it] = sim(i, 1000, 1, True)
        print(i)




    #plt.plot(peds, kl)
    #plt.plot(peds, avg_speed)
    plt.plot(peds, density)
    plt.show()
    print(kl)

