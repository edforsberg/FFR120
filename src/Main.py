import numpy as np
import matplotlib.pyplot as plt


x_max = 10
y_max = 5
p_max = np.array([x_max, y_max])

max_num_agents = 100
num_agents = 100
data = np.zeros((max_num_agents, 4))
data[0:num_agents, [0, 1]] = (np.random.random(size=(num_agents, 2)) * p_max * 2-p_max)*0.9

def get_drift_velocty(i):
    drift_velocity = 0.1
    if i<=num_agents/2:
        return np.array([drift_velocity, 0])
    else:
        return np.array([-drift_velocity, 0])

def get_closest_wall_point(p):
    if p[1] >= 0:
        return np.array([p[0], y_max])
    else:
        return np.array([p[0], -y_max])



agent_repulsion_strength = 0.01
wall_repulsion_strength = 0.05

def update_agents(data):
    for i in range(num_agents):
        p = data[i, [0, 1]]
        v = data[i, [2, 3]]
        v += get_drift_velocty(i)
        max_force = 0
        for j in range(num_agents):
            if j == i:
                continue
            diff = p - data[j, [0, 1]]
            len_sq = np.dot(diff, diff)
            f = diff / (len_sq*np.sqrt(len_sq)) * agent_repulsion_strength
            if np.dot(f, f) > np.dot(max_force, max_force):
                max_force = f

        diff = p-get_closest_wall_point(p)
        f = diff / (len_sq * np.sqrt(len_sq)) * wall_repulsion_strength
        if np.dot(f, f) > np.dot(max_force, max_force):
            max_force = f
        v += max_force
        p += v
        v *= 0.5

        p[0] = (p[0] + x_max) % (x_max*2) - x_max

        data[i, [0, 1]] = p
        data[i, [2, 3]] = v

def plot_agents(data):
    x = data[0:num_agents, 0]
    y = data[0:num_agents, 1]
    plt.cla()
    plt.scatter(x, y)
    plt.xlim(-x_max, x_max)
    plt.ylim(-y_max, y_max)

    plt.show()
    plt.pause(.001)


def main():
    plt.ion()
    for i in range(100):
        update_agents(data)
        plot_agents(data)




if __name__ == "__main__":
    main()

