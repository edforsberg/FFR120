import numpy as np
import matplotlib.pyplot as plt

search_radius = 0.5
body_radius = 0.1
extra = 0.05
effective_radius = search_radius + extra

num_agents = 30
w = 10
h = 5
t = 0.01

v = 1

x_coordinates = np.zeros((num_agents,1))
y_coordinates = np.zeros((num_agents,1))
direction = np.zeros((num_agents, 1))

for i in range(num_agents):
	r =  np.random.rand()*10
	x_coordinates[i] = r
	if (r < 5):
		direction[i] = 0
	else:
		direction[i] = np.pi
	
	y_coordinates[i] = 1  +np.random.rand()*3


def try_move(x, y, d):
	desired_x = x + v*t*np.cos(d)
	desired_y = y + v*t*np.sin(d)
	return desired_x, desired_y
	

def check_for_collisions(x, y, index, d):
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
		netto_y += tmp
	
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
	
def plot(x, y):
	c = plt.scatter(x,y, c = 'b', s = 15)
	plt.gca().set_aspect('equal', adjustable='box')
	plt.xlim(-0.5, w+0.5)
	plt.ylim(-0.5, h+0.5)
	plt.show()
	plt.pause(0.001)
	c.remove()
		

if __name__ == '__main__':
	plt.ion()
	
	for i in range(300):
		for j in range(len(x_coordinates)):
			x = x_coordinates[j]
			y = y_coordinates[j]
			d = direction[j]
			desired_x, desired_y = try_move(x, y, d)
			x, y = check_for_collisions(desired_x, desired_y, j, d)
			x_coordinates[j] = x
			y_coordinates[j] = y
		
		plot(x_coordinates, y_coordinates)
	
	
	
