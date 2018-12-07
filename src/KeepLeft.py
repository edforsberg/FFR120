import numpy as np

def keep_left(ys, h, angles):
	tmp = np.sign(ys-h/2)*np.cos(angles)
	average_keep_left = np.sum(tmp)/len(ys)
	return average_keep_left
