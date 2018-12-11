import numpy as np
from scipy.spatial import distance_matrix

def calc_density(xs, ys):

npts = len(xs)
dist__mat = np.zeros(npts)
counter = 0

for i in range(npts):
    for j in range(i):
        counter += np.sqrt((xs(i)-xs(j))^2 + (ys(i)-ys(j))^2)

counter = counter/(npts*(npts-1))

return counter