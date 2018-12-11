import numpy as np
from scipy.spatial import distance_matrix

def calc_density(xs, ys):

    npts = len(xs)
    counter = 0

    for i in range(npts):
            counter += np.sum((xs[i] - xs[0:i])**2 + (ys[i] - ys[0:i])**2)

    counter = 2*counter/(npts*(npts-1))

    return counter