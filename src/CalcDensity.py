import numpy as np
from scipy.spatial import distance_matrix

def calc_density(xs, ys, w):

    npts = len(xs)
    counter = 0

    for i in range(npts):
            tmp = abs(xs[i] - xs[0:i])
            tmp = np.minimum(tmp, w-tmp)**2
            counter += np.sum(tmp + (ys[i] - ys[0:i])**2)

    counter = 2*counter/(npts*(npts-1))

    return counter