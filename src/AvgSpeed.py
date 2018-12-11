import numpy as np

def get_average_speed(dx,v,t,angle):
    avg_speed = dx*v*t*np.cos(angle)
    return avg_speed

