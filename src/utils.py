
import numpy as np


def rotate(theta, point):
    '''
    Rotate about the y-axis. Clockwise is positive, counter is negative.
    '''
    mat = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    return np.dot(mat, point)

def get_normalized_vec(vector):
    '''
    Normalize a vector relative to its maximum value.
    '''
    max_val = np.max(np.abs(vector))
    if max_val != 0: return vector/max_val
    else: return vector

def wrap_print(text):
    print('\n')
    print('-'*50)
    print(text)
    print('-'*50)
    print('\n')