#!/usr/bin/env python

import scipy.optimize
import numpy as np

def load_measurements(filename):
    measurements = []

    with open(filename, 'r') as csv_file:
        for line in csv_file.readlines():
            values = [float(v) for v in line.split(',')]

            tracker_pose = np.reshape(values[:16], (4, 4))
            tool_pose = np.reshape(values[16:], (4, 4))

            measurements.append((tracker_pose, tool_pose))

    return measurements

def make_4x4_from_3x4(mat_3x4):
    mat_4x4 = np.identity(4)
    mat_4x4[0:3, :] = np.reshape(mat_3x4, (3, 4))[:]
    return mat_4x4

def fitness(transform, measurements):
    transform_mat = make_4x4_from_3x4(transform)
 
    score = 0

    for tracker_pose, tool_pose in measurements:
        guess = np.matmul(tracker_pose, transform_mat)
        score += np.sum((guess - tool_pose) ** 2)

    return score

"""
def main():
    measurements = load_measurements('measurements.csv')
    result = scipy.optimize.minimize(fitness, np.ndarray.flatten(np.identity(4)), args=(measurements,), tol=0.001, method='BFGS', options={'disp': True})
    
    if not result.success:
        print('Optimization failed: {}'.format(result.message))
        return

    print('Optimization succeeded after {} iterations.'.format(result.nit))
    print('Fitness is {}'.format(fitness(result.x, measurements)))
    print('Solution:')
    print(repr(np.reshape(result.x, (4, 4))))
    """

def main():
    measurements = load_measurements('measurements.csv')

    result = scipy.optimize.differential_evolution(fitness, [(-10, 10)] * 12, args=(measurements,), atol=1, tol=0, polish=True)

    if not result.success:
        print('Optimization failed: {}'.format(result.message))
        return

    print('Optimization succeeded after {} iterations.'.format(result.nit))
    print('Fitness is {}'.format(fitness(result.x, measurements)))
    print('Solution:')
    print(repr(make_4x4_from_3x4(np.reshape(result.x, (4, 3)))))

main()

