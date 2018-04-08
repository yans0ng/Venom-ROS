import numpy as np
import matplotlib.pyplot as plt

def CircleTrajectory(resolution, radius, height):
    traj = []
    theta = 0.0
    tick = 2. * np.pi / float(resolution)
    for i in range(resolution):
        theta += tick
        traj.append([radius * np.cos(theta), radius * np.sin(theta)])
    return traj

if __name__ == '__main__':
    path = CircleTrajectory(40, 3, 3)
    path = np.array(path)
    plt.plot(path[:,0], path[:,1])
    plt.axis('equal')
    plt.show()
