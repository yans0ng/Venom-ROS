# References
# https://stackoverflow.com/questions/27023068/plotting-3d-vectors-using-matplot-lib
# https://answers.ros.org/question/69754/quaternion-transformations-in-python/
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def draw_arrows(soa):
    X, Y, Z, U, V, W = zip(*soa)
    ax.quiver(X, Y, Z, U, V, W, pivot = 'tail',color='b')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

def get_rotation (quaternion):
    orientation_list = [quaternion.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print roll,pitch,yaw

def get_quaternion( n, theta):
    nx = n[0] * np.sin(theta/2.)
    ny = n[1] * np.sin(theta/2.)
    nz = n[2] * np.sin(theta/2.)
    return np.array([nx, ny, nz, np.cos(theta/2.)])


if __name__ == '__main__':
    # Initialize plot and original frame
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    soa = np.array([[0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]])
    X, Y, Z, U, V, W = zip(*soa)
    ax.quiver(X, Y, Z, U, V, W, pivot = 'tail',color='b')

    q = get_quaternion( [0., 0., 1.], -np.pi/2.)
    print q

    # Apply rotation matrix
    t = tf.TransformerROS()
    R = t.fromTranslationRotation([0,0,0],q)

    x = np.array([1,0,0,0])
    y = np.array([0,1,0,0])
    z = np.array([0,0,1,0])
    x = [0,0,0] + [m for m in R.dot(x)[:3]]
    y = [0,0,0] + [m for m in R.dot(y)[:3]]
    z = [0,0,0] + [m for m in R.dot(z)[:3]]
    soa = np.array([x,y,z])
    X, Y, Z, U, V, W = zip(*soa)
    ax.quiver(X, Y, Z, U, V, W, pivot = 'tail',color='g')

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
