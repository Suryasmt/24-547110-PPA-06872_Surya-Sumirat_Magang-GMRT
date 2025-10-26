import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def deg2rad(d):
    return np.deg2rad(d)

def forward_kinematics_3d(theta0, theta1, theta2, L1, L2):
    t0 = deg2rad(theta0)
    t1 = deg2rad(theta1)
    t2 = deg2rad(theta2)
    t12 = t1 + t2

    r = L1 * np.cos(t1) + L2 * np.cos(t12)
    x = r * np.cos(t0)
    y = r * np.sin(t0)
    z = L1 * np.sin(t1) + L2 * np.sin(t12)

    return x, y, z

def plot_3d_fk(x, y, z, L1, L2, th0, th1, th2):
    t0 = deg2rad(th0)
    t1 = deg2rad(th1)
    t12 = t1 + deg2rad(th2)

    r1 = L1 * np.cos(t1)
    x1 = r1 * np.cos(t0)
    y1 = r1 * np.sin(t0)
    z1 = L1 * np.sin(t1)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([0, x1, x], [0, y1, y], [0, z1, z], 'ro-', linewidth=3, markersize=8)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Forward Kinematics 3D (3-DoF)')
    plt.savefig('fk_3d_result.png', dpi=150)
    plt.show()

if __name__ == "__main__":
    L1 = 10
    L2 = 72

    theta0 = float(input("Sudut coxa (θ0, derajat): ") or 0)
    theta1 = float(input("Sudut femur (θ1, derajat): ") or 40)
    theta2 = float(input("Sudut tibia (θ2, derajat): ") or 30)

    x, y, z = forward_kinematics_3d(theta0, theta1, theta2, L1, L2)
    print(f"Posisi 3D: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

    plot_3d_fk(x, y, z, L1, L2, theta0, theta1, theta2)