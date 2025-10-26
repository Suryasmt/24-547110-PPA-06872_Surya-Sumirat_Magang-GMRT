import numpy as np
import matplotlib.pyplot as plt

import matplotlib
matplotlib.use('TkAgg')

def deg2rad(d):
    return np.deg2rad(d)

def forward_kinematics_2dof(theta1_deg, theta2_deg, L1, L2):
    t1 = deg2rad(theta1_deg)
    t2 = deg2rad(theta2_deg)
    t12 = t1 + t2

    x = L1 * np.cos(t1) + L2 * np.cos(t12)
    y = L1 * np.sin(t1) + L2 * np.sin(t12)

    # Homogeneous Transformation Matrices
    T01 = np.array([
        [np.cos(t1), -np.sin(t1), L1 * np.cos(t1)],
        [np.sin(t1),  np.cos(t1), L1 * np.sin(t1)],
        [0,           0,          1]
    ])
    T02 = np.array([
        [np.cos(t12), -np.sin(t12), x],
        [np.sin(t12),  np.cos(t12), y],
        [0,            0,           1]
    ])
    return (x, y), T01, T02

def inverse_kinematics_2dof(x, y, L1, L2):
    r2 = x**2 + y**2
    if r2 > (L1 + L2)**2:
        raise ValueError("Target di luar jangkauan robot!")
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    return np.rad2deg(theta1), np.rad2deg(theta2)

def plot_robot(L1, L2, x, y, theta1_deg):
    t1 = deg2rad(theta1_deg)
    joint1 = (L1 * np.cos(t1), L1 * np.sin(t1))
    plt.figure(figsize=(7, 7))
    plt.plot([0, joint1[0], x], [0, joint1[1], y], 'ro-', linewidth=3, markersize=10)
    plt.grid(True, alpha=0.4)
    plt.axis('equal')
    plt.title('Forward Kinematics – 2-DoF Robot Leg')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig('fk_result.png', dpi=150)
    plt.show(block=True)

# === MAIN PROGRAM ===
if __name__ == "__main__":
    print("=== Forward Kinematics 2-DoF ===")
    L1 = float(input("Panjang femur (L1): ") or 10)
    L2 = float(input("Panjang tibia (L2): ") or 72)
    th1 = float(input("Sudut servo 1 (θ1 dalam derajat): ") or 40)
    th2 = float(input("Sudut servo 2 (θ2 dalam derajat): ") or 30)

    (x, y), T01, T02 = forward_kinematics_2dof(th1, th2, L1, L2)
    print(f"\nPosisi end-effector: X = {x:.3f}, Y = {y:.3f}")
    print("\nMatriks Homogen T0→1:")
    print(T01)
    print("\nMatriks Homogen T0→2 (end-effector):")
    print(T02)

    plot_robot(L1, L2, x, y, th1)

    # Inverse Kinematics
    xt, yt = 50, 30
    print(f"\n=== Inverse Kinematics untuk target ({xt}, {yt}) ===")
    try:
        ik1, ik2 = inverse_kinematics_2dof(xt, yt, L1, L2)
        print(f"Hasil IK: θ1 = {ik1:.2f}°, θ2 = {ik2:.2f}°")
    except ValueError as e:
        print("Error:", e)