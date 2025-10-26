import numpy as np
import matplotlib.pyplot as plt

def deg2rad(d):
    return np.deg2rad(d)

def forward_kinematics_3dof(theta1_deg, theta2_deg, theta3_deg, L1, L2, L3):
    t1 = deg2rad(theta1_deg)
    t2 = deg2rad(theta2_deg)
    t3 = deg2rad(theta3_deg)
    
    t12 = t1 + t2
    t123 = t1 + t2 + t3

    x = L1 * np.cos(t1) + L2 * np.cos(t12) + L3 * np.cos(t123)
    y = L1 * np.sin(t1) + L2 * np.sin(t12) + L3 * np.sin(t123)

    # Homogeneous matrices
    T01 = np.array([
        [np.cos(t1), -np.sin(t1), L1 * np.cos(t1)],
        [np.sin(t1),  np.cos(t1), L1 * np.sin(t1)],
        [0,           0,          1]
    ])
    T02 = np.array([
        [np.cos(t12), -np.sin(t12), x - L3 * np.cos(t123)],
        [np.sin(t12),  np.cos(t12), y - L3 * np.sin(t123)],
        [0,            0,           1]
    ])
    T03 = np.array([
        [np.cos(t123), -np.sin(t123), x],
        [np.sin(t123),  np.cos(t123), y],
        [0,             0,            1]
    ])
    return (x, y), T01, T02, T03

def inverse_kinematics_3dof(x, y, L1, L2, L3):
    r = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    
    xe = x - L3 * np.cos(phi)
    ye = y - L3 * np.sin(phi)

    r2 = xe**2 + ye**2
    if r2 > (L1 + L2)**2:
        raise ValueError("Target di luar jangkauan!")

    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(ye, xe) - np.arctan2(k2, k1)

    theta3 = phi - (theta1 + theta2)

    return np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)

def plot_robot_3dof(L1, L2, L3, x, y, theta1_deg, theta2_deg, theta3_deg):
    t1 = deg2rad(theta1_deg)
    t2 = deg2rad(theta2_deg)
    t12 = t1 + t2

    j1 = (L1 * np.cos(t1), L1 * np.sin(t1))
    j2 = (j1[0] + L2 * np.cos(t12), j1[1] + L2 * np.sin(t12))

    plt.figure(figsize=(7,7))
    plt.plot([0, j1[0], j2[0], x], [0, j1[1], j2[1], y], 'ro-', linewidth=3, markersize=10)
    plt.grid(True)
    plt.axis('equal')
    plt.title('Forward Kinematics 3-DoF Planar')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig('fk_3dof_result.png', dpi=150)
    plt.show()

# MAIN
if __name__ == "__main__":
    print("=== Forward Kinematics 3-DoF ===")
    L1 = float(input("Panjang femur (L1): ") or 10)
    L2 = float(input("Panjang tibia (L2): ") or 72)
    L3 = float(input("Panjang tarsus (L3): ") or 20)

    th1 = float(input("Sudut servo 1 (θ1): ") or 40)
    th2 = float(input("Sudut servo 2 (θ2): ") or 30)
    th3 = float(input("Sudut servo 3 (θ3): ") or -70)

    (x, y), T01, T02, T03 = forward_kinematics_3dof(th1, th2, th3, L1, L2, L3)
    print(f"\nEnd-effector: X = {x:.3f}, Y = {y:.3f}")
    print("\nT0→3 (Homogeneous Matrix):")
    print(T03)

    plot_robot_3dof(L1, L2, L3, x, y, th1, th2, th3)

    # Inverse Kinematics
    xt, yt = 50, 30
    ik1, ik2, ik3 = inverse_kinematics_3dof(xt, yt, L1, L2, L3)
    print(f"Hasil IK: θ1 = {ik1:.2f}°, θ2 = {ik2:.2f}°, θ3 = {ik3:.2f}°")
    plot_robot_3dof(L1, L2, L3, xt, yt, ik1, ik2, ik3)
