import numpy as np

# -------- Robot geometry (meters) --------
L1 = 0.05
L2 = 0.15
L3 = 0.15
L4 = 0.04
Lp = 0.05


def ik_lynxmotion(xd, yd, zd, theta_des=np.pi/2):
    """
    Paul type-6 IK
    Elbow-UP
    Gripper always pointing DOWN
    """

    # --- q1 ---
    q1 = np.arctan2(yd, xd)

    # --- target in frame 1 ---
    Rz = np.array([
        [np.cos(q1), -np.sin(q1), 0],
        [np.sin(q1),  np.cos(q1), 0],
        [0,           0,          1]
    ])

    p0 = np.array([xd, yd, zd])
    p1 = Rz.T @ (p0 - np.array([0, 0, L1]))

    x1 = p1[0]
    z1 = p1[2]

    # --- wrist center ---
    Le = L4 + Lp
    xw = x1 - Le * np.cos(theta_des)
    zw = z1 + Le * np.sin(theta_des)

    # --- planar 2R (Paul) ---
    x = xw
    y = -zw

    D = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if abs(D) > 1:
        raise ValueError("Target unreachable")

    # COUDE HAUT
    q3 = np.arctan2(+np.sqrt(1 - D**2), D)

    q2 = np.arctan2(y, x) - np.arctan2(
        L3 * np.sin(q3),
        L2 + L3 * np.cos(q3)
    )

    # --- orientation constraint ---
    q4 = theta_des - q2 - q3

    # roll
    q5 = 0.0

    return np.array([q1, q2, q3, q4, q5])
