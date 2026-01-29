import time
import numpy as np
import ssc32

# =========================================================
# GEOMETRY (EXACTLY AS MATLAB)
# =========================================================
L1 = 0.04
L2 = 0.15
L3 = 0.18
L4 = 0.035
Lp = 0.05
Le = L4 + Lp

theta_des = np.pi / 2
q5_des = 0.0

# =========================================================
# TARGET (PINCE TIP)
# =========================================================
xd = 0.3
yd = 0
zd = 0.05

# =========================================================
# INVERSE KINEMATICS (COPY-PASTE FROM MATLAB LOGIC)
# =========================================================

# --- STEP 1: q1 ---
q1 = np.arctan2(yd, xd)

# --- STEP 2: target in frame 1 ---
Rz = np.array([
    [np.cos(q1), -np.sin(q1), 0],
    [np.sin(q1),  np.cos(q1), 0],
    [0,           0,          1]
])

p0 = np.array([xd, yd, zd])
p1 = Rz.T @ (p0 - np.array([0, 0, L1]))

x1 = p1[0]
z1 = p1[2]

# --- STEP 3: wrist center ---
xw = x1 - Le * np.cos(theta_des)
zw = z1 + Le * np.sin(theta_des)

# --- STEP 4: planar 2R ---
x = xw
y = -zw

D = (x*x + y*y - L2*L2 - L3*L3) / (2*L2*L3)

if abs(D) > 1:
    raise RuntimeError("IK failed: |D| > 1")

# COUDE HAUT (EXACT MATCH MATLAB)
q3 = np.arctan2(np.sqrt(1 - D*D), D)

q2 = np.arctan2(y, x) - np.arctan2(
    L3 * np.sin(q3),
    L2 + L3 * np.cos(q3)
)

# --- STEP 5: orientation constraint ---
q4 = theta_des - q2 - q3
q5 = q5_des

q = np.array([q1, q2, q3, q4, q5])

print("IK solution (rad):")
print(q)

# =========================================================
# CALIBRATION (YOUR WORKING ONE)
# =========================================================
US_ZERO = [1500, 900, 500, 1675, 1500]
GAIN = [
    600/(np.pi/2),
   -1700/np.pi,
    1800/np.pi,
   -2100/np.pi,
    2000/np.pi
]
US_MIN = [900, 500, 500, 625, 1500]
US_MAX = [2100, 1750, 1400, 1675, 2500]

def q_to_us(q):
    us = []
    for i in range(5):
        u = US_ZERO[i] + GAIN[i] * q[i]
        u = int(max(US_MIN[i], min(US_MAX[i], u)))
        us.append(u)
    return us

us = q_to_us(q)

print("Servo µs:")
print(us)

# =========================================================
# SEND TO ROBOT
# =========================================================
ssc = ssc32.Ssc32("/dev/ttyUSB0", 115200, 0.1)
ssc.verbose = True

ssc.gotoUs([0,1,2,3,4], [1500]*5, 2000)
time.sleep(3)

ssc.gotoUs([0,1,2,3,4], us, 3000)
ssc.waitForNextCommand()
ssc.close()
