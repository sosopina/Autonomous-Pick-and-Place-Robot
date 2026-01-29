import time
import numpy as np
import ssc32

L1 = 0.04
L2 = 0.15
L3 = 0.18
L4 = 0.035
Lp = 0.04
Le = L4 + Lp

theta_des = np.pi / 2
q5_des = 0.0

xd = 0.05
yd = 0.05
zd = 0.07

q1 = np.arctan2(yd, xd)

Rz = np.array([
    [np.cos(q1), -np.sin(q1), 0],
    [np.sin(q1),  np.cos(q1), 0],
    [0,           0,          1]
])

p0 = np.array([xd, yd, zd])
p1 = Rz.T @ (p0 - np.array([0, 0, L1]))

x1 = p1[0]
z1 = p1[2]

xw = x1 - Le * np.cos(theta_des)
zw = z1 + Le * np.sin(theta_des)

x = xw
y = -zw

D = (x*x + y*y - L2*L2 - L3*L3) / (2*L2*L3)
if abs(D) > 1:
    raise RuntimeError(f"IK failed: |D|>1 (D={D:.3f})")

q3 = np.arctan2(np.sqrt(1 - D*D), D)

q2 = np.arctan2(y, x) - np.arctan2(
    L3 * np.sin(q3),
    L2 + L3 * np.cos(q3)
)

q4 = theta_des - q2 - q3
q5 = q5_des

q = np.array([q1, q2, q3, q4, q5])
print("\nIK solution (rad):")
print(q)

def q2_to_us(q2):
    if q2 >= -np.pi/2:
        return 900 + (1750 - 900) * (q2 / (-np.pi/2))
    else:
        return 1750 + (2500 - 1750) * ((q2 + np.pi/2) / (-np.pi/2))

def q_to_us(q):
    us = [0]*5

    #  FIXED q1 mapping (YOUR CALIBRATION)
    # 0->1500, +pi/2->500, -pi/2->2500
    us[0] = 1500 - (2000/np.pi) * q[0]

    us[1] = q2_to_us(q[1])

    # q3: 0 -> 500, pi/2 -> 1400
    us[2] = 500 + (1400 - 500) * (q[2] / (np.pi/2))

    # q4: 0 -> 1675, pi/2 -> 625
    us[3] = 1675 + (625 - 1675) * (q[3] / (np.pi/2))

    # q5: 0 -> 1500, pi/2 -> 2500
    us[4] = 1500 + (2500 - 1500) * (q[4] / (np.pi/2))

    US_MIN = [500, 900, 500, 625, 1500]
    US_MAX = [2500, 2500, 2100, 1675, 2500]

    for i in range(5):
        us[i] = int(max(US_MIN[i], min(US_MAX[i], us[i])))

    return us

us = q_to_us(q)
print("\nServo commands (µs):")
print(us)

ssc = ssc32.Ssc32("/dev/ttyUSB0", 115200, 0.1)
ssc.verbose = True

# your home pose
HOME_US = [1500, 2100, 1800, 1500, 1500]

# dummy command to avoid first jerk
ssc.gotoUs([0,1,2,3,4], HOME_US, 2000)
time.sleep(6)

ssc.gotoUs([0,1,2,3,4], us, 2000)
ssc.waitForNextCommand()
ssc.close()

print("\nDONE.")
