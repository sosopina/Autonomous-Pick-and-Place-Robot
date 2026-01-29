import time
import numpy as np
import ssc32

# =========================================================
# SSC-32 CONNECTION
# =========================================================
PORT = "/dev/ttyUSB0"
BAUD = 115200

ssc = ssc32.Ssc32(PORT, BAUD, 0.1)
ssc.verbose = True

SERVO_IDS = [0, 1, 2, 3, 4]

# =========================================================
# TEST JOINT VALUES (EDIT HERE)
# radians – SAME meaning as MATLAB FK
# =========================================================
q = np.array([
    0.0,            # q1 base
    -np.pi/6,       # q2 shoulder
    np.pi/4,        # q3 elbow
    np.pi/6,        # q4 wrist pitch
    np.pi/4         # q5 roll
])

# =========================================================
# CALIBRATION (FINAL, REAL)
# =========================================================

US_ZERO = [
    1500,   # q1 base
    900,    # q2 shoulder
    500,    # q3 elbow
    1675,   # q4 wrist pitch
    1500    # q5 roll
]

GAIN = [
    600 / (np.pi/2),     # q1 base (safe approx)
   -1700 / np.pi,       # q2 shoulder
    1800 / np.pi,       # q3 elbow
   -2100 / np.pi,       # q4 wrist pitch
    2000 / np.pi        # q5 roll
]

# Per-servo limits (IMPORTANT)
US_MIN = [900, 900, 500, 625, 1500]
US_MAX = [2100, 1750, 1400, 1675, 2500]

# =========================================================
# CONVERSION FUNCTION
# =========================================================
def q_to_us(q):
    us = []
    for i in range(len(q)):
        u = US_ZERO[i] + GAIN[i] * q[i]
        u = int(max(US_MIN[i], min(US_MAX[i], u)))
        us.append(u)
    return us

# =========================================================
# SAFE START
# =========================================================
print("Moving to safe pose...")
ssc.gotoUs(SERVO_IDS, [1500]*5, 2000)
time.sleep(3)

# =========================================================
# MOVE ROBOT
# =========================================================
us = q_to_us(q)

print("q (rad):", q)
print("servo µs:", us)

ssc.gotoUs(SERVO_IDS, us, 3000)
ssc.waitForNextCommand()
ssc.close()

print("Done.")
