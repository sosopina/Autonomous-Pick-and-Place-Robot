import numpy as np

# -------- Servo limits --------
US_MIN = 900
US_MAX = 2100

# -------- Gain (from your tests) --------
# ±45° = ±600 µs
GAIN_US_RAD = 600 / (np.pi / 4)

# -------- ZERO OFFSETS (IMPORTANT) --------
# q = 0 rad corresponds to THESE µs values
US_ZERO = [
    1500,  # q1 base (CONFIRMED)
    1500,  # q2 shoulder (CONFIRMED)
    1500,  # q3 elbow  (TEMP – adjust later if needed)
    1500,  # q4 wrist pitch
    1500   # q5 wrist roll / pince
]

# -------- SIGNS --------
SIGN = [
    +1,   # q1 base
    -1,   # q2 shoulder (BACKWARDS – CONFIRMED)
    +1,   # q3 elbow
    +1,   # q4 wrist
    +1    # q5 roll
]


def q_to_us(q):
    """
    Convert joint angles (rad) → SSC-32 microseconds
    """
    us = []
    for i in range(len(q)):
        u = US_ZERO[i] + SIGN[i] * GAIN_US_RAD * q[i]
        u = int(max(US_MIN, min(US_MAX, u)))
        us.append(u)
    return us
