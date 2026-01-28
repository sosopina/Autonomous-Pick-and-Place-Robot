import time
import ssc32
from ik_lynxmotion import ik_lynxmotion
from servo_mapping import q_to_us

# -------- SSC-32 connection --------
ssc = ssc32.Ssc32("/dev/ttyUSB0", 115200, 0.1)
ssc.verbose = True

SERVO_IDS = [0, 1, 2, 3, 4]

# -------- SAFE START --------
ssc.gotoUs(SERVO_IDS, [1500]*5, 3000)
time.sleep(4)

# -------- TARGET (meters) --------
xd = 0.20
yd = 0.00
zd = 0.10

# -------- IK --------
q = ik_lynxmotion(xd, yd, zd)
print("IK angles (rad):", q)

# -------- Convert to µs --------
us = q_to_us(q)
print("Servo µs:", us)

# -------- MOVE ROBOT --------
ssc.gotoUs(
    numServos=SERVO_IDS,
    valuesUS=us,
    timeTravelms=3000
)

ssc.waitForNextCommand()
ssc.close()
