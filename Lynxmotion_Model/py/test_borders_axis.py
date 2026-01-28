import ssc32
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

ssc = ssc32.Ssc32(PORT, BAUD, 0.1)
ssc.verbose = True

# Include gripper as last servo (adjust if needed)
SERVO_IDS = [0, 1, 2, 3, 4, 5]

CENTER = 1500
STEP = 50
DELAY = 0.6
MIN_US = 1000
MAX_US = 2000


def test_servo(ch):
    print("\n==============================")
    print(f"Testing SERVO {ch}")
    print("==============================")

    # Go to center before test
    print(f"Servo {ch}: go to center {CENTER} us")
    ssc.gotoUs([ch], [CENTER], 1500)
    time.sleep(2)

    # + direction
    print(f"Servo {ch}: testing + direction")
    us = CENTER
    while us + STEP <= MAX_US:
        us += STEP
        print(f"  -> {us} us")
        ssc.gotoUs([ch], [us], 500)
        time.sleep(DELAY)

        stop = input("Press ENTER to continue, type 's' to stop + direction: ")
        if stop.lower() == 's':
            print(f"  STOP + at {us} us")
            break

    # Return to center
    ssc.gotoUs([ch], [CENTER], 1000)
    time.sleep(1)

    # - direction
    print(f"Servo {ch}: testing - direction")
    us = CENTER
    while us - STEP >= MIN_US:
        us -= STEP
        print(f"  -> {us} us")
        ssc.gotoUs([ch], [us], 500)
        time.sleep(DELAY)

        stop = input("Press ENTER to continue, type 's' to stop - direction: ")
        if stop.lower() == 's':
            print(f"  STOP - at {us} us")
            break

    print(f"Servo {ch}: test finished")


# ===============================
# SEQUENTIAL TESTING
# ===============================

# --- SERVO 0 ---
test_servo(0)
ssc.gotoUs([0], [CENTER], 1500)
time.sleep(2)

# --- SERVO 1 ---
test_servo(1)

# IMPORTANT: leave servo 1 at 900 before continuing
print("Locking SERVO 1 at 900 us")
ssc.gotoUs([1], [MIN_US], 1500)
time.sleep(2)

# --- SERVO 2 ---
test_servo(2)
ssc.gotoUs([2], [CENTER], 1500)
time.sleep(2)

# --- SERVO 3 ---
test_servo(3)
ssc.gotoUs([3], [CENTER], 1500)
time.sleep(2)

# --- SERVO 4 ---
test_servo(4)
ssc.gotoUs([4], [CENTER], 1500)
time.sleep(2)

# --- SERVO 5 (GRIPPER / PINCE) ---
test_servo(5)
ssc.gotoUs([5], [CENTER], 1500)
time.sleep(2)

# ===============================
# FINAL SAFE POSE
# ===============================
print("\nAll tests done. Returning all servos to CENTER.")
ssc.gotoUs(SERVO_IDS, [CENTER]*len(SERVO_IDS), 3000)
ssc.close()
