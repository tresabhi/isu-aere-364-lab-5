import dynamic_model
import pulseio
import board
import time
import math
import adafruit_bno055
import busio
import pwmio

MIN_INPUT = 1
MAX_INPUT = 2
DEFAULT_INPUT = (MIN_INPUT + MAX_INPUT) / 2
FREQUENCY = 40
LOOP_RATE = 60 # Hz
PWM_MAX = 65536
MAX_LIST = 10
prev_error = 0
gyrocurrent = 0

dynamic_model.enable_wind = False

def to_duty_cycle(input_ms):
    return input_ms * PWM_MAX * FREQUENCY * 1e-3

rotation_command = pulseio.PulseIn(board.D5)
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055_I2C(i2c)
left_motor = pwmio.PWMOut(board.D9, frequency=40, duty_cycle=to_duty_cycle(DEFAULT_INPUT))
right_motor = pwmio.PWMOut(board.D10, frequency=40, duty_cycle=to_duty_cycle(DEFAULT_INPUT))

# -1 = max left, 0 = stop, 1 = max right

def power(input):
    input = max(-1, min(1, input))
    left_motor.duty_cycle = to_duty_cycle(1.5 - input / 2)
    right_motor.duty_cycle = to_duty_cycle(1.5 + input / 2)

def to_rad(deg):
    return deg * math.pi / 180

def normalize_angle(rad):
    return (rad + 2 * math.pi) % (2 * math.pi)

print("Waiting for IMU to calibrate...")

while not imu.calibrated:
    time.sleep(5)
    print("IMU calibrated")
    last_theta = 0
    theta_history = []

last_time = time.monotonic()
last_theta = 0
integral = 0
KP, KI, KD = 0.5, 0.0, 0.05   # tune these

desired_theta = 0.0
last_time = time.monotonic()
integral = 0

while True:
    time.sleep(1 / LOOP_RATE)

    # 1. Update the command from the newest valid pulse (if any)
    while len(rotation_command) > 0:
        try:
            p = rotation_command.popleft()   # add * 1e-3 if the sim returns microseconds
        except IndexError:                   # sim thread emptied it between the check and the pop
            break
        if MIN_INPUT <= p <= MAX_INPUT:
            x = (p - MIN_INPUT) / (MAX_INPUT - MIN_INPUT)
            desired_theta = math.pi * (2 * x - 1)

    # 2. Read sensors
    euler, gyro = imu.euler, imu.gyro
    if euler is None or euler[0] is None or gyro is None:
        continue

    # 3. PID
    now = time.monotonic()
    dt = now - last_time
    last_time = now

    error = (desired_theta - to_rad(euler[0]) + math.pi) % (2 * math.pi) - math.pi
    error /= math.pi
    integral = max(-1, min(1, integral + error * dt))

    u = KP * error + KI * integral - KD * gyro[2]
    print("euler:", euler[0], "gyro:", gyro[2], "u:", u)
    power(u)