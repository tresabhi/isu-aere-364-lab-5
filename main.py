import dynamic_model
import pulseio
import board
import time
import math
import adafruit_bno055
import busio
import pwmio
import adafruit_bmp3xx

"""
Constants are assigned here for usage throughout the whole code not
global variables though.
"""
MIN_INPUT = 1.0 #ms
MAX_INPUT = 2.0 #ms
NEUTRAL = 1.5 #ms
FREQUENCY = 40
LOOP_RATE = 60 #hz
PWM_MAX = 65536
I_LIMIT = 0.05
HEIGHT_MAX = 8.0 # In meters

"""
Pins are assigned and can be changed SCL, SDA are used for the IMU and
ultrasonic sensor. Motors use pins 9-12. Order might need to change
based on how the motors are wired to the pi. 
"""
def to_duty_cycle(input_ms):
    return int(input_ms * PWM_MAX * FREQUENCY * 1e-3)

rotation_command = pulseio.PulseIn(board.D5)
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055_I2C(i2c)
board.bmp3xx_i2c = True
altimiter = adafruit_bmp3xx.BMP3XX_I2C(i2c)
left_motor1 = pwmio.PWMOut(board.D9, frequency=FREQUENCY, duty_cycle=to_duty_cycle(NEUTRAL))
right_motor1 = pwmio.PWMOut(board.D10, frequency=FREQUENCY, duty_cycle=to_duty_cycle(NEUTRAL))
left_motor2 = pwmio.PWMOut(board.D11, frequency=FREQUENCY, duty_cycle=to_duty_cycle(NEUTRAL))
right_motor2 = pwmio.PWMOut(board.D12, frequency=FREQUENCY, duty_cycle=to_duty_cycle(NEUTRAL))


# NOTE: PID class is assgined created here. Used for all loops
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0.0

    def reset(self):
        self.i = 0.0

    def clamp(value, low, high):
        return max(low, min(high, value))
    
    def update(self, error, rate, dt):
        self.i = PID.clamp(self.i + self.ki * error * dt, -I_LIMIT, I_LIMIT)
        return self.kp * error - self.kd * rate + self.i

# NOTE functions etablisjing control for the motors 
def set_motor(motor, cmd):
    motor.duty_cycle = to_duty_cycle(NEUTRAL + 0.5 * PID.clamp(cmd, -1, 1))

def mix(throttle, roll, pitch, yaw):
    set_motor(left_motor1, throttle - roll - pitch - yaw)
    set_motor(right_motor1, throttle + roll + pitch - yaw)
    set_motor(left_motor2, throttle - roll + pitch + yaw)
    set_motor(right_motor2, throttle + roll - pitch + yaw)

# NOTE Angle wrap function for roll, pitch, yaw
def wrap(error):
    return ((error + math.pi) % (2 * math.pi) - math.pi) / math.pi



""" PID Variables are commanded here """
yaw_pid = PID(0.0, 0.0, 0.0)
roll_pid = PID(0.0, 0.0, 0.0)
pitch_pid = PID(0.0, 0.0, 0.0)
height_pid = PID(0.85, 0.0, 0.85)

""" Requirement for the simulation to read the IMU """
print("Waiting for IMU to calibrate...")
while not imu.calibrated:
    time.sleep(1)
print("IMU calibrated")


""" Setting Zeros so that no errors are thrown in the while loop """
cmd_height = 0.0
target_height = 0.0
climb = 0.0
target_yaw = math.radians(imu.euler[0])
last_time = time.monotonic()
last_alt = altimiter.altitude
last_alt_time = last_time
target_height = 0.0

while True:
    # NOTE: Set sleep point
    time.sleep(1 / LOOP_RATE)

    # NOTE: Grabs newest possible commands. Ensures the range is between desired
    while len(rotation_command) > 0:
        try:
            p = rotation_command.popleft() * 1e-3
        except IndexError:
            break
        if MIN_INPUT <= p <= MAX_INPUT:
            x = (p - MIN_INPUT) / (MAX_INPUT - MIN_INPUT)
            cmd_height = HEIGHT_MAX * x

    # NOTE: Assing values to ueler anfles and gyro
    euler, gyro , acceleration = imu.euler, imu.gyro, imu.acceleration

    # NOTE: dt calulation
    now = time.monotonic()
    dt = max(now - last_time, 1e-3)
    last_time = now

    # NOTE: Error calculations. Normalization to account for wrapping
    error_yaw = wrap(target_yaw - math.radians(euler[0]))
    error_roll = wrap(0.0 - math.radians(euler[1]))
    error_pitch = wrap(0.0 - math.radians(euler[2])) 

    yaw_cmd = yaw_pid.update(error_yaw, gyro[2], dt)
    roll_cmd = roll_pid.update(error_roll, gyro[0], dt)
    pitch_cmd = pitch_pid.update(error_pitch, gyro[1], dt)

    
    # Throttle limit for now
    alt = altimiter.altitude
    if alt != last_alt:
        climb = (alt - last_alt) / (now - last_alt_time)
        last_alt, last_alt_time = alt, now

    step = 0.1 * dt
    target_height += PID.clamp(cmd_height - target_height, -step, step)
    error_height = target_height - alt
    height_cmd = height_pid.update(error_height, climb, dt)

    mix(height_cmd, roll_cmd, pitch_cmd, yaw_cmd)
    print("Altitiude:", alt)