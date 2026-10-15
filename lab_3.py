import time
import board
import busio
import adafruit_pca9685
import adafruit_bno055
import measure_servofreq_adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
servo_breakout = adafruit_pca9685.PCA9685(i2c)
sensor = adafruit_bno055.BNO055_I2C(i2c)

"""
# The commanded frequency will be 40 Hz
servo_breakout.frequency = int(40)
servo_breakout.channels[15].duty_cycle=int(1.5e-3*servo_breakout.frequency*65536.0) 
PWMfreq = measure_servofreq_adafruit_pca9685.measure_servofreq(servo_breakout,15,board.D26)
"""

# Then let’s suppose you are commanding channel #0 (LEFT OUT):
# LEFT_OUT = servo_breakout.channels[0]
# desired_left_pulsewidth_seconds = 1.5e-3
# Assign the duty cycle using the measured PWM frequency to get an

# accurate pulse width
# LEFT_OUT.duty_cycle = int(desired_left_pulsewidth_seconds * 65536* 40)

while True:
    yaw, roll, pitch = sensor.euler  # degrees
    gx, gy, gz = sensor.gyro  # rad/s
    calibrated = sensor.calibration_status  # True when fully calibrated

    print("Euler (deg)  Y: {}  R: {}  P: {}".format(yaw, roll, pitch))
    print("Gyro (rad/s) X: {}  Y: {}  Z: {}".format(gx, gy, gz))
    print("Calibrated: {}".format(calibrated))
    print()

    time.sleep(0.5)
