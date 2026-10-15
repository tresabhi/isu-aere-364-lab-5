import board
import busio
import adafruit_pca9685
import measure_servofreq_adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
servo_breakout = adafruit_pca9685.PCA9685(i2c)
# The commanded frequency will be 40 Hz
servo_breakout.frequency = int(40)
# The PCA9685 has very inaccurate timing. So in calculations
# we need to use a measured frequency in order to reliably be
# able to initialize the ESC’s
# Perform measurement:
servo_breakout.channels[15].duty_cycle=int(1.5e-3*servo_breakout.frequency*65536.0) # Set a reasonable duty cycle on the frequency measurement channel
PWMfreq = measure_servofreq_adafruit_pca9685.measure_servofreq(
servo_breakout,15,board.D26) # Measure actual servo frequency
# Then let’s suppose you are commanding channel #0 (LEFT OUT):
LEFT_OUT = servo_breakout.channels[0]
desired_left_pulsewidth_seconds = 1.5e-3
# Assign the duty cycle using the measured PWM frequency to get an
# accurate pulse width
LEFT_OUT.duty_cycle = int(desired_left_pulsewidth_seconds * 65536*PWMfreq
