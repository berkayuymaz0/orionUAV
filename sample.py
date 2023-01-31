# Rotate servo motor 180 degrees and back to 0
import time
import Adafruit_PCA9685

# Initialize the PCA9685 using the default address
# PCA9685_address = 0x40
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

# Set the servo to the desired angle
def set_servo_angle(angle):
    pwm.set_pwm(angle, 0, servo_min + (servo_max - servo_min) * angle / 180)

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set servo to 0 degrees
set_servo_angle(0)

# Rotate the servo 180 degrees
for i in range(180, 0, -1):
    set_servo_angle(i)
    time.sleep(0.001)

# Rotate back to 0 degrees
for i in range(0, 180):
    set_servo_angle(i)
    time.sleep(0.001)


#sample code of servo motor movements for Jetson 

# Import the necessary libraries
import RPi.GPIO as GPIO
import time

# Setup the GPIO Pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)

# Set the frequency of PWM
freq = 50

# Setup the servo motor
servo = GPIO.PWM(7, freq)
servo.start(0)

# Function to move servo
def move_servo(angle):
    duty = float(angle) / 10.0 + 2.5
    servo.ChangeDutyCycle(duty)
    time.sleep(0.3)
    servo.ChangeDutyCycle(0)

# Move servo to 90 degrees
move_servo(90)

# Cleanup the GPIO pins
GPIO.cleanup()


#sample code of buzzer
# Import the GPIO and time library
import time
import RPi.GPIO as GPIO

# Set pins
buzzer_pin = 21

# Set board mode 
GPIO.setmode(GPIO.BCM)

# Set buzzer pin as output
GPIO.setup(buzzer_pin, GPIO.OUT)

# Main section
try:
    while True:
        GPIO.output(buzzer_pin, GPIO.HIGH)
        time.sleep(0.5)  # Delay in seconds
        GPIO.output(buzzer_pin, GPIO.LOW)
        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()

#sample code buzzer 

import Jetson.GPIO as GPIO

# choose which pin to use
pin = 12

# set pin to use GPIO as output
GPIO.setup(pin, GPIO.OUT)

# loop while buzzer is on
while True:
    GPIO.output(pin, 1)
    time.sleep(.25)
    GPIO.output(pin, 0)
    time.sleep(.25)