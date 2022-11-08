import RPi.GPIO as GPIO
import time
from DC_Motor_pi import DC_Motor
# hardware pwm: 12, 32, 33, 35

# pins setup
clockwise_pin_1 = 11
counterclockwise_pin_1 = 13
pwm_pin_1 = 12

clockwise_pin_2 = 16
counterclockwise_pin_2 = 15
pwm_pin_2 = 18

motor_1 = DC_Motor(clockwise_pin_1, counterclockwise_pin_1, pwm_pin_1)

motor_2 = DC_Motor(clockwise_pin_2, counterclockwise_pin_2, pwm_pin_2)

try:
    motor_1.forward(40)
    motor_2.forward(40)
    time.sleep(0.1)

    while True:
        motor_1.forward(15)
        motor_2.forward(15)

        # motor_1.forward(40)
        # motor_2.forward(40)
    

except:
    del motor_1
    del motor_2
    GPIO.cleanup()
