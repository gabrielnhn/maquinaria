import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime
from DC_Motor_pi import DC_Motor
# hardware pwm: 12, 32, 33, 35

CLOCKWISE = 1
COUNTERCLOCKWISE = -1

KNOB_NUMBER = 11

# motor pins setup
clockwise_pin = 11
counterclockwise_pin = 13
pwm_pin = 12
motor = DC_Motor(clockwise_pin, counterclockwise_pin, pwm_pin)

# encoder pin setuo
encoder_a = 14
encoder_b = 15 
GPIO.setup(encoder_a, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(encoder_b, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

# init state variables
last_a_state = GPIO.input(encoder_a)
counter = 0
current_dir = 0
knob_counter = 0

last_ts = datetime.timestamp(datetime.now())

i = 0   # motor speed
while i <= 100:
    motor.forward(i)

    current_a_state = GPIO.input(encoder_a)
    b_state = GPIO.input(encoder_b)
    
    # check if encoder detected a turn
    if current_a_state != last_a_state  and current_a_state == 1:
        knob_counter += 1

        # check direction 
        if b_state != current_a_state:
            current_dir = COUNTERCLOCKWISE
        else:
            current_dir = CLOCKWISE

    last_a_state = current_a_state

    # output string
    dir_name = "horário" if current_dir == CLOCKWISE else "anti-horário"
    rotations = knob_counter // KNOB_NUMBER
    print(f"Direção: {dir_name}, Contador: {knob_counter}, Rotações {rotations}")

    # increment motor speed each second
    curr_ts = datetime.timestamp(datetime.now())
    if (curr_ts - last_ts) > 1.0:
        i += 1
        last_ts = curr_ts