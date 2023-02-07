import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime
from DC_Motor_pi import DC_Motor

# hardware pwm: 12, 32, 33, 35

CLOCKWISE = 1
COUNTERCLOCKWISE = -1

KNOB_NUMBER = 460

# motor pins setup
clockwise_pin = 16
counterclockwise_pin = 15
pwm_pin = 18
motor = DC_Motor(clockwise_pin, counterclockwise_pin, pwm_pin)

# encoder pin setuo
encoder_a = 19
encoder_b = 21
GPIO.setup(encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# init state variables
last_a_state = GPIO.input(encoder_a)
counter = 0
current_dir = 0
knob_counter = 0

last_ts = datetime.timestamp(datetime.now())
start_ts = last_ts

i = 1  # motor speed
while i < 100:
    if (i > 30):
        motor.forward(10)
    else:
        motor.backwards(10)
    #motor.forward(100)

    current_a_state = GPIO.input(encoder_a)
    b_state = GPIO.input(encoder_b)

    # check if encoder detected a turn
    if current_a_state != last_a_state and current_a_state == 1:
        knob_counter += 1

        # check direction
        if b_state != current_a_state:
            current_dir = COUNTERCLOCKWISE
        else:
            current_dir = CLOCKWISE

    last_a_state = current_a_state

    curr_ts = datetime.timestamp(datetime.now())

    # output string
    dir_name = "horário" if current_dir == CLOCKWISE else "anti-horário"
    rotations = knob_counter // KNOB_NUMBER
    rpm = (knob_counter * 60) // 130
    print(
        f"Tempo: {i}, RPM: {rpm}, Direção: {current_dir}, Contador: {knob_counter}, Rotações {rotations}"
    )

    # increment motor speed each second
    # print(last_ts, curr_ts)
    if (curr_ts - last_ts) > 1.0:
        i += 1
        last_ts = curr_ts
