import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime
from DC_Motor_pi import DC_Motor

# hardware pwm: 12, 32, 33, 35

CLOCKWISE = 1
COUNTERCLOCKWISE = -1

LINE_NUMBER = 46
RPM = 1300

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
current_dir = 0
pulse_counter = 0
total_pulse_counter = 0

last_ts = datetime.timestamp(datetime.now())
start_ts = last_ts

i = 1  # motor speed
while i < 100:
    if (i > 30):
        motor.forward(10)
    else:
        motor.backwards(10)

    current_a_state = GPIO.input(encoder_a)
    b_state = GPIO.input(encoder_b)

    # check if encoder detected a turn
    if current_a_state and not last_a_state:
        total_pulse_counter += 1

        # check direction
        if b_state:
            current_dir = "anti-horário"
            pulse_counter += 1
        else:
            current_dir = "horário"
            pulse_counter -= 1

    last_a_state = current_a_state

    # some rotory encoder calculations
    calc_line_num = total_pulse_counter // LINE_NUMBER
    frquency = RPM * LINE_NUMBER/60
    calc_rpm = frquency * 60 // LINE_NUMBER

    rotations = total_pulse_counter // LINE_NUMBER

    print(f"Tempo: {i}, RPM: {calc_rpm}, Direção: {current_dir}, Pulsos totais: {total_pulse_counter}, Rotações {rotations}, Pulsos: {pulse_counter}")

    # increment motor speed each second
    curr_ts = datetime.timestamp(datetime.now())
    if (curr_ts - last_ts) > 1.0:
        i += 1
        last_ts = curr_ts
