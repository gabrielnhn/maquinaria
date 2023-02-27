import RPi.GPIO as GPIO
import time
from datetime import datetime
from DC_Motor_pi import DC_Motor


# hardware pwm: 12, 32, 33, 35

LINE_NUMBER = 7
RPM = 800

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

last_ts = time.time()
start_ts = last_ts

direc = 0
period = 1
pstart = 0
wave_state = 0

i = 10  # motor speed
while i < 100:
    if (i // 10 % 2) == 0:
        direc = 1
        motor.forward(100)
    else:
        direct = -1
        motor.backwards(100)

    current_a_state = GPIO.input(encoder_a)


    if wave_state == 0:
        if current_a_state == 1 and last_a_state == 0:
            period = (time.time_ns() / 1000) - pstart
            pstart = time.time_ns() / 1000
            wave_state = 1

    elif wave_state == 1:
        if current_a_state == 0 and last_a_state == 1:
            wave_state = 0

    # check if encoder detected a turn
    if current_a_state != last_a_state and current_a_state == 1:
        total_pulse_counter += 1

        b_state = GPIO.input(encoder_b)
        # check direction
        if b_state != current_a_state:
            current_dir = "anti-horário"
            pulse_counter += 1
        else:
            current_dir = "horário"
            pulse_counter -= 1

    last_a_state = current_a_state

    curr_ts = time.time()
    # some rotory encoder calculations
    frequency = total_pulse_counter / (curr_ts - start_ts)
    # calc_line_num = frequency * 60 // RPM
    calc_rpm = frequency * 60 // LINE_NUMBER

    rotations = total_pulse_counter // LINE_NUMBER


    print(f"Speed: {i}, RPM: {calc_rpm}, Periodo: {period}, Frequencia: {1/period}, Dir: {direc}, M_Direc: {current_dir}")

    # increment motor speed each second
    if (curr_ts - last_ts) > 1.0:
        i += 1
        last_ts = curr_ts
