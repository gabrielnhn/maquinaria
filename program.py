#!/usr/bin/env python3
"""
A program used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
You may change the parameters BECAUSE IT IS NECESSARY.
"""
__author__ = "Gabriel Hishida, Gabriel Pontarolo, Tiago Serique and Isadora Botassari"

import numpy as np
import cv2
import signal
import RPi.GPIO as GPIO
from DC_Motor_pi import DC_Motor
import requests
from datetime import datetime, timedelta
import argparse
import time
import csv

# init arg parser
parser = argparse.ArgumentParser()
parser.add_argument("-s", "--start", action="store_true", help="Follow line")
parser.add_argument("-r", "--record", action="store_true", help="Record masked image")
parser.add_argument("-o", "--output", metavar="address", action="store", help="Show output image to ip address")
parser.add_argument("-p", "--stop", metavar="store_true", help="Stop the robot in `RUNTIME` seconds")
args = parser.parse_args()

# pins setup
clockwise_pin_1 = 11
counterclockwise_pin_1 = 13
pwm_pin_1 = 12

clockwise_pin_2 = 16
counterclockwise_pin_2 = 15
pwm_pin_2 = 18

motor_left = DC_Motor(clockwise_pin_1, counterclockwise_pin_1, pwm_pin_1)
motor_right = DC_Motor(clockwise_pin_2, counterclockwise_pin_2, pwm_pin_2)

# encoder pin setup
encoder_a = 19
encoder_b = 21
GPIO.setup(encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# encoder values
# hardware pwm: 12, 32, 33, 35
STEPS_NUMBER = 7
RPM = 800
# 
RADIUS_WHEEL = 5.0

# init state variables
last_a_state = GPIO.input(encoder_a)
current_dir = 0
pulse_counter = 0
total_pulse_counter = 0
direc = 0
period = 1
pstart = 0
wave_state = 0

# Global vars. initial values
runtime = 0 # time until it should stop
init_time = int(datetime.now().timestamp())
init_time_iso = datetime.now()
image_input = None
error = 0
just_seen_line = False
should_move = False
should_record = False
should_show = False
should_stop = False
record_writer = None
ip_addr = "0.0.0.0"
no_movement_count = 0

# finalization_countdown = None
# right_mark_count = 0
# just_seen_right_mark = False

## User-defined parameters: (Update these values as necessary)
# Minimum size for a contour to be considered anything
MIN_AREA = 400

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 900

MAX_CONTOUR_VERTICES = 30

# Robot's speed when following the line
# LINEAR_SPEED = 14.0
LINEAR_SPEED = 15.0
LINEAR_SPEED_ON_LOSS = 5.0
LINEAR_SPEED_ON_CURVE = 7.5

# error when the curve starts
CURVE_ERROR_THRH =  22
LOSS_THRH =  40


FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS = 10
after_loss_count = FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS + 1

# mininum speed to keep the robot running without rampup
MIN_SPEED = 7

# Proportional constant to be applied on speed when turning
# (Multiplied by the error value)
# KP = 26/100
KP = 27.5/100

# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# frames without diff in the speed
NO_MOVEMENT_FRAMES = 3

RESIZE_SIZE = 4


# CTR_CENTER_SIZE_FACTOR = 10 
# CTR_CENTER_SIZE_FACTOR = 1 
# Send messages every $TIMER_PERIOD seconds
# TIMER_PERIOD = 0.06

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
# FINALIZATION_PERIOD = 4

# Time the robot takes to finish the track in seconds
#RUNTIME = 127.0

# The maximum error value for which the robot is still in a straight line
# MAX_ERROR = 30


# BGR values to filter only the selected color range
lower_bgr_values = np.array([170,  170,  170])
upper_bgr_values = np.array([255, 255, 255])


def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
    Width_left_boundary, Width_right_boundary)
    """
    ## Update these parameters as well.

    return (0, 3*height//5, 0, width)


def show_callback():
    global should_show
    global ip_addr
    should_show = True
    ip_addr = args.output
    print("SHOWING")
    print(">>", end="")

def record_callback(width, height):
    global should_record
    global record_writer
    should_record = True
    record_writer = cv2.VideoWriter(f"out-{datetime.now().minute}.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 20, (width, height))
    print("RECORDING")
    print(">>", end="")

def end_record():
    global should_record
    global record_writer
    if should_record:
        record_writer.release()

def stop_callback():
    global should_stop
    global runtime
    runtime = int(args.stop)
    runtime = timedelta(seconds=runtime)

    should_stop = True
    print("WILL STOP")
    print(">>", end="")

def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    print("STARTING!")
    global should_move
    global right_mark_count
    global finalization_countdown
    lost = False
    should_move = True
    # right_mark_count = 0
    # finalization_countdown = None

    print(">>", end="")
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    print("STOPPED!")
    global should_move
    # global finalization_countdown
    should_move = False
    # finalization_countdown = None

    print(">>", end="")
    return response


def get_contour_data(mask, out, previous_pos):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line)
    and return the side in which the smaller contour is (the track mark)
    (If there are any of these contours),
    and draw all contours on 'out' image
    """

    # erode image (filter excessive brightness noise)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)

    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}
    over = False
    tried_once = False

    possible_tracks = []

    while not over:

        for contour in contours:
            M = cv2.moments(contour)
            # Search more about Image Moments on Wikipedia :)

            contour_vertices = len(cv2.approxPolyDP(contour, 1.0, True))
            # print("vertices: ", contour_vertices)

            if (M['m00'] < MIN_AREA):
                continue

            if (contour_vertices < MAX_CONTOUR_VERTICES) and (M['m00'] > MIN_AREA_TRACK):
                # Contour is part of the track
                line['x'] = crop_w_start + int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])

                possible_tracks.append(line)

                # plot the amount of vertices in light blue
                cv2.drawContours(out, contour, -1, (255,255,0), 1)
                # cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                #     cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (100,200,150), 1)

                cv2.putText(out, str(contour_vertices), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (100,200,150), 1)

            else:
                # plot the area in pink
                cv2.drawContours(out, contour, -1, (255,0,255), 1)
                cv2.putText(out, f"{contour_vertices}-{M['m00']}", (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (255,0,255), 2)

        if line:
            over = True

        # Did not find the line. Try eroding more?
        elif not tried_once:
            mask = cv2.erode(mask, kernel, iterations=1)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            tried_once = True

        # Did not find anything
        else:
            over = True

    if not possible_tracks:
        chosen_line = None
    else:
        chosen_line = min(possible_tracks, key=lambda line: abs(line["x"] - previous_pos))

    return chosen_line


def process_frame(image_input, last_res_v):
    """
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    debug_str2 = ""
    global error
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global should_stop
    global right_mark_count
    global init_time
    global lost
    global no_movement_count
    global after_loss_count

    res_v = {   
        "left" : 0, # left motor resulting speed
        "right" : 0 # right motor resulting speed
    } 

    height, width, _ = image_input.shape
    image = image_input

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    cx = width//2

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]

    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)


    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    line = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop], error + cx)
    # also get the side in which the track mark "is"

    x = None

    if line:
        x = line['x']
        new_error = x - cx
    else:
        new_error = None

    # (((error < 0) and (new_error < 0)) or ((error > 0) and (new_error > 0)))):

    if (line) and ((not lost) or (
        abs(new_error - error) < LOSS_THRH)):

        # error:= The difference between the center of the image and the center of the line
        error = new_error

        if lost:
            after_loss_count = 0

        lost = False

        if abs(error) > CURVE_ERROR_THRH:
            linear = LINEAR_SPEED_ON_CURVE
        else:
            linear = LINEAR_SPEED

        if after_loss_count < FRAMES_TO_USE_LINEAR_SPEED_ON_LOSS:
            linear = LINEAR_SPEED_ON_LOSS
            after_loss_count += 1
        
        just_seen_line = True

    else:
        if new_error:
            debug_str2 = f"last={error}| new={new_error}"
        # print("LOST", end=". ")
        lost = True
        # There is no line in the image.
        # Turn on the spot to find it again.
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
        
        linear = LINEAR_SPEED_ON_LOSS


    global runtime
    # Check for final countdown
    if should_move and should_stop:
        if (datetime.now() - init_time_iso) >= runtime:
            should_move = False
            print(f"STOPPED AT {datetime.now()}")


    # Determine the speed to turn and get the line in the center of the camera.
    angular = float(error) * -KP

    # resulting speed
    res_v["left"] = int(linear - angular)
    res_v["right"] = int(linear + angular)
    
    left_should_rampup = False
    right_should_rampup = False

    if (last_res_v["left"] == res_v["left"]) and (last_res_v["right"] == res_v["right"]):
        no_movement_count += 1
    else:
        no_movement_count = 0

    if no_movement_count > NO_MOVEMENT_FRAMES:
        left_should_rampup = True
        right_should_rampup = True
        no_movement_count = 0


    if (last_res_v["left"] <= MIN_SPEED) and (res_v["left"] > last_res_v["left"]): 
        left_should_rampup = True

    if (last_res_v["right"] <= MIN_SPEED) and (res_v["right"] > last_res_v["right"]): 
        right_should_rampup = True

    if should_move:

        if left_should_rampup:
            motor_left.run(90)
        if right_should_rampup:
            motor_right.run(90)
        if left_should_rampup or right_should_rampup:
            time.sleep(0.008)

        motor_left.run(res_v["left"])
        motor_right.run(res_v["right"])
    
    else:
        motor_left.stop()
        motor_right.stop()


    #Show the output image to the user
    global should_record
    global should_show
    global ip_addr
    global record_writer

    now = f"{datetime.now().strftime('%M:%S.%f')[:-4]}"
    debug_str = f"A: {int(angular)}|L: {linear}|E: {error}"
    debug_str += f"lft{res_v['left']}"
    if left_should_rampup:
        debug_str += "(rmp)"
    debug_str += f"rgt{res_v['right']}"
    if right_should_rampup:
        debug_str += "(rmp)"

    if should_record or should_show:
        text_size, _ = cv2.getTextSize(debug_str, cv2.FONT_HERSHEY_PLAIN, 2, 2)
        text_w, text_h = text_size

        cv2.rectangle(output, (0, 90), (text_w, 110 + text_h), (255,255,255), -1)
        # Plot the boundaries where the image was cropped
        cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
        # center of the image
        cv2.circle(output, (cx, crop_h_start + (height//2)), 1, (75,0,130), 1)
        # cv2.putText(output, now, (0, text_h - 10), cv2.FONT_HERSHEY_PLAIN, 0.5, (50, 255, 255), 1)
        cv2.putText(output, debug_str, (0, text_h - 10), cv2.FONT_HERSHEY_PLAIN, 0.5, (0, 255, 100), 1)
        cv2.putText(output, debug_str2, (0, text_h), cv2.FONT_HERSHEY_PLAIN, 0.5, (0, 255, 100), 1)

        # plot the rectangle around contour center
        if x:
            cv2.circle(output, (line['x'], crop_h_start + line['y']), 1, (0,255,0), 1)
            cv2.rectangle(output, (x - width, crop_h_start), (x + width, crop_h_stop), (0,0,255), 2)

        if should_show:
            # cv2.imshow("output", output)
            # Print the image for 5milis, then resume execution
            # cv2.waitKey(5)
            _, imdata = cv2.imencode('.jpg', output)    
            # _, imdata = cv2.imencode('.jpg', mask)    
            requests.put(f"http://{ip_addr}:5000/upload", data=imdata.tobytes()) # send image to webserver

        if should_record:
            record_writer.write(output)
    
    # Uncomment to show the binary picture
    #cv2.imshow("mask", mask)
    
    return res_v # return speed of the current iteration
    
# this is a highly experimental function
# this should be able to detect encoder readings according to encoder_test.py
def read_encoders():
    global current_dir
    global pulse_counter
    global total_pulse_counter
    global direc
    global period
    global pstart
    global wave_state

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
    calc_rpm = frequency * 60 // STEPS_NUMBER
    rotations = total_pulse_counter // STEPS_NUMBER 


def timeout(signum, frame):
    raise TimeoutError

def main():
    print(datetime.now())
    global lost
    global error
    global no_movement_count
    lost = False

    last_ts = time.time()
    start_ts = last_ts
    f = open('csvfile.csv', 'w')
    writer = csv.writer(f)
    header = ['distancia', 'linear', 'angular', 'erro']
    # write the header
    writer.writerow(header)


    signal.signal(signal.SIGALRM, timeout)
    # Use system signals to stop input()

    video = cv2.VideoCapture(0)

    # print(video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920))
    # print(video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080))

    retval, image = video.read()
    print(image.shape)
    height, width, _ = image.shape
    print(">>", end="")
    error = width//(RESIZE_SIZE*2)

    if args.start:  # should start following line
        start_follower_callback(None, None)

    if args.record: # should record image
        record_callback(width//RESIZE_SIZE, height//RESIZE_SIZE)

    if args.output != None: # should show image
        show_callback()

    if args.stop != None: # should show image
        stop_callback()

    last_res_v = {
        "left" : 0,
        "right" : 0
    }

    while retval:
        try:

            image = cv2.resize(image, (width//RESIZE_SIZE, height//RESIZE_SIZE), interpolation= cv2.INTER_LINEAR)
            last_res_v = process_frame(image, last_res_v)

            # I don't know if read_encoder should be called here
            read_encoder()
            data = [(((2 * 3.14 * RADIUS_WHEEL) / STEPS_NUMBER) * pulse_counter), linear, angular, error]
            writer.writerow(data)

            retval, image = video.read()


        except TimeoutError:
            pass

    GPIO.cleanup()
    f.close()
    end_record()
    print("Exiting...")


try:
    main()

except KeyboardInterrupt:
    end_record()
    now = datetime.now()
    print(now)
    print(f"TOTAL TIME {now - init_time_iso}")
    print("\nExiting...")

except Exception as e:
    print(e)

finally:
    del motor_left
    del motor_right
    GPIO.cleanup()
    #video.close()

