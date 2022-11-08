#!/usr/bin/env python3
"""
A ROS2 node used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
You may change the parameters to your liking.
"""
__author__ = "Gabriel Hishida, Gabriel Pontarolo, Tiago Serique and Isadora Botassari"

from codecs import latin_1_encode
import numpy as np
import cv2
import signal
import RPi.GPIO as GPIO
from DC_Motor_pi import DC_Motor
import requests
from datetime import datetime
import argparse
import time

# init arg parser
parser = argparse.ArgumentParser()
parser.add_argument("-s", "--start", action="store_true", help="Follow line")
parser.add_argument("-r", "--record", action="store_true", help="Record masked image")
parser.add_argument("-o", "--output", metavar="address", action="store", help="Show output image to ip address")
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


# Global vars. initial values
image_input = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None
should_record = False
should_show = False
record_writer = None
ip_addr = "0.0.0.0"

## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
# MIN_AREA = 20000
MIN_AREA = 50


# Minimum size for a contour to be considered part of the track
# MIN_AREA_TRACK = 40000
# MIN_AREA_TRACK = 20000
MIN_AREA_TRACK = 80

# CTR_CENTER_SIZE_FACTOR = 10 
CTR_CENTER_SIZE_FACTOR = 1 


MAX_CONTOUR_VERTICES = 45


# Robot's speed when following the line
LINEAR_SPEED = 20.0
LINEAR_SPEED_ON_LOSS = 13.0

# mininum speed to keep the robot
MIN_SPEED = 15

# Proportional constant to be applied on speed when turning
# (Multiplied by the error value)
KP = 33/100
# KP = 3/100


# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 2.6

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.06

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
FINALIZATION_PERIOD = 4

# The maximum error value for which the robot is still in a straight line
MAX_ERROR = 30

RESIZE_SIZE = 6


# BGR values to filter only the selected color range
# lower_bgr_values = np.array([185,  190,  191])
lower_bgr_values = np.array([180,  185,  185])

# lower_bgr_values = np.array([192,  193,  193])

upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.

    #return (1*height//3, height, width//4, 3*width//4)
    # return (0, height, 0, width)
    # return (2*height//5, height, 0, width)
    # return (0, 3*height//5, 0, width)
    return (1*height//3, height, 0, width)



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
    record_writer = cv2.VideoWriter(f"output-{datetime.now()}.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 25, (width, height))
    print("RECORDING")
    print(">>", end="")

def end_record():
    global should_record
    global record_writer
    if should_record:
        record_writer.release()

def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    print("STARTING!")
    global should_move
    global right_mark_count
    global finalization_countdown
    should_move = True
    right_mark_count = 0
    finalization_countdown = None


    print(">>", end="")
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    print("STOPPED!")
    global should_move
    global finalization_countdown
    should_move = False
    finalization_countdown = None

    print(">>", end="")
    return response


def get_contour_data(mask, out):
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

    while not over:

        for contour in contours:
            contour_vertices = len(cv2.approxPolyDP(contour, 3.0, True))

            if contour_vertices > MAX_CONTOUR_VERTICES:
                continue


            M = cv2.moments(contour)
            # Search more about Image Moments on Wikipedia :)


            if M['m00'] > MIN_AREA:
            # if countor.area > MIN_AREA:

                if (M['m00'] > MIN_AREA_TRACK):
                    # Contour is part of the track
                    line['x'] = crop_w_start + int(M["m10"]/M["m00"])
                    line['y'] = int(M["m01"]/M["m00"])

                    # plot the area in light blue
                    cv2.drawContours(out, contour, -1, (255,255,0), 1)
                    cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (100,200,150), 1)

                else:
                    # Contour is a track mark
                    if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                        # if there are more than one mark, consider only
                        # the one closest to the robot
                        mark['y'] = int(M["m01"]/M["m00"])
                        mark['x'] = crop_w_start + int(M["m10"]/M["m00"])

                        # plot the area in pink
                        cv2.drawContours(out, contour, -1, (255,0,255), 1)
                        cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                            cv2.FONT_HERSHEY_PLAIN, 2/(RESIZE_SIZE/3), (255,0,255), 2)


        if mark and line:
        # if both contours exist
            if mark['x'] > line['x']:
                mark_side = "right"
            else:
                mark_side = "left"
        else:
            mark_side = None

        if line:
            over = True

        # Did not find the line. Try eroding more?
        elif not tried_once:
            mask = cv2.erode(mask, kernel, iterations=7)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            tried_once = True

        # Did not find anything
        else:
        
            over = True


    return (line, mark_side)


def process_frame(image_input):
    """
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    global error
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global right_mark_count
    global finalization_countdown

    v_left = 0  # left motor resulting speed
    v_right = 0 # right motor resulting speed

    height, width, _ = image_input.shape
    in_line = False

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
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
    # also get the side in which the track mark "is"

    # message = Twist()

    x = None

    if line:
    # if there even is a line in the image:
    # (as the camera could not be reading any lines)
        x = line['x']

        # error:= The difference between the center of the image
        # and the center of the line
        error = x - cx
        # print(f"x: {x}, error {error}")

        linear = LINEAR_SPEED
        just_seen_line = True

        # check if image center is inside a square around the line center
        in_line = ((cx > (x - width//CTR_CENTER_SIZE_FACTOR)) and (cx < (x + width//CTR_CENTER_SIZE_FACTOR)))

        # plot the line centroid on the image
        # cv2.circle(output, (line['x'], crop_h_start + line['y']), 5, (0,255,0), 7)
        

    else:
        print("LOST", end=". ")
        # There is no line in the image.
        # Turn on the spot to find it again.
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
        
        linear = LINEAR_SPEED_ON_LOSS

    if mark_side != None:
        # print("mark_side: {}".format(mark_side))

        if (mark_side == "right") and (finalization_countdown == None) and \
            (abs(error) <= MAX_ERROR) and (not just_seen_right_mark):

            right_mark_count += 1

            if right_mark_count > 1:
                # Start final countdown to stop the robot
                finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
                print("\nFinalization Process has begun!\n>>", end="")


            just_seen_right_mark = True
    else:
        just_seen_right_mark = False


    # Determine the speed to turn and get the line in the center of the camera.
    angular = float(error) * -KP

    # result speed
    v_left = int(linear - angular)
    v_right = int(linear + angular)

    # if image center is inside the contour, angular = 0
    # if in_line:
    #     angular = 0.0

    now = f"{datetime.now()}"
    debug_str = f"A: {int(angular)}|L: {linear}|E: {error}"

    #Show the output image to the user

    global should_record
    global should_show
    global ip_addr
    global record_writer
    
    if should_record or should_show:
        text_size, _ = cv2.getTextSize(debug_str, cv2.FONT_HERSHEY_PLAIN, 2, 2)
        text_w, text_h = text_size

        cv2.rectangle(output, (0, 90), (text_w, 110 + text_h), (255,255,255), -1)
        # Plot the boundaries where the image was cropped
        cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
        # center of the image
        cv2.circle(output, (cx, crop_h_start + (height//2)), 1, (75,0,130), 1)
        cv2.putText(output, now, (0, text_h - 10), cv2.FONT_HERSHEY_PLAIN, 0.5, (50, 255, 255), 1)
        cv2.putText(output, debug_str, (0, text_h), cv2.FONT_HERSHEY_PLAIN, 0.5, (50, 255, 255), 1)
        # plot the rectangle around contour center
        if x:
            cv2.circle(output, (line['x'], crop_h_start + line['y']), 1, (0,255,0), 1)
            cv2.rectangle(output, (x - width//CTR_CENTER_SIZE_FACTOR, crop_h_start), (x + width//CTR_CENTER_SIZE_FACTOR, crop_h_stop), (0,0,255), 2)

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


    # Check for final countdown
    if finalization_countdown != None:
        if finalization_countdown > 0:
            finalization_countdown -= 1

        elif finalization_countdown == 0:
            #should_move = False
            pass

    # Publish the message to 'cmd_vel'
    print(f"{now}\n{debug_str}\nLEFT: { v_left} |  RIGHT: {v_right}\n --- \n")


    if should_move:
        if (v_left > MIN_SPEED or v_right > MIN_SPEED):
            rampup()

        motor_left.run(v_left)
        motor_right.run(v_right)
    else:
        motor_left.stop()
        motor_right.stop()

        

def rampup():
    motor_left.run(40)
    motor_right.run(40)
    time.sleep(0.1)


def timeout(signum, frame):
    raise TimeoutError

def main():
    signal.signal(signal.SIGALRM, timeout)
    # Use system signals to stop input()

    video = cv2.VideoCapture(0)

    # print(video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920))
    # print(video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080))

    retval, image = video.read()
    print(image.shape)
    height, width, _ = image.shape

    print(">>", end="")

    if args.start:  # should start following line
        start_follower_callback(None, None)

    if args.record: # should record image
        record_callback(width//RESIZE_SIZE, height//RESIZE_SIZE)

    if args.output != None: # should show image
        show_callback()

    if should_move:
        rampup()


    while retval:
        try:

            image = cv2.resize(image, (width//RESIZE_SIZE, height//RESIZE_SIZE), interpolation= cv2.INTER_LINEAR)
            process_frame(image)

            retval, image = video.read()


        except TimeoutError:
            pass

    GPIO.cleanup()
    end_record()
    print("Exiting...")


try:
    main()

except KeyboardInterrupt:
    end_record()
    print("\nExiting...")

except Exception as e:
    print(e)

finally:
    del motor_left
    del motor_right
    GPIO.cleanup()
    #video.close()

