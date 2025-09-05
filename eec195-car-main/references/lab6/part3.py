"""
EEC 195A Lab 6
Part 3: Line Detection and PWM Control

This program uses the OpenMV Cam to determine whether the car needs to go left, right, or straight based on the position of the detected line.
The program will print the direction the car needs to go and the corresponding servo and motor pulse widths.

Mark Kim
Joseph Melman
Hanson Nguyen
Adam Wong
"""

import sensor
import time
import math
from machine import LED

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (225, 255)

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROI = (0, 0, 160, 120)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()
red_led = LED("LED_RED")
green_led = LED("LED_GREEN")
blue_led = LED("LED_BLUE")

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

count = 0
cur_servo_pw = 0

while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot()  # Take a picture and return the image.

    blobs = img.find_blobs(
        [thresholds], roi=ROI[0:4], area_threshold=80, merge=True
    )  # r[0:4] is roi tuple.

    if blobs:
        # Find the blob with the most pixels.
        closest_blob = min(blobs, key=lambda b: math.sqrt(abs(b.cx() - 80) ** 2 + abs(b.cy() - 10) ** 2))

        # Draw a rect around the blob.
        img.draw_rectangle(closest_blob.rect(), color=127)
        img.draw_cross(closest_blob.cx(), closest_blob.cy(), color=127)

        x_diff = closest_blob.cx() - 80

        if abs(x_diff) < 20:
            cur_servo_pw = 1500
            print(f"CENTER: SERVO_PW={cur_servo_pw}us | MOTOR_PW=1650us | LED=GREEN")
            red_led.off()
            green_led.on()
            blue_led.off()
        elif x_diff < 0:
            cur_servo_pw = 1100
            print(f"LEFT: SERVO_PW={cur_servo_pw}us | MOTOR_PW=1575us | LED=BLUE")
            red_led.off()
            green_led.off()
            blue_led.on()
        else:
            cur_servo_pw = 1900
            print(f"RIGHT: SERVO_PW={cur_servo_pw}us | MOTOR_PW=1575us | LED=RED")
            red_led.on()
            green_led.off()
            blue_led.off()
    else:
        print(f"NO TRACK: SERVO_PW={cur_servo_pw}us | MOTOR_PW=1500us | LED=GREEN")
        red_led.off()
        green_led.off()
        blue_led.off()


    print(clock.fps())  # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.
