"""
EEC 195A Lab 6
Part 2: Blob Detection - Program 1

This program uses the OpenMV Cam to detect blobs in the top portion of the image.
The program will draw a rectangle around the blobs and a cross at the centroid of the blobs.
The program will also toggle the red LED on the OpenMV Cam every 50 frames.

Mark Kim
Joseph Melman
Hanson Nguyen
Adam Wong
"""

import sensor
import time
from machine import LED

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)

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

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

count = 0

while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot()  # Take a picture and return the image.

    centroid_sum = 0

    blobs = img.find_blobs(
        [thresholds], pixels_threshold=50, area_threshold=50, roi=ROI[0:4], merge=True
    )  # ROI[0:4] is roi tuple.

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=127)
        img.draw_cross(blob.cx(), blob.cy(), color=127)

    count += 1
    if count == 50:
        count = 0
        red_led.toggle()

    print(clock.fps())  # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.
