"""
EEC 195A Lab 6
Part 2: Blob Detection - Program 3

This program uses the OpenMV Cam to detect the centermost blob in the top and bottom portions of the image.
The program will calculate the deflection angle between the two blobs and print the angle in degrees.

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
ROIS = [
    (0, 0, 160, 20),
    (0, 100, 160, 20)
]

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

    cbs = []

    for r in ROIS:
        blobs = img.find_blobs(
            [thresholds], roi=r[0:4], merge=True
        )  # r[0:4] is roi tuple.

        if blobs:
            # Find the blob with the most pixels.
            closest_blob = min(blobs, key=lambda b: math.sqrt(abs(b.cx() - 80) ** 2 + abs(b.cy() - 10) ** 2))

            # Draw a rect around the blob.
            img.draw_rectangle(closest_blob.rect(), color=127)
            img.draw_cross(closest_blob.cx(), closest_blob.cy(), color=127)

            cbs.append(closest_blob)

    if len(cbs) != 2:
        continue
    deflection_angle = -math.atan((cbs[0].cx() - cbs[1].cx()) / (cbs[0].cy() - cbs[1].cy()))

    # Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)

    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
    print("Turn Angle: %f" % deflection_angle)

    count += 1
    if count == 50:
        count = 0
        red_led.toggle()

    print(clock.fps())  # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.
