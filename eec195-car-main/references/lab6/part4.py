"""
EEC 195A Lab 6
Part 4: Deflection Angle Using Linear Regression

This program uses the OpenMV Cam to calculate the deflection angle between the direction of travel of the car and the linear regression line of the detected line.
The program will print the deflection angle in degrees and turn on the corresponding LED based on the position of the detected line.

Mark Kim
Joseph Melman
Hanson Nguyen
Adam Wong
"""

import sensor
import time
import math
from machine import LED

THRESHOLD = (225, 255)  # Grayscale threshold for dark things.
BINARY_VISIBLE = True  # Binary pass first to see what linear regression is running on.

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)  # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
sensor.skip_frames(time=2001)  # WARNING: If you use QQVGA it may take seconds
clock = time.clock()  # to process a frame sometimes.
red_led = LED("LED_RED")
green_led = LED("LED_GREEN")
blue_led = LED("LED_BLUE")

while True:
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    # Returns a line object similar to line objects returned by find_lines() and
    # find_line_segments(). You have x1(), y1(), x2(), y2(), length(),
    # theta() (rotation in degrees), rho(), and magnitude().
    #
    # magnitude() represents how well the linear regression worked. It means something
    # different for the robust linear regression. In general, the larger the value the
    # better...
    line = img.get_regression(
        [(255, 255) if BINARY_VISIBLE else THRESHOLD], robust=True
    )

    if line:
        img.draw_line(line.line(), color=127)
        deflection_angle = -math.atan((line.x1() - line.x2()) / (line.y1() - line.y2()))
        deflection_angle = math.degrees(deflection_angle)

        if (line.x1() < 25 and line.x2() < 25 and line.x1() >= 0 and line.x2() >= 0):
            print(f"ANGLE={deflection_angle} | LEFT")
            red_led.off()
            green_led.off()
            blue_led.on()
        elif (line.x1() < 55 and line.x2() < 55 and line.x1() >= 25 and line.x2() >= 25):
            print(f"ANGLE={deflection_angle} | CENTER")
            red_led.off()
            green_led.on()
            blue_led.off()
        elif (line.x1() < 80 and line.x2() < 80 and line.x1() >= 55 and line.x2() >= 55):
            print(f"ANGLE={deflection_angle} | RIGHT")
            red_led.on()
            green_led.off()
            blue_led.off()
        else:
            print(f"ANGLE={deflection_angle} | NONE")
            red_led.off()
            green_led.off()
            blue_led.off()

    print(clock.fps())
