"""
EEC 195A Senior Design Project

Mark Kim
Joseph Melman
Hanson Nguyen
Adam Wong
"""

import sensor
import time
import math
from machine import Pin, PWM


# ---------- Constants ----------

# Clock for FPS
CLOCK = time.clock()
TIME_DELTA = 22  # Time delta for the loop in milliseconds

# Image settings
# Position
WIDTH = 160
HEIGHT = 120
TRACK_WIDTH = 90
THRESHOLD = (225, 255)  # Grayscale threshold for dark things.
BINARY_VISIBLE = True  # Binary pass first to see what linear regression is running on.
# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
ROIS = [(0, int(HEIGHT * (1 / 4)), WIDTH // 2 - 10, int(HEIGHT * (3 * 4))), (WIDTH // 2 + 10, int(HEIGHT * (1 / 4)), WIDTH // 2 - 10, int(HEIGHT * (3 * 4)))]

# DC Motor PWM range: [0, 65535]
FORWARD_MAX = 65535
BRAKE = 0  # 0% duty cycle
MAX_SPEED = int(0.350 * FORWARD_MAX)  # 17.5% duty cycle
MIN_SPEED = int(0.125 * FORWARD_MAX)  # 12.5% duty cycle

# Servo Motor PWM range: [1100000, 1900000]
STEER_RIGHT = 1900000
STEER_STRAIGHT = 1500000
STEER_LEFT = 1100000

# P7 and P8 may share the same PWM module they need
# to have the same frequency.
DC_MOTOR = PWM("P7", freq=1600, duty_u16=0)

# P9 and P10 may share the same PWM module they need
# to have the same frequency.
SERVO_MOTOR = PWM("P9", freq=100, duty_ns=STEER_STRAIGHT)

# Set the GPIO pins for the DC motor
INA = Pin("P8", Pin.OUT)  # Direction pin for the DC motor
INB = Pin("P10", Pin.OUT)  # Direction pin for the DC motor
CS = Pin("P2", Pin.IN)  # Current sense pin for the DC motor
INA.value(1)  # Set the direction to forward
INB.value(0)

# Maximum deflection angle for PID control
MAX_ANGLE_ABS = 30.0

# PID
LINE_CENTER_WEIGHT = 2.0
DEFLECTION_ANGLE_WEIGHT = 1.2

STEER_FACTOR = 1.05
SPEED_FACTOR = 2.50

PID_MAX = 500000
PID_MIN = -500000

KP = 0.8
KI = 0
KD = 2

integral = 0
prev_error = 0
curr_error = 0


# ---------- Functions ----------
def init():
    """
    Initialize the speed of the DC motor.
    """
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)  # 160x120
    sensor.skip_frames(time=2001)  # WARNING: If you use QQVGA it may take seconds

    # Ramp up the speed of the DC motor
    cur_speed = BRAKE
    DC_MOTOR.duty_u16(cur_speed)
    time.sleep_ms(1000)  # Wait for the motor to stop (1s)
    while cur_speed < MIN_SPEED:
        cur_speed += 200
        cur_speed = min(cur_speed, MIN_SPEED)
        DC_MOTOR.duty_u16(cur_speed)
        time.sleep_ms(10)


def _clamp(value, min_value, max_value):
    """
    Clamp a value between min_value and max_value.
    """
    return max(min(value, max_value), min_value)


def steer_pid(curr_error: float):
    """
    PID control for steering based on current error where current error is between -1 and 1.
    """
    global integral
    global prev_error

    # PID calculations
    integral = integral + curr_error  # I = intergral
    derivative = curr_error - prev_error  # D = derivative
    prev_error = curr_error  # Save

    pid = (KP * curr_error) + (KI * integral) + (KD * derivative)

    # Limit PID to max/min values
    pid = _clamp(pid, -1, 1) * -1  # Invert PID for physical vehicle

    # Convert to Servo PWM range
    servo_pwm = int(STEER_STRAIGHT + (pid * STEER_FACTOR * 500000))
    servo_pwm = _clamp(servo_pwm, STEER_LEFT, STEER_RIGHT)

    # If servo_pwm is within ±5% of STEER_STRAIGHT, set it to STEER_STRAIGHT
    if abs(servo_pwm - STEER_STRAIGHT) <= 0.05 * STEER_STRAIGHT:
        servo_pwm = STEER_STRAIGHT
    SERVO_MOTOR.duty_ns(servo_pwm)
    return servo_pwm


def speed_control(error: float):
    """
    Control the speed of the DC motor based on the error.
    """
    # Calculate speed based on error
    speed = int(
        MAX_SPEED - (abs(error) * SPEED_FACTOR * (MAX_SPEED - MIN_SPEED))
    )  # Speed decreases as error increases
    speed = _clamp(speed, MIN_SPEED, MAX_SPEED)

    # Set the DC motor speed
    DC_MOTOR.duty_u16(speed)
    return speed


def reset_steer():
    """
    Reset the steering to straight.
    """
    SERVO_MOTOR.duty_ns(STEER_STRAIGHT)


def stop():
    """
    Stop the DC motor.
    """
    DC_MOTOR.duty_u16(BRAKE)
    time.sleep_ms(100)  # Wait for the motor to stop (0.1s)
    DC_MOTOR.duty_u16(0)  # Set the duty cycle to 0% to stop the motor
    reset_steer()
    time.sleep_ms(100)  # Wait for the servo to stop (0.1s)


def line_center_error(cur_x: float):
    """
    Calculate the error based on the center of the line.

    Returns a value between -1 and 1 where -1 is error indicating a left turn and 1 is a right turn.
    """
    center_x = WIDTH / 2  # Center of the image
    return (cur_x - center_x) / center_x


def deflection_angle_error(deflection_angle: float):
    """
    Calculate the error based on the deflection angle.

    Returns a value between -1 and 1 where -1 is error indicating a left turn and 1 is a right turn.
    """
    return -_clamp(deflection_angle, -MAX_ANGLE_ABS, MAX_ANGLE_ABS) / MAX_ANGLE_ABS


def combined_error(lc_error: float, da_error: float, lc_weight: float = LINE_CENTER_WEIGHT, da_weight: float = DEFLECTION_ANGLE_WEIGHT):
    """
    Combine the line center error and deflection angle error based on their weights.
    """
    return _clamp(
        (lc_weight * lc_error) + (da_weight * da_error), -1, 1
    )


# ---------- Main Loop ----------
init()
while True:
    # Start straight
    SERVO_MOTOR.duty_ns(STEER_STRAIGHT)
    CLOCK.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    lines = []

    for r in ROIS:
        line = img.get_regression([THRESHOLD], roi=r[0:4], robust=True)

        if line:
            img.draw_line(line.line(), color=127)
            lines.append(line)

    if len(lines) < 1 or len(lines) > 2:
        continue

    # Calculate the middle line between the two lines
    if len(lines) == 1:
        middle_line = {
            "x1": lines[0].x1() + (TRACK_WIDTH // 2 if lines[0].x1() < WIDTH // 2 else -TRACK_WIDTH // 2),
            "x2": lines[0].x2() + (TRACK_WIDTH // 2 if lines[0].x1() < WIDTH // 2 else -TRACK_WIDTH // 2),
            "y1": lines[0].y1(),
            "y2": lines[0].y2(),
        }
    else:
                middle_line = {
            "x1": (lines[0].x1() + lines[1].x1()) // 2,
            "x2": (lines[0].x2() + lines[1].x2()) // 2,
            "y1": (lines[0].y1() + lines[1].y1()) // 2,
            "y2": (lines[0].y2() + lines[1].y2()) // 2,
        }
    img.draw_line(
        (middle_line["x1"], middle_line["y1"], middle_line["x2"], middle_line["y2"]),
        color=200,
    )
    # print(
    #     f"[Middle Line]: x1={middle_line['x1']}, y1={middle_line['y1']}, x2={middle_line['x2']}, y2={middle_line['y2']} | [Line 1]: x1={lines[0].x1()}, y1={lines[0].y1()}, x2={lines[0].x2()}, y2={lines[0].y2()} | [Line 2]: x1={lines[1].x1()}, y1={lines[1].y1()}, x2={lines[1].x2()}, y2={lines[1].y2()}"
    # )

    # Avoid division by zero
    if middle_line["y1"] == middle_line["y2"]:
        continue

    # Determine errors
    da = math.degrees(
        math.atan(
            (middle_line["x1"] - middle_line["x2"])
            / (middle_line["y1"] - middle_line["y2"])
        )
    )
    da_error = deflection_angle_error(da)
    lc = (middle_line["x1"] + middle_line["x2"]) / 2
    lc_error = line_center_error(lc)

    # Calculate combined error
    error = combined_error(lc_error, da_error)

    servo_pwm = steer_pid(error)
    dc_pwm = speed_control(error)
    # print(
    #     f"[Values]: servo_pwm={servo_pwm}, dc_duty_cycle={(dc_pwm / FORWARD_MAX) * 100}%, error={error}, lc_error={lc_error}, da_error={da_error}"
    # )
