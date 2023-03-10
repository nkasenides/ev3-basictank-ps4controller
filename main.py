#!/usr/bin/env pybricks-micropython


# MOTOR & SENSOR BINDINGS:
# Motor Left: B
# Motor Right: C
# Front Claw: A

# Touch button: 4

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch

import math
import struct
import _thread
import sys
import os

print("STARTED")
ev3 = EV3Brick()
ev3.speaker.beep()

# A helper function for converting stick values (0 - 255)
# to more usable numbers (-100 - 100)
def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
 
    val: float or int
    src: tuple
    dst: tuple
 
    example: print(scale(99, (0.0, 99.0), (-1.0, +1.0)))
    """
    return (float(val-src[0]) / (src[1]-src[0])) * (dst[1]-dst[0])+dst[0]


def get_speeds(x, y, radius=100.0):
        """
        Convert ``x``,``y`` joystick coordinates to left/right motor speed percentages
        and move the motors.
        This will use a classic "arcade drive" algorithm: a full-forward joystick
        goes straight forward and likewise for full-backward. Pushing the joystick
        all the way to one side will make it turn on the spot in that direction.
        Positions in the middle will control how fast the vehicle moves and how
        sharply it turns.
        ``x``, ``y``:
            The X and Y coordinates of the joystick's position, with
            (0,0) representing the center position. X is horizontal and Y is vertical.
        ``radius`` (default 100):
            The radius of the joystick, controlling the range of the input (x, y) values.
            e.g. if "x" and "y" can be between -1 and 1, radius should be set to "1".
        """

        # If joystick is in the middle stop the tank
        if not x and not y:
            # self.off()
            return

        vector_length = math.sqrt((x * x) + (y * y))
        angle = math.degrees(math.atan2(y, x))

        if angle < 0:
            angle += 360

        # Should not happen but can happen (just by a hair) due to floating point math
        if vector_length > radius:
            vector_length = radius

        (init_left_speed_percentage, init_right_speed_percentage) = angle_to_speed_percentage(angle)

        # scale the speed percentages based on vector_length vs. radius
        left_speed_percentage = (init_left_speed_percentage * vector_length) / radius
        right_speed_percentage = (init_right_speed_percentage * vector_length) / radius
        return left_speed_percentage, right_speed_percentage


def angle_to_speed_percentage(angle):
        """
        The following graphic illustrates the **motor power outputs** for the
        left and right motors based on where the joystick is pointing, of the
        form ``(left power, right power)``::
                                     (1, 1)
                                  . . . . . . .
                               .        |        .
                            .           |           .
                   (0, 1) .             |             . (1, 0)
                        .               |               .
                       .                |                 .
                      .                 |                  .
                     .                  |                   .
                    .                   |                   .
                    .                   |     x-axis        .
            (-1, 1) .---------------------------------------. (1, -1)
                    .                   |                   .
                    .                   |                   .
                     .                  |                  .
                      .                 | y-axis          .
                        .               |               .
                  (0, -1) .             |             . (-1, 0)
                            .           |           .
                               .        |        .
                                  . . . . . . .
                                     (-1, -1)
        The joystick is a circle within a circle where the (x, y) coordinates
        of the joystick form an angle with the x-axis.  Our job is to translate
        this angle into the percentage of power that should be sent to each motor.
        For instance if the joystick is moved all the way to the top of the circle
        we want both motors to move forward with 100% power...that is represented
        above by (1, 1).  If the joystick is moved all the way to the right side of
        the circle we want to rotate clockwise so we move the left motor forward 100%
        and the right motor backwards 100%...so (1, -1).  If the joystick is at
        45 degrees then we move apply (1, 0) to move the left motor forward 100% and
        the right motor stays still.
        The 8 points shown above are pretty easy. For the points in between those 8
        we do some math to figure out what the percentages should be. Take 11.25 degrees
        for example. We look at how the motors transition from 0 degrees to 45 degrees:
        - the left motor is 1 so that is easy
        - the right motor moves from -1 to 0
        We determine how far we are between 0 and 45 degrees (11.25 is 25% of 45) so we
        know that the right motor should be 25% of the way from -1 to 0...so -0.75 is the
        percentage for the right motor at 11.25 degrees.
        """

        if 0 <= angle <= 45:

            # left motor stays at 1
            left_speed_percentage = 1

            # right motor transitions from -1 to 0
            right_speed_percentage = -1 + (angle / 45.0)

        elif 45 < angle <= 90:

            # left motor stays at 1
            left_speed_percentage = 1

            # right motor transitions from 0 to 1
            percentage_from_45_to_90 = (angle - 45) / 45.0
            right_speed_percentage = percentage_from_45_to_90

        elif 90 < angle <= 135:

            # left motor transitions from 1 to 0
            percentage_from_90_to_135 = (angle - 90) / 45.0
            left_speed_percentage = 1 - percentage_from_90_to_135

            # right motor stays at 1
            right_speed_percentage = 1

        elif 135 < angle <= 180:

            # left motor transitions from 0 to -1
            percentage_from_135_to_180 = (angle - 135) / 45.0
            left_speed_percentage = -1 * percentage_from_135_to_180

            # right motor stays at 1
            right_speed_percentage = 1

        elif 180 < angle <= 225:

            # left motor transitions from -1 to 0
            percentage_from_180_to_225 = (angle - 180) / 45.0
            left_speed_percentage = -1 + percentage_from_180_to_225

            # right motor transitions from 1 to -1
            # right motor transitions from 1 to 0 between 180 and 202.5
            if angle < 202.5:
                percentage_from_180_to_202 = (angle - 180) / 22.5
                right_speed_percentage = 1 - percentage_from_180_to_202

            # right motor is 0 at 202.5
            elif angle == 202.5:
                right_speed_percentage = 0

            # right motor transitions from 0 to -1 between 202.5 and 225
            else:
                percentage_from_202_to_225 = (angle - 202.5) / 22.5
                right_speed_percentage = -1 * percentage_from_202_to_225

        elif 225 < angle <= 270:

            # left motor transitions from 0 to -1
            percentage_from_225_to_270 = (angle - 225) / 45.0
            left_speed_percentage = -1 * percentage_from_225_to_270

            # right motor stays at -1
            right_speed_percentage = -1

        elif 270 < angle <= 315:

            # left motor stays at -1
            left_speed_percentage = -1

            # right motor transitions from -1 to 0
            percentage_from_270_to_315 = (angle - 270) / 45.0
            right_speed_percentage = -1 + percentage_from_270_to_315

        elif 315 < angle <= 360:

            # left motor transitions from -1 to 1
            # left motor transitions from -1 to 0 between 315 and 337.5
            if angle < 337.5:
                percentage_from_315_to_337 = (angle - 315) / 22.5
                left_speed_percentage = (1 - percentage_from_315_to_337) * -1

            # left motor is 0 at 337.5
            elif angle == 337.5:
                left_speed_percentage = 0

            # left motor transitions from 0 to 1 between 337.5 and 360
            elif angle > 337.5:
                percentage_from_337_to_360 = (angle - 337.5) / 22.5
                left_speed_percentage = percentage_from_337_to_360

            # right motor transitions from 0 to -1
            percentage_from_315_to_360 = (angle - 315) / 45.0
            right_speed_percentage = -1 * percentage_from_315_to_360

        else:
            raise Exception(
                'You created a circle with more than 360 degrees ({})...that is quite the trick'.format(angle))

        return (left_speed_percentage * 100, right_speed_percentage * 100)

# Declare motors 
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
claw_motor = Motor(Port.D)

#Declare sensors
touch_sensor = TouchSensor(Port.S1)
# object_methods = [method_name for method_name in dir(touch_sensor)
#                   if callable(getattr(touch_sensor, method_name))]
# for a in object_methods:
#     print(a)

speedY = 0
speedX = 0
left_speed = 0
right_speed = 0

def handle_sensors(is_running, touch_sensor):
    while(True):
        if touch_sensor.pressed():
            ev3.speaker.beep()
            os._exit(1)


_thread.start_new_thread(handle_sensors, (True, touch_sensor))

# Open the Gamepad event file:
# /dev/input/event3 is for PS3 gamepad
# /dev/input/event4 is for PS4 gamepad
# look at contents of /proc/bus/input/devices if either one of them doesn't work.
# use 'cat /proc/bus/input/devices' and look for the event file.
infile_path = "/dev/input/event4"

# open file in binary mode
in_file = open(infile_path, "rb")

# Read from the file
# long int, long int, unsigned short, unsigned short, unsigned int
FORMAT = 'llHHI'    
EVENT_SIZE = struct.calcsize(FORMAT)
event = in_file.read(EVENT_SIZE)

while event:

    (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)

    if ev_type == 3:
        if code == 4: # Right Stick Y
            speedY = scale(value, (0,255), (100,-100))
            left_speed, right_speed = get_speeds(speedX, speedY)
        elif code == 3: # Right Stick X
            speedX = scale(value, (0,255), (-100,100))
            left_speed, right_speed = get_speeds(speedX, speedY)
        
    # if code == 16:
    #     if value == -1: # D-Pad Up Pressed
    #         claw_motor.dc(-100)
    #     elif value == 0: # D-Pad Released
    #         claw_motor.dc(0)
    #     elif value == 1: # D-Pad Down Pressed
    #         claw_motor.dc(100)
    

    if code == 307 and value == 1: # Triangle button is pressed
        claw_motor.dc(-100)
    elif code == 307 and value == 0: #Triangle released
        claw_motor.dc(0)

    if code == 304 and value == 1: # X button is pressed
        claw_motor.dc(100)
    elif code == 304 and value == 0: # X button is released
        claw_motor.dc(0)

    # Set motor voltages. 
    left_motor.dc(left_speed)
    right_motor.dc(right_speed)

    # Finally, read another event
    event = in_file.read(EVENT_SIZE)

in_file.close()
