# Lego Mindstorms Robot
De robot kan een lijn volgen, obstakels ontwijken en een knop indrukken.

Website van de robot: https://placeholder.text

Lego Mindstorms Python code
```python
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase


# Main Variables
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
arm = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
button = TouchSensor(Port.S2)
line_sensor = ColorSensor(Port.S3)
obstacle_sensor = UltrasonicSensor(Port.S4)


# Arm Variables
ARM_SPEED = 250
ARM_TARGET_ANGLE = 120


# Wheels & Colour Sensor Variables
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
DRIVE_SPEED = 100
PROPORTIONAL_GAIN = -2.0
TURN_DEGREES = 70
DISTANCE = 150
BLACK = 12
WHITE = 100
threshold = (BLACK + WHITE) / 2


# Setup
# Background Calibration
while True:
    background = line_sensor.reflection()
    if button.pressed():
        break
    
    ev3.screen.clear()
    ev3.screen.draw_text(0, 0, "Calibration")
    ev3.screen.draw_text(0, 30, "Background: " + str(background))
    wait(50)

wait(500)
# Line Calibration
while True:
    line = line_sensor.reflection()
    if button.pressed():
        break
    
    ev3.screen.clear()
    ev3.screen.draw_text(0, 0, "Calibration")
    ev3.screen.draw_text(0, 30, "Line: " + str(line))
    wait(50)

# Set variables to new values and recalculate threshold
BLACK = line
WHITE = background
threshold = (BLACK + WHITE) / 2

ev3.screen.clear()
ev3.screen.draw_text(0, 0, "Done")
ev3.screen.draw_text(0, 30, "Background: " + str(WHITE))
ev3.screen.draw_text(0, 50, "Line: " + str(BLACK))
ev3.screen.draw_text(0, 80, "Press button to start")
wait(500)

# Execute code after 2 seconds if button is pressed
while True:
    if button.pressed():
        break

ev3.screen.clear()
ev3.screen.draw_text(0, 50, "Executing code")
ev3.screen.draw_text(0, 70, "In 2 seconds")
wait(1000)
ev3.screen.clear()
ev3.screen.draw_text(0, 50, "Executing code")
ev3.screen.draw_text(0, 70, "In 1 second")
wait(1000)
ev3.screen.clear()


# 
# ALL FUNCTIONS
#
def move_arm():
    while arm.angle() < ARM_TARGET_ANGLE:
        arm.run_target(ARM_SPEED, ARM_TARGET_ANGLE)

    while arm.angle() > 0:
        arm.run_target(ARM_SPEED, 0)

def avoid_obstacle():
    if obstacle_sensor.distance() < DISTANCE:
        robot.stop()
        wait(100)
        robot.straight(-100)
        robot.turn(TURN_DEGREES)
        wait(20)


# 
# MAIN CODE
# 
while True:
    avoid_obstacle()
    
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)
    
    ev3.screen.clear()
    ev3.screen.draw_text(0, 50, str(line_sensor.color()))
    if line_sensor.color() == Color.GREEN:
        robot.stop()
        break
    # wait(500)

move_arm()
ev3.screen.clear()
ev3.screen.draw_text(0, 50, "Done")
```
