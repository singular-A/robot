'''
Example program of a simple line follower robot with 2 motors, 1 OC RGB sensor using a PID regulator.
Fill TODO parts with your own code.
'''
from lib.robot import *
import time, machine
from lib.robot_consts import Sensor, Port, Button

from lib.robot_consts import Button, Port, Sensor, Light

# TODO Change these constants
REGULATION_PERIOD_MS = 10       # Regulation period 0.010 s <=> 100 Hz
MOTOR_BASE_POWER = 30           # 0-100 %
LIGHT_SETPOINT = 500           #  
Kp, Ki, Kd = 1, 0, 0            # PID constants

robot.init_sensor(sensor_type=Sensor.OC_COLOR,port=Port)
current = sensor.reflection_raw()
print(current)


# If the program is run from the menu, access global robot variable
# If it is run independently, robot is initialized
independent_run = False
#global robot
try :
    robot
except :
    robot = Robot()
    independent_run = True

# Initialize motors on ports M2 and M3
robot.init_motor(Port.M1)
robot.init_motor(Port.M2)

# Initialize Open-Cube RGB sensor on port S1
robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)

# Regulator variables
e, e_sum, e_prev, motor_pwr = 0, 0, 0, 0

# Follower regulation loop
while True:
    # Read light intensity
    light_intensity = robot.sensors.light[Port.S1].reflection_raw() 

    #TODO - PID regulator
    # 1. calculate error value e from light setpoint and measured ligth intensity
    #e = 
    black = sensor.reflection()
    target = (white + black) / 2
    half = (white - black) / 2

    # 2. calculate motor_pwr using error variables (e, e_sum, e_prev) and PID constants (Kp, Ki, Kd)
   # motor_pwr = 

    # 3. save previous error for the derivative part
    #e_prev = 

    # 4. Save sum of errors for the integral part
    # e_sum = 


    # Set motor power
    robot.motors[Port.M1].set_power(MOTOR_BASE_POWER + motor_pwr)
    robot.motors[Port.M2].set_power(MOTOR_BASE_POWER - motor_pwr)

    # Do nothing
    time.sleep_ms(REGULATION_PERIOD_MS)
    print("ss")
    # Exit program if left cube button is pressed
    buttons = robot.buttons.pressed()
    if buttons[Button.LEFT]:
        break
    
# Reset back to menu
if independent_run:
    machine.reset()
