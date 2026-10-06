from lib.robot import *
import time, machine
from lib.robot_consts import Button, Port, Sensor, Light

current = sensor.reflection_raw()
robot.init_sensor(sensor_type=Sensor.OC_COLOR,port=Port)