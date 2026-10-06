# Import sleep function and port constants
from time import sleep
from lib.robot_consts import Port

# Initialize motors on motor ports 1 and 2
robot.init_motor(Port.M1)
robot.init_motor(Port.M2)

# Initialize motors encoders
robot.motors[Port.M1].init_encoder()
robot.motors[Port.M2].init_encoder()

# Set power on the motors to 50%, and 20% in the opposite direction of rotation
robot.motors[Port.M1].set_power(50)
robot.motors[Port.M2].set_power(-20)

# Keep the program running, otherwise the motors are deinitialized (stopped)
# as soon as the program exits. Report position and speed once per second.
for _ in range(5):
    sleep(1)
    pos1 = robot.motors[Port.M1].position()
    pos2 = robot.motors[Port.M2].position()
    speed1 = robot.motors[Port.M1].speed()
    speed2 = robot.motors[Port.M2].speed()
    robot.esp.bt_write("M1 {} deg {} deg/s | M2 {} deg {} deg/s;".format(pos1, speed1, pos2, speed2))

# Stop the motors
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)

