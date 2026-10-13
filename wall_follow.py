# Left wall follower for the block maze (grid of 28 x 28 cm cells, corridors 28 cm wide).
#
# Robot: drives with the caster end first, so the big wheels are at the BACK.
#   LIDAR (S2): front-left, 12.5 cm ahead of the wheel axle, pointing straight LEFT, low
#   TOUCH (S1): front bumper, 18 cm ahead of the wheel axle
# Usage: put the robot in a corridor with a wall on its LEFT and run. LEFT button = stop.
#
# Left-hand rule:
#   wall on the left        -> follow it at TARGET_MM
#   opening on the left     -> drive on until the axle is in the middle of the opening,
#                              pivot left 90 degrees, go on
#   bumper pressed          -> back up a little, pivot right 90 degrees, go on
# Distances and turns are measured with the wheel encoders.
from time import sleep_ms
from lib.robot_consts import Sensor, Port, Button

# Motors: driving "backwards", so forward = negative power and the wheels swap sides.
LEFT_MOTOR = Port.M1    # left wheel in the driving direction (swap with RIGHT if it turns the wrong way)
RIGHT_MOTOR = Port.M2
FORWARD = -1            # set to 1 if it drives the wrong way
LIDAR_PORT = Port.S2
TOUCH_PORT = Port.S1

# Robot and maze, cm
CELL_CM = 28
LIDAR_AHEAD_CM = 12.5   # lidar ahead of the wheel axle
WHEEL_CM = 8.16         # wheel diameter
DEG_PER_CM = 360 / (3.1416 * WHEEL_CM)
TURN_90_DEG = 210       # wheel degrees each wheel turns for a 90 degree pivot - tune this first

# Wall following
TARGET_MM = 55          # lidar distance to the left wall (robot about centred in the corridor)
OPEN_MM = 150           # further than this = opening on the left
BASE_POWER = 30         # forward power
KP = 0.4                # steering power per mm off the target
KD = 3.0                # steering power per mm the distance changed since the last reading
TURN_MAX = 20           # max steering power while following
MIN_VALID_MM = 20       # readings below this are invalid and skipped

# Moves
MOVE_POWER = 30
TURN_POWER = 30
BACK_CM = 2             # back up after a bump, so the front clears the wall while pivoting
SKIP_CM = 3             # after a left turn, drive this far before looking at the lidar again


def log(message):
    robot.esp.bt_write(message + ";")
    print(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def run():
    robot.init_sensor(sensor_type=Sensor.OC_LASER, port=LIDAR_PORT)
    robot.init_sensor(sensor_type=Sensor.NXT_TOUCH, port=TOUCH_PORT)
    robot.init_motor(LEFT_MOTOR)
    robot.init_motor(RIGHT_MOTOR)
    lidar = robot.sensors.laser[LIDAR_PORT]
    bumper = robot.sensors.touch[TOUCH_PORT]
    left = robot.motors[LEFT_MOTOR]
    right = robot.motors[RIGHT_MOTOR]
    left.init_encoder()
    right.init_encoder()

    def wheels(left_power, right_power):
        left.set_power(FORWARD * left_power)
        right.set_power(FORWARD * right_power)

    def stop_pressed():
        return robot.buttons.pressed()[Button.LEFT]

    def travelled(l0, r0):
        # Wheel degrees turned since l0, r0 (average of both wheels, direction ignored)
        return (abs(left.position() - l0) + abs(right.position() - r0)) / 2

    def move(cm, power):
        # Drive straight cm (power > 0 forward, < 0 back). Returns True if the bumper hit a wall.
        l0, r0 = left.position(), right.position()
        wheels(power, power)
        while travelled(l0, r0) < cm * DEG_PER_CM and not stop_pressed():
            if power > 0 and bumper.pressed():
                wheels(0, 0)
                return True
            sleep_ms(5)
        wheels(0, 0)
        return False

    def turn(direction):
        # Pivot 90 degrees on the spot: +1 right, -1 left
        sleep_ms(100)
        l0, r0 = left.position(), right.position()
        wheels(direction * TURN_POWER, -direction * TURN_POWER)
        while travelled(l0, r0) < TURN_90_DEG and not stop_pressed():
            sleep_ms(5)
        wheels(0, 0)
        sleep_ms(100)

    sleep_ms(1000)
    log("start, lidar {} mm".format(lidar.distance()))
    d_prev = None
    open_count = 0
    loops = 0
    while not stop_pressed():
        # Wall ahead: back up a little, turn right
        if bumper.pressed():
            log("wall ahead -> right")
            wheels(0, 0)
            move(BACK_CM, -MOVE_POWER)
            turn(1)
            d_prev = None
            open_count = 0
            continue

        d = lidar.distance()
        if d < MIN_VALID_MM:
            sleep_ms(10)
            continue

        # Opening on the left: drive on to its middle, turn left
        if d > OPEN_MM:
            open_count += 1
            if open_count >= 2:
                log("opening on the left ({} mm) -> left".format(d))
                if move(LIDAR_AHEAD_CM + CELL_CM / 2, MOVE_POWER):
                    move(BACK_CM, -MOVE_POWER)      # wall ahead too: back up so it can pivot
                turn(-1)
                move(SKIP_CM, MOVE_POWER)
                d_prev = None
                open_count = 0
                continue
            wheels(BASE_POWER, BASE_POWER)
            sleep_ms(20)
            continue
        open_count = 0

        # Follow the left wall: too far -> turn left, too close -> turn right
        change = 0 if d_prev is None else d - d_prev
        d_prev = d
        u = clamp(KP * (d - TARGET_MM) + KD * change, -TURN_MAX, TURN_MAX)
        wheels(BASE_POWER - u, BASE_POWER + u)

        loops += 1
        if loops % 25 == 0:
            log("wall {} mm".format(d))
        sleep_ms(20)

    wheels(0, 0)
    log("Stopped")


try:
    run()
except Exception as ex:
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
