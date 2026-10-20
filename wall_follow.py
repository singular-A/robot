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
#
# Log: saved on the cube in LOG_FILE (run read_log.py afterwards to print it to the Thonny
# log panel). Events (EV ...) are also sent live over Bluetooth. Every LOG_MS one data line:
#   D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power
#   mode: follow / invalid (lidar reading skipped) / open (opening seen, not confirmed yet) /
#   gap (driving to the middle of the opening) / turnL / turnR / back / skip.
#   Encoders in wheel degrees, powers as sent to the wheels in driving direction (+ = forward).
from time import sleep_ms, ticks_ms, ticks_diff
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
LOG_MS = 40             # one data line every this many ms (25 per second)
LOG_FILE = "wall_log.csv"   # on the cube; overwritten at every start
STALL_MS = 600          # wheels hardly turned for this long while moving/turning = stuck


log_file = [None]


def log(message, live=True):
    # To the log file, and (live) also over Bluetooth to the Thonny log panel
    if log_file[0]:
        log_file[0].write(message + "\n")
    if live:
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

    t0 = ticks_ms()
    power = [0, 0]          # last powers sent, in driving direction
    last_row = [0]
    last_flush = [0]

    def wheels(left_power, right_power):
        power[0], power[1] = left_power, right_power
        left.set_power(FORWARD * left_power)
        right.set_power(FORWARD * right_power)

    def event(text):
        log("EV,{},{}".format(ticks_diff(ticks_ms(), t0), text))

    def row(mode, d=None, force=False):
        # One data line, at most every LOG_MS (force: always)
        now = ticks_ms()
        if not force and ticks_diff(now, last_row[0]) < LOG_MS:
            return
        last_row[0] = now
        if d is None:
            d = lidar.distance()
        log("D,{},{},{},{},{},{},{:.0f},{:.0f}".format(
            ticks_diff(now, t0), mode, d, 1 if bumper.pressed() else 0,
            left.position(), right.position(), power[0], power[1]), live=False)
        if ticks_diff(now, last_flush[0]) >= 1000:   # save to flash about once a second
            log_file[0].flush()
            last_flush[0] = now

    def stop_pressed():
        return robot.buttons.pressed()[Button.LEFT]

    def travelled(l0, r0):
        # Wheel degrees turned since l0, r0 (average of both wheels, direction ignored)
        return (abs(left.position() - l0) + abs(right.position() - r0)) / 2

    def stalled(l0, r0, progress):
        # progress = [last travelled value, time of last real progress]
        t = travelled(l0, r0)
        now = ticks_ms()
        if t > progress[0] + 3:
            progress[0], progress[1] = t, now
        return ticks_diff(now, progress[1]) > STALL_MS

    def move(cm, move_power, mode):
        # Drive straight cm (power > 0 forward, < 0 back).
        # Returns True if it hit something going forward (bumper, or wheels stuck).
        event("{} {} cm start".format(mode, cm))
        l0, r0 = left.position(), right.position()
        progress = [0, ticks_ms()]
        wheels(move_power, move_power)
        while travelled(l0, r0) < cm * DEG_PER_CM and not stop_pressed():
            if move_power > 0 and bumper.pressed():
                wheels(0, 0)
                event("{} bumped after {:.1f} cm".format(mode, travelled(l0, r0) / DEG_PER_CM))
                row(mode, force=True)
                return True
            if stalled(l0, r0, progress):
                wheels(0, 0)
                event("{} STUCK after {:.1f} cm".format(mode, travelled(l0, r0) / DEG_PER_CM))
                row(mode, force=True)
                return move_power > 0
            row(mode)
            sleep_ms(5)
        wheels(0, 0)
        event("{} done {:.1f} cm".format(mode, travelled(l0, r0) / DEG_PER_CM))
        row(mode, force=True)
        return False

    def turn(direction):
        # Pivot 90 degrees on the spot: +1 right, -1 left
        mode = "turnR" if direction > 0 else "turnL"
        sleep_ms(100)
        event("{} start".format(mode))
        done = 0                 # wheel degrees of the turn done so far
        for attempt in range(3):
            l0, r0 = left.position(), right.position()
            progress = [0, ticks_ms()]
            wheels(direction * TURN_POWER, -direction * TURN_POWER)
            while done + travelled(l0, r0) < TURN_90_DEG and not stop_pressed():
                if stalled(l0, r0, progress):
                    break
                row(mode)
                sleep_ms(5)
            wheels(0, 0)
            done += travelled(l0, r0)
            if done >= TURN_90_DEG or stop_pressed():
                break
            # Stuck on something while turning: back up a little and finish the turn
            event("{} STUCK at {:.0f}/{} deg -> back up, retry".format(mode, done, TURN_90_DEG))
            row(mode, force=True)
            move(BACK_CM, -MOVE_POWER, "back")
        event("{} done {:.0f}/{} deg".format(mode, done, TURN_90_DEG))
        row(mode, force=True)
        sleep_ms(100)

    sleep_ms(1000)
    log("CFG,TARGET_MM={},OPEN_MM={},BASE={},KP={},KD={},TURN_90_DEG={},WHEEL_CM={}".format(
        TARGET_MM, OPEN_MM, BASE_POWER, KP, KD, TURN_90_DEG, WHEEL_CM))
    log("HDR,D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power")
    t0 = ticks_ms()
    event("start")
    d_prev = None
    open_count = 0
    while not stop_pressed():
        # Wall ahead: back up a little, turn right
        if bumper.pressed():
            wheels(0, 0)
            event("bump -> back, turn right")
            row("follow", force=True)
            move(BACK_CM, -MOVE_POWER, "back")
            turn(1)
            d_prev = None
            open_count = 0
            continue

        d = lidar.distance()
        if d < MIN_VALID_MM:
            row("invalid", d)
            sleep_ms(10)
            continue

        # Opening on the left: drive on to its middle, turn left
        if d > OPEN_MM:
            open_count += 1
            if open_count >= 2:
                event("opening on the left ({} mm) -> to its middle, turn left".format(d))
                row("open", d, force=True)
                if move(LIDAR_AHEAD_CM + CELL_CM / 2, MOVE_POWER, "gap"):
                    move(BACK_CM, -MOVE_POWER, "back")      # wall ahead too: back up so it can pivot
                turn(-1)
                move(SKIP_CM, MOVE_POWER, "skip")
                d_prev = None
                open_count = 0
                continue
            wheels(BASE_POWER, BASE_POWER)
            row("open", d)
            sleep_ms(20)
            continue
        open_count = 0

        # Follow the left wall: too far -> turn left, too close -> turn right
        change = 0 if d_prev is None else d - d_prev
        d_prev = d
        u = clamp(KP * (d - TARGET_MM) + KD * change, -TURN_MAX, TURN_MAX)
        wheels(BASE_POWER - u, BASE_POWER + u)
        row("follow", d)
        sleep_ms(20)

    wheels(0, 0)
    event("stopped")


try:
    log_file[0] = open(LOG_FILE, "w")
    run()
except Exception as ex:
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
if log_file[0]:
    log_file[0].close()
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
