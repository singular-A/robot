# Left wall follower for the block maze (grid of 28 x 28 cm cells, corridors 28 cm wide).
#
# Robot: drives with the caster end first, so the big wheels are at the BACK.
#   LIDAR (S2): front-left, 13.5 cm ahead of the wheel axle, pointing straight LEFT, low
#   TOUCH (S1): front bumper, 16.5 cm ahead of the wheel axle
# Usage: put the robot in the middle of the start box C5, facing out of it (toward D5), and
#        run. LEFT button = stop.
#
# The robot is too long to pivot in the middle of a cell that has a wall ahead (the bumper
# touches the wall before the axle reaches the middle). So there are two kinds of turns:
#   pivot   - on the spot, only in the middle of a cell with nothing ahead
#   corner  - after the bumper touched a wall ahead: back up ARC_BACK_CM, then turn with the
#             inner wheel standing still. That moves the axle forward and sideways during the
#             turn, so it ends near the middle of the cell, facing the new way.
# The robot keeps track of where it is in the cell grid along the corridor (p = axle position
# from the middle of the current cell, cm, from the encoders), so it turns in the middle of
# a cell. Every bump corrects p (the wall ahead is always at the end of a cell).
#
# Start: drive straight out of C5 until the bumper touches block E5, corner turn right
# (now in D5 facing D6). The left-hand rule then turns left at D6 into E6 by itself.
#
# Left-hand rule:
#   wall on the left        -> follow it at TARGET_MM
#   opening on the left     -> drive on to the middle of that cell, turn left
#   bumper pressed          -> corner turn left if the left is open, else right
#
# Log: saved on the cube in LOG_FILE (run read_log.py afterwards to print it to the Thonny
# log panel). Events (EV ...) are also sent live over Bluetooth. Every LOG_MS one data line:
#   D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power,p_cm
#   mode: follow / invalid / open / gap / back / pivotL / pivotR / cornerL / cornerR / out
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
LIDAR_AHEAD_CM = 13.5   # lidar ahead of the wheel axle
TOUCH_AHEAD_CM = 16.5   # bumper ahead of the wheel axle
TRACK_CM = 19.0         # distance between the middles of the two wheels - measure it
WHEEL_CM = 8.16         # wheel diameter
DEG_PER_CM = 360 / (3.1416 * WHEEL_CM)
TURN_90_DEG = 210       # wheel degrees each wheel turns for a 90 degree pivot - tune this first
# When the bumper touches the wall ahead, the axle is BUMP_POS_CM from the middle of the cell
# (-2.5: just before it). A corner turn moves the axle TRACK_CM / 2 forward and sideways, so
# it backs up to -TRACK_CM / 2 first and ends in the middle.
BUMP_POS_CM = CELL_CM / 2 - TOUCH_AHEAD_CM
ARC_BACK_CM = BUMP_POS_CM + TRACK_CM / 2

# Wall following
TARGET_MM = 55          # lidar distance to the left wall (robot about centred in the corridor)
OPEN_MM = 150           # further than this = opening on the left
OPEN_CONFIRM = 2        # this many readings in a row before it believes an opening
BASE_POWER = 30         # forward power
KP = 0.4                # steering power per mm off the target
KD = 3.0                # steering power per mm the distance changed since the last reading
TURN_MAX = 20           # max steering power while following
MIN_VALID_MM = 20       # readings below this are invalid and skipped

# Moves
MOVE_POWER = 30
TURN_POWER = 30
STRAIGHT_K = 0.5        # straight moves: power correction per wheel degree one wheel is ahead
STRAIGHT_MAX = 10       # max power correction
STALL_MS = 600          # wheels hardly turned for this long while moving/turning = stuck
START = True            # False: skip the start out of C5 (start anywhere, wall on the left)

LOG_MS = 40             # one data line every this many ms (25 per second)
LOG_FILE = "wall_log.csv"   # on the cube; overwritten at every start


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
    # p: axle position from the middle of the current cell along the driving direction, cm
    # min_k: cells before this one (0 = the current cell) are ignored when looking for an
    #        opening on the left (right after a turn, the left of the current cell is where
    #        the robot came from)
    st = {"p": 0.0, "min_k": 1}

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
        log("D,{},{},{},{},{},{},{:.0f},{:.0f},{:.1f}".format(
            ticks_diff(now, t0), mode, d, 1 if bumper.pressed() else 0,
            left.position(), right.position(), power[0], power[1], st["p"]), live=False)
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
        # Drive straight cm (power > 0 forward, < 0 back), kept straight with the encoders.
        # Returns True if it hit something going forward (bumper, or wheels stuck).
        event("{} {:.1f} cm start (p {:.1f})".format(mode, cm, st["p"]))
        l0, r0 = left.position(), right.position()
        progress = [0, ticks_ms()]
        sign = 1 if move_power > 0 else -1
        hit = False
        while travelled(l0, r0) < cm * DEG_PER_CM and not stop_pressed():
            if move_power > 0 and bumper.pressed():
                event("{} bumped".format(mode))
                hit = True
                break
            if stalled(l0, r0, progress):
                event("{} STUCK".format(mode))
                hit = move_power > 0
                break
            ahead = abs(left.position() - l0) - abs(right.position() - r0)
            u = clamp(STRAIGHT_K * ahead, -STRAIGHT_MAX, STRAIGHT_MAX)
            wheels(move_power - sign * u, move_power + sign * u)
            row(mode)
            sleep_ms(5)
        wheels(0, 0)
        st["p"] += sign * travelled(l0, r0) / DEG_PER_CM
        event("{} done {:.1f} cm (p {:.1f})".format(mode, travelled(l0, r0) / DEG_PER_CM, st["p"]))
        row(mode, force=True)
        return hit

    def turn(direction, arc):
        # 90 degrees: direction +1 right, -1 left.
        # arc False: pivot on the spot (both wheels, opposite ways).
        # arc True: inner wheel stands still, outer wheel turns twice as far (corner turn).
        mode = ("corner" if arc else "pivot") + ("R" if direction > 0 else "L")
        sleep_ms(100)
        event("{} start".format(mode))
        if arc:
            outer = left if direction > 0 else right
            goal = 2 * TURN_90_DEG
            powers = (TURN_POWER, 0) if direction > 0 else (0, TURN_POWER)
        else:
            goal = TURN_90_DEG
            powers = (direction * TURN_POWER, -direction * TURN_POWER)
        done = 0                 # wheel degrees of the turn done so far
        for attempt in range(3):
            l0, r0 = left.position(), right.position()
            progress = [0, ticks_ms()]
            now_done = done
            wheels(*powers)
            while not stop_pressed():
                if arc:
                    now_done = done + abs(outer.position() - (l0 if outer is left else r0))
                else:
                    now_done = done + travelled(l0, r0)
                if now_done >= goal or stalled(l0, r0, progress):
                    break
                row(mode)
                sleep_ms(5)
            wheels(0, 0)
            done = now_done
            if done >= goal or stop_pressed():
                break
            # Stuck on something while turning: back up a little and finish the turn
            event("{} STUCK at {:.0f}/{} deg -> back up, retry".format(mode, done, goal))
            row(mode, force=True)
            move(2, -MOVE_POWER, "back")
        event("{} done {:.0f}/{} deg".format(mode, done, goal))
        row(mode, force=True)
        sleep_ms(100)

    def corner_turn(direction):
        # Called when the bumper touched the wall ahead
        st["p"] = BUMP_POS_CM
        move(ARC_BACK_CM, -MOVE_POWER, "back")
        turn(direction, True)
        st["p"] = TRACK_CM / 2           # the turn moved the axle this far in the new direction
        st["min_k"] = 1

    sleep_ms(1000)
    log("CFG,TARGET_MM={},OPEN_MM={},BASE={},KP={},KD={},TURN_90_DEG={},TRACK_CM={},ARC_BACK_CM={}".format(
        TARGET_MM, OPEN_MM, BASE_POWER, KP, KD, TURN_90_DEG, TRACK_CM, ARC_BACK_CM))
    log("HDR,D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power,p_cm")
    t0 = ticks_ms()
    event("start")

    if START:
        # Out of C5 until the bumper touches E5, then corner turn right (into D5, facing D6)
        st["p"] = -CELL_CM              # roughly: the middle of C5, one cell before D5
        if move(4 * CELL_CM, MOVE_POWER, "out"):
            corner_turn(1)
        else:
            event("WARNING: no wall found ahead after leaving C5")

    d_prev = None
    open_count = 0
    l_last, r_last = left.position(), right.position()
    while not stop_pressed():
        # Where along the corridor is the axle (only forward moves while following)
        l, r = left.position(), right.position()
        st["p"] += (abs(l - l_last) + abs(r - r_last)) / 2 / DEG_PER_CM
        l_last, r_last = l, r

        # Wall ahead: corner turn, left if the left is open, else right
        if bumper.pressed():
            wheels(0, 0)
            d = lidar.distance()
            left_open = d > OPEN_MM
            event("bump at p {:.1f}, lidar {} mm -> corner {}".format(
                st["p"], d, "left" if left_open else "right"))
            row("follow", d, force=True)
            corner_turn(-1 if left_open else 1)
            d_prev = None
            open_count = 0
            l_last, r_last = left.position(), right.position()
            continue

        d = lidar.distance()
        if d < MIN_VALID_MM:
            row("invalid", d)
            sleep_ms(10)
            continue

        # Opening on the left: which cell is the lidar next to? (k = 0 current, 1 next, ...)
        k = round((st["p"] + LIDAR_AHEAD_CM) / CELL_CM)
        if d > OPEN_MM and k >= st["min_k"]:
            open_count += 1
            if open_count >= OPEN_CONFIRM:
                to_go = k * CELL_CM - st["p"]
                event("opening on the left ({} mm), cell +{}, {:.1f} cm to its middle".format(d, k, to_go))
                row("open", d, force=True)
                if to_go >= 0:
                    hit = move(to_go, MOVE_POWER, "gap")
                else:
                    hit = False
                    move(-to_go, -MOVE_POWER, "gap")
                if hit:
                    corner_turn(-1)                 # wall ahead there too
                else:
                    turn(-1, False)                 # pivot in the middle of the cell
                    st["p"] = 0
                    st["min_k"] = 1
                d_prev = None
                open_count = 0
                l_last, r_last = left.position(), right.position()
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
