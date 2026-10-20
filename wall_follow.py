# Left wall follower for the block maze (grid of 28 x 28 cm cells, corridors 28 cm wide).
#
# Robot: drives with the caster end first, so the big wheels are at the BACK.
#   LIDAR (S2): front-left, 13.5 cm ahead of the wheel axle, pointing straight LEFT, low
#   TOUCH (S1): front bumper, 16.5 cm ahead of the wheel axle
# Usage: put the robot in the middle of the start box C5, facing out of it (toward D5), and
#        run. LEFT button = stop.
#
# 1. Start (fixed moves): C5 -> D5 -> D6 -> E6
#    - straight out of C5 until the bumper touches block E5
#    - corner turn right: back up ARC_BACK_CM, turn with the right wheel standing still
#      (the robot is too long to pivot in D5 with E5 right in front of it)
#    - straight to the middle of D6, pivot left, a little into E6: block E5 is on the left
# 2. Then it just follows that one wall edge on its LEFT, all the way around:
#    - wall on the left: keep TARGET_MM from it
#    - the wall ends (corner of a block): drive on until the axle is at the corner, then
#      curve left around it until the lidar sees the wall again
#    - bumper touches a wall ahead (inside corner): back up BUMP_BACK_CM, pivot 90 degrees
#      right on the spot (one wheel forward, the other backward = the sharpest turn)
#    - wheels stuck without the bumper pressed (e.g. the front-left corner hit a block):
#      back up STUCK_BACK_CM, pivot 90 degrees right
#    - dead end (it bumps into a wall again during that turn): undo the turn, back out to
#      the middle of the cell before, turn around on the spot (swinging the front away from
#      the wall on the left if there is one) and go on
#
# Log: printed to the console (Thonny Open-Cube log panel). Events start with EV, and every
# LOG_MS one data line:
#   D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power
#   mode: out / back / cornerR / toD6 / pivotL / intoE6 / follow / invalid / open / tocorner /
#         around / undo / deadback / pivotR
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
# Corner turn after a bump: when the bumper touches the wall ahead, the axle is
# TOUCH_AHEAD_CM - CELL_CM / 2 (2.5 cm) short of the middle of the cell. The turn around the
# inner wheel moves the axle TRACK_CM / 2 forward and sideways, so it first backs up to
# TRACK_CM / 2 before the middle and ends in the middle.
BUMP_SHORT_CM = TOUCH_AHEAD_CM - CELL_CM / 2   # axle this far before the cell middle at a bump
ARC_BACK_CM = TRACK_CM / 2 - BUMP_SHORT_CM
BUMP_BACK_CM = 2        # in the maze: back up this far after a bump, then pivot right

# Wall following
TARGET_MM = 55          # lidar distance to the left wall (robot about centred in the corridor)
OPEN_MM = 150           # further than this = opening on the left
OPEN_CONFIRM = 2        # this many readings in a row before it believes the wall ended
AROUND_INNER = 0.2      # going around a block corner: left wheel power = this x right wheel power
AROUND_MAX_DEG = 200    # give up curving around a corner after this many degrees of robot turn
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
STUCK_BACK_CM = 4       # stuck while following (no bump): back up this far, then pivot right
START = True            # False: skip the start (start anywhere, wall on the left)
START_INTO_E6_CM = 5    # after the pivot in D6: this far, so the lidar is beside block E5

LOG_MS = 100            # one data line every this many ms (10 per second)


def log(message):
    # To the console: over Bluetooth to the Thonny log panel, and USB
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
            left.position(), right.position(), power[0], power[1]))

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
        event("{} {:.1f} cm start".format(mode, cm))
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
        event("{} done {:.1f} cm".format(mode, travelled(l0, r0) / DEG_PER_CM))
        row(mode, force=True)
        return hit

    def turn(direction, arc, part=1.0, blockable=False):
        # 90 degrees (or part of it): direction +1 right, -1 left.
        # arc False: pivot on the spot (both wheels, opposite ways).
        # arc True: inner wheel stands still, outer wheel turns twice as far.
        # blockable (always for arcs): stop if the bumper touches a wall or the wheels get
        #   stuck on the way, instead of backing up and retrying.
        # Returns (finished, wheel degrees done).
        blockable = blockable or arc
        mode = ("corner" if arc else "pivot") + ("R" if direction > 0 else "L")
        sleep_ms(100)
        event("{} start".format(mode))
        if arc:
            outer = left if direction > 0 else right
            goal = 2 * TURN_90_DEG
            powers = (TURN_POWER, 0) if direction > 0 else (0, TURN_POWER)
        else:
            goal = TURN_90_DEG * part
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
                if blockable and bumper.pressed() and now_done > 20:
                    break
                row(mode)
                sleep_ms(5)
            wheels(0, 0)
            done = now_done
            if done >= goal or stop_pressed():
                break
            if blockable:
                event("{} blocked at {:.0f}/{} deg".format(mode, done, goal))
                row(mode, force=True)
                return False, done
            # Stuck on something while turning: back up a little and finish the turn
            event("{} STUCK at {:.0f}/{} deg -> back up, retry".format(mode, done, goal))
            row(mode, force=True)
            move(2, -MOVE_POWER, "back")
        event("{} done {:.0f}/{} deg".format(mode, done, goal))
        row(mode, force=True)
        sleep_ms(100)
        return True, done

    def corner_turn(direction, arc):
        # Called when the bumper touched the wall ahead.
        # arc False: back up a little, pivot (sharpest turn). arc True: back up more, turn
        # around the inner wheel (ends nearer the middle of the cell; used at the start).
        back = ARC_BACK_CM if arc else BUMP_BACK_CM
        move(back, -MOVE_POWER, "back")
        finished, done = turn(direction, arc, blockable=True)
        if not finished and not stop_pressed():
            dead_end(direction, arc, done, back)

    def dead_end(direction, arc, done, back):
        # The turn ran into a wall too: walls ahead and to the side
        event("DEAD END -> undo the turn, back out, turn around")
        # Undo the part of the turn that was done
        if arc:
            outer = left if direction > 0 else right
            start = outer.position()
            wheels(-TURN_POWER, 0) if direction > 0 else wheels(0, -TURN_POWER)
            t_start = ticks_ms()
            while abs(outer.position() - start) < done and ticks_diff(ticks_ms(), t_start) < 4000 \
                    and not stop_pressed():
                row("undo")
                sleep_ms(5)
            wheels(0, 0)
        else:
            turn(-direction, False, done / TURN_90_DEG)
        # Back out to the middle of the cell before
        move(CELL_CM - BUMP_SHORT_CM - back, -MOVE_POWER, "deadback")
        # Turn around, swinging the front away from the wall on the left (if there is one)
        d = lidar.distance()
        side = 1 if MIN_VALID_MM <= d <= OPEN_MM else -1
        event("turn around {} (lidar {} mm)".format("right" if side > 0 else "left", d))
        turn(side, False)
        turn(side, False)

    def around_corner():
        # The wall on the left ended: curve left around its end until the lidar sees it again.
        # Returns quietly; a bump on the way is handled by the main loop.
        event("around start")
        l0, r0 = left.position(), right.position()
        wheels(MOVE_POWER * AROUND_INNER, MOVE_POWER)
        seen = 0
        while not stop_pressed() and not bumper.pressed():
            # robot turn so far, degrees: (right - left wheel travel) / track
            turned = (abs(right.position() - r0) - abs(left.position() - l0)) / DEG_PER_CM \
                * 180 / 3.1416 / TRACK_CM
            if turned > AROUND_MAX_DEG:
                event("around: no wall found after {:.0f} deg".format(turned))
                break
            d = lidar.distance()
            seen = seen + 1 if MIN_VALID_MM <= d <= OPEN_MM else 0
            row("around", d)
            if seen >= OPEN_CONFIRM:
                event("around done: wall again at {} mm after {:.0f} deg".format(d, turned))
                break
            sleep_ms(10)
        wheels(0, 0)

    sleep_ms(1000)
    log("CFG,TARGET_MM={},OPEN_MM={},BASE={},KP={},KD={},TURN_90_DEG={},TRACK_CM={},ARC_BACK_CM={}".format(
        TARGET_MM, OPEN_MM, BASE_POWER, KP, KD, TURN_90_DEG, TRACK_CM, ARC_BACK_CM))
    log("HDR,D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power")
    t0 = ticks_ms()
    event("start")

    if START:
        event("start: C5 -> D5")
        if move(4 * CELL_CM, MOVE_POWER, "out"):        # until the bumper touches E5
            corner_turn(1, True)                         # now in D5, facing D6
            move(CELL_CM - TRACK_CM / 2, MOVE_POWER, "toD6")   # to the middle of D6
            turn(-1, False)                              # facing E6
            move(START_INTO_E6_CM, MOVE_POWER, "intoE6")
            event("start done, following the wall on the left")
        else:
            event("WARNING: no wall ahead after leaving C5")

    d_prev = None
    open_count = 0
    stall = [left.position(), right.position(), ticks_ms()]   # last place it was moving
    while not stop_pressed():
        # Driving forward but the wheels do not turn: something touched the robot where the
        # bumper is not (front-left corner, side). Back up a little, pivot right.
        if travelled(stall[0], stall[1]) > 5:
            stall[0], stall[1], stall[2] = left.position(), right.position(), ticks_ms()
        elif ticks_diff(ticks_ms(), stall[2]) > STALL_MS:
            wheels(0, 0)
            event("STUCK without bump -> back up, pivot right")
            row("follow", force=True)
            move(STUCK_BACK_CM, -MOVE_POWER, "back")
            turn(1, False)
            d_prev = None
            open_count = 0
            stall[0], stall[1], stall[2] = left.position(), right.position(), ticks_ms()
            continue

        # Wall ahead (inside corner): corner turn right
        if bumper.pressed():
            wheels(0, 0)
            event("bump -> back up, pivot right")
            row("follow", force=True)
            corner_turn(1, False)
            d_prev = None
            open_count = 0
            stall[0], stall[1], stall[2] = left.position(), right.position(), ticks_ms()
            continue

        d = lidar.distance()
        if d < MIN_VALID_MM:
            row("invalid", d)
            sleep_ms(10)
            continue

        # The wall on the left ended (outside corner): drive on until the axle is at the
        # corner (the lidar is LIDAR_AHEAD_CM ahead of it), then curve around it
        if d > OPEN_MM:
            open_count += 1
            if open_count >= OPEN_CONFIRM:
                event("wall ended ({} mm)".format(d))
                row("open", d, force=True)
                if not move(LIDAR_AHEAD_CM, MOVE_POWER, "tocorner"):
                    around_corner()
                d_prev = None
                open_count = 0
                stall[0], stall[1], stall[2] = left.position(), right.position(), ticks_ms()
                continue
            wheels(BASE_POWER, BASE_POWER)
            row("open", d)
            sleep_ms(20)
            continue
        open_count = 0

        # Follow the wall: too far -> turn left, too close -> turn right
        change = 0 if d_prev is None else d - d_prev
        d_prev = d
        u = clamp(KP * (d - TARGET_MM) + KD * change, -TURN_MAX, TURN_MAX)
        wheels(BASE_POWER - u, BASE_POWER + u)
        row("follow", d)
        sleep_ms(20)

    wheels(0, 0)
    event("stopped")


try:
    run()
except Exception as ex:
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
