# Left wall follower for the block maze - simple version.
#
# What it does:
#  1. Start, fixed moves: the robot stands in the middle of the start box C5, facing D5.
#     Straight out until the bumper touches block E5, turn right, one cell to D6, turn left.
#  2. Then it goes cell by cell (28 cm) with the wall on its LEFT (left-hand rule):
#       opening on the left of the new cell       -> turn left
#       bumper touches a wall (or wheels stuck)    -> turn right
#       can't turn right either (dead end)         -> back up one cell, try turning right there
#                                                     (can't back up any more: turn left there)
#       otherwise                                  -> straight on to the next cell
#  3. Every turn is a sharp 90 degree pivot: one wheel forward, the other backward.
#  4. Everything is printed to the console (Thonny Open-Cube log). LEFT button = stop.
#
# How it stays in the middle: it keeps track of where its axle is in the current cell
# (s = along the corridor from the middle of the cell, lat = sideways, + = left; cm) from the
# encoders. A wall on the left tells lat exactly (lidar), a bump tells s exactly (the wall is
# at the end of the cell). It drives at a small angle (at most MAX_ANGLE) toward the middle.
# Along a wall it also checks its heading: if the lidar sees it moving sideways differently
# than the encoders say, the heading is off (a pivot was not exactly 90 degrees) and it
# corrects it.
#
# Robot: drives with the caster end first, the big wheels are at the back.
#   lidar (S2): front-left, 13.5 cm ahead of the wheel axle, pointing straight left
#   touch (S1): front bumper, 16.5 cm ahead of the wheel axle
#
# Tune TURN_90_DEG first: with CALIBRATE = True it only turns right 4 times on the spot (put it
# on open floor, e.g. along a straight edge) and has to end up facing exactly the way it
# started. Turned too far: lower TURN_90_DEG; not far enough: raise it. The whole program
# depends on accurate turns.
#
# Log: CFG,... = the settings, EV,time_ms,cell+heading,text for every event (cell and heading as the robot thinks it
# is, e.g. D6->F = in D6 facing column F; the map is only used for the log), and every
# LOG_MS a data line:
#   D,time_ms,mode,lidar_mm,bump,left_enc,right_enc,left_power,right_power,s_cm,lat_cm,angle_deg
import math
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

# Motors: caster end first, so forward = negative power and the wheels swap sides
LEFT_MOTOR = Port.M1    # left wheel in the driving direction (swap with RIGHT if turns go the wrong way)
RIGHT_MOTOR = Port.M2
FORWARD = -1            # set to 1 if it drives the wrong way
LIDAR_PORT = Port.S2
TOUCH_PORT = Port.S1

CELL_CM = 28
LIDAR_AHEAD_CM = 13.5   # lidar ahead of the wheel axle
TOUCH_AHEAD_CM = 16.5   # bumper ahead of the wheel axle
WHEEL_CM = 8.16         # wheel diameter
DEG_PER_CM = 360 / (3.1416 * WHEEL_CM)
TURN_90_DEG = 210       # wheel degrees (each wheel) for a 90 degree pivot - tune with CALIBRATE

POWER = 30              # driving power
TURN_POWER = 30         # pivot power
TARGET_MM = 55          # lidar distance to the wall on the left when the robot is in the middle
OPEN_MM = 150           # further than this = no wall on the left
TRUST_MM = 100          # only readings up to this tell where the robot is, only if they
TRUST_STEP_MM = 6       # changed less than this since the last one, and only if the lidar is
EDGE_CM = 4             # at least EDGE_CM from a cell border (walls start and end at cell
                        # borders, and there the lidar sees the corner at a slant)
FIX_CM = 10             # heading check along a wall: compare lidar and encoders over this distance
FIX_GAIN = 0.7          # how much of the heading error found to correct
ANGLE_PER_CM = 3        # degrees to steer back toward the middle per cm off it
MAX_ANGLE = 15          # never drive at more than this angle to the corridor
K_HEADING = 1.5         # steering power per degree off the wanted angle
STEER_MAX = 15
MIN_VALID_MM = 20       # closer than this = (almost) touching the wall on the left
BUMP_BACK_CM = 0.8      # after a bump: back up this far, so the front clears the wall when pivoting
STALL_MS = 600          # wheels hardly move for this long = stuck
MAX_BACKOUTS = 8        # dead end: back up at most this many cells
LOG_MS = 100
CALIBRATE = False

# Map, only for the log: blocks and the thin wall of the start box
BLOCKS = ("E1", "B2", "D2", "E3", "A4", "C4", "E5", "F5", "A6", "C6", "E7", "B8", "D8", "E9")
THIN_WALLS = (("B5", "C5"),)
STEP = ((1, 0), (0, 1), (-1, 0), (0, -1))     # heading 0..3: toward column F, row 9, column A, row 1
HEADING = ("->F", "->9", "->A", "->1")


def cell_name(x, y):
    return "ABCDEF"[x] + str(y + 1) if 0 <= x < 6 and 0 <= y < 9 else "out"


def map_wall(x, y, h):
    # Is there a wall between cell x, y and the next cell in direction h (on the map)?
    a, b = cell_name(x, y), cell_name(x + STEP[h][0], y + STEP[h][1])
    return b == "out" or b in BLOCKS or (a, b) in THIN_WALLS or (b, a) in THIN_WALLS


def clamp(value, low, high):
    return max(low, min(high, value))


class Bot:
    def __init__(self):
        robot.init_sensor(sensor_type=Sensor.OC_LASER, port=LIDAR_PORT)
        robot.init_sensor(sensor_type=Sensor.NXT_TOUCH, port=TOUCH_PORT)
        robot.init_motor(LEFT_MOTOR)
        robot.init_motor(RIGHT_MOTOR)
        self.lidar = robot.sensors.laser[LIDAR_PORT]
        self.bumper = robot.sensors.touch[TOUCH_PORT]
        self.left = robot.motors[LEFT_MOTOR]
        self.right = robot.motors[RIGHT_MOTOR]
        self.left.init_encoder()
        self.right.init_encoder()
        self.power = [0, 0]
        self.t0 = ticks_ms()
        self.last_row = 0
        self.done = False
        self.x, self.y, self.h = 2, 4, 0          # cell C5, facing D5
        self.s, self.lat = 0.0, 0.0                # axle in the middle of the cell
        self.ref = self.pos()                      # encoders at the start (placed straight)
        self.turns = 0                             # 90 degree pivots so far (+ right, - left)
        self.fix = 0.0                             # heading correction found along walls, degrees

    # --- log ---
    def log(self, text):
        robot.esp.bt_write(text + ";")
        print(text)

    def event(self, text):
        self.log("EV,{},{}{},{}".format(ticks_diff(ticks_ms(), self.t0),
                                        cell_name(self.x, self.y), HEADING[self.h], text))

    def row(self, mode, d=None):
        now = ticks_ms()
        if ticks_diff(now, self.last_row) < LOG_MS:
            return
        self.last_row = now
        if d is None:
            d = self.lidar.distance()
        self.log("D,{},{},{},{},{},{},{:.0f},{:.0f},{:.1f},{:.1f},{:.0f}".format(
            ticks_diff(now, self.t0), mode, d, 1 if self.bumper.pressed() else 0,
            self.left.position(), self.right.position(), self.power[0], self.power[1],
            self.s, self.lat, self.angle()))

    # --- wheels and encoders ---
    def wheels(self, left_power, right_power):
        self.power[0], self.power[1] = left_power, right_power
        self.left.set_power(FORWARD * left_power)
        self.right.set_power(FORWARD * right_power)

    def pos(self):
        return [self.left.position(), self.right.position(), ticks_ms()]

    def moved(self, start):
        # Wheel degrees since start (average of both wheels, direction ignored)
        return (abs(self.left.position() - start[0]) + abs(self.right.position() - start[1])) / 2

    def angle(self):
        # Angle to the corridor, degrees, + = turned right: everything turned since the start
        # (a 90 degree pivot turns each wheel TURN_90_DEG, one forward and one backward)
        # minus 90 for every pivot. A pivot started at an angle keeps that angle.
        fl = FORWARD * (self.left.position() - self.ref[0])
        fr = FORWARD * (self.right.position() - self.ref[1])
        return (fl - fr) * 45 / TURN_90_DEG - 90 * self.turns + self.fix

    def stuck(self, ref):
        # ref = pos() of the last place it moved; True if it has not moved for STALL_MS
        if self.moved(ref) > 4:
            ref[:] = self.pos()
        return ticks_diff(ticks_ms(), ref[2]) > STALL_MS

    def stop_pressed(self):
        if robot.buttons.pressed()[Button.LEFT]:
            self.done = True
        return self.done

    # --- moves ---
    def go(self, to_s, power, mode, look=False):
        # Drive straight on (power > 0) or back (power < 0) until s = to_s, at a small angle
        # toward the middle of the corridor (+ angle = right). On the way it updates s and lat
        # from the encoders, and lat from the lidar where there is a wall on the left.
        # look: watch whether the left side of the next cell is open.
        # Returns (how, left_open): how = "done", "bump", "stuck" or "stop".
        sign = 1 if power > 0 else -1
        ref, last = self.pos(), self.pos()
        n_open = n_wall = 0
        side, d_prev = None, None
        check = None                    # heading check: [lat from lidar, sideways from encoders, distance]
        how = "done"
        while (self.s - to_s) * sign < 0:
            if self.stop_pressed():
                how = "stop"
                break
            if sign > 0 and self.bumper.pressed():
                how = "bump"
                break
            if self.stuck(ref):
                how = "stuck"
                break
            # Where is it now (encoders since the last loop)
            a = math.radians(self.angle())
            ds = sign * self.moved(last) / DEG_PER_CM
            last = self.pos()
            self.s += ds * math.cos(a)
            self.lat -= ds * math.sin(a)
            # A wall on the left tells exactly how far it is from the middle (the lidar is
            # LIDAR_AHEAD_CM in front of the axle, so at an angle it is off to one side of it)
            d = self.lidar.distance()
            side = "open" if d > OPEN_MM else "wall"
            border = (self.s + LIDAR_AHEAD_CM + CELL_CM / 2) % CELL_CM     # lidar from the cell border behind it
            near_border = min(border, CELL_CM - border) < EDGE_CM
            if MIN_VALID_MM <= d <= TRUST_MM and d_prev is not None and abs(d - d_prev) < TRUST_STEP_MM \
                    and not near_border:
                self.lat = (TARGET_MM - d * math.cos(a)) / 10 + LIDAR_AHEAD_CM * math.sin(a)
                # Heading check: the sideways movement the encoders did not see = heading error
                if check is None:
                    check = [self.lat, 0.0, 0.0]
                else:
                    check[1] -= ds * math.sin(a)
                    check[2] += ds
                    if abs(check[2]) >= FIX_CM:
                        missed = (self.lat - check[0]) - check[1]
                        self.fix -= FIX_GAIN * math.degrees(missed / check[2])
                        check = [self.lat, 0.0, 0.0]
            else:
                check = None
            d_prev = d
            # Left side of the next cell: count while the lidar is beside its middle
            if look and abs(self.s - (CELL_CM - LIDAR_AHEAD_CM)) <= 6:
                if side == "open":
                    n_open += 1
                else:
                    n_wall += 1
            # Steer back toward the middle (backing up: the other way round)
            if side == "wall" and d < MIN_VALID_MM:
                want = sign * MAX_ANGLE             # touching the wall: away from it
            else:
                want = clamp(sign * ANGLE_PER_CM * self.lat, -MAX_ANGLE, MAX_ANGLE)
            u = clamp(K_HEADING * (self.angle() - want), -STEER_MAX, STEER_MAX)
            self.wheels(power - u, power + u)
            self.row(mode, d)
            sleep_ms(10)
        self.wheels(0, 0)
        left_open = n_open > n_wall if n_open + n_wall else side == "open"
        self.event("{} {} (s {:.1f} lat {:.1f})".format(mode, how, self.s, self.lat))
        return how, left_open

    def reindex(self):
        # s is from the middle of the cell the axle is in: moved into the next/previous cell?
        while self.s > CELL_CM / 2:
            self.x += STEP[self.h][0]
            self.y += STEP[self.h][1]
            self.s -= CELL_CM
        while self.s < -CELL_CM / 2:
            self.x -= STEP[self.h][0]
            self.y -= STEP[self.h][1]
            self.s += CELL_CM

    def back(self, cm):
        self.go(self.s - cm, -POWER, "back")

    def back_straight(self, cm):
        # Just back up (both wheels the same), e.g. when stuck in the middle of a pivot
        start, ref = self.pos(), self.pos()
        self.wheels(-POWER, -POWER)
        while self.moved(start) < cm * DEG_PER_CM and not self.stuck(ref) and not self.stop_pressed():
            self.row("back")
            sleep_ms(5)
        self.wheels(0, 0)

    def pivot(self, direction, part=1.0, blockable=False):
        # Turn on the spot, one wheel forward and one backward: direction 1 = right, -1 = left,
        # part = part of 90 degrees. blockable: if it hits something (bumper or stuck), stop and
        # return False. Otherwise back up a little and finish the turn.
        # Returns (finished, wheel degrees done).
        mode = "pivR" if direction > 0 else "pivL"
        goal = TURN_90_DEG * part
        done = 0
        sleep_ms(100)
        for attempt in range(3):
            start, ref = self.pos(), self.pos()
            self.wheels(direction * TURN_POWER, -direction * TURN_POWER)
            hit = False
            while done + self.moved(start) < goal and not self.stop_pressed():
                if self.stuck(ref) or (blockable and done + self.moved(start) > 20 and self.bumper.pressed()):
                    hit = True
                    break
                self.row(mode)
                sleep_ms(5)
            self.wheels(0, 0)
            done += self.moved(start)
            if not hit or self.done:
                break
            if blockable:
                self.event("{} blocked at {:.0f}/{:.0f} deg".format(mode, done, goal))
                return False, done
            self.event("{} stuck at {:.0f}/{:.0f} deg -> back up, finish the turn".format(mode, done, goal))
            self.back_straight(2)
        if part == 1.0:
            # New heading: along and sideways swap (right turn: old left = new back)
            self.h = (self.h + direction) % 4
            self.turns += direction
            if direction > 0:
                self.s, self.lat = -self.lat, self.s
            else:
                self.s, self.lat = self.lat, -self.s
        self.event("{} done {:.0f}/{:.0f} deg".format(mode, done, goal))
        sleep_ms(100)
        return True, done

    def move_cell(self):
        # Drive to the middle of the next cell, looking at the left side of it on the way.
        # Returns (how, left_open): how = "arrived", "bump", "stuck" or "stop".
        how, left_open = self.go(CELL_CM, POWER, "cell", look=True)
        self.reindex()
        if how in ("bump", "stuck"):
            self.s = CELL_CM / 2 - TOUCH_AHEAD_CM  # the wall is at the end of this cell
        return ("arrived" if how == "done" else how), left_open

    def turn_right(self):
        # Turn right (wall ahead and on the left). If that is blocked too (dead end): undo, back
        # up to the middle of the cell before and try again there. If it can't back up any more
        # (wall behind), turn left there instead.
        for backouts in range(MAX_BACKOUTS + 1):
            ok, done = self.pivot(1, blockable=True)
            if ok or self.done:
                return
            self.pivot(-1, done / TURN_90_DEG)         # undo the part of the turn that was done
            if backouts == MAX_BACKOUTS:
                break
            self.event("dead end -> back up to the middle of the cell before")
            how, _ = self.go(-CELL_CM, -POWER, "backcell")
            self.reindex()
            if how == "stuck":
                self.event("wall behind -> to the middle of the cell, turn left")
                self.go(0, POWER if self.s < 0 else -POWER, "center")
                ok, done = self.pivot(-1, blockable=True)
                if ok or self.done:
                    return
                self.pivot(1, done / TURN_90_DEG)
                break
        self.event("can't get out - stopping")
        self.done = True

    # --- program ---
    def start(self):
        self.event("start")
        how, _ = self.go(3 * CELL_CM, POWER, "out")  # until the bumper touches E5
        if how not in ("bump", "stuck"):
            self.event("WARNING: did not touch E5")
        self.reindex()                               # D5
        self.s = CELL_CM / 2 - TOUCH_AHEAD_CM
        self.back(BUMP_BACK_CM)
        self.pivot(1)                                # facing D6
        self.move_cell()                             # to D6
        self.pivot(-1)                               # facing E6
        self.event("start done")

    def run(self):
        self.log("CFG,TURN_90_DEG={},WHEEL_CM={},TARGET_MM={},OPEN_MM={},POWER={},TURN_POWER={},BUMP_BACK_CM={}".format(
            TURN_90_DEG, WHEEL_CM, TARGET_MM, OPEN_MM, POWER, TURN_POWER, BUMP_BACK_CM))
        if CALIBRATE:
            for i in range(4):
                self.pivot(1)
                sleep_ms(1000)
            return
        self.start()
        while not self.stop_pressed():
            how, left_open = self.move_cell()
            if how == "stop":
                break
            self.event("{}: left {} (map {}), ahead (map {})".format(
                how, "open" if left_open else "wall",
                "wall" if map_wall(self.x, self.y, (self.h - 1) % 4) else "open",
                "wall" if map_wall(self.x, self.y, self.h) else "open"))
            if how != "arrived":
                self.back(BUMP_BACK_CM)
            if left_open:
                self.pivot(-1)
            elif how != "arrived":
                self.turn_right()
        self.wheels(0, 0)
        self.event("stopped")


try:
    Bot().run()
except Exception as ex:
    robot.esp.bt_write("Crashed: {} {};".format(type(ex).__name__, ex))
    print("Crashed:", type(ex).__name__, ex)
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
