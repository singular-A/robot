# Line follower that drives around an obstacle (a bottle, 6 cm diameter) on the line
# Line following is the same as line_follow_safe.py (BASE_POWER 88, KP 1.2, KD 7).
#
# Setup: Open-Cube color sensor on S1 (pointing down, CAM_AHEAD_CM in front of the wheel axis)
#        Open-Cube laser distance sensor (LIDAR) on LASER_PORT, at the front, pointing forward
#        (read in its wide field-of-view mode, distance_fov(), so it also sees a narrow bottle
#        that is not exactly in front of the middle of the robot)
#        Left motor M1, right motor M2
# Usage: put the color sensor on the EDGE of the line (half on black, half on white), run the
#        program. It measures white and black (pivots over the line) and follows the left
#        edge. When the bottle is ahead it slows down, stops, drives around it on the left
#        (the white side) and continues on the line behind it. Press LEFT to stop.
#
# Going around (all distances measured with the wheel encoders):
#   turn 90 left -> drive SIDE_CM -> turn 90 right -> drive PASS_CM (past the bottle)
#   -> turn 90 right -> drive until the color sensor sees the line (max RETURN_CM)
#   -> drive CAM_AHEAD_CM more (wheels over the edge) -> turn 90 left -> follow the line
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

# --- Line following (same as line_follow_safe.py) ---
BASE_POWER = 88     # forward power, 0-100 %
KP = 1.2            # steering strength; raise if it leaves the line, lower if it wobbles
KI = 0.01           # corrects steady drift in long curves; 0 turns it off
KD = 7.0            # damping; raise if it wobbles
D_FILTER = 0.7      # derivative smoothing 0..1; higher = smoother but slower
I_LIMIT = 50        # max steering power from the integral term
SLOWDOWN = 1        # forward power lost per unit of error, so it slows down in curves
MIN_POWER = -25     # lowest wheel power, so a wheel cannot spin hard backwards
LOST_TURN = 40      # steering power when only white is seen (keeps the last direction)
PERIOD_MS = 10      # regulation period
CAL_POWER = 25      # pivot power while measuring white and black
CAL_SWEEP = 150     # how far it pivots each way (wheel degrees difference)
MIN_CONTRAST = 20   # white minus black must be at least this, else the sensor is not over a line

# --- Obstacle ---
LASER_PORT = Port.S2  # port of the laser distance sensor (change if it is plugged in elsewhere)
SLOW_MM = 400       # obstacle closer than this: slow down
STOP_MM = 200       # obstacle closer than this: stop and drive around it (room to turn in place)
SLOW_MIN = 0.3      # slowest speed while approaching, as a fraction of BASE_POWER
CONFIRM = 3         # this many readings in a row below STOP_MM before it reacts (no false alarms)
MIN_VALID_MM = 20   # readings below this are invalid (no target / too close to measure)

# --- Going around (robot measurements and distances, in cm) ---
WHEEL_CM = 8.0      # wheel diameter
TRACK_CM = 19.0     # distance between the wheels
CAM_AHEAD_CM = 4.0  # color sensor in front of the wheel axis
SIDE_CM = 25        # sideways away from the line (half robot width + bottle + margin)
PASS_CM = 60        # forward past the bottle (far enough that the robot does not hit it when turning back)
RETURN_CM = 60      # max distance back toward the line before it gives up
AVOID_POWER = 40    # motor power while going around
TURN_POWER = 30     # motor power while turning in place

DEG_PER_CM = 360 / (3.1416 * WHEEL_CM)          # wheel degrees per cm driven
DEG_PER_TURN_DEG = 2 * TRACK_CM / WHEEL_CM      # left-right wheel degrees per degree of robot turn


def log(message):
    robot.esp.bt_write(message + ";")
    print(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def run():
    robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
    robot.init_sensor(sensor_type=Sensor.OC_LASER, port=LASER_PORT)
    robot.init_motor(Port.M1)
    robot.init_motor(Port.M2)
    light = robot.sensors.light[Port.S1]
    laser = robot.sensors.laser[LASER_PORT]
    left = robot.motors[Port.M1]
    right = robot.motors[Port.M2]
    left.init_encoder()
    right.init_encoder()

    def stop():
        left.set_power(0)
        right.set_power(0)

    def heading():
        return left.position() - right.position()

    def stopped_by_user():
        return robot.buttons.pressed()[Button.LEFT]

    # --- Moves for going around (encoders) ---
    def turn(angle):
        # Turn in place by angle degrees (> 0 right, < 0 left); slower near the end. False on LEFT.
        begin = heading()
        goal = begin + angle * DEG_PER_TURN_DEG
        direction = 1 if angle > 0 else -1
        start = ticks_ms()
        while (goal - heading()) * direction > 0 and ticks_diff(ticks_ms(), start) < 4000:
            if stopped_by_user():
                return False
            rest = abs(goal - heading())
            power = TURN_POWER if rest > 60 else max(15, TURN_POWER * rest / 60)
            left.set_power(direction * power)
            right.set_power(-direction * power)
            sleep_ms(5)
        stop()
        log("  turn {} deg: turned {:.0f} deg in {} ms".format(
            angle, (heading() - begin) / DEG_PER_TURN_DEG, ticks_diff(ticks_ms(), start)))
        sleep_ms(150)
        return True

    def drive(cm, until=None):
        # Drive straight forward cm (keeping the heading); stops early when until() is True.
        # Returns "done", "found" (until() became True) or "user" (LEFT).
        start_pos = (left.position() + right.position()) / 2
        start_heading = heading()
        goal = cm * DEG_PER_CM
        start = ticks_ms()
        while (left.position() + right.position()) / 2 - start_pos < goal \
                and ticks_diff(ticks_ms(), start) < 8000:
            if stopped_by_user():
                stop()
                return "user"
            if until is not None and until():
                stop()
                moved = ((left.position() + right.position()) / 2 - start_pos) / DEG_PER_CM
                log("  drive up to {} cm: line found after {:.0f} cm".format(cm, moved))
                return "found"
            correction = (heading() - start_heading) * 0.5   # keep driving straight
            left.set_power(AVOID_POWER - correction)
            right.set_power(AVOID_POWER + correction)
            sleep_ms(5)
        stop()
        moved = ((left.position() + right.position()) / 2 - start_pos) / DEG_PER_CM
        log("  drive {} cm: drove {:.0f} cm in {} ms".format(cm, moved, ticks_diff(ticks_ms(), start)))
        sleep_ms(150)
        return "done"

    # --- Measure white and black (same as line_follow_safe.py) ---
    seen = [100, 0]

    def pivot_to(goal):
        direction = 1 if goal > heading() else -1
        start = ticks_ms()
        while (heading() - goal) * direction < 0 and ticks_diff(ticks_ms(), start) < 3000:
            left.set_power(direction * CAL_POWER)
            right.set_power(-direction * CAL_POWER)
            r = light.reflection()
            seen[0] = min(seen[0], r)
            seen[1] = max(seen[1], r)
            sleep_ms(5)
        stop()

    sleep_ms(1000)
    home = heading()
    pivot_to(home + CAL_SWEEP)
    pivot_to(home - CAL_SWEEP)
    pivot_to(home)
    black, white = seen
    if white - black < MIN_CONTRAST:
        log("white={:.0f} black={:.0f}: no line found - start with the sensor on the edge of the line".format(white, black))
        return
    setpoint = (white + black) / 2
    lost_error = 0.8 * (white - setpoint)
    log("white={:.0f} black={:.0f} setpoint={:.0f}".format(white, black, setpoint))

    def align():
        # Pivot slowly until the sensor is exactly on the edge
        side = 1 if light.reflection() > setpoint else -1
        start = ticks_ms()
        while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
            left.set_power(side * CAL_POWER)
            right.set_power(-side * CAL_POWER)
            sleep_ms(5)
        stop()

    align()

    def go_around():
        # Drive around the obstacle on the left and come back onto the left edge of the line.
        # Returns True when back on the line, False if not (or LEFT pressed).
        log("Obstacle! Going around")
        stop()
        sleep_ms(200)
        if not turn(-90):
            return False
        if drive(SIDE_CM) == "user":
            return False
        if not turn(90):
            return False
        if drive(PASS_CM) == "user":
            return False
        if not turn(90):
            return False
        found = drive(RETURN_CM, until=lambda: light.reflection() < setpoint)
        if found != "found":
            log("Line not found after the obstacle ({})".format(found))
            return False
        drive(CAM_AHEAD_CM)          # wheels over the edge, then turn back along the line
        if not turn(-90):
            return False
        log("Back on the line")
        return True

    # --- Follow the line, watch for the obstacle ---
    e_prev = 0
    d = 0
    integral = 0
    last_sign = 1
    close = 0                        # valid readings in a row closer than STOP_MM
    near = SLOW_MM + 1               # last valid distance (used while a reading is invalid)
    last_log = 0
    last_tick = ticks_ms()
    while not stopped_by_user():
        now = ticks_ms()
        dt = max(ticks_diff(now, last_tick), 1)
        last_tick = now
        if dt > 50:
            # A loop normally takes ~10 ms; a long one means a sensor read was blocked
            # (the firmware waits up to 3 s if a sensor drops out) and the robot drove blind
            log("  SLOW LOOP {} ms - a sensor was not responding".format(dt))

        # Obstacle ahead? Invalid readings (no target / too close to measure) are skipped:
        # they neither count nor reset the count, and the last valid distance is kept.
        dist = laser.distance_fov()      # wide field of view: also sees a bottle a bit off-centre
        if dist >= MIN_VALID_MM:
            near = dist
            close = close + 1 if dist < STOP_MM else 0
        if near < SLOW_MM and ticks_diff(now, last_log) >= 100:
            log("  laser {} mm (valid {} mm), close {}/{}".format(dist, near, close, CONFIRM))
            last_log = now
        if close >= CONFIRM:
            if not go_around():
                break
            close = 0
            near = SLOW_MM + 1
            e_prev = d = integral = 0
            last_tick = ticks_ms()
            continue

        # Line following (same as line_follow_safe.py)
        e = light.reflection() - setpoint         # > 0: too much white -> turn right
        d = D_FILTER * d + (1 - D_FILTER) * (e - e_prev) * 10 / dt
        e_prev = e
        if (e > 0) != (integral > 0):
            integral = 0
        integral = clamp(integral + e * dt / 10, -I_LIMIT / max(KI, 1e-9), I_LIMIT / max(KI, 1e-9))
        if e > lost_error:
            u = last_sign * LOST_TURN
        else:
            u = KP * e + KI * integral + KD * d
            if abs(u) > 2:
                last_sign = 1 if u > 0 else -1

        # Slow down when the obstacle gets close
        base = BASE_POWER
        if near < SLOW_MM:
            base *= clamp((near - STOP_MM) / (SLOW_MM - STOP_MM), SLOW_MIN, 1)

        speed = max(0, base - SLOWDOWN * abs(e))
        left.set_power(clamp(speed + u, MIN_POWER, 100))
        right.set_power(clamp(speed - u, MIN_POWER, 100))
        sleep_ms(PERIOD_MS)

    stop()
    log("Stopped")


try:
    run()
except Exception as ex:
    # Report why the program crashed instead of stopping silently
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
