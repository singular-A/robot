# Line follower that drives around an obstacle (a bottle, 6 cm diameter) on the line
# Line following is the same as line_follow_safe.py (BASE_POWER 88, KP 1.2, KD 7).
#
# Setup: Open-Cube color sensor on S1 (pointing down)
#        Open-Cube laser distance sensor (LIDAR) on LASER_PORT, at the front, pointing forward
#        (read in its wide field-of-view mode, distance_fov(), so it also sees a narrow bottle
#        that is not exactly in front of the middle of the robot)
#        Left motor M1, right motor M2
# Usage: put the color sensor on the EDGE of the line (half on black, half on white), run the
#        program. It measures white and black (pivots over the line) and follows the left
#        edge. When the bottle is ahead it slows down and swerves around it on the left (the
#        white side) without stopping, then continues on the line behind it. LEFT = stop.
#
# Going around = a smooth swerve, steered with the wheel encoders (heading and position):
#   1. out:   curve to SWERVE_DEG left until the middle of the robot is about SIDE_CM beside
#             the line (never more than MAX_SIDE_CM)
#   2. past:  straight, parallel to the line, until the back of the robot is past the bottle
#   3. in:    curve to SWERVE_DEG right, toward the line, until the color sensor sees it
#   4. the normal line following takes over and straightens the robot out on the line
import math
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

# --- Straight-line mode: faster and calmer on straight parts of the line ---
# straight = for STRAIGHT_MS the robot hardly turns (wheel encoders: turning per distance
# below STRAIGHT_RATIO) and stays close to the edge. Then: STRAIGHT_BOOST extra power
# (added gradually) and a softer KP (KP * STRAIGHT_KP) against the wobble. As soon as it
# turns or drifts off the edge (a curve begins), back to normal at once. Off near obstacles.
# STRAIGHT_BOOST = 0 and STRAIGHT_KP = 1 turn it off.
STRAIGHT_BOOST = 25     # extra forward power on straights
STRAIGHT_KP = 0.6       # KP is multiplied by this on straights
STRAIGHT_RATIO = 0.25   # straight = turning less than this per distance driven
STRAIGHT_MS = 150       # must be straight this long before it speeds up
STRAIGHT_ERROR = 0.6    # leave straight mode at once when the error is above this part of the range
TURN_MEMORY = 0.9       # smoothing of the turning measurement
BOOST_RAMP = 1          # extra power added per loop (10 ms) until STRAIGHT_BOOST is reached

# --- Obstacle ---
LASER_PORT = Port.S2  # port of the laser distance sensor (change if it is plugged in elsewhere)
SLOW_MM = 600       # obstacle closer than this: slow down
SWERVE_MM = 300     # obstacle closer than this: swerve around it
SLOW_MIN = 0.5      # slowest speed while approaching, as a fraction of BASE_POWER
CONFIRM = 3         # this many valid readings in a row below SWERVE_MM before it reacts
MIN_VALID_MM = 20   # readings below this are invalid (no target / too close to measure)

# --- Going around (robot measurements in cm) ---
WHEEL_CM = 8.0      # wheel diameter
TRACK_CM = 19.0     # distance between the wheels
LIDAR_AHEAD_CM = 10 # laser sensor in front of the wheel axis
REAR_CM = 12        # robot length behind the wheel axis
BOTTLE_R_CM = 3     # bottle radius
SIDE_CM = 19        # middle of the robot this far beside the line while passing the bottle
MAX_SIDE_CM = 20    # never further than this from the line
SWERVE_DEG = 35     # angle away from / back toward the line
AVOID_POWER = 45    # forward power while swerving
HEADING_GAIN = 1.5  # steering power per degree of heading error while swerving
RETURN_CM = 80      # max forward distance looking for the line before it gives up

DEG_PER_CM = 360 / (math.pi * WHEEL_CM)          # wheel degrees per cm driven
DEG_PER_TURN_DEG = 2 * TRACK_CM / WHEEL_CM       # left-right wheel degrees per degree of robot turn


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

    # Encoder directions: checked at the start (some motors count backwards)
    sign = {"l": 1, "r": 1}

    def lpos():
        return sign["l"] * left.position()

    def rpos():
        return sign["r"] * right.position()

    def heading():
        return lpos() - rpos()

    def check_encoders():
        # Short wiggle: left wheel forward, right wheel backward, then back again.
        # Forward must count up; if a wheel counts down, flip its sign.
        l0, r0 = left.position(), right.position()
        left.set_power(CAL_POWER)
        right.set_power(-CAL_POWER)
        sleep_ms(250)
        stop()
        sleep_ms(100)
        dl, dr = left.position() - l0, right.position() - r0
        left.set_power(-CAL_POWER)
        right.set_power(CAL_POWER)
        sleep_ms(250)
        stop()
        sleep_ms(100)
        sign["l"] = 1 if dl >= 0 else -1
        sign["r"] = 1 if dr <= 0 else -1
        log("encoders: left {} deg, right {} deg -> signs left {:+d} right {:+d}{}".format(
            dl, dr, sign["l"], sign["r"],
            "  WARNING: an encoder is not counting" if abs(dl) < 10 or abs(dr) < 10 else ""))

    def stopped_by_user():
        return robot.buttons.pressed()[Button.LEFT]

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
    check_encoders()
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
    half = white - setpoint
    log("white={:.0f} black={:.0f} setpoint={:.0f}".format(white, black, setpoint))

    # Pivot slowly until the sensor is exactly on the edge
    side = 1 if light.reflection() > setpoint else -1
    start = ticks_ms()
    while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
        left.set_power(side * CAL_POWER)
        right.set_power(-side * CAL_POWER)
        sleep_ms(5)
    stop()

    def go_around(dist_mm):
        # Swerve around the obstacle on the left without stopping. The position is tracked
        # with the encoders: x = along the line, y = to the left of it (cm), from here.
        # Returns True when the color sensor is back on the line, False if not (or LEFT).
        bottle_x = dist_mm / 10 + LIDAR_AHEAD_CM + BOTTLE_R_CM    # bottle centre ahead of the axis
        log("Obstacle {} mm ahead, swerving around it".format(dist_mm))
        h0 = heading()
        pos = {"x": 0.0, "y": 0.0, "l": lpos(), "r": rpos(), "max_y": 0.0}

        def angle():
            # Robot heading relative to the line, degrees (> 0 = turned right)
            return (heading() - h0) / DEG_PER_TURN_DEG

        def update():
            l, r = lpos(), rpos()
            ds = ((l - pos["l"]) + (r - pos["r"])) / 2 / DEG_PER_CM
            pos["l"], pos["r"] = l, r
            a = angle() * math.pi / 180
            pos["x"] += ds * math.cos(a)
            pos["y"] -= ds * math.sin(a)
            pos["max_y"] = max(pos["max_y"], pos["y"])

        def steer(target, done):
            # Drive forward while turning toward heading `target` until done() is True.
            # Returns "done", "user" (LEFT) or "time".
            start = ticks_ms()
            while ticks_diff(ticks_ms(), start) < 8000:
                if stopped_by_user():
                    return "user"
                update()
                if done():
                    return "done"
                u = clamp(HEADING_GAIN * (target - angle()), -AVOID_POWER, AVOID_POWER)
                left.set_power(AVOID_POWER + u)       # u > 0: left faster = turn right
                right.set_power(AVOID_POWER - u)
                sleep_ms(5)
            return "time"

        # 1. Out: curve left until far enough beside the line (the turn back adds a little)
        if steer(-SWERVE_DEG, lambda: pos["y"] >= SIDE_CM - 4 or pos["y"] >= MAX_SIDE_CM - 2) != "done":
            return False
        log("  out: {:.0f} cm beside the line after {:.0f} cm".format(pos["y"], pos["x"]))
        # 2. Past: parallel to the line until the back of the robot is past the bottle
        if steer(0, lambda: pos["x"] >= bottle_x + BOTTLE_R_CM + REAR_CM + 2) != "done":
            return False
        log("  past the bottle: x={:.0f} cm, max {:.0f} cm beside the line".format(pos["x"], pos["max_y"]))
        # 3. In: curve toward the line until the color sensor sees it
        x_in = pos["x"]
        found = []

        def line_or_give_up():
            if light.reflection() < setpoint:
                found.append(True)
                return True
            return pos["x"] - x_in > RETURN_CM

        if steer(SWERVE_DEG, line_or_give_up) != "done" or not found:
            stop()
            log("Line not found after the obstacle")
            return False
        log("  back on the line after {:.0f} cm (max {:.0f} cm beside the line)".format(pos["x"], pos["max_y"]))
        return True

    # --- Follow the line, watch for the obstacle ---
    e_prev = 0
    d = 0
    integral = 0
    last_sign = 1
    last_l, last_r = lpos(), rpos()
    turn_avg = 0         # smoothed turning (left - right wheel degrees per loop)
    drive_avg = 0        # smoothed driving (average wheel degrees per loop)
    straight_since = None
    boost = 0            # current extra power on a straight
    close = 0                        # valid readings in a row closer than SWERVE_MM
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
            close = close + 1 if dist < SWERVE_MM else 0
        if near < SLOW_MM and ticks_diff(now, last_log) >= 100:
            log("  laser {} mm (valid {} mm), close {}/{}".format(dist, near, close, CONFIRM))
            last_log = now
        if close >= CONFIRM:
            if not go_around(near):
                break
            close = 0
            near = SLOW_MM + 1
            e_prev = d = integral = 0
            last_sign = -1               # it came in from the left: if it overshoots, the line is left
            last_l, last_r = lpos(), rpos()
            turn_avg = drive_avg = boost = 0
            straight_since = None
            last_tick = ticks_ms()
            continue

        # Line following (same as line_follow_safe.py)
        e = light.reflection() - setpoint         # > 0: too much white -> turn right

        # Straight-line detection: how much is the robot turning compared to how far it drives?
        l_pos, r_pos = lpos(), rpos()
        dl, dr = l_pos - last_l, r_pos - last_r
        last_l, last_r = l_pos, r_pos
        turn_avg = TURN_MEMORY * turn_avg + (1 - TURN_MEMORY) * (dl - dr)
        drive_avg = TURN_MEMORY * drive_avg + (1 - TURN_MEMORY) * (dl + dr) / 2
        if (drive_avg > 0 and abs(turn_avg) < STRAIGHT_RATIO * drive_avg
                and abs(e) < STRAIGHT_ERROR * half and near >= SLOW_MM):
            if straight_since is None:
                straight_since = now
        else:
            straight_since = None
        on_straight = straight_since is not None and ticks_diff(now, straight_since) >= STRAIGHT_MS
        boost = min(STRAIGHT_BOOST, boost + BOOST_RAMP) if on_straight else 0
        d = D_FILTER * d + (1 - D_FILTER) * (e - e_prev) * 10 / dt
        e_prev = e
        if (e > 0) != (integral > 0):
            integral = 0
        integral = clamp(integral + e * dt / 10, -I_LIMIT / max(KI, 1e-9), I_LIMIT / max(KI, 1e-9))
        if e > lost_error:
            u = last_sign * LOST_TURN
        else:
            kp = KP * STRAIGHT_KP if on_straight else KP
            u = kp * e + KI * integral + KD * d
            if abs(u) > 2:
                last_sign = 1 if u > 0 else -1

        # Slow down when the obstacle gets close
        base = BASE_POWER + boost
        if near < SLOW_MM:
            base *= clamp((near - SWERVE_MM) / (SLOW_MM - SWERVE_MM), SLOW_MIN, 1)

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
