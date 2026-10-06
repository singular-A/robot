# SCORE TEST: the same line follower as line_follow_auto.py, but it drives for TEST_S
# seconds, stops by itself and reports how well it kept to the edge of the line:
#   err   = average distance from the edge, 0 % = always on the edge, 100 % = always
#           fully on black or white
#   edge  = % of time close to the edge (within 25 %)
#   off   = % of time fully off the edge (more than 80 %, sensor sees only black or white)
#   wob   = side changes per second (black side <-> white side), how much it wobbles
#   speed = average speed in cm/s (from the encoders, WHEEL_CM wheels)
#   score = speed * (1 - err): higher is better, rewards fast AND close to the edge
#
# Setup: Open-Cube color sensor on S1 (pointing down), left motor M1, right motor M2
# Usage: put the sensor on the EDGE of the line (half on black, half on white) and run the
#        program. At every start the robot first pivots left and right over the line to
#        measure white and black, turns back to the edge and then follows it.
#        Press LEFT on the cube to stop.
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

BASE_POWER = 75     # forward power, 0-100 %
KP = 0.8            # steering strength; raise if it leaves the line, lower if it wobbles
KI = 0.01           # corrects steady drift in long curves; 0 turns it off
KD = 3.0            # damping; raise if it wobbles
D_FILTER = 0.7      # derivative smoothing 0..1; higher = smoother but slower
I_LIMIT = 10        # max steering power from the integral term
SLOWDOWN = 1      # forward power lost per unit of error, so it slows down in curves
MIN_POWER = -10     # lowest wheel power, so a wheel cannot spin hard backwards
LOST_TURN = 40      # steering power when only white is seen (keeps the last direction)
PERIOD_MS = 10      # regulation period
CAL_POWER = 25      # pivot power while measuring white and black
CAL_SWEEP = 150     # how far it pivots each way (wheel degrees difference, about 35 degrees)
MIN_CONTRAST = 20   # white minus black must be at least this, else the sensor is not over a line
LOG_EVERY = 100     # log every N loops (100 = about once a second)
TEST_S = 60         # test length in seconds (then it stops and prints the result)
REPORT_S = 5        # also print a partial result every this many seconds
WHEEL_CM = 5.6      # wheel diameter, only used to show the speed in cm/s


def log(message):
    robot.esp.bt_write(message + ";")
    print(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def run():
    robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
    robot.init_motor(Port.M1)
    robot.init_motor(Port.M2)
    light = robot.sensors.light[Port.S1]
    left = robot.motors[Port.M1]
    right = robot.motors[Port.M2]

    left.init_encoder()
    right.init_encoder()

    seen = [100, 0]      # darkest and brightest reading during the calibration sweep

    def pivot_to(goal):
        # Pivot in place until the heading (left - right wheel position) reaches goal,
        # recording the darkest and brightest readings on the way (max 3 s)
        direction = 1 if goal > left.position() - right.position() else -1
        start = ticks_ms()
        while (left.position() - right.position() - goal) * direction < 0 \
                and ticks_diff(ticks_ms(), start) < 3000:
            left.set_power(direction * CAL_POWER)
            right.set_power(-direction * CAL_POWER)
            r = light.reflection()
            seen[0] = min(seen[0], r)
            seen[1] = max(seen[1], r)
            sleep_ms(5)
        left.set_power(0)
        right.set_power(0)

    # Measure black and white: pivot right (onto the line), left (across to the white
    # side), then back right to about where it started
    sleep_ms(1000)
    home = left.position() - right.position()
    pivot_to(home + CAL_SWEEP)
    pivot_to(home - CAL_SWEEP)
    pivot_to(home)
    black, white = seen
    if white - black < MIN_CONTRAST:
        log("white={:.0f} black={:.0f}: no line found - start with the sensor on the edge of the line".format(white, black))
        return
    setpoint = (white + black) / 2       # the reading on the edge, which the regulator holds
    log("white={:.0f} black={:.0f} setpoint={:.0f}".format(white, black, setpoint))

    # Pivot slowly until the sensor is exactly on the edge (reading = setpoint)
    side = 1 if light.reflection() > setpoint else -1     # white: pivot right, black: pivot left
    start = ticks_ms()
    while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
        left.set_power(side * CAL_POWER)
        right.set_power(-side * CAL_POWER)
        sleep_ms(5)
    left.set_power(0)
    right.set_power(0)

    lost_error = 0.8 * (white - setpoint)   # error at which the sensor sees (almost) only white

    e_prev = 0
    half = white - setpoint          # error when the sensor sees only white (or only black)
    stats = [0, 0, 0, 0, 0]          # samples, sum |e|/half, on edge, fully off, side changes
    block = [0, 0, 0, 0, 0]          # the same, for the current REPORT_S block
    side = 0                         # +1 white side, -1 black side (for counting wobbles)
    start_pos = None
    d = 0                # filtered derivative
    integral = 0
    last_sign = 1        # last steering direction: +1 right, -1 left
    loops = 0
    last_tick = ticks_ms()
    run_start = last_tick
    block_start = last_tick
    block_pos = (left.position() + right.position()) / 2
    start_pos = block_pos

    def cm(deg):
        return deg / 360 * 3.1416 * WHEEL_CM

    def result(name, st, seconds, deg):
        n = max(st[0], 1)
        err = st[1] / n
        speed = cm(deg) / max(seconds, 0.01)
        log("{} err={:.0f}% edge={:.0f}% off={:.0f}% wob={:.1f}/s speed={:.1f}cm/s score={:.1f}".format(
            name, 100 * err, 100 * st[2] / n, 100 * st[3] / n, st[4] / max(seconds, 0.01),
            speed, speed * (1 - min(err, 1))))
    while not robot.buttons.pressed()[Button.LEFT]:
        now = ticks_ms()
        if ticks_diff(now, run_start) >= TEST_S * 1000:
            break
        dt = max(ticks_diff(now, last_tick), 1)   # real loop time in ms
        last_tick = now

        e = light.reflection() - setpoint         # > 0: too much white -> turn right

        # Measure how close it keeps to the edge
        a = abs(e) / half
        new_side = 1 if e > 0.25 * half else (-1 if e < -0.25 * half else side)
        for st in (stats, block):
            st[0] += 1
            st[1] += min(a, 1)
            st[2] += a < 0.25
            st[3] += a > 0.8
            st[4] += side != 0 and new_side != side
        side = new_side
        if ticks_diff(now, block_start) >= REPORT_S * 1000:
            pos = (left.position() + right.position()) / 2
            result("  {:.0f}s:".format(ticks_diff(now, run_start) / 1000), block,
                   ticks_diff(now, block_start) / 1000, pos - block_pos)
            block = [0, 0, 0, 0, 0]
            block_start = now
            block_pos = pos

        # Derivative scaled to a 10 ms step so an uneven loop time does not change it
        d = D_FILTER * d + (1 - D_FILTER) * (e - e_prev) * 10 / dt
        e_prev = e

        # Integral, clamped, and reset when the error changes sign
        if (e > 0) != (integral > 0):
            integral = 0
        integral = clamp(integral + e * dt / 10, -I_LIMIT / max(KI, 1e-9), I_LIMIT / max(KI, 1e-9))

        if e > lost_error:
            # Only white: keep steering the way it was steering before it lost the line
            u = last_sign * LOST_TURN
        else:
            u = KP * e + KI * integral + KD * d
            if abs(u) > 2:
                last_sign = 1 if u > 0 else -1

        speed = max(0, BASE_POWER - SLOWDOWN * abs(e))
        left_power = clamp(speed + u, MIN_POWER, 100)
        right_power = clamp(speed - u, MIN_POWER, 100)
        left.set_power(left_power)
        right.set_power(right_power)

        loops += 1

        sleep_ms(PERIOD_MS)

    left.set_power(0)
    right.set_power(0)
    seconds = ticks_diff(ticks_ms(), run_start) / 1000
    result("RESULT BASE={} KP={} KI={} KD={} ({:.0f}s):".format(BASE_POWER, KP, KI, KD, seconds),
           stats, seconds, (left.position() + right.position()) / 2 - start_pos)


try:
    run()
except Exception as ex:
    # Report why the program crashed instead of stopping silently
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
    robot.motors[Port.M1].set_power(0)
    robot.motors[Port.M2].set_power(0)
