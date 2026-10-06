# GAP TEST: line follower (same as line_follow_auto.py) that also detects a gap in the line
# (white tape across the black line) and logs it, with the time between gaps = lap time.
#
# How a gap is recognised: the robot was following the edge steadily and the reading
# suddenly jumps to white. It then drives STRAIGHT; if the line comes back within
# GAP_MAX_DEG of wheel travel, it was the gap. If not, the robot really left the line
# and the normal "only white" steering takes over.
#
# Setup: Open-Cube color sensor on S1 (pointing down), left motor M1, right motor M2
# Usage: put the sensor on the EDGE of the line (half on black, half on white) and run the
#        program. At every start the robot first pivots left and right over the line to
#        measure white and black, turns back to the edge and then follows it.
#        Press LEFT on the cube to stop.
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

BASE_POWER = 40     # forward power, 0-100 %
KP = 0.8            # steering strength; raise if it leaves the line, lower if it wobbles
KI = 0.01           # corrects steady drift in long curves; 0 turns it off
KD = 3.0            # damping; raise if it wobbles
D_FILTER = 0.7      # derivative smoothing 0..1; higher = smoother but slower
I_LIMIT = 10        # max steering power from the integral term
SLOWDOWN = 0.5      # forward power lost per unit of error, so it slows down in curves
MIN_POWER = -10     # lowest wheel power, so a wheel cannot spin hard backwards
LOST_TURN = 40      # steering power when only white is seen (keeps the last direction)
PERIOD_MS = 10      # regulation period
CAL_POWER = 25      # pivot power while measuring white and black
CAL_SWEEP = 150     # how far it pivots each way (wheel degrees difference, about 35 degrees)
MIN_CONTRAST = 20   # white minus black must be at least this, else the sensor is not over a line
LOG_EVERY = 100     # log every N loops (100 = about once a second)
STABLE_LOOPS = 15   # following the edge steadily this many loops before a jump to white = gap
STABLE_ERROR = 0.4  # "steadily" = error within this fraction of (white - setpoint)
GAP_MAX_DEG = 120   # longest gap in wheel degrees (about 6 cm with 5.6 cm wheels)
GAP_MIN_DEG = 600   # ignore a new gap until the robot drove this far after the last one


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
    stable = 0           # loops in a row spent close to the edge
    gap_start = None     # wheel distance where the current possible gap started
    gap_peak = 0         # brightest reading in the current possible gap
    gaps = 0
    last_gap_pos = -GAP_MIN_DEG
    last_gap_time = None
    d = 0                # filtered derivative
    integral = 0
    last_sign = 1        # last steering direction: +1 right, -1 left
    loops = 0
    last_tick = ticks_ms()
    run_start = last_tick
    while not robot.buttons.pressed()[Button.LEFT]:
        now = ticks_ms()
        dt = max(ticks_diff(now, last_tick), 1)   # real loop time in ms
        last_tick = now

        r = light.reflection()
        e = r - setpoint                          # > 0: too much white -> turn right
        pos = (left.position() + right.position()) / 2   # distance driven, wheel degrees

        # Gap detection: a sudden jump to white after following the edge steadily
        if gap_start is None:
            if e > lost_error and stable >= STABLE_LOOPS and pos - last_gap_pos > GAP_MIN_DEG:
                gap_start = pos
                gap_peak = r
            stable = stable + 1 if abs(e) < STABLE_ERROR * (white - setpoint) else 0
        if gap_start is not None:
            gap_peak = max(gap_peak, r)
            if e < lost_error:
                # Line is back: it was the gap
                gaps += 1
                last_gap_pos = pos
                lap = "" if last_gap_time is None else " lap={:.2f}s".format(ticks_diff(now, last_gap_time) / 1000)
                last_gap_time = now
                log("GAP #{} at t={:.2f}s length={:.0f}deg peak={:.0f}{}".format(
                    gaps, ticks_diff(now, run_start) / 1000, pos - gap_start, gap_peak, lap))
                gap_start = None
                stable = 0
            elif pos - gap_start > GAP_MAX_DEG:
                # Still white after the longest gap: really off the line
                log("not a gap: white for {:.0f}deg at t={:.2f}s, off the line".format(
                    pos - gap_start, ticks_diff(now, run_start) / 1000))
                gap_start = None
                stable = 0
            else:
                # Inside the gap: drive straight at the normal speed
                left.set_power(BASE_POWER)
                right.set_power(BASE_POWER)
                e_prev = 0
                d = 0
                loops += 1
                sleep_ms(PERIOD_MS)
                continue

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
        if loops % LOG_EVERY == 0:
            log("e={:.0f} u={:.0f} L={:.0f} R={:.0f} dt={} bat={:.2f}V".format(
                e, u, left_power, right_power, dt, robot.battery.voltage()))

        sleep_ms(PERIOD_MS)

    left.set_power(0)
    right.set_power(0)
    log("Stopped, {} gaps found".format(gaps))


try:
    run()
except Exception as ex:
    # Report why the program crashed instead of stopping silently
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
    robot.motors[Port.M1].set_power(0)
    robot.motors[Port.M2].set_power(0)
