# Line follower that drives around one obstacle (a bottle) on the line.
#
# Setup: color sensor on S1 (pointing down), laser distance sensor on S2 (pointing forward)
# Usage: put the color sensor on the edge of the line and run. It measures white and black,
#        then follows the line. The first time the laser sees something closer than
#        AVOID_MM it drives around it with a fixed, timed move (AROUND):
#          turn right, out, turn left (parallel), past the bottle, turn left (toward the line)
#        then drives straight until the sensor sees the line, and follows the line again. Only one obstacle: the laser is not read after that.
#        LEFT button = stop.
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

# Motors: on this robot M1 is on the right side. Swap these two if it swerves left.
LEFT_MOTOR = Port.M2
RIGHT_MOTOR = Port.M1

# Line following (same regulator as line_follow_safe.py)
BASE_POWER = 88     # forward power, 0-100 %
KP = 1.0            # steering strength (line_follow_safe.py uses 1.2; lower = less wobble)
KI = 0.01           # corrects steady drift in long curves
KD = 7.0            # damping
D_FILTER = 0.7      # derivative smoothing 0..1
I_LIMIT = 50        # max steering power from the integral term
SLOWDOWN = 1        # forward power lost per unit of error
SPEED_MEMORY = 0.95 # smoothing of that slowdown, so the speed does not pulse with every wobble
MIN_POWER = -25     # lowest wheel power
LOST_TURN = 40      # steering power when only white is seen
PERIOD_MS = 10      # regulation period
CAL_POWER = 25      # pivot power while measuring white and black

# Obstacle
AVOID_MM = 300      # obstacle closer than this: drive around it
CONFIRM = 3         # this many valid readings in a row before it reacts (ignores single glitches)
LASER_EVERY = 3     # read the laser only every this many loops, so line following stays fast
MIN_VALID_MM = 20   # readings below this are invalid (no target / too close): skipped, not counted

# The move around the obstacle, step by step: (left power, right power, time in ms).
# Tune the times on the track: longer turn = bigger angle, longer straight = further.
AROUND = [
    (50, -50, 300),     # pivot right, about 45 degrees
    (50, 50, 1000),     # drive out, away from the line
    (-50, 50, 300),     # pivot left, back parallel to the line
    (50, 50, 2400),     # drive past the bottle
    (-50, 50, 180),     # pivot left, about 25 degrees toward the line (shallow, so it meets the line gently)
]
# Straight steps (both powers equal) are kept straight with the wheel encoders.
RETURN = (30, 30)       # then slowly straight toward the line until the sensor sees it
STRAIGHT_K = 0.5        # power correction per wheel degree that one wheel is ahead
STRAIGHT_MAX = 15       # max power correction
STEP_PAUSE_MS = 250     # stop this long before every step, so each one starts from standstill
AFTER_POWER = 62        # forward power right after the obstacle (a curve follows it)
AFTER_MS = 4000         # how long it stays at AFTER_POWER after it is back on the line
RETURN_MAX_MS = 6000    # give up and stop if no line is found by then (longer: shallow and slow)


def log(message):
    robot.esp.bt_write(message + ";")
    print(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def run():
    robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
    robot.init_sensor(sensor_type=Sensor.OC_LASER, port=Port.S2)
    robot.init_motor(LEFT_MOTOR)
    robot.init_motor(RIGHT_MOTOR)
    light = robot.sensors.light[Port.S1]
    laser = robot.sensors.laser[Port.S2]
    left = robot.motors[LEFT_MOTOR]
    right = robot.motors[RIGHT_MOTOR]
    left.init_encoder()
    right.init_encoder()
    sign = [1, 1]        # encoder directions (some motors count backwards), set at calibration

    def straight(power, l0, r0):
        # Same power on both wheels, corrected so both wheels have turned equally since l0, r0
        ahead = sign[0] * (left.position() - l0) - sign[1] * (right.position() - r0)
        u = clamp(STRAIGHT_K * ahead, -STRAIGHT_MAX, STRAIGHT_MAX)
        left.set_power(power - u)
        right.set_power(power + u)

    def drive(left_power, right_power, ms):
        if left_power != right_power or left_power == 0:   # turn or stop: no correction
            left.set_power(left_power)
            right.set_power(right_power)
            sleep_ms(ms)
            return
        l0, r0 = left.position(), right.position()
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < ms:
            straight(left_power, l0, r0)
            sleep_ms(5)

    def stop_pressed():
        return robot.buttons.pressed()[Button.LEFT]

    # Measure white and black: pivot right, left, back right, remember darkest and brightest
    sleep_ms(1000)
    black, white = 100, 0
    l0, r0 = left.position(), right.position()
    for i, (direction, ms) in enumerate(((1, 400), (-1, 800), (1, 400))):
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < ms:
            left.set_power(direction * CAL_POWER)
            right.set_power(-direction * CAL_POWER)
            r = light.reflection()
            black, white = min(black, r), max(white, r)
            sleep_ms(5)
        if i == 0:
            # First pivot: left wheel went forward, right wheel backward
            sign[0] = 1 if left.position() >= l0 else -1
            sign[1] = 1 if right.position() <= r0 else -1
    drive(0, 0, 100)
    setpoint = (white + black) / 2
    lost_error = 0.8 * (white - setpoint)
    log("white={:.0f} black={:.0f} setpoint={:.0f}".format(white, black, setpoint))

    # Pivot until the sensor is on the edge
    side = 1 if light.reflection() > setpoint else -1
    start = ticks_ms()
    while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
        left.set_power(side * CAL_POWER)
        right.set_power(-side * CAL_POWER)
        sleep_ms(5)
    drive(0, 0, 0)

    e_prev = 0
    d = 0
    integral = 0
    last_sign = 1        # last steering direction: +1 right, -1 left
    close = 0            # valid readings in a row closer than AVOID_MM
    last_log = 0
    avoided = False
    loops = 0
    e_avg = 0            # smoothed |error|, for the slowdown
    last_tick = ticks_ms()
    while not stop_pressed():
        # Obstacle? (only until it has driven around one)
        loops += 1
        if not avoided and loops % LASER_EVERY == 0:
            dist = laser.distance_fov()
            if dist >= MIN_VALID_MM:                 # invalid readings neither count nor reset
                close = close + 1 if dist < AVOID_MM else 0
            if dist < 2 * AVOID_MM and ticks_diff(ticks_ms(), last_log) >= 100:
                log("  laser {} mm, close {}/{}".format(dist, close, CONFIRM))
                last_log = ticks_ms()
            if close >= CONFIRM:
                log("Obstacle at {} mm".format(dist))
                for step in AROUND:
                    drive(0, 0, STEP_PAUSE_MS)
                    drive(*step)
                drive(0, 0, STEP_PAUSE_MS)
                l0, r0 = left.position(), right.position()
                start = ticks_ms()
                while light.reflection() > setpoint:
                    straight(RETURN[0], l0, r0)
                    if stop_pressed() or ticks_diff(ticks_ms(), start) > RETURN_MAX_MS:
                        drive(0, 0, 0)
                        log("Line not found after the obstacle")
                        return
                    sleep_ms(5)
                log("Back on the line")
                avoided = True
                back_at = ticks_ms()
                e_prev = d = integral = 0
                last_sign = 1            # came in from the right: if it overshoots, turn right
                last_tick = ticks_ms()

        # Line following
        now = ticks_ms()
        dt = max(ticks_diff(now, last_tick), 1)
        last_tick = now
        e = light.reflection() - setpoint         # > 0: too much white -> turn right
        d = D_FILTER * d + (1 - D_FILTER) * (e - e_prev) * 10 / dt
        e_prev = e
        if (e > 0) != (integral > 0):
            integral = 0
        integral = clamp(integral + e * dt / 10, -I_LIMIT / KI, I_LIMIT / KI)
        if e > lost_error:
            u = last_sign * LOST_TURN
        else:
            u = KP * e + KI * integral + KD * d
            if abs(u) > 2:
                last_sign = 1 if u > 0 else -1
        slow = avoided and ticks_diff(now, back_at) < AFTER_MS
        e_avg = SPEED_MEMORY * e_avg + (1 - SPEED_MEMORY) * abs(e)
        speed = max(0, (AFTER_POWER if slow else BASE_POWER) - SLOWDOWN * e_avg)
        left.set_power(clamp(speed + u, MIN_POWER, 100))
        right.set_power(clamp(speed - u, MIN_POWER, 100))
        sleep_ms(PERIOD_MS)

    drive(0, 0, 0)
    log("Stopped")


try:
    run()
except Exception as ex:
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
