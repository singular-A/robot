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
KP = 1.2            # steering strength
KI = 0.01           # corrects steady drift in long curves
KD = 7.0            # damping
D_FILTER = 0.7      # derivative smoothing 0..1
I_LIMIT = 50        # max steering power from the integral term
SLOWDOWN = 1        # forward power lost per unit of error
MIN_POWER = -25     # lowest wheel power
LOST_TURN = 40      # steering power when only white is seen
PERIOD_MS = 10      # regulation period
CAL_POWER = 25      # pivot power while measuring white and black

# Obstacle
AVOID_MM = 250      # obstacle closer than this: drive around it
CONFIRM = 3         # this many readings in a row before it reacts (ignores single glitches)

# The move around the obstacle, step by step: (left power, right power, time in ms).
# Tune the times on the track: longer turn = bigger angle, longer straight = further.
AROUND = [
    (50, -50, 300),     # pivot right, about 45 degrees
    (50, 50, 700),      # drive out, away from the line
    (-50, 50, 300),     # pivot left, back parallel to the line
    (50, 50, 1200),     # drive past the bottle
    (-50, 50, 300),     # pivot left, about 45 degrees toward the line
]
RETURN = (40, 40)       # then straight toward the line until the sensor sees it
RETURN_MAX_MS = 3000    # give up and stop if no line is found by then


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

    def drive(left_power, right_power, ms):
        left.set_power(left_power)
        right.set_power(right_power)
        sleep_ms(ms)

    def stop_pressed():
        return robot.buttons.pressed()[Button.LEFT]

    # Measure white and black: pivot right, left, back right, remember darkest and brightest
    sleep_ms(1000)
    black, white = 100, 0
    for direction, ms in ((1, 400), (-1, 800), (1, 400)):
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < ms:
            left.set_power(direction * CAL_POWER)
            right.set_power(-direction * CAL_POWER)
            r = light.reflection()
            black, white = min(black, r), max(white, r)
            sleep_ms(5)
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
    close = 0            # readings in a row closer than AVOID_MM
    avoided = False
    last_tick = ticks_ms()
    while not stop_pressed():
        # Obstacle? (only until it has driven around one)
        if not avoided:
            dist = laser.distance_fov()
            close = close + 1 if 20 <= dist < AVOID_MM else 0
            if close >= CONFIRM:
                log("Obstacle at {} mm".format(dist))
                for step in AROUND:
                    drive(*step)
                left.set_power(RETURN[0])
                right.set_power(RETURN[1])
                start = ticks_ms()
                while light.reflection() > setpoint:
                    if stop_pressed() or ticks_diff(ticks_ms(), start) > RETURN_MAX_MS:
                        drive(0, 0, 0)
                        log("Line not found after the obstacle")
                        return
                    sleep_ms(5)
                log("Back on the line")
                avoided = True
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
        speed = max(0, BASE_POWER - SLOWDOWN * abs(e))
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
