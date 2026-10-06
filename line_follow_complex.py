# Basic line follower (PID control, follows the left edge of a black line)
#
# Setup:  Open-Cube color sensor on S1, pointing down, in front of the wheels
#         Left motor on M1, right motor on M2
#
# Usage:  1. Place the sensor over WHITE and press OK (or RIGHT)
#         2. Place the sensor over BLACK and press OK (or RIGHT)
#         3. Put the robot on the left edge of the line and press OK (or RIGHT) to start
#         The cube LED toggles each time a press is registered.
#         Press LEFT at any time to stop.
#
# If the robot runs off the line, it never drives back: it keeps going forward and
# steers hard (pivots) in the direction it was turning while it last saw the line,
# until the sensor finds the line again. If the line is not found that way, it tries
# the other direction.

from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button
BASE_POWER = 48   # forward power in %
KP = 0.82          # how hard to steer; raise if it loses the line, lower if it wobbles
KI = 0.01         # integral gain; corrects steady drift in long curves, lower if it oscillates slowly
I_LIMIT = 10      # max steering power from the integral term (anti-windup)
KD = 3.49          # derivative gain; damps wobbling, lower if steering gets jittery
D_FILTER = 0.68    # derivative smoothing 0..1; higher = smoother but slower to react
MIN_POWER = -10   # lowest wheel power; keeps the inner wheel from spinning hard backwards
LOG_EVERY = 10    # log every N loops (10 = every ~100 ms)
LOST_MS = 100     # sensor on white this long = off the line, steer hard back to it
SEARCH_POWER = 70 # outer wheel power while steering hard back to the line
INNER_POWER = 0   # inner wheel power meanwhile (0 = pivot on the inner wheel, never backwards)
TURN_MEMORY = 0.9 # how long it remembers which way it was turning (0..1, higher = longer)
FIND_MS = 1500    # steer this long in the remembered direction, then try the other way
CHECK_RIGHT_MS = 150  # before steering left, steer right this long (it may only have swung past the edge)
RECOVER_MS = 1000 # after finding the line, drive slower for this long ...
RECOVER_SPEED = 0.5  # ... at this fraction of BASE_POWER, so it can take the curve

robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
robot.init_motor(Port.M1)
robot.init_motor(Port.M2)

sensor = robot.sensors.light[Port.S1]
left = robot.motors[Port.M1]
right = robot.motors[Port.M2]

# Encoders let us log whether the wheels actually turn, not just the commanded power
left.init_encoder()
right.init_encoder()


def any_pressed():
    buttons = robot.buttons.pressed()
    return buttons[Button.OK] or buttons[Button.RIGHT] or buttons[Button.LEFT]


def wait_for_ok(message):
    # Wait for a full press and release of OK (or RIGHT).
    # Returns False if LEFT was pressed instead.
    robot.esp.bt_write(message + ";")
    print(message)

    # Wait until all buttons are released (e.g. the previous OK press)
    while any_pressed():
        sleep_ms(20)

    # Wait for a new press
    while True:
        buttons = robot.buttons.pressed()
        if buttons[Button.LEFT]:
            return False
        if buttons[Button.OK] or buttons[Button.RIGHT]:
            break
        sleep_ms(20)

    # Wait for release so the same press is not counted twice
    while any_pressed():
        sleep_ms(20)

    robot.led.toggle()  # visual confirmation that the press was registered
    return True


def recover(direction, target, half):
    # Steer hard back to the line without ever driving backwards: the outer wheel drives
    # forward and the inner wheel stops, so the robot pivots toward `direction`
    # (+1 right, -1 left), the way it was turning while it last saw the line.
    #  - Right: the line is to the right, stop as soon as the sensor reaches it.
    #  - Left: the robot probably shot across the line in a left curve and is on its right
    #    side. It could also have only swung past the left edge, so first steer right
    #    briefly (finds the line at once in that case), then steer left across the black
    #    line until the sensor comes out on the left edge.
    # If the line is not found within FIND_MS, try the other direction for twice as long.
    # Returns True when back on the left edge, False if not found, "abort" on LEFT.
    if direction > 0:
        attempts = ((1, FIND_MS), (-1, 2 * FIND_MS))
    else:
        attempts = ((1, CHECK_RIGHT_MS), (-1, FIND_MS), (1, 2 * FIND_MS))
    for d, limit in attempts:
        if limit <= 0:
            continue
        left.set_power(SEARCH_POWER if d > 0 else INNER_POWER)
        right.set_power(INNER_POWER if d > 0 else SEARCH_POWER)
        start = ticks_ms()
        seen_black = False
        while True:
            if robot.buttons.pressed()[Button.LEFT]:
                return "abort"
            refl = sensor.reflection()
            if d > 0:
                if refl < target + 0.3 * half:
                    return True
            elif refl < target - 0.4 * half:
                seen_black = True
            elif seen_black and refl > target:
                return True
            # Give up this direction after `limit`, but never while half-way across the line
            elapsed = ticks_diff(ticks_ms(), start)
            crossing = d < 0 and refl < target + 0.3 * half
            if elapsed > 2 * limit or (elapsed > limit and not crossing):
                break
            sleep_ms(5)
    return False


def run():
    # Calibration
    if not wait_for_ok("Place sensor on WHITE, press OK"):
        return
    white = sensor.reflection()
    if not wait_for_ok("Place sensor on BLACK, press OK"):
        return
    black = sensor.reflection()
    target = (white + black) / 2
    half = (white - black) / 2
    robot.esp.bt_write("white={} black={} target={};".format(white, black, target))

    if not wait_for_ok("Put robot on left edge of line, press OK to start"):
        return

    # Follow the line
    robot.esp.bt_write("t_ms,loop,refl_raw,err,P,I,D,turn,Lpow,Rpow,Lspd,Rspd,Lpos,Rpos,dt_ms,bat;")
    integral = 0
    # Start from the real error so the first derivative is not a big kick
    last_error = max(black, min(white, sensor.reflection())) - target
    derivative = 0
    loops = 0
    start = ticks_ms()
    last_tick = start
    stalled = False
    lost_since = None
    last_l, last_r = left.position(), right.position()
    turn_avg = 0               # recent turning (wheel difference) while the sensor sees the line
    drive_avg = 0              # recent driving (wheel average) at the same time
    steer_avg = 0              # recent steering command at the same time (+ right, - left)
    recovered_at = None        # when the line was found again (drive slower for a moment)
    while True:
        loops += 1
        now = ticks_ms()
        dt = ticks_diff(now, last_tick)  # loop duration; big values mean the loop froze
        last_tick = now

        if robot.buttons.pressed()[Button.LEFT]:
            robot.esp.bt_write("Stopped: LEFT pressed at t={} ms;".format(ticks_diff(now, start)))
            print("Stopped: LEFT pressed")
            break

        # error > 0: too much white (drifted left) -> turn right
        # error < 0: too much black (drifted right) -> turn left
        # Clamp to the calibrated range so readings darker than the line (e.g. off the mat)
        # do not produce huge errors
        refl_raw = sensor.reflection()
        refl = max(black, min(white, refl_raw))
        error = refl - target

        # Remember which way the robot is turning while the sensor sees the line, measured
        # with the encoders (its real turning, e.g. into a curve). Not updated on white,
        # so it still holds the direction from before the robot ran off the line.
        l_pos, r_pos = left.position(), right.position()
        dl, dr = l_pos - last_l, r_pos - last_r
        last_l, last_r = l_pos, r_pos
        if refl < target + 0.3 * half:
            turn_avg = TURN_MEMORY * turn_avg + (1 - TURN_MEMORY) * (dl - dr)
            drive_avg = TURN_MEMORY * drive_avg + (1 - TURN_MEMORY) * (dl + dr) / 2

        # Off the line: sensor (almost) fully on white for too long -> steer hard back to it
        # in the direction it was turning. Left only when the robot was clearly turning left
        # AND still steering left (it shot across the line in a left curve). Otherwise the
        # line is to the right: going straight, in a right curve, or at the end of a left
        # curve (still turning left, but already steering right).
        if refl > target + 0.85 * half:
            if lost_since is None:
                lost_since = now
            elif ticks_diff(now, lost_since) > LOST_MS:
                turning_left = turn_avg < -0.2 * abs(drive_avg)
                direction = -1 if turning_left and steer_avg < 0 else 1
                robot.esp.bt_write("Off line at t={} ms, steering hard {};".format(
                    ticks_diff(now, start), "left" if direction < 0 else "right"))
                found = recover(direction, target, half)
                if found == "abort":
                    robot.esp.bt_write("Stopped: LEFT pressed during search;")
                    break
                if found:
                    robot.esp.bt_write("Line found at t={} ms;".format(ticks_diff(ticks_ms(), start)))
                else:
                    left.set_power(0)
                    right.set_power(0)
                    if not wait_for_ok("Line not found. Put robot on left edge of line, press OK"):
                        break
                # Continue following from a clean state, slower for a moment
                lost_since = None
                turn_avg = drive_avg = steer_avg = 0
                last_l, last_r = left.position(), right.position()
                recovered_at = ticks_ms()
                integral = 0
                last_error = max(black, min(white, sensor.reflection())) - target
                derivative = 0
                last_tick = ticks_ms()
                continue
        else:
            lost_since = None

        # Integral accumulates error over time, clamped so KI * integral stays within I_LIMIT
        # Reset when crossing the edge, so old error does not keep pulling the robot
        if (error > 0) != (integral > 0):
            integral = 0
        integral += error
        max_integral = I_LIMIT / KI
        integral = max(-max_integral, min(max_integral, integral))

        # Derivative reacts to how fast the error changes, scaled to a 10 ms step so
        # slower loops (e.g. while logging) do not produce spikes, then smoothed
        # because sensor noise would otherwise make the steering jerk
        raw_derivative = (error - last_error) * 10 / max(dt, 1)
        last_error = error
        derivative = D_FILTER * derivative + (1 - D_FILTER) * raw_derivative

        p_term = KP * error
        i_term = KI * integral
        d_term = KD * derivative
        turn = p_term + i_term + d_term
        if refl < target + 0.3 * half:
            steer_avg = TURN_MEMORY * steer_avg + (1 - TURN_MEMORY) * turn

        # Drive slower for a moment after finding the line again, so it can take the curve
        base = BASE_POWER
        if recovered_at is not None and ticks_diff(now, recovered_at) < RECOVER_MS:
            base = BASE_POWER * RECOVER_SPEED

        left_power = max(MIN_POWER, min(100, base + turn))
        right_power = max(MIN_POWER, min(100, base - turn))
        left.set_power(left_power)
        right.set_power(right_power)

        # Actual wheel motion from the encoders
        left_speed = left.speed()
        right_speed = right.speed()

        # Stall detection: motors are powered but the wheels are not turning
        is_stalled = (abs(left_power) > 20 and abs(left_speed) < 30) or                      (abs(right_power) > 20 and abs(right_speed) < 30)
        if is_stalled != stalled:
            stalled = is_stalled
            robot.esp.bt_write("{} at t={} ms;".format(
                "STALL START" if stalled else "STALL END", ticks_diff(now, start)))

        if loops % LOG_EVERY == 0 or dt > 50:
            robot.esp.bt_write(
                "{},{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.0f},{:.0f},{},{},{},{},{},{:.2f};".format(
                    ticks_diff(now, start), loops, refl_raw, error,
                    p_term, i_term, d_term, turn,
                    left_power, right_power, left_speed, right_speed,
                    left.position(), right.position(), dt, robot.battery.voltage()))

        sleep_ms(10)

try:
    run()
except Exception as e:
    # Report why the program crashed instead of silently stopping
    robot.esp.bt_write("Crashed: {} {};".format(type(e).__name__, e))
    import sys
    sys.print_exception(e)

left.set_power(0)
right.set_power(0)