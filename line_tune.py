# Self-tuning line follower: finds the fastest settings by trial and error
#
# Setup:  same as line_follow.py (color sensor on S1, left motor M1, right motor M2)
#         Use a CLOSED track (loop), so the robot can keep driving between trials.
#
# How it works:
#   1. Calibrate white and black like in line_follow.py.
#   2. The robot follows the line in trials of TRIAL_MS. Each trial uses slightly
#      different settings (random changes around the best settings found so far).
#   3. Score = how fast the robot moves along the line (encoder distance per second),
#      reduced if it wobbles a lot. If it runs off the line it never drives back: it
#      steers hard (pivots) in the direction it was turning until the sensor finds the
#      line again, then continues slower for a moment. Every run-off halves the trial's
#      score, so the tuner prefers settings that stay on the line. If the line is not
#      found or the robot stalls, the trial only gets half credit for the distance it
#      covered, and you are asked to put it back on the line.
#   4. If a trial beats the best score, its settings become the new best and are
#      saved to SAVE_FILE on the cube. The best settings are re-tested now and then
#      so one lucky run cannot stay on top forever.
#   Press LEFT between or during trials to stop. Progress is kept in SAVE_FILE and
#   the next run continues from there.

import json
import math
from random import getrandbits
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

SAVE_FILE = "/line_tune.json"

TRIAL_MS = 6000       # length of one trial
WARMUP_MS = 1000      # start of each trial that is not scored (acceleration, settling)
LOOP_MS = 10          # control loop period
LOST_MS = 100         # sensor on white this long = off the line, steer hard back to it
STALL_MS = 800        # wheels not moving this long while powered = stalled
SEARCH_POWER = 70     # outer wheel power while steering hard back to the line
INNER_POWER = 0       # inner wheel power meanwhile (0 = pivot on the inner wheel, never backwards)
TURN_MEMORY = 0.9     # how long it remembers which way it was turning (0..1, higher = longer)
FIND_MS = 1500        # steer this long in the remembered direction, then try the other way
CHECK_RIGHT_MS = 150  # before steering left, steer right this long (it may only have swung past the edge)
RECOVER_MS = 1000     # after finding the line, drive slower for this long ...
RECOVER_SPEED = 0.5   # ... at this fraction of the base power, so it can take the curve
MAX_TRIALS = 100
RETEST_EVERY = 5      # re-run the best settings every N trials
WOBBLE_PENALTY = 0.5  # score *= 1 - WOBBLE_PENALTY * (average |error| as 0..1)
OFF_LINE_PENALTY = 0.5  # score *= this for every time the robot ran off the line in a trial

MIN_POWER = -30       # lowest wheel power
D_FILTER = 0.7        # derivative smoothing (same as line_follow.py)

# Tuned settings: start value, allowed range, and typical step size
START = {"base": 35.0, "kp": 0.8, "kd": 3.0, "slowdown": 0.5}
LIMITS = {"base": (20.0, 100.0), "kp": (0.2, 3.0), "kd": (0.0, 10.0), "slowdown": (0.0, 1.5)}
STEP = {"base": 5.0, "kp": 0.15, "kd": 0.5, "slowdown": 0.1}

robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
robot.init_motor(Port.M1)
robot.init_motor(Port.M2)

sensor = robot.sensors.light[Port.S1]
left = robot.motors[Port.M1]
right = robot.motors[Port.M2]
left.init_encoder()
right.init_encoder()


def log(message):
    robot.esp.bt_write(message + ";")
    print(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def stop_motors():
    left.set_power(0)
    right.set_power(0)


def any_pressed():
    buttons = robot.buttons.pressed()
    return buttons[Button.OK] or buttons[Button.RIGHT] or buttons[Button.LEFT]


def wait_for_ok(message):
    # Wait for a full press and release of OK (or RIGHT).
    # Returns False if LEFT was pressed instead.
    log(message)
    while any_pressed():
        sleep_ms(20)
    while True:
        buttons = robot.buttons.pressed()
        if buttons[Button.LEFT]:
            return False
        if buttons[Button.OK] or buttons[Button.RIGHT]:
            break
        sleep_ms(20)
    while any_pressed():
        sleep_ms(20)
    robot.led.toggle()
    return True


def gauss():
    # Standard normal random number (Box-Muller), using only getrandbits
    u1 = (getrandbits(24) + 1) / 16777217
    u2 = (getrandbits(24) + 1) / 16777217
    return math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)


def mutate(params, sigma):
    # New candidate: random change of every setting around the current best
    new = {}
    for name in params:
        low, high = LIMITS[name]
        new[name] = clamp(params[name] + gauss() * STEP[name] * sigma, low, high)
    return new


def fmt(params):
    return "base={:.1f} kp={:.2f} kd={:.2f} sd={:.2f}".format(
        params["base"], params["kp"], params["kd"], params["slowdown"])


def distance():
    # Average wheel position in degrees; abs() in the result makes the sign irrelevant
    return (left.position() + right.position()) / 2


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


def run_trial(p, white, black, target):
    # Follow the line for TRIAL_MS with settings p.
    # Returns (status, score, speed, wobble, recoveries); status is "ok", "lost", "stall" or "abort".
    half = (white - black) / 2
    start = ticks_ms()
    last_tick = start
    last_error = clamp(sensor.reflection(), black, white) - target
    derivative = 0
    lost_since = None
    last_l, last_r = left.position(), right.position()
    turn_avg = 0               # recent turning (wheel difference) while the sensor sees the line
    drive_avg = 0              # recent driving (wheel average) at the same time
    steer_avg = 0              # recent steering command at the same time (+ right, - left)
    recovered_at = None        # when the line was found again (drive slower for a moment)
    recoveries = 0
    stall_ref_t = start        # stall check: wheel positions at the start of the window
    stall_ref_l = left.position()
    stall_ref_r = right.position()
    start_pos = None
    start_t = 0
    err_sum = 0
    samples = 0
    t = 0

    first_pos = distance()

    def failed(status):
        # Partial credit: distance covered before failing, at half value, so that
        # among failing settings the ones that got further still rank higher
        return status, 0.5 * abs(distance() - first_pos) / (TRIAL_MS / 1000), 0, 0, recoveries

    while True:
        now = ticks_ms()
        t = ticks_diff(now, start)
        dt = ticks_diff(now, last_tick)
        last_tick = now
        if t >= TRIAL_MS:
            break

        if robot.buttons.pressed()[Button.LEFT]:
            return "abort", 0, 0, 0, 0

        refl = clamp(sensor.reflection(), black, white)
        error = refl - target
        raw_derivative = (error - last_error) * LOOP_MS / max(dt, 1)
        last_error = error
        derivative = D_FILTER * derivative + (1 - D_FILTER) * raw_derivative

        turn = p["kp"] * error + p["kd"] * derivative
        if refl < target + 0.3 * half:
            steer_avg = TURN_MEMORY * steer_avg + (1 - TURN_MEMORY) * turn
        # Drive slower for a moment after finding the line again, so it can take the curve
        base = p["base"]
        if recovered_at is not None and ticks_diff(now, recovered_at) < RECOVER_MS:
            base *= RECOVER_SPEED
        speed = max(0, base - p["slowdown"] * abs(error))
        left_power = clamp(speed + turn, MIN_POWER, 100)
        right_power = clamp(speed - turn, MIN_POWER, 100)
        left.set_power(left_power)
        right.set_power(right_power)

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
                recoveries += 1
                turning_left = turn_avg < -0.2 * abs(drive_avg)
                direction = -1 if turning_left and steer_avg < 0 else 1
                log("  off line at t={} ms, steering hard {}".format(
                    t, "left" if direction < 0 else "right"))
                found = recover(direction, target, half)
                if found == "abort":
                    return "abort", 0, 0, 0, 0
                if not found:
                    return failed("lost")
                log("  line found at t={} ms".format(ticks_diff(ticks_ms(), start)))
                # Continue following from a clean state, slower for a moment
                lost_since = None
                turn_avg = drive_avg = steer_avg = 0
                last_l, last_r = left.position(), right.position()
                recovered_at = ticks_ms()
                last_error = clamp(sensor.reflection(), black, white) - target
                derivative = 0
                last_tick = ticks_ms()
                stall_ref_t = last_tick
                stall_ref_l = left.position()
                stall_ref_r = right.position()
                continue
        else:
            lost_since = None

        # Stalled: powered but the wheel positions barely changed over STALL_MS.
        # Uses encoder positions instead of speed() so short speed dips do not count.
        if max(abs(left_power), abs(right_power)) <= 20:
            stall_ref_t = now
            stall_ref_l = left.position()
            stall_ref_r = right.position()
        elif ticks_diff(now, stall_ref_t) >= STALL_MS:
            moved = abs(left.position() - stall_ref_l) + abs(right.position() - stall_ref_r)
            if moved < 30:
                log("  STALL: Lpow={:.0f} Rpow={:.0f} Lspd={} Rspd={} moved={} refl={:.0f} err={:.0f} bat={:.2f}".format(
                    left_power, right_power, left.speed(), right.speed(), moved,
                    refl, error, robot.battery.voltage()))
                return failed("stall")
            stall_ref_t = now
            stall_ref_l = left.position()
            stall_ref_r = right.position()

        # Scoring starts after warmup
        if t >= WARMUP_MS:
            if start_pos is None:
                start_pos = distance()
                start_t = t
            err_sum += abs(error) / half
            samples += 1

        sleep_ms(LOOP_MS)

    seconds = (t - start_t) / 1000
    speed = abs(distance() - start_pos) / seconds  # wheel degrees per second
    wobble = err_sum / max(samples, 1)             # 0 = always on the edge, 1 = always fully off
    score = speed * (1 - WOBBLE_PENALTY * wobble) * OFF_LINE_PENALTY ** recoveries
    return "ok", score, speed, wobble, recoveries


def load():
    try:
        with open(SAVE_FILE) as f:
            data = json.load(f)
        return data["params"], data.get("trials", 0)
    except Exception:
        return dict(START), 0


def save(params, score, trials):
    try:
        with open(SAVE_FILE, "w") as f:
            json.dump({"params": params, "score": score, "trials": trials}, f)
    except Exception as e:
        log("Save failed: {}".format(e))


def main():
    if not wait_for_ok("Place sensor on WHITE, press OK"):
        return
    white = sensor.reflection()
    if not wait_for_ok("Place sensor on BLACK, press OK"):
        return
    black = sensor.reflection()
    target = (white + black) / 2
    log("white={:.1f} black={:.1f} target={:.1f}".format(white, black, target))

    best, total_trials = load()
    log("Starting from " + fmt(best) + " (previous trials: {})".format(total_trials))

    if not wait_for_ok("Put robot on left edge of line, press OK to start tuning"):
        return

    # Battery sags over time, which makes later trials slower; scale scores to the start voltage
    ref_voltage = robot.battery.voltage()

    best_score = None
    best_runs = 0     # how many times the best settings were measured (score is their average)
    sigma = 1.0       # step size multiplier: grows after improvements, shrinks after failures

    for trial in range(1, MAX_TRIALS + 1):
        retest = best_score is None or trial % RETEST_EVERY == 0
        candidate = best if retest else mutate(best, sigma)

        status, score, speed, wobble, recoveries = run_trial(candidate, white, black, target)
        if status == "abort":
            break
        if status == "ok":
            score *= ref_voltage / robot.battery.voltage()

        # Short stop between trials so saving to flash cannot disturb driving
        stop_motors()
        total_trials += 1
        tag = "RETEST" if retest else "TRY"
        log("#{} {} {} -> {} score={:.0f} spd={:.0f} wob={:.2f} rec={} sigma={:.2f}".format(
            trial, tag, fmt(candidate), status, score, speed, wobble, recoveries, sigma))

        if retest:
            # Average the best's repeated scores so one lucky run does not dominate
            best_score = score if best_score is None else (best_score * best_runs + score) / (best_runs + 1)
            best_runs += 1
        elif score > best_score:
            best, best_score, best_runs = candidate, score, 1
            sigma = min(sigma * 1.3, 3.0)
            log("NEW BEST score={:.0f} {}".format(best_score, fmt(best)))
            save(best, best_score, total_trials)
        else:
            sigma = max(sigma * 0.92, 0.2)

        if status != "ok":
            sigma = max(sigma * 0.8, 0.2)
            if not wait_for_ok("Line {}. Put robot back on the line, press OK".format(status)):
                break
        else:
            sleep_ms(300)

    stop_motors()
    save(best, best_score, total_trials)
    log("DONE. Best: " + fmt(best) + " score={}".format(best_score))
    log("Copy to line_follow.py: BASE_POWER={:.0f} KP={:.2f} KD={:.2f} (SLOWDOWN={:.2f}, KI=0)".format(
        best["base"], best["kp"], best["kd"], best["slowdown"]))


try:
    main()
except Exception as e:
    robot.esp.bt_write("Crashed: {} {};".format(type(e).__name__, e))
    import sys
    sys.print_exception(e)

stop_motors()
