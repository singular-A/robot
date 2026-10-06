# SELF-LEARNING line follower for a CLOSED track (a loop): let it circle on its own
# (separate project; the safe versions and the start/finish learner are not touched)
#
# How it works:
#   1. Put the robot on the EDGE of the line, run the program, press OK (or RIGHT).
#      It measures white and black (pivots over the line) and drives one warm-up lap.
#   2. Laps are detected automatically, without any marker: on a simple loop the robot
#      turns exactly one full circle (360 degrees) per lap, and the wheel encoders measure
#      the turning. (Only for a simple loop - a figure eight has no net turning.)
#   3. Every lap is one round with slightly changed settings (BASE_POWER, KP, KD,
#      SLOWDOWN, MIN_POWER) around the best ones. Goal = fastest lap:
#      score = 100 / (lap time + OFF_TRACK_PENALTY_S for every time it left the line).
#   5. Leaving the line is handled automatically, so it can train alone: when the sensor
#      sees only white for LOST_MS, it steers hard back (pivots on the inner wheel, never
#      drives backwards) in the direction it was turning, finds the line, continues slower
#      for a moment and keeps going. The penalty teaches it to avoid settings that leave it.
#   4. Faster -> the new settings become the best (saved on the cube in SAVE_FILE, the
#      next start continues from there). Every RETEST_EVERY laps the best settings are
#      driven again and averaged, so one lucky lap cannot fool it.
#   The robot never stops between laps; the settings change while it drives.
#
# Only if it cannot find the line by itself (or you press LEFT while it drives) it stops,
#   the lap scores 0, and it waits: OK = continue (put it back on the edge first), LEFT = quit.
# Check the "dist" in the log: if every lap shows about the same distance, the automatic
# lap detection works. If not, adjust TRACK_CM (distance between the wheels).
# Setup: color sensor on S1, left motor M1, right motor M2.
import json
import math
from random import getrandbits
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

SAVE_FILE = "/line_learn_loop.json"
MAX_ROUNDS = 50     # stop after this many scored laps
RETEST_EVERY = 4    # drive the best settings again every N laps
MAX_LAP_S = 90      # safety: a lap longer than this scores 0 (lap not detected?)
LOST_MS = 100       # sensor on white this long = off the line: steer hard back to it
SEARCH_POWER = 70   # outer wheel power while steering back (inner wheel stops, never backwards)
FIND_MS = 1500      # steer back this long in the remembered direction, then try the other way
CHECK_RIGHT_MS = 150  # before steering back left, steer right this long (may only have swung past)
TURN_MEMORY = 0.9   # how long it remembers which way it was turning (0..1, higher = longer)
RECOVER_MS = 1000   # after finding the line, drive slower for this long ...
RECOVER_SPEED = 0.5 # ... at this fraction of the base power, so it can take the curve
OFF_TRACK_PENALTY_S = 2.0  # seconds added to the lap time (for the score) per time it left the line
WHEEL_CM = 8.0      # wheel diameter
TRACK_CM = 19.0     # distance between the wheels (centre of the tyres)
# One full robot turn = each wheel travels pi*TRACK_CM in opposite directions:
# left - right encoder difference = 2 * TRACK_CM / WHEEL_CM * 360 degrees
LAP_TURN = 2 * TRACK_CM / WHEEL_CM * 360

# Fixed parts of the line follower (same as line_follow_safe.py)
KI = 0.01
D_FILTER = 0.7
I_LIMIT = 10
LOST_TURN = 40
PERIOD_MS = 10
CAL_POWER = 25
CAL_SWEEP = 150
MIN_CONTRAST = 20

# Learned settings: start values, allowed range, step size
# SAFE STAGE: speed capped at MAX_BASE so it rarely leaves the line and can train alone.
# When it is reliable, raise MAX_BASE step by step (60 -> 70 -> 80 -> 100); it continues
# from the best settings learned so far (as long as START is not changed).
MAX_BASE = 60.0
START = {"base": 55.0, "kp": 1.19, "kd": 7.2, "slowdown": 0.70, "min_power": -31.0}
LIMITS = {"base": (25.0, MAX_BASE), "kp": (0.2, 3.0), "kd": (4.0, 12.0),
          "slowdown": (0.0, 2.0), "min_power": (-50.0, 0.0)}
STEP = {"base": 4.0, "kp": 0.1, "kd": 0.5, "slowdown": 0.15, "min_power": 5.0}


def log(message):
    robot.esp.bt_write(message + ";")
    print(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def gauss():
    # Normal random number (Box-Muller)
    u1 = (getrandbits(24) + 1) / 16777217
    u2 = (getrandbits(24) + 1) / 16777217
    return math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)


def mutate(params, sigma):
    # New settings: a small random change of every setting around the best ones
    return {k: clamp(v + gauss() * STEP[k] * sigma, LIMITS[k][0], LIMITS[k][1])
            for k, v in params.items()}


def fmt(p):
    return "base={:.0f} kp={:.2f} kd={:.1f} sd={:.2f} minp={:.0f}".format(
        p["base"], p["kp"], p["kd"], p["slowdown"], p["min_power"])


def load():
    # Continue from the saved best settings - unless START was changed in this file
    # since they were saved: then start fresh from the new START values.
    try:
        with open(SAVE_FILE) as f:
            data = json.load(f)
    except Exception:
        return dict(START), 0
    if data.get("start") != START:
        log("START was changed in the file - starting fresh from START")
        return dict(START), 0
    params = dict(START)
    params.update(data["params"])
    # Keep saved settings within the current LIMITS (e.g. a lower MAX_BASE)
    params = {k: clamp(v, LIMITS[k][0], LIMITS[k][1]) for k, v in params.items()}
    return params, data.get("rounds", 0)


def save(params, rounds):
    try:
        with open(SAVE_FILE, "w") as f:
            json.dump({"params": params, "rounds": rounds, "start": START}, f)
    except Exception as ex:
        log("Save failed: {}".format(ex))


def buttons():
    # (OK or RIGHT pressed, LEFT pressed)
    b = robot.buttons.pressed()
    return b[Button.OK] or b[Button.RIGHT], b[Button.LEFT]


def wait_for_ok():
    # Wait for a full press and release of OK (or RIGHT). Returns False if LEFT was pressed.
    while any(buttons()):
        sleep_ms(20)
    while True:
        ok, quit_ = buttons()
        if quit_:
            return False
        if ok:
            break
        sleep_ms(20)
    while any(buttons()):
        sleep_ms(20)
    robot.led.toggle()
    return True


def run():
    robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
    robot.init_motor(Port.M1)
    robot.init_motor(Port.M2)
    light = robot.sensors.light[Port.S1]
    left = robot.motors[Port.M1]
    right = robot.motors[Port.M2]
    left.init_encoder()
    right.init_encoder()

    def stop():
        left.set_power(0)
        right.set_power(0)

    # --- Measure white and black (same as line_follow_safe.py) ---
    seen = [100, 0]

    def pivot_to(goal):
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
        stop()

    def align(setpoint):
        # Pivot slowly until the sensor is exactly on the edge
        side = 1 if light.reflection() > setpoint else -1
        start = ticks_ms()
        while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
            left.set_power(side * CAL_POWER)
            right.set_power(-side * CAL_POWER)
            sleep_ms(5)
        stop()

    log("Closed-track learner. Robot on the EDGE of the line, press OK to start (LEFT = quit)")
    if not wait_for_ok():
        return
    sleep_ms(300)
    home = left.position() - right.position()
    pivot_to(home + CAL_SWEEP)
    pivot_to(home - CAL_SWEEP)
    pivot_to(home)
    black, white = seen
    if white - black < MIN_CONTRAST:
        log("white={:.0f} black={:.0f}: no line found - start on the edge of the line".format(white, black))
        return
    setpoint = (white + black) / 2
    half = white - setpoint
    lost_error = 0.8 * half
    log("white={:.0f} black={:.0f} setpoint={:.0f}, lap = {:.0f} deg of turning".format(
        white, black, setpoint, LAP_TURN))
    align(setpoint)

    def recover(direction):
        # Steer hard back to the line without ever driving backwards: the outer wheel drives
        # forward and the inner wheel stops, so the robot pivots toward `direction`
        # (+1 right, -1 left), the way it was turning while it last saw the line.
        #  - Right: the line is to the right, stop as soon as the sensor reaches it.
        #  - Left: the robot probably shot across the line in a left curve and is on its right
        #    side. It could also have only swung past the left edge, so first steer right
        #    briefly (finds the line at once in that case), then steer left across the black
        #    line until the sensor comes out on the left edge.
        # If not found within FIND_MS, try the other direction for twice as long.
        # Returns True when back on the left edge, False if not found, "fail" on LEFT.
        if direction > 0:
            attempts = ((1, FIND_MS), (-1, 2 * FIND_MS))
        else:
            attempts = ((1, CHECK_RIGHT_MS), (-1, FIND_MS), (1, 2 * FIND_MS))
        for d, limit in attempts:
            left.set_power(SEARCH_POWER if d > 0 else 0)
            right.set_power(0 if d > 0 else SEARCH_POWER)
            start = ticks_ms()
            seen_black = False
            while True:
                if buttons()[1]:
                    return "fail"
                r = light.reflection()
                if d > 0:
                    if r < setpoint + 0.3 * half:
                        return True
                elif r < setpoint - 0.4 * half:
                    seen_black = True
                elif seen_black and r > setpoint:
                    return True
                # Give up this direction after `limit`, but never while half-way across the line
                elapsed = ticks_diff(ticks_ms(), start)
                crossing = d < 0 and r < setpoint + 0.3 * half
                if elapsed > 2 * limit or (elapsed > limit and not crossing):
                    break
                sleep_ms(5)
        stop()
        return False

    # --- Line follower state (kept between laps: the robot never stops) ---
    st = {"e_prev": 0, "d": 0, "integral": 0, "last_sign": 1,
          "last_tick": ticks_ms(), "last_l": left.position(), "last_r": right.position(),
          "turn_avg": 0, "drive_avg": 0, "steer_avg": 0, "recovered_at": None}

    def reset_follower():
        st.update(e_prev=0, d=0, integral=0, last_tick=ticks_ms(),
                  last_l=left.position(), last_r=right.position(),
                  turn_avg=0, drive_avg=0, steer_avg=0)

    def drive_lap(p):
        # Drive until the robot has turned one full circle (= one lap) with settings p.
        # Returns (end, seconds, err, dist_cm, recoveries);
        # end is "lap", "fail" (LEFT), "lost" (could not find the line again) or "time".
        start = ticks_ms()
        turn = 0           # robot turning since the lap started (left - right wheel degrees)
        dist = 0           # distance driven in this lap (wheel degrees, average of both wheels)
        n = 0
        err_sum = 0
        recoveries = 0     # times it left the line and steered back by itself
        white_since = None
        end = "time"
        t = 0
        while True:
            now = ticks_ms()
            t = ticks_diff(now, start)
            if t >= MAX_LAP_S * 1000:
                break
            if buttons()[1]:
                end = "fail"
                break

            # The safe line follower with this lap's settings
            dt = max(ticks_diff(now, st["last_tick"]), 1)
            st["last_tick"] = now
            e = light.reflection() - setpoint
            st["d"] = D_FILTER * st["d"] + (1 - D_FILTER) * (e - st["e_prev"]) * 10 / dt
            st["e_prev"] = e
            if (e > 0) != (st["integral"] > 0):
                st["integral"] = 0
            st["integral"] = clamp(st["integral"] + e * dt / 10, -I_LIMIT / KI, I_LIMIT / KI)
            if e > lost_error:
                u = st["last_sign"] * LOST_TURN
            else:
                u = p["kp"] * e + KI * st["integral"] + p["kd"] * st["d"]
                if abs(u) > 2:
                    st["last_sign"] = 1 if u > 0 else -1
            base = p["base"]
            if st["recovered_at"] is not None and ticks_diff(now, st["recovered_at"]) < RECOVER_MS:
                base *= RECOVER_SPEED       # just found the line again: slower for a moment
            speed = max(0, base - p["slowdown"] * abs(e))
            left.set_power(clamp(speed + u, p["min_power"], 100))
            right.set_power(clamp(speed - u, p["min_power"], 100))

            # Lap detection: add up how much the robot turned
            l_pos, r_pos = left.position(), right.position()
            dl, dr = l_pos - st["last_l"], r_pos - st["last_r"]
            st["last_l"], st["last_r"] = l_pos, r_pos
            turn += dl - dr
            dist += (dl + dr) / 2
            if abs(turn) >= LAP_TURN:
                end = "lap"
                break

            # Remember which way it turns (encoders) and steers while it sees the line
            if e < 0.3 * half:
                m = TURN_MEMORY
                st["turn_avg"] = m * st["turn_avg"] + (1 - m) * (dl - dr)
                st["drive_avg"] = m * st["drive_avg"] + (1 - m) * (dl + dr) / 2
                st["steer_avg"] = m * st["steer_avg"] + (1 - m) * u

            n += 1
            err_sum += min(abs(e) / half, 1)

            # Off the line: steer hard back to it, in the direction it was turning. Left only
            # when it was clearly turning left AND still steering left (shot across the line in
            # a left curve); otherwise the line is to the right.
            if e > lost_error:
                if white_since is None:
                    white_since = now
                elif ticks_diff(now, white_since) > LOST_MS:
                    turning_left = st["turn_avg"] < -0.2 * abs(st["drive_avg"])
                    direction = -1 if turning_left and st["steer_avg"] < 0 else 1
                    recoveries += 1
                    found = recover(direction)
                    if found == "fail":
                        end = "fail"
                        break
                    if not found:
                        end = "lost"
                        break
                    # Found it. The search may have spun the robot around: full circles do not
                    # move it along the track, so only count the turning modulo one circle
                    # (between -180 and +180 degrees) for the lap detection.
                    l_pos, r_pos = left.position(), right.position()
                    spin = (l_pos - st["last_l"]) - (r_pos - st["last_r"])
                    turn += (spin + LAP_TURN / 2) % LAP_TURN - LAP_TURN / 2
                    dist += ((l_pos - st["last_l"]) + (r_pos - st["last_r"])) / 2
                    reset_follower()
                    st["recovered_at"] = ticks_ms()
                    white_since = None
                    continue
            else:
                white_since = None
            sleep_ms(PERIOD_MS)
        dist_cm = dist / 360 * math.pi * WHEEL_CM
        return end, t / 1000, err_sum / max(n, 1), dist_cm, recoveries

    # --- Warm-up lap (from standstill, not scored) ---
    best, total = load()
    log("Learning from " + fmt(best) + " (laps so far: {}). Warm-up lap...".format(total))
    end, seconds, err, dist_cm, rec = drive_lap(best)
    log("Warm-up lap: {} time={:.1f}s dist={:.0f}cm".format(end.upper(), seconds, dist_cm))

    best_score = None
    best_runs = 0
    sigma = 1.0          # step size: bigger after improvements, smaller after failures
    rnd = 0
    while rnd < MAX_ROUNDS:
        if end != "lap":
            # Failed / lost / not detected: stop and wait for the person
            stop()
            log("Stopped ({}). Put the robot on the EDGE of the line, OK = continue, LEFT = quit".format(end))
            if not wait_for_ok():
                break
            sleep_ms(300)
            align(setpoint)
            reset_follower()
        rnd += 1
        retest = best_score is None or rnd % RETEST_EVERY == 0
        cand = best if retest else mutate(best, sigma)
        end, seconds, err, dist_cm, rec = drive_lap(cand)
        total += 1
        # Fastest lap wins; every time it left the line counts as OFF_TRACK_PENALTY_S extra
        score = 100 / (seconds + OFF_TRACK_PENALTY_S * rec) if end == "lap" else 0
        line = "#{} {} {} -> {} time={:.1f}s left_line={} score={:.2f} (dist={:.0f}cm err={:.0f}%)".format(
            rnd, "BEST" if retest else "TRY ", fmt(cand), end.upper(), seconds, rec, score, dist_cm, 100 * err)
        if retest:
            best_score = score if best_score is None else (best_score * best_runs + score) / (best_runs + 1)
            best_runs += 1
            line += " best avg {:.2f}".format(best_score)
        elif score > best_score:
            best, best_score, best_runs = cand, score, 1
            sigma = min(sigma * 1.3, 3.0)
            save(best, total)
            line += " NEW BEST"
        else:
            sigma = max(sigma * 0.9, 0.3)
        log(line)
    stop()
    save(best, total)
    log("Best: " + fmt(best))
    log("Best settings: BASE_POWER={:.0f} KP={:.2f} KD={:.1f} SLOWDOWN={:.2f} MIN_POWER={:.0f}".format(
        best["base"], best["kp"], best["kd"], best["slowdown"], best["min_power"]))


try:
    run()
except Exception as ex:
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
