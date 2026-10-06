# SELF-LEARNING line follower (separate project; the safe versions are not touched)
#
# Same line follower as line_follow_safe.py, but it improves its own settings while it
# drives laps on a closed track:
#   1. At the start it measures white and black (pivots over the line) like the safe version.
#   2. It drives in rounds of ROUND_S seconds without stopping. Every round it tries
#      slightly changed settings (BASE_POWER, KP, KD, SLOWDOWN, MIN_POWER) around the
#      best settings found so far.
#   3. Each round gets a score = speed * (1 - error), multiplied by OFF_PENALTY for every
#      time the sensor ran off the edge onto white for longer than OFF_MS.
#   4. Better score -> the new settings become the best. Every RETEST_EVERY rounds the
#      best settings are driven again and their score averaged, so one lucky round
#      cannot fool it.
#   5. The best settings are saved to SAVE_FILE on the cube; the next start continues
#      from there. Delete the file (Thonny file manager) to start learning from scratch.
#
# Stops when: LEFT is pressed, MAX_ROUNDS are done, or the line is lost for GIVE_UP_S
# (then put the robot back on the edge and start it again; nothing learned is lost).
# Setup: color sensor on S1, left motor M1, right motor M2, robot on the EDGE of the line.
import json
import math
from random import getrandbits
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

SAVE_FILE = "/line_learn.json"
ROUND_S = 15        # length of one round; best at least one lap, so every round sees every curve
SETTLE_S = 1        # start of a round that is not scored (new settings settle in)
MAX_ROUNDS = 40     # stop after this many rounds (40 x 15 s = 10 minutes)
RETEST_EVERY = 4    # drive the best settings again every N rounds
OFF_MS = 200        # sensor on white longer than this = ran off the edge
OFF_PENALTY = 0.8   # score is multiplied by this for every run-off
GIVE_UP_S = 2       # sensor on white this long = line lost, stop
WHEEL_CM = 8.0      # wheel diameter, for the speed in cm/s

# Fixed parts of the line follower (same as line_follow_safe.py)
KI = 0.01
D_FILTER = 0.7
I_LIMIT = 10
LOST_TURN = 40
PERIOD_MS = 10
CAL_POWER = 25
CAL_SWEEP = 150
MIN_CONTRAST = 20

# Learned settings: start values (= line_follow_safe.py), allowed range, step size
START = {"base": 60.0, "kp": 0.8, "kd": 3.0, "slowdown": 1.2, "min_power": -10.0}
LIMITS = {"base": (25.0, 100.0), "kp": (0.2, 3.0), "kd": (0.0, 10.0),
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
    try:
        with open(SAVE_FILE) as f:
            data = json.load(f)
        params = dict(START)
        params.update(data["params"])
        return params, data.get("rounds", 0)
    except Exception:
        return dict(START), 0


def save(params, rounds):
    try:
        with open(SAVE_FILE, "w") as f:
            json.dump({"params": params, "rounds": rounds}, f)
    except Exception as ex:
        log("Save failed: {}".format(ex))


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

    sleep_ms(1000)
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
    log("white={:.0f} black={:.0f} setpoint={:.0f}".format(white, black, setpoint))

    side = 1 if light.reflection() > setpoint else -1
    start = ticks_ms()
    while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
        left.set_power(side * CAL_POWER)
        right.set_power(-side * CAL_POWER)
        sleep_ms(5)
    stop()

    # --- Line follower state (kept between rounds, the robot never stops) ---
    st = {"e_prev": 0, "d": 0, "integral": 0, "last_sign": 1, "last_tick": ticks_ms()}

    def control(p, now):
        # One step of the safe line follower with settings p; returns the error
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
        speed = max(0, p["base"] - p["slowdown"] * abs(e))
        left.set_power(clamp(speed + u, p["min_power"], 100))
        right.set_power(clamp(speed - u, p["min_power"], 100))
        return e

    def drive_round(p):
        # Drive ROUND_S seconds with settings p and score it.
        # Returns (score, err, speed, offs), or "stop" (LEFT pressed) / "lost".
        start = ticks_ms()
        pos0 = None
        t0 = 0
        n = 0
        err_sum = 0
        offs = 0
        white_since = None
        counted = False
        t = 0
        while True:
            now = ticks_ms()
            t = ticks_diff(now, start)
            if t >= ROUND_S * 1000:
                break
            if robot.buttons.pressed()[Button.LEFT]:
                return "stop"
            e = control(p, now)
            if t >= SETTLE_S * 1000:
                if pos0 is None:
                    pos0 = (left.position() + right.position()) / 2
                    t0 = t
                n += 1
                err_sum += min(abs(e) / half, 1)
            # Ran off the edge onto white?
            if e > lost_error:
                if white_since is None:
                    white_since = now
                    counted = False
                w = ticks_diff(now, white_since)
                if w > OFF_MS and not counted and t >= SETTLE_S * 1000:
                    offs += 1
                    counted = True
                if w > GIVE_UP_S * 1000:
                    return "lost"
            else:
                white_since = None
            sleep_ms(PERIOD_MS)
        seconds = (t - t0) / 1000
        dist = (left.position() + right.position()) / 2 - pos0
        speed = dist / 360 * math.pi * WHEEL_CM / max(seconds, 0.01)
        err = err_sum / max(n, 1)
        return speed * (1 - err) * OFF_PENALTY ** offs, err, speed, offs

    # --- Learning ---
    best, total = load()
    log("Learning from " + fmt(best) + " (rounds so far: {})".format(total))
    best_score = None
    best_runs = 0
    sigma = 1.0          # step size: bigger after improvements, smaller after failures
    for rnd in range(1, MAX_ROUNDS + 1):
        retest = best_score is None or rnd % RETEST_EVERY == 0
        cand = best if retest else mutate(best, sigma)
        res = drive_round(cand)
        if res == "stop":
            log("Stopped (LEFT)")
            break
        if res == "lost":
            stop()
            log("#{} {} -> line LOST, stopping. Put the robot on the edge and start again.".format(rnd, fmt(cand)))
            break
        score, err, speed, offs = res
        total += 1
        line = "#{} {} {} -> score={:.1f} err={:.0f}% speed={:.1f}cm/s offs={}".format(
            rnd, "BEST" if retest else "TRY ", fmt(cand), score, 100 * err, speed, offs)
        if retest:
            best_score = score if best_score is None else (best_score * best_runs + score) / (best_runs + 1)
            best_runs += 1
            line += " (best avg {:.1f})".format(best_score)
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
    log("For line_follow_safe.py: BASE_POWER={:.0f} KP={:.2f} KD={:.1f} SLOWDOWN={:.2f} MIN_POWER={:.0f}".format(
        best["base"], best["kp"], best["kd"], best["slowdown"], best["min_power"]))


try:
    run()
except Exception as ex:
    log("Crashed: {} {}".format(type(ex).__name__, ex))
    import sys
    sys.print_exception(ex)
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
