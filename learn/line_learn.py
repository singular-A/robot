# SELF-LEARNING line follower for a track with a START and a FINISH (not a loop)
# (separate project; the safe versions are not touched)
#
# Every round is one run from the start to the finish with slightly different settings:
#   1. Put the robot on the EDGE of the line at the start, press OK (or RIGHT).
#      In the very first round it also measures white and black (pivots over the line).
#   2. It follows the line with this round's settings (BASE_POWER, KP, KD, SLOWDOWN,
#      MIN_POWER). Press OK (or RIGHT) at the finish: it stops and prints the score.
#   3. Goal = FASTEST run: score = 100 / finish time in seconds (higher = faster).
#      Running off the line for a moment is fine as long as it finishes. If it loses the
#      line (white for GIVE_UP_S) or runs out of time, the run ends by itself: score 0.
#      err and offs (run-offs) are only printed for information.
#   4. Better score -> the new settings become the best. Every RETEST_EVERY rounds the best
#      settings are driven again and their score averaged, so one lucky run cannot fool it.
#   5. The best settings are saved to SAVE_FILE on the cube; the next start continues from
#      there. Delete the file (Thonny file manager) to start learning from scratch.
# LEFT while DRIVING = this run failed (e.g. it went off the track): the robot stops, the
#   run counts as score 0 (so it learns those settings are bad) and the next round follows.
# LEFT while WAITING at the start = quit the program; nothing learned is lost.
# Do not stop it from Thonny in the middle of a run: that run would not be learned from.
# Setup: color sensor on S1, left motor M1, right motor M2.
import json
import math
from random import getrandbits
from time import sleep_ms, ticks_ms, ticks_diff
from lib.robot_consts import Sensor, Port, Button

SAVE_FILE = "/line_learn_fast.json"    # learning for the fastest time (older files are not used)
MAX_ROUNDS = 30     # stop after this many rounds
MIN_RUN_S = 2       # OK presses in the first seconds of a run are ignored (no accidental finish)
RETEST_EVERY = 4    # drive the best settings again every N rounds
SETTLE_S = 0.5      # first part of a run that is not scored (starting from standstill)
MAX_ROUND_S = 120   # safety: a run ends by itself after this many seconds
OFF_MS = 200        # sensor on white longer than this = counted as a run-off (info only)
GIVE_UP_S = 2       # sensor on white this long = line lost, the run ends (score 0)
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

# Learned settings: start values (= the fastest run so far, 24.3 s), allowed range, step size
START = {"base": 75.0, "kp": 1.19, "kd": 7.2, "slowdown": 0.70, "min_power": -31.0}
LIMITS = {"base": (25.0, 100.0), "kp": (0.2, 3.0), "kd": (4.0, 12.0),
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

    # --- Measure white and black (same as line_follow_safe.py), done once ---
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

    cal = {}

    def calibrate():
        home = left.position() - right.position()
        pivot_to(home + CAL_SWEEP)
        pivot_to(home - CAL_SWEEP)
        pivot_to(home)
        black, white = seen
        if white - black < MIN_CONTRAST:
            log("white={:.0f} black={:.0f}: no line found - start on the edge of the line".format(white, black))
            return False
        cal["setpoint"] = (white + black) / 2
        cal["half"] = white - cal["setpoint"]
        cal["lost_error"] = 0.8 * cal["half"]
        log("white={:.0f} black={:.0f} setpoint={:.0f}".format(white, black, cal["setpoint"]))
        return True

    def align():
        # Pivot slowly until the sensor is exactly on the edge
        setpoint = cal["setpoint"]
        side = 1 if light.reflection() > setpoint else -1
        start = ticks_ms()
        while (light.reflection() - setpoint) * side > 0 and ticks_diff(ticks_ms(), start) < 3000:
            left.set_power(side * CAL_POWER)
            right.set_power(-side * CAL_POWER)
            sleep_ms(5)
        stop()

    def drive_round(p):
        # One run from start to finish with settings p (the safe line follower).
        # Returns (end, score, err, speed, offs, seconds); end is "finish", "fail" (LEFT), "lost" or "time".
        setpoint, half, lost_error = cal["setpoint"], cal["half"], cal["lost_error"]
        e_prev = 0
        d = 0
        integral = 0
        last_sign = 1
        start = ticks_ms()
        last_tick = start
        pos0 = None
        t0 = 0
        n = 0
        err_sum = 0
        offs = 0
        white_since = None
        counted = False
        end = "time"
        t = 0
        while True:
            now = ticks_ms()
            t = ticks_diff(now, start)
            if t >= MAX_ROUND_S * 1000:
                break
            ok, failed = buttons()
            if failed:
                end = "fail"       # LEFT while driving: this run failed (score 0)
                break
            if ok and t >= MIN_RUN_S * 1000:
                end = "finish"
                break

            # The safe line follower with this round's settings
            dt = max(ticks_diff(now, last_tick), 1)
            last_tick = now
            e = light.reflection() - setpoint
            d = D_FILTER * d + (1 - D_FILTER) * (e - e_prev) * 10 / dt
            e_prev = e
            if (e > 0) != (integral > 0):
                integral = 0
            integral = clamp(integral + e * dt / 10, -I_LIMIT / KI, I_LIMIT / KI)
            if e > lost_error:
                u = last_sign * LOST_TURN
            else:
                u = p["kp"] * e + KI * integral + p["kd"] * d
                if abs(u) > 2:
                    last_sign = 1 if u > 0 else -1
            speed = max(0, p["base"] - p["slowdown"] * abs(e))
            left.set_power(clamp(speed + u, p["min_power"], 100))
            right.set_power(clamp(speed - u, p["min_power"], 100))

            # Measure how well it follows
            if t >= SETTLE_S * 1000:
                if pos0 is None:
                    pos0 = (left.position() + right.position()) / 2
                    t0 = t
                n += 1
                err_sum += min(abs(e) / half, 1)
            if e > lost_error:
                if white_since is None:
                    white_since = now
                    counted = False
                w = ticks_diff(now, white_since)
                if w > OFF_MS and not counted:
                    offs += 1
                    counted = True
                if w > GIVE_UP_S * 1000:
                    end = "lost"
                    break
            else:
                white_since = None
            sleep_ms(PERIOD_MS)
        stop()
        if pos0 is None:
            return end, 0, 1, 0, offs, t / 1000
        seconds = (t - t0) / 1000
        dist = (left.position() + right.position()) / 2 - pos0
        speed = dist / 360 * math.pi * WHEEL_CM / max(seconds, 0.01)
        err = err_sum / max(n, 1)
        # Fastest wins: only a finished run counts
        score = 100 / (t / 1000) if end == "finish" else 0
        return end, score, err, speed, offs, t / 1000

    # --- Learning ---
    best, total = load()
    log("Learning from " + fmt(best) + " (rounds so far: {})".format(total))
    best_score = None
    best_runs = 0
    sigma = 1.0          # step size: bigger after improvements, smaller after failures
    for rnd in range(1, MAX_ROUNDS + 1):
        retest = best_score is None or rnd % RETEST_EVERY == 0
        cand = best if retest else mutate(best, sigma)
        log("Round #{} ({}): {}. Robot on the edge at the START, press OK. While driving: OK = finish, LEFT = failed run. Now LEFT = quit".format(
            rnd, "best" if retest else "try", fmt(cand)))
        if not wait_for_ok():
            log("Quit")
            break
        sleep_ms(300)
        if not cal:
            if not calibrate():
                break
        align()
        end, score, err, speed, offs, seconds = drive_round(cand)
        total += 1
        line = "#{} {} {} -> {} time={:.1f}s score={:.2f} (err={:.0f}% speed={:.1f}cm/s offs={})".format(
            rnd, "BEST" if retest else "TRY ", fmt(cand), end.upper(), seconds, score, 100 * err, speed, offs)
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
