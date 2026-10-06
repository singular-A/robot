# Sensor speed test (motors are not used): how often does the color sensor's value
# actually change? Reads as fast as possible for 2 seconds, both reflection()
# (0-100 %, capped) and reflection_raw() (raw LED-on / LED-off values, not capped).
from time import ticks_us, ticks_diff, sleep_ms
from lib.robot_consts import Sensor, Port

robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
light = robot.sensors.light[Port.S1]
sleep_ms(500)


def measure(read, name):
    reads = 0
    changes = []            # time between value changes, microseconds
    call_us = 0
    last = read()
    first = last
    last_change = ticks_us()
    start = ticks_us()
    while ticks_diff(ticks_us(), start) < 2000000:
        t0 = ticks_us()
        r = read()
        call_us += ticks_diff(ticks_us(), t0)
        reads += 1
        if r != last:
            now = ticks_us()
            changes.append(ticks_diff(now, last_change))
            last_change = now
            last = r
    msg = "{}: first={} last={} reads={} one read {:.2f} ms, changed {} times".format(
        name, first, last, reads, call_us / max(reads, 1) / 1000, len(changes))
    if changes:
        changes.sort()
        msg += ", new value every {:.1f} ms avg (min {:.1f}, median {:.1f}, max {:.1f})".format(
            sum(changes) / len(changes) / 1000, changes[0] / 1000,
            changes[len(changes) // 2] / 1000, changes[-1] / 1000)
    robot.esp.bt_write(msg + ";")
    print(msg)


measure(light.reflection_raw_green, "green_raw")
measure(light.reflection_raw, "reflection_raw")
