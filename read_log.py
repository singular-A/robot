# Prints the log file saved by wall_follow.py (wall_log.csv on the cube) to the Thonny
# Open-Cube log panel, line by line. Run it after a run, then copy the whole panel.
from time import sleep_ms

LOG_FILE = "wall_log.csv"


def send(message):
    robot.esp.bt_write(message + ";")
    print(message)


try:
    count = 0
    with open(LOG_FILE) as f:
        for line in f:
            send(line.rstrip())
            count += 1
            sleep_ms(8)          # do not flood the Bluetooth link
    send("END {} lines".format(count))
except OSError:
    send("No {} on the cube - run wall_follow.py first".format(LOG_FILE))
