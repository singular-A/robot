# Sensor channel test (motors are not used): which LED colour saturates on white?
# Prints the raw readings of the red, green and blue LEDs (value with LED on, LED off)
# and the calibrated rgb(), 5 times, 0.2 s apart. 65535 = saturated (maximum).
from time import sleep_ms
from lib.robot_consts import Sensor, Port

robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
light = robot.sensors.light[Port.S1]
sleep_ms(500)
for i in range(5):
    msg = "red={} green={} blue={} rgb_raw={} reflection={}".format(
        light.reflection_raw(), light.reflection_raw_green(), light.reflection_raw_blue(),
        light.rgb_raw(), light.reflection())
    robot.esp.bt_write(msg + ";")
    print(msg)
    sleep_ms(200)
