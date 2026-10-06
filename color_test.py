from lib.robot_consts import Sensor, Port

robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)

reflection = robot.sensors.light[Port.S1].reflection()
buffer = "senzor"+str(reflection)+";"
robot.esp.bt_write(buffer)