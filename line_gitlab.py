'''
Example program of line follower robot with PID regulator.
Start robot with light sensor on black line
After startup robot measures light intensity of line and white paper
and starts following the line afterwards.
Optional ESP communication in binary mode.
'''
import utime
import struct

from lib.robot_consts import Button, Port, Sensor, Light

# Regulation period 0.01 s
REGUL_PERIOD_US = 10000
REGUL_PERIOD_S = REGUL_PERIOD_US / 1000000

# ESP communication
BINARY_HEADER = (203, 98, 116, 140)
NUM_RECEIVED_VALS = 6
DATA_SIZE = 4

# Light sensor settings
ADC_SAMPLE_TIME_US = 1100
ADC_WAIT_TIME_US = 0
LIGHT_CALIB_MEASUREMENTS = 100
LIGHT_NORM_MAX = 100

# Measure mean of light sensor values
def measure_mean(robot):
    light = 0
    for i in range(LIGHT_CALIB_MEASUREMENTS):
        # Wait until new data is available
        while not robot.sensors.light[Port.S2].new_data(Light.ON):
            pass
        light += robot.sensors.light[Port.S2].get_value(Light.ON)
    light_mean = light/LIGHT_CALIB_MEASUREMENTS
    return light_mean

def calibrate_light(robot):
    # Start with robot's light sensor on line and measure black
    black_mean = measure_mean(robot)

    # Move robot away from line and measure white
    robot.motors[Port.M2].init_regulator()
    robot.motors[Port.M2].regulator_pos_set_consts(0.35, 0.03, 0)
    robot.motors[Port.M2].set_regulator_position(300)
    while abs(robot.motors[Port.M2].regulator_error()) > 20:
        pass
    white_mean = measure_mean(robot)

    # Move robot back to the edge of line
    robot.motors[Port.M2].set_regulator_position(100)
    while abs(robot.motors[Port.M2].regulator_error()) > 20:
        pass
    robot.motors[Port.M2].deinit_regulator()

    # Calculate mean and spread for light normalization
    light_mean = (black_mean + white_mean)/2
    light_spread = abs(black_mean - white_mean)/2
    return light_mean, light_spread

# Normalize light (-100 for black, 100 for white)
def normalize_light(light_mean, light_spread, light):
    light_norm = (light - light_mean)/light_spread*LIGHT_NORM_MAX
    if light > LIGHT_NORM_MAX:
        light = LIGHT_NORM_MAX
    if light < -LIGHT_NORM_MAX:
        light = -LIGHT_NORM_MAX
    return light
        
# Main program
def main():
    global robot
    # Initialize regulator parameters
    Kp, Ki, Kd = 0.3, 0.0, 0.004
    light_error_sum, light_error_prev = 0, 0
    light_setpoint, motor_base_pwr, motor_max_variable_pwr = 0, 45, 30
    
    # Initialize ESP BT communication
    robot.init_esp_uart()
    robot.esp_uart.set_binary(BINARY_HEADER, NUM_RECEIVED_VALS*DATA_SIZE)
    
    # Initialize motors at ports M2 and M3
    robot.init_motor(Port.M2)
    robot.init_motor(Port.M3)

    # Initialize light sensor at port S2
    # LED is always on (no modulation due to long settling time of light sensor)
    robot.init_sensor(sensor_type=Sensor.NXT_LIGHT, port=Port.S2)
    robot.sensors.light[Port.S2].on()
    robot.sensors.light[Port.S2].set_continuous(ADC_SAMPLE_TIME_US, ADC_WAIT_TIME_US)

    # Measure light intensity of black and white starting with robot on line (black)
    light_mean, light_spread = calibrate_light(robot)

    # Follower loop
    while True:
        regul_start_time = utime.ticks_us()
        # Read light sensor and normalize (-100 for black, 100 for white)
        light_meas = robot.sensors.light[Port.S2].get_value(Light.ON)
        light_norm = normalize_light(light_mean, light_spread, light_meas)

        # PID regulator
        light_error = light_setpoint - light_norm
        P = Kp*light_error
        I = Ki*light_error_sum*REGUL_PERIOD_S
        D = Kd*(light_error - light_error_prev)/REGUL_PERIOD_S
        motor_pwr = P + I + D

        # Save previous error for derivative part
        light_error_prev = light_error
        # Anti-windup for integral part
        if ((light_error > 0 and motor_pwr < motor_max_variable_pwr) 
            or (light_error < 0 and motor_pwr > -motor_max_variable_pwr)):
            light_error_sum += Ki*light_error*REGUL_PERIOD_S 

        # Limit motor power to motor_max_variable_pwr
        if motor_pwr > motor_max_variable_pwr:
            motor_pwr = motor_max_variable_pwr
        elif motor_pwr < -motor_max_variable_pwr:
            motor_pwr = -motor_max_variable_pwr

        # Set motor power
        robot.motors[Port.M2].set_power(motor_base_pwr + motor_pwr)
        robot.motors[Port.M3].set_power(motor_base_pwr - motor_pwr)

        # Receive data from ESP and update regulator parameters
        new_data = robot.esp_uart.read()
        if new_data:
            values = struct.unpack('<ffffff', new_data)
            (Kp,Ki,Kd,light_setpoint,motor_base_pwr,motor_max_variable_pwr) = values
        
        # Send data to ESP
        send_data = struct.pack('<ffffff', light_setpoint,light_norm,motor_pwr,P,I,D)
        robot.esp_uart.write(send_data)

        # Exit program if left cube button is pressed
        buttons = robot.buttons.pressed()
        if buttons[Button.LEFT]:
            break

        # Sleep for the rest of the regulation period
        regul_end_time = utime.ticks_us()
        regul_time = utime.ticks_diff(regul_end_time, regul_start_time)
        utime.sleep_us(REGUL_PERIOD_US-regul_time)
        
    # Initialize ESP BT communication, motors and light sensor
    robot.deinit_all()

# Start the program
main()