# IMPORTS #
from machine import Pin, PWM, ADC, I2C
from servo import Servo
from hcsr04 import HCSR04
from imu import MPU6050
from time import sleep, ticks_ms, ticks_diff
import math

# CONSTANTS #
SPEED = 45 # ~ 35 for fresh battery, ~ 45 for worn down battery
STOP_DISTANCE = 12
RIGHT_ANGLE = 85
SENSOR_CHECKS = 3
TURN_CLEAR_TIME = 5 # Time to keep advancing to fully clear the obstacle before turning (hundredths of a second?)
SIDE_OBSTACLE_BUFFER = 8 # cm

# INITIALIZE COMPONENTS #

# Motor A
motor_a_in1 = Pin(6, Pin.OUT)
motor_a_in2 = Pin(7, Pin.OUT)
motor_a_en = PWM(Pin(8))
motor_a_en.freq(1000)
MOTOR_A_CORRECTION = 1.0 # Adjustment so both motors have same speed

# Motor B
motor_b_in3 = Pin(4, Pin.OUT)
motor_b_in4 = Pin(3, Pin.OUT)
motor_b_en = PWM(Pin(2))
motor_b_en.freq(1000)
MOTOR_B_CORRECTION = 1.0 # Adjustment so both motors have same speed

# Ultrasonic sensors
sensor_front = HCSR04(trigger_pin=21, echo_pin=20)
sensor_side = HCSR04(trigger_pin=16, echo_pin=17)

ir_sensor = ADC(28)
# You may need to change the sensitivity of the IR Proximity sensor using the built in potentiometer.
# NOTE: Test the sensor using the built in LED to determine the actvation range.

reed_switch = Pin(0, Pin.IN, Pin.PULL_DOWN)

line_sensor = Pin(10, Pin.IN)

pico_led = Pin("LED", Pin.OUT) # Controls LED on the raspberry pi pico

# Servos
sg901 = Servo(Pin(12))
sg902 = Servo(Pin(13))

# Gyroscope sensor
i2c = I2C(1, scl=Pin(15), sda=Pin(14))  # Adjust pins if needed
imu = MPU6050(i2c)

# Constants
G_TO_MS2 = 9.80665  # 1g to m/s^2
CALIBRATION_SAMPLES = 500

# State variables
yaw = 0.0 

# Calibration offsets
gyro_z_offset = 0.0

print("Calibrating... Please keep the robot still.")

gyro_z_sum = 0.0

for i in range(CALIBRATION_SAMPLES):
    # Flash LED to indicate calibration
    if (i % 20 == 0) or (i == 0):
        pico_led.value(1)
    elif (i % 10 == 0) and (i % 20 != 0):
        pico_led.value(0)
        
    gyro_z_sum += imu.gyro.z
    sleep(0.01)

gyro_z_offset = gyro_z_sum / CALIBRATION_SAMPLES

print("Calibration complete!")

# Time tracking
last_time = ticks_ms()

# FUNCTIONS #

def flash_led(times=3, interval=0.05):
    for i in range(times):
        pico_led.value(1)
        sleep(interval)
        pico_led.value(0)
        sleep(interval)

# Servo control functions
def slow_down(LIMIT):
    for i in range(0, LIMIT, 10):
        sg902.move(LIMIT - (i + 15)) # move to 0 degree postion
        sg901.move(i)
        sleep(0.5)
    
    sleep(3)
def slow_up(LIMIT):
    for i in range(0, LIMIT, 10):
        sg902.move(i) # move to 90 degree position
        sg901.move(LIMIT - (i + 5))
        sleep(0.5)
    sleep(3)

def arm_up(LIMIT):
    sg902.move(LIMIT + 15)
    sg901.move(0)
    sleep(0.5)

def arm_down(LIMIT):
    sg902.move(15)
    sg901.move(LIMIT)
    sleep(0.5)

# Function to control Motor A
def motor_a(direction = "stop", speed = 0):
    adjusted_speed = int(speed * MOTOR_A_CORRECTION)  # Apply correction
    if direction == "forward":
        motor_a_in1.value(0)
        motor_a_in2.value(1)
    elif direction == "backward":
        motor_a_in1.value(1)
        motor_a_in2.value(0)
    else:  # Stop
        motor_a_in1.value(0)
        motor_a_in2.value(0)
    motor_a_en.duty_u16(int(adjusted_speed * 65535 / 100))  # Speed: 0-100%

# Function to control Motor B
def motor_b(direction = "stop", speed = 0):
    adjusted_speed = int(speed * MOTOR_B_CORRECTION)  # Apply correction
    if direction == "forward":
        motor_b_in3.value(1)
        motor_b_in4.value(0)
    elif direction == "backward":
        motor_b_in3.value(0)
        motor_b_in4.value(1)
    else:  # Stop
        motor_b_in3.value(0)
        motor_b_in4.value(0)
    motor_b_en.duty_u16(int(adjusted_speed * 65535 / 100))  # Speed: 0-100%

def advance():
    motor_a("backward", SPEED)
    motor_b("forward", SPEED)

def reverse():
    motor_a("forward", SPEED)
    motor_b("backward", SPEED)
    
def stop():
    motor_a()
    motor_b()

def turn(direction="right", duration=0.5):
    if direction == "right":
        motor_a("forward", SPEED)
        motor_b("forward", SPEED)
    else:
        motor_a("backward", SPEED)
        motor_b("backward", SPEED)
    sleep(duration)
    stop()
    
def gyro_advance(yaw, target_yaw, reverse=False):
    CORRECTION_SENSITIVITY = 0.25 # Adjust this value to change the sensitivity of the correction
    REVERSE_SPEED = SPEED * 1.1 # Reverses slower without correction due to friction
    motor_speed = SPEED
    
    if reverse:
        motor_speed = REVERSE_SPEED

    yaw_error = target_yaw - yaw
    correction = yaw_error * CORRECTION_SENSITIVITY # Calculate speed correction based on yaw error

    right_speed = min(100, max(35, motor_speed + correction))
    left_speed = min(100, max(35, motor_speed - correction))
    
    # Apply correction to motor speeds
    if reverse:
        motor_a("forward", left_speed) # Motor a becomes the right side when reversing
        motor_b("backward", right_speed) # Motor b becomes the left side when reversing
    else:
        motor_a("backward", right_speed)
        motor_b("forward", left_speed)
        
def gyro_turn(target_yaw=-90):
    global last_time, yaw, gyro_z_offset

    TOLERANCE = 1.5
    TURN_SPEED_ADJUSTMENT = 1.15 # Need more power to turn well

    while True:
        try:
            # Get yaw value
            current_time = ticks_ms()
            dt = ticks_diff(current_time, last_time) / 1000.0  # seconds
            last_time = current_time

            # Safely read gyroscope data
            try:
                gyro_z = imu.gyro.z - gyro_z_offset
                # Still integrate all values, even large ones
                yaw += gyro_z * dt
            except Exception as e:
                print(f"Gyro read error: {e}")
                # Don't modify yaw if we can't read the sensor
                # Just continue with previous value
            
            sleep(0.01) # Short delay to avoid flooding the console

            if yaw < target_yaw - TOLERANCE:
                motor_a("backward", SPEED * TURN_SPEED_ADJUSTMENT)
                motor_b("backward", SPEED * TURN_SPEED_ADJUSTMENT)
            elif yaw > target_yaw + TOLERANCE:
                motor_a("forward", SPEED * TURN_SPEED_ADJUSTMENT)
                motor_b("forward", SPEED * TURN_SPEED_ADJUSTMENT)
            else:
                stop()
                return
                    
        except Exception as e:
            # Catch any exceptions in the main loop
            print(f"Loop error: {e}")

def payload_bonus():
    global last_time, yaw, gyro_z_offset
    
    i = 0

    arm_up(60)
    while True:
        try:
            # Get yaw value
            current_time = ticks_ms()
            dt = ticks_diff(current_time, last_time) / 1000.0  # seconds
            last_time = current_time

            # Safely read gyroscope data
            try:
                gyro_z = imu.gyro.z - gyro_z_offset
                # Still integrate all values, even large ones
                yaw += gyro_z * dt
            except Exception as e:
                print(f"Gyro read error: {e}")
                # Don't modify yaw if we can't read the sensor
                # Just continue with previous value
            
            
            sleep(0.01) # Short delay to avoid flooding the console

            if i < 250:
                gyro_advance(yaw, 0, reverse=True)
                i +=1
            elif i < 500:
                gyro_advance(yaw, 0)
                i +=1
            else:
                stop()
                sleep(2)
                break
     
        except Exception as e:
            # Catch any exceptions in the main loop
            print(f"Loop error: {e}")
    arm_down(60)
    reverse()
    sleep(1)
    stop()

def complete_course(obstacle_to_pickup=True, obstacle_bonus=False):
    global last_time, yaw, gyro_z_offset
    mode = "line search"
    prev_mode = "line search"
    i = 1
    obstacle_flag = 0
    angle = 0
    obstacle_cleared = not obstacle_to_pickup
    line_search_angle = 0

    if obstacle_bonus:
        mode = "go straight"
        line_search_angle = RIGHT_ANGLE

    while True:
        try:
            # Get distance from sensors with error handling
            try:
                distance_front = sensor_front.distance_cm()
            except Exception as e:
                print(f"Front sensor error: {e}")
                distance_front = 100  # Safe default
                
            try:
                distance_side = sensor_side.distance_cm()
            except Exception as e:
                print(f"Side sensor error: {e}")
                distance_side = 100  # Safe default


            over_line = line_sensor.value() == 0
            obstacle_in_front = distance_front < STOP_DISTANCE and distance_front > 0
            obstacle_on_side = distance_side < STOP_DISTANCE + SIDE_OBSTACLE_BUFFER and distance_side > 0
            payload_detected = reed_switch.value() == 1
            in_dropoff_zone = ir_sensor.read_u16() > 7000

            # Get yaw value
            current_time = ticks_ms()
            dt = ticks_diff(current_time, last_time) / 1000.0  # seconds
            last_time = current_time

            # Safely read gyroscope data
            try:
                gyro_z = imu.gyro.z - gyro_z_offset
                # Still integrate all values, even large ones
                yaw += gyro_z * dt
            except Exception as e:
                print(f"Gyro read error: {e}")
                # Don't modify yaw if we can't read the sensor
                # Just continue with previous value

            # Print key information
            print(f"Distance front: {distance_front:.2f} cm, Distance side: {distance_side:.2f} cm, Black line: {over_line}, Yaw: {yaw:.2f}, Payload: {payload_detected}, Dropoff zone: {in_dropoff_zone}")
            
            if obstacle_in_front:
                print("OBSTACLE!!!")
            
            sleep(0.01) # Short delay to avoid flooding the console

            if payload_detected:
                stop()
                flash_led()
                mode = "pickup"

            if in_dropoff_zone:
                stop()
                flash_led()
                mode = "dropoff"
            
            if mode == "line search":
                gyro_advance(yaw, angle)
                if over_line:
                    stop()
                    flash_led()
                    angle += -RIGHT_ANGLE
                    gyro_turn(angle)
                    if obstacle_cleared:
                        mode = "detect payload"
                    else:
                        mode = "go straight"
                elif obstacle_in_front:
                    obstacle_flag += 1
                    stop()
                    flash_led()
                    prev_mode = mode
                    mode = "confirm front obstacle"
            elif mode == "go straight":
                gyro_advance(yaw, angle)
                
                if obstacle_in_front:
                    obstacle_flag += 1
                    stop()
                    flash_led()
                    prev_mode = mode
                    mode = "confirm front obstacle"
            elif mode == "confirm front obstacle":
                if obstacle_in_front:
                    obstacle_flag += 1
                else:
                    mode = prev_mode

                if obstacle_flag >= SENSOR_CHECKS:
                    obstacle_flag = 0
                    stop()
                    angle += -RIGHT_ANGLE
                    gyro_turn(angle)
                    flash_led()
                    mode = "obstacle"
            elif mode == "obstacle":
                gyro_advance(yaw, angle)
                
                # Check if the obstacle is beside the robot
                if obstacle_on_side:
                    obstacle_flag += 1

                if obstacle_in_front:
                    obstacle_flag += 1
                    stop()
                    flash_led()
                    prev_mode = mode
                    mode = "confirm front obstacle"

                # If there's no obstacle on the side (and it actually passed the obstacle)
                if not obstacle_on_side and obstacle_flag >= 3:
                    obstacle_flag = 1
                    stop()
                    flash_led()
                    mode = "confirm no side obstacle"
            elif  mode == "confirm no side obstacle":
                if not obstacle_on_side:
                    obstacle_flag += 1
                else:
                    obstacle_flag = 3 # Since it was already 3, we need to reset it to 3 because it cleared the side of the obstacle
                    mode = "obstacle"
                    
                if obstacle_flag >= SENSOR_CHECKS:
                    obstacle_flag = 0 # Reset the flag
                    clear_time = TURN_CLEAR_TIME

                    if angle == -RIGHT_ANGLE:
                        clear_time = TURN_CLEAR_TIME * 2 # If the robot is turning its back to the obstacle, it needs more time to clear it
                        
                    if i < clear_time:
                        gyro_advance(yaw, angle)
                        i += 1
                    else:
                        i = 0
                        stop()
                        angle += RIGHT_ANGLE
                        gyro_turn(angle)
                        flash_led()
                        if angle == line_search_angle: # If the angle is such that it's headed for the line
                            mode = "line search"
                            obstacle_cleared = True # Set flag to indicate the obstacle was cleared and we can try to detect the payload
                        else:
                            mode = "obstacle"
            elif mode == "detect payload":
                gyro_advance(yaw, angle)
                
                if payload_detected:
                    stop()
                    flash_led()
                    mode = "pickup"
            elif mode == "pickup":
                # Pick up the payload
                if i < 50:
                    gyro_advance(yaw, angle, reverse=True)
                elif i == 51:
                    stop()
                    arm_down(60)
                elif i > 100 and i < 170:
                    gyro_advance(yaw, angle)
                elif i == 171:
                    stop()
                    arm_up(60)
                    angle += (2 * RIGHT_ANGLE) # Turn 180 degrees
                    gyro_turn(angle)
                    obstacle_cleared = False
                    mode = "go straight" # Might not work if payload blocks the front sensor
                    i = -1 # Resets i to 0

                i += 1
            elif mode == "dropoff":
                arm_down(60)

                if i < 50:
                    gyro_advance(yaw, angle, reverse=True)
                else:
                    stop()
                    break
                
                i += 1
      
        except Exception as e:
            # Catch any exceptions in the main loop
            print(f"Loop error: {e}")
        
    # Celebrate completion!!!
    flash_led(20)

# MAIN PROGRAM #
# Uncomment the function you want to use

complete_course() # Args: obstacle_to_pickup=True, obstacle_bonus=False
#payload_bonus()