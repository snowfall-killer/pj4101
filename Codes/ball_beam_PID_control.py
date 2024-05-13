# pip install pyzmq cbor keyboard
# zmqRemoteApi_IPv6 為將 zmq 通訊協定修改為 IPv4 與 IPv6 相容
from zmqRemoteApi_IPv6 import RemoteAPIClient
import keyboard
import time

# Import numpy for Ziegler-Nichols tuning
import numpy as np

#time.sleep(15)

# Initialize PID control constants
kp = 0.0
ki = 0.0
kd = 0.0

# Function to calculate PID control signal
def calculatePID(target_distance, dt):
    global error_sum, last_error, kp, ki, kd

    # Get the current distance
    distance = getDistance() + 0.05

    if distance is not None:
        # Calculate the error term
        error = target_distance - distance

        # Update the error sum
        error_sum += error

        # Calculate the derivative term
        derivative = (error - last_error) / dt

        # Calculate the control signal using PID parameters
        control_signal = kp * error + ki * error_sum + kd * derivative

        # Update the last error
        last_error = error

        return control_signal
    else:
        return 0.0
        
# Function to perform Ziegler-Nichols tuning
def ZieglerNicholsTuning():
    global kp, ki, kd

    # Initialize gains
    kp = 0.0
    ki = 0.0
    kd = 0.0

    # Set a small increment value for tuning
    delta = 0.001

    # Step response parameters
    time_step = 0.05  # Time step for simulation
    max_time = 100.0    # Maximum simulation time
    target_distance = 0.9  # Desired distance

    # Initial control signal
    control_signal = 0.0

    # Perform Ziegler-Nichols tuning for proportional gain (Kp)
    while True:
        control_signal = calculatePID(target_distance, time_step)
        if abs(control_signal) >= 1.0:
            break
        kp += delta

    # Calculate critical gain (Kc)
    Kc = kp

    # Set proportional gain (Kp) to 0.6 * Kc for Ziegler-Nichols ultimate gain
    kp = 0.6 * Kc

    # Perform Ziegler-Nichols tuning for integral gain (Ki)
    while True:
        control_signal = calculatePID(target_distance, time_step)
        if abs(control_signal) >= 1.0:
            break
        ki += delta

    # Set integral gain (Ki) to 2 * Kp / Ti for Ziegler-Nichols ultimate period
    Ti = 3.0 * time_step
    ki = 2 * kp / Ti

    # Perform Ziegler-Nichols tuning for derivative gain (Kd)
    while True:
        control_signal = calculatePID(target_distance, time_step)
        if abs(control_signal) >= 1.0:
            break
        kd += delta

    # Set derivative gain (Kd) to Kp * Td / 3 for Ziegler-Nichols ultimate period
    Td = time_step
    kd = kp * Td / 3.0
   
#time.sleep(15)

# Initialize error and integral term for PID control
error_sum = 0.0
last_error = 0.0

#time.sleep(15)

# 利用 zmqRemoteAPI 以 23000 對場景伺服器進行連線
client = RemoteAPIClient('localhost', 23000)

sim = client.getObject('sim')

# Get the handle of the proximity sensor and the ball
laser_handle = sim.getObject('/laser')
# Set the ball object in the property common tab to be detectable
ball_handle = sim.getObject('/ball')
motor_handle = sim.getObject('/motor')
# motor rotation angle plus to make ball move to the right
# motor rotation angle minus to make ball move to the left

sim.startSimulation()
print('Simulation started')

# Function to get the distance between the laser and the ball
def getDistance():
    result, distance, _, _, _ = sim.readProximitySensor(laser_handle, ball_handle)
    if result == 1:
        return round(distance, 4) + 0.05  # Ensure the distance is not None and add a small offset
    else:
        return 0.0  # Return a default value if distance is not available

# Function to control motor using PID
def controlMotor(target_distance, dt):
    # Call Ziegler-Nichols tuning to optimize PID parameters
    ZieglerNicholsTuning()

    while True:
        if keyboard.is_pressed('q'):
            # Stop simulation
            sim.stopSimulation()
            break

        # Control the motor to maintain the desired distance
        control_signal = calculatePID(target_distance, dt)

        # Set the motor rotation angle
        sim.setJointTargetPosition(motor_handle, control_signal)

        # Wait for the specified time step
        time.sleep(dt)

# Control the motor
controlMotor(0.9, 0.05)  # Adjust the time step (dt) as needed

# 終止模擬
# sim.stopSimulation()
