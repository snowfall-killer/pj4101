# 導入所需的庫
import numpy as np
import random
import time
import keyboard
from zmqRemoteApi_IPv6 import RemoteAPIClient

# 初始化 zmqRemoteAPI 連接
client = RemoteAPIClient('localhost', 23000)

# 獲取模擬物件的控制權
sim = client.getObject('sim')
laser_handle = sim.getObject('/laser')
ball_handle = sim.getObject('/ball')
motor_handle = sim.getObject('/motor')

# 初始化 PID 控制參數
kp = 1.0
ki = 0.1
kd = 0.5

# 初始化誤差和積分項
error_sum = 0.0
last_error = 0.0

# 開始模擬
sim.startSimulation()
print('模擬開始')

# 函數定義
def getDistance():
    result, distance, _, _, _ = sim.readProximitySensor(laser_handle, ball_handle)
    return round(distance, 4) if result == 1 else None

def controlMotor(target_distance, dt):
    global error_sum, last_error
    distance = getDistance()
    if distance is not None:
        error = target_distance - distance
        error_sum += error
        derivative = (error - last_error) / dt
        control_signal = kp * error + ki * error_sum + kd * derivative
        sim.setJointTargetPosition(motor_handle, control_signal)
        last_error = error

def run_simulation():
    error = 0.0
    simulation_time = 10.0
    dt = 0.05
    sim.startSimulation()
    for _ in range(int(simulation_time / dt)):
        controlMotor(0.9, dt)
        distance = getDistance()
        if distance is not None:
            error += abs(0.9 - distance)
        time.sleep(dt)
    sim.stopSimulation()
    return error / (simulation_time / dt)

def tune_pid():
    global kp, ki, kd
    kp_min, kp_max = 0.0, 2.0
    ki_min, ki_max = 0.0, 2.0
    kd_min, kd_max = 0.0, 2.0
    step_size = 0.01
    best_error = float('inf')
    for kp in np.arange(kp_min, kp_max, step_size):
        for ki in np.arange(ki_min, ki_max, step_size):
            for kd in np.arange(kd_min, kd_max, step_size):
                error = run_simulation()
                if error < best_error:
                    best_error = error
                    best_gains = (kp, ki, kd)
    kp, ki, kd = best_gains

def is_balanced():
    distance = getDistance()
    return 0.8999 <= distance <= 0.9001

# 優化 PID 參數
tune_pid()

# 主迴圈
while True:
    controlMotor(0.9, 0.05)
    if is_balanced():
        print(f"最終的 PID 參數值為：\nKP = {kp}\nKI = {ki}\nKD = {kd}")
        sim.stopSimulation()
        break
