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
kp = 1
ki = 0.1
kd = 0.5
dst = 0.9
# 初始化誤差和積分項
error_sum = 0.0
last_error = 0.0

# 函數定義
def getDistance():
    result, distance, _, _, _ = sim.readProximitySensor(laser_handle)
    return round(distance, 4) if result else None

def controlMotor(target_distance, dt):
    global error_sum, last_error
    distance = getDistance()
    if distance is not None:
        error = target_distance - distance
        error_sum += error * dt
        derivative = (error - last_error) / dt
        control_signal = kp * error + ki * error_sum + kd * derivative
        sim.setJointTargetPosition(motor_handle, control_signal)
        last_error = error

def run_simulation():
    global error_sum, last_error
    error_sum = 0.0
    last_error = 0.0
    total_error = 0.0
    simulation_time = 2.0
    dt = 0.05

    sim.startSimulation()
    start_time = time.time()
    while time.time() - start_time < simulation_time:
        controlMotor(dst, dt)
        distance = getDistance()
        if distance is not None:
            total_error += abs(dst - distance)
        time.sleep(dt)
    sim.stopSimulation()
    time.sleep(0.5)  # 確保模擬完全停止
    return total_error / (simulation_time / dt)

def tune_pid():
    global kp, ki, kd
    kp_min, kp_max = 1, 2.0
    ki_min, ki_max = 0.0, 0.1
    kd_min, kd_max = 0.0, 1.0
    step_size = 0.1
    best_error = float('inf')
    best_gains = (kp, ki, kd)

    for new_kd in np.arange(kd_min, kd_max, step_size):
        for new_ki in np.arange(ki_min, ki_max, step_size):
            for new_kp in np.arange(kp_min, kp_max, step_size):
                kp, ki, kd = new_kp, new_ki, new_kd
                error = run_simulation()
                if error < best_error:
                    best_error = error
                    best_gains = (new_kp, new_ki, new_kd)
                print(f"KP: {new_kp}, KI: {new_ki}, KD: {new_kd}, Error: {error}")
    kp, ki, kd = best_gains
    print(f"最佳PID參數為 KP: {kp}, KI: {ki}, KD: {kd}")

def is_balanced():
    distance = getDistance()
    return distance is not None and dst <= distance <= dst

# 優化 PID 參數
tune_pid()

# 主迴圈
sim.startSimulation()
while True:
    controlMotor(0.9, dt)
    if is_balanced():
        print(f"最終的 PID 參數值為：\nKP = {kp}\nKI = {ki}\nKD = {kd}")
        sim.stopSimulation()
        break
