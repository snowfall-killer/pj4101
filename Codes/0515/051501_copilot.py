# pip install pyzmq cbor keyboard
from zmqRemoteApi_IPv6 import RemoteAPIClient
import keyboard
import time
import random

#time.sleep(15)

# 利用 zmqRemoteAPI 以 23000 對場景伺服器進行連線
client = RemoteAPIClient('localhost', 23000)

print('程序開始')
sim = client.getObject('sim')

# 獲取激光和球的控制權
laser_handle = sim.getObject('/laser')
ball_handle = sim.getObject('/ball')
motor_handle = sim.getObject('/motor')

# PID 控制常數
kp = 1  # 比例增益
ki = 0.1  # 積分增益
kd = 0.5  # 微分增益

# 初始化誤差和積分項
error_sum = 0.0
last_error = 0.0

sim.startSimulation()
print('模擬開始')

# 優化 PID 參數
tune_pid()

# 函數用於獲取激光和球之間的距離
def getDistance():
    result, distance, _, _, _ = sim.readProximitySensor(laser_handle, ball_handle)
    if result == 1:
        return round(distance, 4)
    else:
        return None

def controlMotor(target_distance, dt):
    global error_sum, last_error

    # 獲取當前距離
    distance = getDistance()
    print(target_distance, distance)

    if distance is not None:
        # 計算誤差項
        error = target_distance - distance
        print(error)
        print(kp,ki,kd)
        # 更新誤差總和
        error_sum += error

        # 計算微分項
        derivative = (error - last_error) / dt

        # 計算控制信號
        control_signal = kp * error + ki * error_sum + kd * derivative

        # 設置馬達旋轉角度
        sim.setJointTargetPosition(motor_handle, control_signal)

        # 更新最後的誤差
        last_error = error

# 隨機搜索調整方法
def tune_pid():
    global kp, ki, kd

    # 定義搜索的範圍
    kp_min, kp_max = 0.0, 2.0
    ki_min, ki_max = 0.0, 2.0
    kd_min, kd_max = 0.0, 2.0

    # 定義步階大小
    step_size = 0.01

    best_error = float('inf')

    # 進行網格搜索
    for kp in np.arange(kp_min, kp_max, step_size):
        for ki in np.arange(ki_min, ki_max, step_size):
            for kd in np.arange(kd_min, kd_max, step_size):
                # 使用新的增益運行模擬
                error = run_simulation()

                # 如果新的增益導致較低的誤差，則更新最佳增益
                if error < best_error:
                    best_error = error
                    best_gains = (kp, ki, kd)

    # 將增益設置為找到的最佳增益
    kp, ki, kd = best_gains


# 函數用於運行模擬並計算誤差

def run_simulation():
    # 初始化誤差
    error = 0.0

    # 運行模擬的時間
    simulation_time = 10.0  # 你可以根據需要調整這個值

    # 模擬的時間步長
    dt = 0.05  # 你可以根據需要調整這個值

    # 開始模擬
    sim.startSimulation()

    # 運行模擬
    for _ in range(int(simulation_time / dt)):
        # 控制馬達以維持所需的距離
        controlMotor(0.9, dt)

        # 獲取當前距離
        distance = getDistance()

        # 如果距離為 None，則跳過這一步
        if distance is None:
            continue

        # 計算誤差
        error += abs(0.9 - distance)

        # 等待下一個時間步長
        time.sleep(dt)

    # 停止模擬
    sim.stopSimulation()

    # 返回平均誤差
    return error / (simulation_time / dt)

# 函數用於檢查系統是否平衡
def is_balanced():
    # 獲取當前距離
    distance = getDistance()
    # 檢查距離是否在 0.89 到 0.91 的範圍內
    return 0.8999 <= distance <= 0.9001

while True:
    # 控制馬達以維持所需的距離
    controlMotor(0.9, 0.05)  # 根據需要調整時間步長 (dt)

    # 檢查系統是否平衡
    if is_balanced():
        # 停止模擬
        print(f"最終的 PID 參數值為：\nKP = {kp}\nKI = {ki}\nKD = {kd}")
        sim.stopSimulation()
        break  # 現在這個 break 是在 while 迴圈中

# 終止模擬
#sim.stopSimulation()
