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
kp = 1.0  # 比例增益
ki = 0.1  # 積分增益
kd = 0.5  # 微分增益

# 初始化誤差和積分項
error_sum = 0.0
last_error = 0.0

# 隨機搜索調整方法
def tune_pid():
    global kp, ki, kd

    # 定義搜索的範圍
    kp_min, kp_max = 0.0, 2.0
    ki_min, ki_max = 0.0, 2.0
    kd_min, kd_max = 0.0, 2.0

    best_error = float('inf')

    # 進行隨機搜索
    for _ in range(1000):  # 根據需要調整迭代次數
        # 在範圍內生成隨機增益
        kp = random.uniform(kp_min, kp_max)
        ki = random.uniform(ki_min, ki_max)
        kd = random.uniform(kd_min, kd_max)

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
    global error_sum, last_error

    # 初始化誤差和積分項
    error_sum = 0.0
    last_error = 0.0

    # 運行模擬的時間（例如：10 秒）
    simulation_time = 10

    # 模擬的時間步長（例如：0.05 秒）
    dt = 0.05

    # 目標距離（例如：0.9）
    target_distance = 0.9

    # 初始化誤差平方和
    error_squared_sum = 0.0

    # 運行模擬
    for _ in range(int(simulation_time / dt)):
        # 控制馬達以維持所需的距離
        controlMotor(target_distance, dt)

        # 獲取當前距離
        distance = getDistance()

        if distance is not None:
            # 計算誤差
            error = target_distance - distance

            # 更新誤差總和
            error_sum += error

            # 計算微分項
            derivative = (error - last_error) / dt

            # 計算控制信號
            control_signal = kp * error + ki * error_sum + kd * derivative

            # 更新最後的誤差
            last_error = error

            # 累加誤差平方
            error_squared_sum += error ** 2

    # 返回平均平方誤差
    return error_squared_sum / (simulation_time / dt)


# 函數用於檢查系統是否平衡
def is_balanced():
    # 獲取當前距離
    distance = getDistance()
    # 檢查距離是否在 0.89 到 0.91 的範圍內
    return 0.8999 <= distance <= 0.9001

# 在開始模擬之前，調用 tune_pid 函數來優化 PID 參數
tune_pid()

sim.startSimulation()
print('模擬開始')

# 函數用於獲取激光和球之間的距離
def getDistance():
    result, distance, _, _, _ = sim.readProximitySensor(laser_handle, ball_handle)
    if result == 1:
        return round(distance, 4)
    else:
        return float('inf')  # 當無法獲取距離時，返回一個大數

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

while True:
    # 控制馬達以維持所需的距離
    controlMotor(0.9, 0.05)  # 根據需要調整時間步長 (dt)

    # 檢查系統是否平衡
    if is_balanced():
        # 停止模擬
        print(f"最終的 PID 參數值為：\nKP = {kp}\nKI = {ki}\nKD = {kd}")
        sim.stopSimulation()
        break  # 現在這個 break 是在 while 迴圈中

    # 如果按下 'q' 鍵，則停止模擬
    if keyboard.is_pressed('q'):
        sim.stopSimulation()
        break

# 終止模擬
#sim.stopSimulation()
