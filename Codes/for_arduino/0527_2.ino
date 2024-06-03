#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo motor;

// PID 控制常數
const float kp = 38.2;  // 比例增益
const float ki = 5;   // 積分增益
const float kd = 31.6;   // 微分增益
const float initial_delay = 500; // 初始延遲時間 (微秒)

// 初始化誤差和積分項
float error_sum = 0.0;
float last_error = 0.0;
unsigned long last_time = 0;

// 指數平滑濾波器參數
const float alpha = 1;  // 平滑因子
float filtered_distance = 0.0;  // 初始化過濾後的距離

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.startContinuous();
  
  motor.attach(9);  // 連接馬達到9號引腳
  motor.write(90);
  delay(initial_delay); // 延遲以確保馬達回到中間位置
  
  last_time = micros();  // 初始化時間
}

float getDistance() {
  float distance = sensor.readRangeContinuousMillimeters(); // 讀取距離，單位毫米
  if (sensor.timeoutOccurred()) {
    Serial.print("Timeout");
    return -1;
  } else {
    distance = constrain(distance, 35, 300); // 將距離值約束在有效範圍內
    return distance;
  }
}

void resetControlVariables() {
  error_sum = 0.0;
  last_error = 0.0;
}

void controlMotor(float target_distance, float dt) {
  float distance = getDistance();
  if (distance > 0) {
    // 更新過濾後的距離
    filtered_distance = alpha * distance + (1 - alpha) * filtered_distance;

    // 檢測碰撞
    if (distance <= 35 || distance >= 130) {
      resetControlVariables();
      Serial.println("Collision detected, resetting control variables.");
      motor.write(90); // 重置馬達位置
      return;
    }

    Serial.print("Target: ");
    Serial.print(target_distance);
    Serial.print(" mm Distance: ");
    Serial.print(filtered_distance);

    // 計算誤差
    float error = target_distance - filtered_distance;
    Serial.print(" Error: ");
    Serial.print(error);

    // 更新誤差總和
    error_sum += error * dt;

    // 防止積分項累積過多
    error_sum = constrain(error_sum, -1000, 1000);

    // 計算微分項
    float derivative = (dt > 0) ? (error - last_error) / dt : 0;

    // 計算控制信號
    float control_signal = (kp * error + ki * error_sum + kd * derivative);

    // 將控制信號映射到有效的角度範圍內
    int angle = constrain(map(control_signal, -10000, 10000, 0, 180), 36 , 144);
    Serial.print(" Control_signal: ");
    Serial.print(control_signal);
    Serial.print(" Angle: ");
    Serial.println(angle);
    motor.write(angle);

    // 更新上一次的誤差
    last_error = error;
  }
}

void loop() {
  if (Serial.available() > 0) {
    char key = Serial.read();
    if (key == 'q') {
      // 停止馬達
      motor.detach();
      while (1);
    }
  }

  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0; // 將微秒轉換為秒
  last_time = current_time;

  controlMotor(90, dt);  // 設置目標距離為 90 mm

  delay(50);  // 延遲以匹配時間步長 (dt)
}