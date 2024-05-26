#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo motor;

// PID 控制常數
const float kp = 1.8;  // 比例增益
const float ki = 0.2;  // 積分增益
const float kd = 1;  // 微分增益
const float tt = 1000;
// 初始化誤差和積分項
float error_sum = 0.0;
float last_error = 0.0;
unsigned long last_time = 0;
unsigned long last_control_time = 0;

const int numReadings = 10;
int readings[numReadings];      // the readings from the VL53L0X
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

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
  delay(tt); // 延遲2秒以確保馬達回到中間位置

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  last_time = micros();  // 初始化時間
  last_control_time = millis();  // 初始化控制時間
}

float getFilteredDistance() {
  // 讀取新的距離數據
  int newDistance = sensor.readRangeContinuousMillimeters();
  
  // 檢查超時
  if (sensor.timeoutOccurred()) {
    Serial.print("Timeout");
    return -1;
  }
  
  // 限制距離範圍
  if (newDistance < 35) {
    newDistance = 35;
  } else if (newDistance > 300) {
    newDistance = 300;
  }

  // 減去上一次的讀數
  total = total - readings[readIndex];
  // 記錄新的讀數
  readings[readIndex] = newDistance;
  // 加入到總和中
  total = total + readings[readIndex];
  // 移動到下一个位置
  readIndex = readIndex + 1;

  // 如果到達末尾，回到開頭
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // 計算平均值
  average = total / numReadings;
  
  return average;
}

void controlMotor(float target_distance, float dt) {
  float distance = getFilteredDistance();
  if (distance > 0) {
    Serial.print("Target: ");
    Serial.print(target_distance);
    Serial.print(" mm Distance: ");
    Serial.print(distance);

    // 計算誤差
    float error = target_distance - distance;
    Serial.print(" Error: ");
    Serial.print(error);

    // 更新誤差總和
    error_sum += error * dt;

    // 防止積分項積累過多
    error_sum = constrain(error_sum, -100, 100);

    // 計算微分項
    float derivative = (error - last_error) / dt;

    // 計算控制信號
    float control_signal = kp * error + ki * error_sum + kd * derivative;

    // 將控制信號映射到有效的角度範圍內
    int angle = constrain(map(control_signal, -1000, 1000, 0, 180), 36 , 144);
    Serial.print(" Control_signal: ");
    Serial.print(control_signal);
    Serial.print(" angle: ");
    Serial.print(angle);
    Serial.print(" error sum: ");
    Serial.println(error_sum);
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

  // 每10毫秒读取一次距离
  if (millis() - last_control_time >= 10) {
    getFilteredDistance();
  }

  // 每100毫秒进行一次反应
  if (millis() - last_control_time >= 100) {
    controlMotor(110, dt);  // 設定目標距離為 110 mm
    last_control_time = millis();
  }

  delay(10);  // 延遲以匹配時間步長
}