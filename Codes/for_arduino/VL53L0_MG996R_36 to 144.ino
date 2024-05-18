#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo motor;

// PID 控制常數
const float kp = 1.0;  // 比例增益
const float ki = 0.1;  // 積分增益
const float kd = 0.5;  // 微分增益
const float tt = 1000;
// 初始化誤差和積分項
float error_sum = 0.0;
float last_error = 0.0;
unsigned long last_time = 0;



// float alpha = 0.2;  // 低通濾波器係數

// float getFilteredDistance() {
//   float distance = sensor.readRangeContinuousMillimeters();
//   static float filteredDistance = distance;

//   filteredDistance = alpha * distance + (1 - alpha) * filteredDistance;

//   return filteredDistance;
// }











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

  // 馬達測試：先回到0度，跑到180度，再回到0度
  motor.write(0);
  delay(tt); // 延遲2秒以確保馬達到達0度
  motor.write(180);
  delay(tt); // 延遲2秒以確保馬達到達180度
  motor.write(0);
  delay(tt);
  motor.write(45);
  delay(tt);
  motor.write(90);
  delay(tt);
  motor.write(135);
  delay(tt);
  motor.write(90);
  delay(2*tt); // 延遲2秒以確保馬達回到0度
}

float getDistance() {
  float distance = sensor.readRangeContinuousMillimeters(); // 讀取距離，單位毫米
  if (sensor.timeoutOccurred()) {
    Serial.print("Timeout");
    return -1;
  } else {
    if (distance < 10) {
      distance = 10;
    } else if (distance > 300) {
      distance = 300;
    }
    return distance;
  }
}

void controlMotor(float target_distance, float dt) {
  float distance = getDistance();
  if (distance > 0) {
    Serial.print("Target: ");
    Serial.print(target_distance);
    Serial.print(" mm Distance: ");
    Serial.print(distance);

    // 計算誤差
    float error = target_distance - distance;
    Serial.print("Error: ");
    Serial.print(error);

    // 更新誤差總和
    error_sum += error * dt;

    // 計算微分項
    float derivative = (error - last_error) / dt;

    // 計算控制信號
    float control_signal = kp * error + ki * error_sum + kd * derivative;

    // 將控制信號映射到有效的角度範圍內
    int angle = constrain(map(control_signal, -100, 100, 0, 180), 36, 144);
    Serial.print("Control_signal: ");
    Serial.print(control_signal);
    Serial.print("angle: ");
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

  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  controlMotor(100, dt);  // 設定目標距離為 100 mm

  delay(10);  // 延遲以匹配時間步長 (dt)
}