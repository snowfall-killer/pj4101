#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo motor;

// PID 控制常数
const float kp = 45;  // 比例增益 45
const float ki = 5;   // 积分增益 5
const float kd = 30;   // 微分增益 30
const float initial_delay = 500; // 初始延迟时间 (微秒)

// 初始化误差和积分项
float error_sum = 0.0;
float last_error = 0.0;
unsigned long last_time = 0;

// 指数平滑滤波器参数
const float alpha = 1;  // 平滑因子
float filtered_distance = 0.0;  // 初始化过滤后的距离

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.startContinuous();
  
  motor.attach(9);  // 连接马达到9号引脚
  motor.write(90);
  delay(initial_delay); // 延迟以确保马达回到中间位置
  
  last_time = micros();  // 初始化时间
}

float getDistance() {
  float distance = sensor.readRangeContinuousMillimeters(); // 读取距离，单位毫米
  if (sensor.timeoutOccurred()) {
    Serial.print("Timeout");
    return -1;
  } else {
    distance = constrain(distance, 35, 300); // 将距离值约束在有效范围内
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
    // 更新过滤后的距离
    filtered_distance = alpha * distance + (1 - alpha) * filtered_distance;

    // 检测碰撞
    if (distance <= 35 || distance >= 130) {
      resetControlVariables();
      Serial.println("Collision detected, resetting control variables.");
      motor.write(90); // 重置马达位置
      return;
    }

    Serial.print("Target: ");
    Serial.print(target_distance);
    Serial.print(" mm Distance: ");
    Serial.print(filtered_distance);

    // 计算误差
    float error = target_distance - filtered_distance;
    Serial.print(" Error: ");
    Serial.print(error);

    // 更新误差总和
    error_sum += error * dt;

    // 防止积分项积累过多
    error_sum = constrain(error_sum, -1000, 1000);

    // 计算微分项
    float derivative = (dt > 0) ? (error - last_error) / dt : 0;

    // 计算控制信号
    float control_signal = kp * error + ki * error_sum + kd * derivative;

    // 将控制信号映射到有效的角度范围内
    int angle = constrain(map(control_signal, -10000, 10000, 0, 180), 36 , 144);
    Serial.print(" Control_signal: ");
    Serial.print(control_signal);
    Serial.print(" Angle: ");
    Serial.println(angle);
    motor.write(angle);

    // 更新上一次的误差
    last_error = error;
  }
}

void loop() {
  if (Serial.available() > 0) {
    char key = Serial.read();
    if (key == 'q') {
      // 停止马达
      motor.detach();
      while (1);
    }
  }

  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0; // 将微秒转换为秒
  last_time = current_time;

  controlMotor(90, dt);  // 设置目标距离为 90 mm

  delay(50);  // 延迟以匹配时间步长 (dt)
}
