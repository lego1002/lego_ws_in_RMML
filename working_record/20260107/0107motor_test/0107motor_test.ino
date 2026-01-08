// --- 接線定義 (Low Active 接法) ---
// XH+ (Pin 17, 21) 接 5V
// XH- (Pin 19, 23) 接 Arduino 腳位
const int PIN_EN  = 4;   // Pin 19 (XH0-)
const int PIN_PWM = 5;   // Pin 19 (XH0-) - 速度
const int PIN_DIR = 6;   // Pin 23 (XH1-) - 方向

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // 初始：停止
  digitalWrite(PIN_EN, HIGH);    // 關閉
  analogWrite(PIN_PWM, 255);     // 速度 0 (Low Active: 255=Stop)
  
  Serial.println("=== Speed Sweep Test Start ===");
  delay(1000);
  
  // 啟動馬達
  digitalWrite(PIN_EN, LOW);     // 開啟
}

void loop() {
  delay(3000);
  // --- 階段 1: 正轉測試 ---
  Serial.println("Direction: Forward (LOW)");
  digitalWrite(PIN_DIR, LOW);    // 設定方向 A
  
  // 加速 (255 -> 0) *數值越小越快
  Serial.println("Ramping UP...");
  for (int i = 255; i >= 50; i -= 2) {
    analogWrite(PIN_PWM, i);
    delay(20); // 這裡決定加速的快慢
  }
  
  delay(1000); // 全速維持 1 秒

  // 減速 (0 -> 255)
  Serial.println("Ramping DOWN...");
  for (int i = 50; i <= 255; i += 2) {
    analogWrite(PIN_PWM, i);
    delay(20);
  }
  
  delay(500); // 停止 1 秒

  // --- 階段 2: 反轉測試 ---
  Serial.println("Direction: Reverse (HIGH)");
  digitalWrite(PIN_DIR, HIGH);   // 設定方向 B (只要切換 High/Low)
  
  // 加速 (255 -> 0)
  Serial.println("Ramping UP...");
  for (int i = 255; i >= 50; i -= 2) {
    analogWrite(PIN_PWM, i);
    delay(20);
  }
  
  delay(500); // 全速維持 1 秒

  // 減速 (0 -> 255)
  Serial.println("Ramping DOWN...");
  for (int i = 50; i <= 255; i += 2) {
    analogWrite(PIN_PWM, i);
    delay(20);
  }

  Serial.println("--- Cycle Complete, Waiting... ---");
  delay(2000); // 休息 2 秒再重來
}