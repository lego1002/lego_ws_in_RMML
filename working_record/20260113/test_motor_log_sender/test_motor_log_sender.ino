// --- 接線定義 (Low Active / Sinking) ---
const int PIN_EN  = 4;    // Enable
const int PIN_PWM = 5;    // Speed PWM
const int PIN_DIR = 6;    // Direction (安全鎖定)
const int PIN_SPD_FB = 2; // Feedback

// --- 參數 ---
const int DRIVER_MAX_RPM = 4000; 

// --- 變數 ---
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float actualRPM = 0;
int targetRPM = 0;
int outputPWM = 0;

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200); // 鮑率
  
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT); 
  pinMode(PIN_SPD_FB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PIN_SPD_FB), countPulse, FALLING);

  // 初始安全狀態
  digitalWrite(PIN_EN, HIGH);    
  analogWrite(PIN_PWM, 255);     
  digitalWrite(PIN_DIR, LOW);    // 固定方向
  
  // 等待 2 秒讓你有時間按錄影
  delay(2000);
  digitalWrite(PIN_EN, LOW); // 啟動
}

void loop() {
  // 30秒 自動測試循環
  unsigned long t = millis(); 
  unsigned long loopTime = t % 30000;
  
  if (loopTime < 5000) {
    targetRPM = 0;    // [0-5s] 靜止 (讓你確認電流歸零)
  } else if (loopTime < 10000) {
    targetRPM = 1000; // [5-10s] 低速暖身
  } else if (loopTime < 15000) {
    targetRPM = 3000; // [10-15s] 高速衝刺 (觀察電流飆升)
  } else if (loopTime < 25000) {
    targetRPM = 500;  // [15-25s] 緩行
  } else {
    targetRPM = 0;    // [25-30s] 結束
  }

  // PWM 計算
  outputPWM = map(targetRPM, 0, DRIVER_MAX_RPM, 255, 0);
  outputPWM = constrain(outputPWM, 0, 255);
  analogWrite(PIN_PWM, outputPWM);

  // 數據回傳 (CSV 格式)
  if (t - lastTime >= 50) { // 50ms 取樣
    detachInterrupt(digitalPinToInterrupt(PIN_SPD_FB));
    unsigned long count = pulseCount;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_FB), countPulse, FALLING);
    
    actualRPM = count * 100.0; 
    
    // *** 這裡改了！只輸出純數字 ***
    // 格式：時間(ms), 目標RPM, 實際RPM
    Serial.print(t);
    Serial.print(",");
    Serial.print(targetRPM);
    Serial.print(",");
    Serial.println(actualRPM);
    
    lastTime = t;
  }
}