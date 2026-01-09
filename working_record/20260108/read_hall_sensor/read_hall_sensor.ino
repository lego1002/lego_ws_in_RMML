// --- 接線定義 (Low Active) ---
const int PIN_EN  = 4;   // Pin 1 (X0)
const int PIN_PWM = 5;   // Pin 19 (XH0-) - 速度控制
const int PIN_DIR = 6;   // Pin 23 (XH1-) - 方向控制
const int PIN_SPD_FB = 2; // Pin 12 (YH1) - 速度回授 (接到 D2)

// --- 變數 ---
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;

// --- 中斷函式 ---
void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SPD_FB, INPUT_PULLUP); // 關鍵：開啟上拉電阻

  // 註冊中斷
  attachInterrupt(digitalPinToInterrupt(PIN_SPD_FB), countPulse, FALLING);

  // 初始：停止
  digitalWrite(PIN_EN, HIGH);    // Disable
  analogWrite(PIN_PWM, 255);     // Speed 0
  digitalWrite(PIN_DIR, LOW);    // 正轉

  Serial.println("=== Feedback Sensor Test Start ===");
  delay(1000);
  
  digitalWrite(PIN_EN, LOW);     // Enable Motor
}

void loop() {
  // --- 讀取並顯示 RPM (每 0.2秒刷新一次) ---
  if (millis() - lastTime >= 200) {
    detachInterrupt(digitalPinToInterrupt(PIN_SPD_FB));
    unsigned long count = pulseCount;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_FB), countPulse, FALLING);
    
    // RPM = (count / 0.2s / 12) * 60
    //     = count * 5 * 5 
    //     = count * 25
    rpm = count * 25.0; 

    Serial.print("Pulses: "); Serial.print(count);
    Serial.print(" | RPM: "); Serial.println(rpm);
    
    lastTime = millis();
  }

  // --- 自動測試邏輯 (使用 millis 控制時間流程) ---
  // 為了不干擾 RPM 讀取，我們用餘數運算來分時段
  
  unsigned long currentTime = millis();
  unsigned long cycleTime = currentTime % 15000; // 15秒一個循環

  if (cycleTime < 5000) {
    // [0~5秒] 階段 1: 低速爬行 (10% Speed)
    // PWM = 255 - (255 * 0.1) = 230
    analogWrite(PIN_PWM, 230);
    if(cycleTime < 200) Serial.println(">>> Mode: Slow Crawl (10%)");
  } 
  else if (cycleTime < 10000) {
    // [5~10秒] 階段 2: 中速巡航 (50% Speed)
    // PWM = 255 - (255 * 0.5) = 127
    analogWrite(PIN_PWM, 127);
    if(cycleTime > 5000 && cycleTime < 5200) Serial.println(">>> Mode: Medium Speed (50%)");
  } 
  else {
    // [10~15秒] 階段 3: 停止 (0% Speed)
    analogWrite(PIN_PWM, 255);
    if(cycleTime > 10000 && cycleTime < 10200) Serial.println(">>> Mode: STOP");
  }
}