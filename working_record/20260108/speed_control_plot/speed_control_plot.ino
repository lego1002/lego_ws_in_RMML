// --- 接線定義 (Low Active / Sinking 接法) ---
const int PIN_EN  = 4;    // Enable (Pin 1) - 啟動/停止
const int PIN_PWM = 5;    // Speed PWM (Pin 19) - 速度
const int PIN_DIR = 6;    // Direction (Pin 23) - 方向 
const int PIN_SPD_FB = 2; // Feedback (Pin 12) - 速度回授

// --- 參數設定 ---
// 驅動器參數 02-09 = 0 (0-5V 模式)
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
  Serial.begin(115200); 
  
  // 1. 定義所有腳位為輸出/輸入
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT); 
  pinMode(PIN_SPD_FB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PIN_SPD_FB), countPulse, FALLING);

  // 2. 初始狀態：安全鎖定
  digitalWrite(PIN_EN, HIGH);    // Disable (Servo OFF)
  analogWrite(PIN_PWM, 255);     // Speed 0 (停止)
  
  // 3. 固定方向
  // LOW = 方向 A (導通), HIGH = 方向 B (不導通)
  // 我們先鎖定一個方向，測試過程不換向
  digitalWrite(PIN_DIR, LOW);    
  
  Serial.println("System Ready. Full Safety Mode.");
  delay(1000);
  
  // 4. 啟動馬達
  digitalWrite(PIN_EN, LOW); 
}

void loop() {
  // ❌ 絕對不能在這裡加 delay(5000); 會導致數據累積錯誤且圖表卡死
  
  // --- 1. 自動產生測試腳本 (30秒大循環) ---
  unsigned long t = millis() % 30000; 
  
  if (t < 5000) {
    // [0~5秒] 暖身：1000 RPM
    targetRPM = 1000;
  } else if (t < 15000) {
    // [5~15秒] 停車檢查：0 RPM (10秒)
    targetRPM = 0;
  } else if (t < 20000) {
    // [15~20秒] 衝刺：3000 RPM
    targetRPM = 3000;
  } else if (t < 25000) {
    // [20~25秒] 緩行：500 RPM
    targetRPM = 500;
  } else {
    // [25~30秒] 循環結束：停車
    targetRPM = 0;
  }

  // --- 2. 計算 PWM ---
  // 映射：0 RPM -> 255 (5V), 4000 RPM -> 0 (0V)
  outputPWM = map(targetRPM, 0, DRIVER_MAX_RPM, 255, 0);
  outputPWM = constrain(outputPWM, 0, 255);
  
  analogWrite(PIN_PWM, outputPWM);

  // --- 3. 讀取回授並畫圖 ---
  // 這裡利用 millis() 控時，每 50ms 執行一次，這才是正確的計時方式
  if (millis() - lastTime >= 50) { 
    detachInterrupt(digitalPinToInterrupt(PIN_SPD_FB));
    unsigned long count = pulseCount;
    pulseCount = 0; // ✅ 有歸零，所以數值不會無限累加
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_FB), countPulse, FALLING);
    
    // RPM = count * 100
    actualRPM = count * 100.0; 
    
    Serial.print("Target:");
    Serial.print(targetRPM);
    Serial.print(",");
    Serial.print("Actual:");
    Serial.println(actualRPM);
    
    lastTime = millis();
  }
}