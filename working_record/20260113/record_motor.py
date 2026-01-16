import serial
import csv
import time

# --- 修改這裡 ---
COM_PORT = 'COM3'   # 改成你的 Arduino Port
BAUD_RATE = 115200
FILE_NAME = 'arduino_data.csv'
# ----------------

ser = serial.Serial(COM_PORT, BAUD_RATE)
print(f"連線成功！數據將存入 {FILE_NAME}")
print("等待 Arduino 傳送數據... (按 Ctrl+C 停止)")

with open(FILE_NAME, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Time_ms", "Target_RPM", "Actual_RPM"]) # 標頭
    
    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 3: # 確保格式正確
                        writer.writerow(parts)
                        print(f"記錄中: {line}")
    except KeyboardInterrupt:
        print("\n錄製結束！")
        ser.close()