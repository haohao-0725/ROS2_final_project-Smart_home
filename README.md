# ROS2 期末專題－智慧家庭系統（RFID × Arduino × Supervisor）

本專案為一套基於 **ROS 2 Humble** 的智慧家庭控制系統，整合：

- **Arduino Mega**
  - RFID 讀卡（MFRC522）
  - LCD 顯示（I2C 20×4）
  - 蜂鳴器提示（登入 / 失敗 / 感應）
  - DC 馬達 PWM 控制
  - 溫濕度（DHT11）
  - 照度（BH1750）
- **ROS 2 Node**
  - `arduino_bridge`：Arduino 與 ROS 之間的序列通訊橋接
  - `supervisor_node`：監督者節點（最高權限）
- **Supervisor GUI（Python / Tkinter）**
  - 取代手動 `ros2 topic pub` 的人機介面

---

## 一、系統架構說明（概念）

### 1️⃣ Arduino → ROS
- RFID UID 上傳
- 感測器資料回傳
- ACK / ERROR 狀態回傳

### 2️⃣ ROS → Arduino
- 使用者登入結果（AUTH）
- 一般使用者馬達控制
- Supervisor 馬達強制控制（不需登入）
- 系統封鎖狀態（BLOCK）

---

## 二、ROS Node 架構

目前系統中主要執行的節點如下：
```bash
/arduino_bridge
/supervisor_node
```

#### 🔹 arduino_bridge 負責：
- 與 Arduino 進行 Serial 通訊
- 將 Arduino 的資料轉成 ROS topic
- 將 ROS topic 轉成 Arduino 指令

#### 🔹 supervisor_node 負責：
- 系統封鎖 / 解鎖
- 監督者馬達控制（SUPER_PWM）
- 臨時使用者管理
- 系統狀態統整與發布

## 三、環境需求
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Arduino IDE

## 四、編譯與啟動
編譯 ROS2 workspace
```bash
cd ~/final_project_ros2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 五、啟動方式（建議流程）
PC A（接 Arduino 的電腦）
```bash
ros2 run smart_home_rfid arduino_bridge
```
PC B（監督者 / 操作端）
```bash
ros2 run smart_home_supervisor supervisor_node
```

可選擇啟動 GUI：
```bash
ros2 run smart_home_supervisor supervisor_gui
```

或使用 launch（推薦）：
```bash
ros2 launch smart_home_supervisor supervisor_with_gui.launch.py
```

## 六、多電腦執行注意事項

兩台電腦需設定相同的 ROS Domain ID，例如：`export ROS_DOMAIN_ID=8`，並確保在同一個網路環境中。

## 七、相關文件
`USAGE_AND_TESTING.md`

👉 詳細的 topic 操作、測試指令、除錯流程