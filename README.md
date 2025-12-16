# ROS2 期末專題－智慧家庭系統
RFID × Arduino × Supervisor Control Architecture

本專案為一套基於 **ROS 2 Humble** 的智慧家庭控制系統，
整合 Arduino 感測與致動、ROS2 分散式節點架構，以及 Supervisor 高權限管理機制。

## 系統設計重點
- ROS2 非阻塞式節點設計
- Arduino 與 ROS 的明確通訊協議
- Supervisor 最高權限節點（可繞過登入限制）
- 可使用 CLI 或 GUI 進行完整測試
- 支援多電腦 ROS2 分散式架構

## 建置與編譯
```bash
cd ~/final_project_ros2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 啟動方式（建議流程）
### 電腦 A（連接 Arduino）
```bash
ros2 run smart_home_rfid arduino_bridge
```

### 電腦 B（Supervisor）
```bash
ros2 run smart_home_supervisor supervisor_node
```

### 啟動 GUI（選用）
```bash
ros2 run smart_home_supervisor supervisor_gui
```

## 多電腦執行注意事項
```bash
export ROS_DOMAIN_ID=8
```

## 測試說明
請參考 USAGE_AND_TESTING.md
