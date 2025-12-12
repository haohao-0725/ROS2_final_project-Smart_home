# 專題名稱（Final Project Title）




---

## 1. 系統概要 (System Overview)

- 專題主題：TODO
- 系統功能：
  - TODO
- 主要技術：
  - ROS2 (版本：TODO，例如 Humble)
  - Arduino / MCU：TODO
  - Web 介面 / Dashboard：TODO
  - 其他：TODO

---

## 2. 硬體架構 (Hardware Setup)

<!-- 描述有哪些電腦、控制板、感測器、致動器 -->

- 電腦：
  - Supervisor PC：TODO（OS、IP、角色）
  - Robot PC #1：TODO
  - Robot PC #2：TODO
- 控制板 / 馬達 / 感測器：
  - Arduino / 型號：TODO
  - 馬達與 Driver：TODO
  - 感測器（RFID、光感測器、相機…）：TODO

---

## 3. 軟體架構與資料夾結構 (Software & Folder Structure)

專案根目錄結構範例：

```bash
final_project_ros2/
├── ros2_ws/
│   ├── src/
│   │   ├── supervisor_pkg/      # 監督者節點
│   │   ├── robot1_pkg/
│   │   └── robot2_pkg/
│   ├── build/                   # colcon 自動產生（不進 git）
│   ├── install/                 # colcon 自動產生（不進 git）
│   └── log/                     # colcon 自動產生（不進 git）
├── firmware/                    # Arduino / MCU 程式
│   └── TODO
├── web/                         # Web 前端 / 後端
│   └── TODO
├── docs/                        # 報告、架構圖、筆記
│   └── TODO
├── scripts/                     # 一鍵啟動、多機同步腳本
│   └── TODO
└── README.md
