# 系統使用方式與測試指南（ROS2 Topic 操作）

本文件說明如何使用 **ros2 topic pub / echo**  
來測試整個智慧家庭系統是否正常運作。

---

## 一、確認目前系統狀態

### 1️⃣ Node 列表
```bash
ros2 node list

/arduino_bridge
/supervisor_node
```

### 2️⃣ Topic 列表
```bash
ros2 topic list
```

##### 目前系統使用的主要 topics：
- /rfid_uid
- /auth_result
- /motor_speed
- /supervisor_cmd
- /supervisor_block
- /supervisor_motor_override
- /supervisor_status
- /temporary_user
- /lux
- /humidity
- /temperature
- /sensor_raw

## 二、RFID 與登入測試
### 1️⃣ 觀察 RFID UID
```bash
ros2 topic echo /rfid_uid
```

刷卡後應看到 UID 字串。

### 2️⃣ 模擬登入成功 / 失敗
```bash
ros2 topic pub /auth_result std_msgs/msg/String "data: 'OK'" -1

ros2 topic pub /auth_result std_msgs/msg/String "data: 'FAIL'" -1
```

預期行為：

| 狀態  |            Arduino 行為            |
| :---: | :--------------------------------: |
|  OK   |   LCD 顯示 Welcome，蜂鳴器兩短音   |
| FAIL  | LCD 顯示 Access Denied，蜂鳴器長音 |
### 3️⃣ 登出測試
```bash
ros2 topic pub /auth_result std_msgs/msg/String "data: 'LOGOUT'" -1
```

LCD 回到：
```text
┌──────────────────────────┐
│ Waiting for card         │
│ Auth: NONE               │
└──────────────────────────┘
```

## 三、一般使用者馬達控制

⚠️ 需先登入成功（AUTH OK）
```bash
ros2 topic pub /motor_speed std_msgs/msg/Int32 "data: 120" -1
```

未登入時，Arduino 會回傳：
```bash
ERR,UNAUTHORIZED_MOTOR_CMD
```

## 四、Supervisor 功能測試
### 1️⃣ 系統封鎖（BLOCK）
```bash
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'BLOCK_SYSTEM'" -1
```

確認：
```bash
ros2 topic echo /supervisor_block
ros2 topic echo /supervisor_status
```

Arduino LCD 顯示：
LCD 回到：
```text
┌──────────────────────────┐
│ SYSTEM BLOCKED           │
└──────────────────────────┘
```

### 2️⃣ 解除封鎖（UNBLOCK）
```bash
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'UNBLOCK_SYSTEM'" -1
```
### 3️⃣ Supervisor 馬達強制控制（不需登入）
```bash
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'SUPER_PWM 180'" -1
```

即使未登入，馬達也應該轉動。

### 4️⃣ 新增臨時使用者
```bash
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'ADD_TEMP_USER 12345678 9999'" -1
```

觀察：
```bash
ros2 topic echo /temporary_user
ros2 topic echo /supervisor_status
```
## 五、感測器資料測試
```bash
ros2 topic echo /sensor_raw
ros2 topic echo /temperature
ros2 topic echo /humidity
ros2 topic echo /lux
```
## 六、快速測試指令總整理
```bash
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'BLOCK_SYSTEM'" -1
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'SUPER_PWM 120'" -1
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'UNBLOCK_SYSTEM'" -1
ros2 topic pub /auth_result std_msgs/msg/String "data: 'OK'" -1
ros2 topic pub /motor_speed std_msgs/msg/Int32 "data: 80" -1
ros2 topic pub /auth_result std_msgs/msg/String "data: 'LOGOUT'" -1
```