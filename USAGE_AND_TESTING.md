# 系統使用方式與測試指南

## 節點確認
```bash
ros2 node list
```

## RFID 與登入測試
```bash
ros2 topic echo /rfid_uid
ros2 topic pub /auth_result std_msgs/msg/String "data: 'OK'" -1
ros2 topic pub /auth_result std_msgs/msg/String "data: 'FAIL'" -1
```

## 馬達控制
```bash
ros2 topic pub /motor_speed std_msgs/msg/Int32 "data: 120" -1
```

## Supervisor 指令
```bash
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'BLOCK_SYSTEM'" -1
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'UNBLOCK_SYSTEM'" -1
ros2 topic pub /supervisor_cmd std_msgs/msg/String "data: 'SUPER_PWM 180'" -1
```
