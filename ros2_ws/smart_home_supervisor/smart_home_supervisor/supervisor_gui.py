#!/usr/bin/env python3
"""
Supervisor GUI 控制面板

功能：
  - 透過 ROS2 topic `supervisor_cmd` (std_msgs/String)
    發送以下指令給 supervisor_node：

    BLOCK_SYSTEM
    UNBLOCK_SYSTEM
    ADD_TEMP_USER <uid> <password>
    SUPER_PWM <value>

使用方式：
  1. 在有 ROS2 環境的終端機中：
       source /opt/ros/humble/setup.bash
       source ~/ros2_ws/install/setup.bash
  2. 執行：
       python3 supervisor_gui.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox


class SupervisorClient:
    """負責和 ROS2 supervisor_cmd topic 溝通的簡單封裝"""

    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(String, 'supervisor_cmd', 10)

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        self._node.get_logger().info(f'Sent supervisor_cmd: {cmd}')


class SupervisorGUI:
    """Tkinter 前端 GUI，使用 SupervisorClient 發送指令"""

    def __init__(self, ros_node: Node):
        self.node = ros_node
        self.client = SupervisorClient(ros_node)

        # 建立 Tk 視窗
        self.root = tk.Tk()
        self.root.title("Supervisor 控制面板")
        self.root.geometry("420x260")

        # ------- 版面配置 -------
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 區塊：系統封鎖 / 解鎖
        block_frame = ttk.LabelFrame(main_frame, text="系統封鎖 / 解鎖", padding=10)
        block_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        btn_block = ttk.Button(
            block_frame,
            text="BLOCK_SYSTEM（封鎖）",
            command=self.on_block_system
        )
        btn_block.grid(row=0, column=0, sticky="ew", pady=5)

        btn_unblock = ttk.Button(
            block_frame,
            text="UNBLOCK_SYSTEM（解除）",
            command=self.on_unblock_system
        )
        btn_unblock.grid(row=1, column=0, sticky="ew", pady=5)

        # 區塊：臨時使用者
        temp_frame = ttk.LabelFrame(main_frame, text="新增臨時使用者（ADD_TEMP_USER）", padding=10)
        temp_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        ttk.Label(temp_frame, text="UID：").grid(row=0, column=0, sticky="e")
        self.entry_uid = ttk.Entry(temp_frame, width=18)
        self.entry_uid.grid(row=0, column=1, sticky="w", padx=5, pady=2)

        ttk.Label(temp_frame, text="Password：").grid(row=1, column=0, sticky="e")
        self.entry_pwd = ttk.Entry(temp_frame, width=18, show="*")
        self.entry_pwd.grid(row=1, column=1, sticky="w", padx=5, pady=2)

        btn_add_temp = ttk.Button(
            temp_frame,
            text="送出 ADD_TEMP_USER",
            command=self.on_add_temp_user
        )
        btn_add_temp.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)

        # 區塊：SUPER_PWM
        pwm_frame = ttk.LabelFrame(main_frame, text="SUPER_PWM（最高權限 PWM）", padding=10)
        pwm_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)

        ttk.Label(pwm_frame, text="PWM 數值 (0–255)：").grid(row=0, column=0, sticky="w")

        self.pwm_var = tk.IntVar(value=0)
        self.scale_pwm = ttk.Scale(
            pwm_frame,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            command=self.on_pwm_scale_changed
        )
        self.scale_pwm.set(0)
        self.scale_pwm.grid(row=1, column=0, sticky="ew", pady=5)

        self.entry_pwm = ttk.Entry(pwm_frame, width=6)
        self.entry_pwm.insert(0, "0")
        self.entry_pwm.grid(row=1, column=1, sticky="w", padx=5)

        btn_send_pwm = ttk.Button(
            pwm_frame,
            text="送出 SUPER_PWM",
            command=self.on_super_pwm
        )
        btn_send_pwm.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)

        # 狀態列
        self.status_var = tk.StringVar(value="就緒")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN)
        status_label.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        # Grid 權重，讓介面可以拉伸
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)

        # 關閉視窗時正確關閉 ROS
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ============ GUI 事件處理 ============

    def set_status(self, text: str):
        self.status_var.set(text)

    def on_block_system(self):
        self.client.send_command("BLOCK_SYSTEM")
        self.set_status("已送出：BLOCK_SYSTEM")

    def on_unblock_system(self):
        self.client.send_command("UNBLOCK_SYSTEM")
        self.set_status("已送出：UNBLOCK_SYSTEM")

    def on_add_temp_user(self):
        uid = self.entry_uid.get().strip()
        pwd = self.entry_pwd.get().strip()

        if not uid or not pwd:
            messagebox.showwarning("輸入不足", "請輸入 UID 和 Password。")
            return

        cmd = f"ADD_TEMP_USER {uid} {pwd}"
        self.client.send_command(cmd)
        self.set_status(f"已送出：{cmd}")

    def on_pwm_scale_changed(self, value_str: str):
        """滑桿變動時同步更新文字框（避免初始化順序造成 entry 尚未建立）"""
        try:
            value = int(float(value_str))
        except ValueError:
            value = 0
        value = max(0, min(255, value))

        # 防呆：初始化階段 entry_pwm 可能尚未建立
        if not hasattr(self, "entry_pwm") or self.entry_pwm is None:
            return

        self.entry_pwm.delete(0, tk.END)
        self.entry_pwm.insert(0, str(value))


    def on_super_pwm(self):
        text = self.entry_pwm.get().strip()
        try:
            value = int(text)
        except ValueError:
            messagebox.showerror("數值錯誤", "請輸入 0～255 的整數。")
            return

        if value < 0 or value > 255:
            messagebox.showerror("範圍錯誤", "PWM 數值必須在 0～255 之間。")
            return

        # 同步滑桿
        self.scale_pwm.set(value)

        cmd = f"SUPER_PWM {value}"
        self.client.send_command(cmd)
        self.set_status(f"已送出：{cmd}")

    def on_close(self):
        # 關閉視窗前，試著關閉 ROS
        try:
            self.node.get_logger().info("Supervisor GUI shutting down...")
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    # 這裡用 create_node 即可，不需要寫一個完整的 Node subclass
    node = rclpy.create_node('supervisor_gui_node')

    gui = SupervisorGUI(node)
    gui.run()

    # 視窗關閉後，安全銷毀 node
    node.destroy_node()


if __name__ == "__main__":
    main()
