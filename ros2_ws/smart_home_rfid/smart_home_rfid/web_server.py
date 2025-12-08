#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int32

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import os
from ament_index_python.packages import get_package_share_directory

# ============================================================
#  Flask + SocketIO 初始化
# ============================================================
# 找到這個 package 在 install 中的 share 目錄
pkg_share = get_package_share_directory('smart_home_rfid')
template_dir = os.path.join(pkg_share, 'templates')

app = Flask(__name__, template_folder=template_dir)
app.config['SECRET_KEY'] = 'secret-key-change-me'
socketio = SocketIO(app, async_mode='threading')



# ============================================================
#  ROS2 Node：負責跟整個 ROS 世界互動
# ============================================================
class WebBridgeNode(Node):
    """
    - 訂閱:
        /lux, /temperature, /humidity (Float32)
        /auth_state (String)
        /rfid_uid   (String)
    - 發布:
        /password_attempt (String)
        /motor_speed      (Int32)
        /logout_request   (String)
    - 每次收到新的資料，就透過 socketio. emit() 推給前端
    """

    def __init__(self):
        super().__init__('web_bridge_node')

        # 最新狀態（也可以給 REST 查）
        self.latest_lux = 0.0
        self. latest_temp = 0.0
        self.latest_hum = 0.0
        self. latest_auth_state = "INIT"
        self. latest_uid = "NONE"

        # 時間戳記（用於前端顯示上次更新時間）
        self.last_sensor_update_time = time.time()
        self.last_auth_update_time = time.time()
        self.last_rfid_update_time = time.time()

        # Publishers
        self.pwd_pub = self. create_publisher(String, 'password_attempt', 10)
        self.motor_pub = self. create_publisher(Int32, 'motor_speed', 10)
        self. logout_pub = self.create_publisher(String, 'logout_request', 10)

        # Subscribers
        self. create_subscription(Float32, 'lux', self.lux_cb, 10)
        self.create_subscription(Float32, 'temperature', self.temp_cb, 10)
        self. create_subscription(Float32, 'humidity', self.hum_cb, 10)
        self.create_subscription(String, 'auth_state', self. auth_state_cb, 10)
        self.create_subscription(String, 'rfid_uid', self.rfid_cb, 10)

        self.get_logger(). info('web_bridge_node started.')

    def _emit_sensor_update(self):
        """發送感測器資料更新到前端"""
        self.last_sensor_update_time = time.time()
        socketio.emit('sensor_update', {
            'lux': self.latest_lux,
            'temperature': self.latest_temp,
            'humidity': self.latest_hum,
            'timestamp': self.last_sensor_update_time,
        }, namespace='/ws')

    # ---- Callbacks ----
    def lux_cb(self, msg: Float32):
        self.latest_lux = msg.data
        self._emit_sensor_update()

    def temp_cb(self, msg: Float32):
        self.latest_temp = msg.data
        self._emit_sensor_update()

    def hum_cb(self, msg: Float32):
        self.latest_hum = msg. data
        self._emit_sensor_update()

    def auth_state_cb(self, msg: String):
        self. latest_auth_state = msg.data
        self.last_auth_update_time = time.time()
        socketio.emit('auth_update', {
            'auth_state': self.latest_auth_state,
            'timestamp': self.last_auth_update_time,
        }, namespace='/ws')

    def rfid_cb(self, msg: String):
        self.latest_uid = msg. data
        self. last_rfid_update_time = time.time()
        socketio.emit('rfid_update', {
            'uid': self.latest_uid,
            'timestamp': self.last_rfid_update_time,
        }, namespace='/ws')

    # ---- 提供給 Flask Websocket 呼叫 ----
    def publish_password(self, password: str):
        msg = String()
        msg.data = password
        self.pwd_pub.publish(msg)
        self.get_logger().info(f'Published password_attempt: {password}')

    def publish_motor_speed(self, spd: int):
        msg = Int32()
        msg. data = spd
        self.motor_pub.publish(msg)
        self. get_logger().info(f'Published motor_speed: {spd}')

    def publish_logout(self):
        msg = String()
        msg.data = 'LOGOUT'
        self.logout_pub.publish(msg)
        self. get_logger().info('Published logout_request')

    def push_status_timer_cb(self):
        """Heartbeat：就算 ROS 資料沒變，也每秒推一次給 Web。"""
        # Sensor 資料
        socketio.emit('sensor_update', {
            'lux': self.latest_lux,
            'temperature': self.latest_temp,
            'humidity': self.latest_hum,
            'timestamp': time.time(),   # 用現在時間
        }, namespace='/ws')

        # Auth 狀態
        socketio.emit('auth_update', {
            'auth_state': self.latest_auth_state,
            'timestamp': self.last_auth_update_time or time.time(),
        }, namespace='/ws')

        # RFID
        socketio.emit('rfid_update', {
            'uid': self.latest_uid,
            'timestamp': self.last_rfid_update_time or time.time(),
        }, namespace='/ws')


# 全域的 ROS Node 實例
ros_node: WebBridgeNode = None


# ============================================================
#  Flask Routes
# ============================================================
@app.route('/')
def index():
    return render_template('index.html')


# ============================================================
#  WebSocket (Socket.IO) 事件處理
# ============================================================
@socketio.on('connect', namespace='/ws')
def ws_connect():
    print('Client connected')
    # 連線時把目前最新資料丟給他
    if ros_node is not None:
        emit('sensor_update', {
            'lux': ros_node.latest_lux,
            'temperature': ros_node.latest_temp,
            'humidity': ros_node.latest_hum,
            'timestamp': ros_node.last_sensor_update_time,
        })
        emit('auth_update', {
            'auth_state': ros_node.latest_auth_state,
            'timestamp': ros_node.last_auth_update_time,
        })
        emit('rfid_update', {
            'uid': ros_node.latest_uid,
            'timestamp': ros_node.last_rfid_update_time,
        })


@socketio.on('disconnect', namespace='/ws')
def ws_disconnect():
    print('Client disconnected')


@socketio.on('submit_password', namespace='/ws')
def handle_submit_password(data):
    """
    data: {'password': '1234'}
    """
    pwd = data. get('password', '')
    print(f'Received password from web: {pwd}')
    if ros_node is not None:
        ros_node.publish_password(pwd)


@socketio.on('set_motor_speed', namespace='/ws')
def handle_set_motor_speed(data):
    """
    data: {'speed': 120}
    """
    spd = int(data.get('speed', 0))
    print(f'Received motor speed from web: {spd}')
    if ros_node is not None:
        ros_node.publish_motor_speed(spd)


@socketio.on('logout', namespace='/ws')
def handle_logout():
    """處理登出請求"""
    print('Received logout request from web')
    if ros_node is not None:
        ros_node.publish_logout()


# ============================================================
#  ROS2 Spin：在背景 Thread 不斷 spin_once
# ============================================================
def ros_spin_thread():
    global ros_node
    rclpy.init()
    ros_node = WebBridgeNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
        rclpy.shutdown()


# ============================================================
#  Main
# ============================================================
def main():
    # 啟動 ROS spin 的背景執行緒
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    # 啟動 Flask + SocketIO Web 伺服器
    # host='0.0.0.0' 讓同一個網域的其他裝置也可以連
    socketio.run(app, host='0.0.0.0', port=5000)


if __name__ == '__main__':
    main()