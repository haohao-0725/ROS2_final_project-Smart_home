# arduino_bridge.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int32, Bool

import serial
import threading


class ArduinoBridgeNode(Node):
    """
    負責：
      - 用 pyserial 讀寫 Arduino
      - 解析 RFID / SENSOR 封包
      - 發布 ROS topic:
          - /rfid_uid        (String)
          - /sensor_raw      (String)
          - /lux             (Float32)
          - /temperature     (Float32)
          - /humidity        (Float32)
      - 訂閱 ROS topic:
          - /auth_result     (String: "OK"/"FAIL")
          - /motor_speed     (Int32: 0~255)
          - /supervisor_block (Bool)
          - /supervisor_motor_override (Int32: 0~255)
    """

    def __init__(self):
        super().__init__('arduino_bridge')

        # 從參數讀取 Serial 設定，方便之後用 ros2 param 調整
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f'Opening serial port {port} @ {baudrate}')
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.05,  # non-blocking-ish
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

        # Publishers
        self.rfid_pub = self.create_publisher(String, 'rfid_uid', 10)
        self.sensor_raw_pub = self.create_publisher(String, 'sensor_raw', 10)
        self.lux_pub = self.create_publisher(Float32, 'lux', 10)
        self.temp_pub = self.create_publisher(Float32, 'temperature', 10)
        self.hum_pub = self.create_publisher(Float32, 'humidity', 10)

        # Subscribers
        self.auth_sub = self.create_subscription(
            String,
            'auth_result',
            self.auth_result_callback,
            10
        )
        self.motor_sub = self.create_subscription(
            Int32,
            'motor_speed',
            self.motor_speed_callback,
            10
        )
        self.supervisor_block_sub = self.create_subscription(
            Bool,
            'supervisor_block',
            self.supervisor_block_callback,
            10,
        )
        self.supervisor_motor_sub = self.create_subscription(
            Int32,
            'supervisor_motor_override',
            self.supervisor_motor_override_callback,
            10,
        )

        # 用 timer 週期性讀取 serial
        self.read_timer = self.create_timer(0.05, self.read_serial_timer_cb)

        # 新增：每 1 秒向 Arduino 要一次感測資料
        self.sensor_req_timer = self.create_timer(1.0, self.request_sensor_timer_cb)

        # 給 Serial 寫入用的 lock（避免多執行緒同時寫）
        self.serial_lock = threading.Lock()
        self.supervisor_blocked = False

        self.get_logger().info('arduino_bridge node started.')

    # -------------------------
    # Timer callback: 讀取 Serial 資料
    # -------------------------
    def read_serial_timer_cb(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            return

        if not line:
            return

        # Debug 用
        self.get_logger().debug(f'Recv line: {line}')

        # 解析封包
        if line.startswith('RFID,UID='):
            uid = line.split('=', 1)[1].strip()
            msg = String()
            msg.data = uid
            self.rfid_pub.publish(msg)
            self.get_logger().info(f'RFID UID received: {uid}')

        elif line.startswith('SENSOR,'):
            # 範例格式: SENSOR,LUX=123,H=45.6,T=23.4
            self.sensor_raw_pub.publish(String(data=line))

            kv_part = line[len('SENSOR,'):]
            parts = kv_part.split(',')

            lux = None
            hum = None
            temp = None
            for p in parts:
                if '=' not in p:
                    continue
                key, val = p.split('=', 1)
                key = key.strip().upper()
                val = val.strip()
                try:
                    fval = float(val)
                except ValueError:
                    continue

                if key == 'LUX':
                    lux = fval
                elif key in ('H', 'HUM', 'HUMIDITY'):
                    hum = fval
                elif key in ('T', 'TEMP', 'TEMPERATURE'):
                    temp = fval

            if lux is not None:
                self.lux_pub.publish(Float32(data=float(lux)))
            if hum is not None:
                self.hum_pub.publish(Float32(data=float(hum)))
            if temp is not None:
                self.temp_pub.publish(Float32(data=float(temp)))
        else:
            # 其他訊息，先當作 debug
            self.get_logger().debug(f'Unknown frame: {line}')

    # -------------------------
    # 收到認證結果（auth_result topic）
    # -------------------------
    def auth_result_callback(self, msg: String):
        """
        msg.data 預期: "OK" 或 "FAIL"
        轉成 Serial 指令:
          AUTH,OK
          AUTH,FAIL
        """
        if self.ser is None or not self.ser.is_open:
            return

        if self.supervisor_blocked:
            self.get_logger().warn('Auth result ignored because system is supervisor-blocked.')
            result = 'FAIL'
        else:
            result = msg.data.strip().upper()

        if result not in ('OK', 'FAIL'):
            self.get_logger().warn(f'Unknown auth_result: {msg.data}')
            return

        cmd = f'AUTH,{result}\n'
        with self.serial_lock:
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.get_logger().info(f'Sent to Arduino: {cmd.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error (AUTH): {e}')

    # -------------------------
    # 收到馬達轉速（motor_speed topic）
    # -------------------------
    def motor_speed_callback(self, msg: Int32):
        """
        msg.data: 0 ~ 255
        轉成 Serial 指令:
          MOTOR,SPD=120
        """
        if self.ser is None or not self.ser.is_open:
            return

        spd = msg.data
        if spd < 0:
            spd = 0
        if spd > 255:
            spd = 255

        if self.supervisor_blocked:
            self.get_logger().warn('Motor speed request blocked by supervisor. Forcing 0.')
            spd = 0

        cmd = f'MOTOR,SPD={spd}\n'
        with self.serial_lock:
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.get_logger().info(f'Sent to Arduino: {cmd.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error (MOTOR): {e}')

    # -------------------------
    # Supervisor: block system toggle
    # -------------------------
    def supervisor_block_callback(self, msg: Bool):
        self.supervisor_blocked = bool(msg.data)
        if self.supervisor_blocked:
            self.get_logger().warn('Supervisor block enabled: stopping motor and denying auth.')
            # stop motor immediately
            stop_msg = Int32()
            stop_msg.data = 0
            self.supervisor_motor_override_callback(stop_msg)
        else:
            self.get_logger().info('Supervisor block disabled.')

    # -------------------------
    # Supervisor: direct motor override
    # -------------------------
    def supervisor_motor_override_callback(self, msg: Int32):
        if self.ser is None or not self.ser.is_open:
            return

        spd = msg.data
        if spd < 0:
            spd = 0
        if spd > 255:
            spd = 255

        # 監督者專用指令：不經過一般 AUTH 檢查
        cmd = f'SUP_MOTOR,SPD={spd}\n'
        with self.serial_lock:
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.get_logger().info(
                    f'Sent to Arduino (supervisor): {cmd.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Serial write error (SUP_MOTOR): {e}')

    # -------------------------
    # 定期向Arduino收取資料
    # -------------------------
    def request_sensor_timer_cb(self):
        """每秒送一個 REQ,SENSOR 給 Arduino，要最新感測資料。"""
        if self.ser is None or not self.ser.is_open:
            return

        cmd = 'REQ,SENSOR\n'
        with self.serial_lock:
            try:
                self.ser.write(cmd.encode('utf-8'))
                # debug：你要的話可以開 log
                # self.get_logger().debug(f'Sent: {cmd.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error (REQ,SENSOR): {e}')



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
