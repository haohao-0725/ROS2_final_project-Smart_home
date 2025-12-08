#!/usr/bin/env python3
"""
auth_node.py
白名單 / 密碼驗證邏輯
"""
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AuthNode(Node):
    def __init__(self):
        super().__init__('auth_node')

        # 宣告參數：白名單檔案路徑
        self.declare_parameter('whitelist_file', '')

        # 載入白名單
        self.whitelist = self._load_whitelist()

        self.last_uid = None

        # Subscribers
        self.rfid_sub = self.create_subscription(
            String,
            'rfid_uid',
            self. rfid_callback,
            10
        )

        self.pwd_sub = self. create_subscription(
            String,
            'password_attempt',
            self. password_callback,
            10
        )

        self.logout_sub = self. create_subscription(
            String,
            'logout_request',
            self.logout_callback,
            10
        )

        # Publishers
        self.auth_result_pub = self. create_publisher(String, 'auth_result', 10)
        self.auth_state_pub = self. create_publisher(String, 'auth_state', 10)

        self.get_logger().info('auth_node started.')

    def _load_whitelist(self):
        """從 JSON 檔案載入白名單"""
        whitelist_file = self.get_parameter('whitelist_file'). get_parameter_value().string_value

        # 如果沒有指定檔案路徑，嘗試預設路徑
        if not whitelist_file:
            possible_paths = [
                os.path. join(os.path.dirname(__file__), 'whitelist.json'),
                os.path.join(os.path. dirname(__file__), '..', 'config', 'whitelist.json'),
                os.path.expanduser('~/ros2_ws/src/smart_home_rfid/config/whitelist.json'),
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    whitelist_file = path
                    break

        if whitelist_file and os.path.exists(whitelist_file):
            try:
                with open(whitelist_file, 'r', encoding='utf-8') as f:
                    whitelist = json.load(f)
                self.get_logger(). info(
                    f'Loaded whitelist from {whitelist_file} with {len(whitelist)} entries.'
                )
                return whitelist
            except (json.JSONDecodeError, IOError) as e:
                self.get_logger().error(f'Failed to load whitelist file: {e}')
                return {}
        else:
            self.get_logger().warn('Whitelist file not found, using empty whitelist.')
            return {}

    def rfid_callback(self, msg):
        """收到最新 RFID UID"""
        self.last_uid = msg.data. strip()
        self.get_logger(). info(f'New card presented, UID={self.last_uid}')
        
        # 更新狀態給 Web UI
        state = String()
        state. data = f'WAIT_PWD,UID={self.last_uid}'
        self.auth_state_pub.publish(state)

    def password_callback(self, msg):
        """收到密碼嘗試"""
        password = msg.data. strip()
        self.get_logger(). info(f'Password attempt: "{password}"')

        # 沒有最近刷卡
        if self.last_uid is None:
            self.get_logger().warn('No recent UID, cannot check password.')
            self._publish_auth('FAIL', 'NO_UID')
            return

        # 查白名單
        uid = self.last_uid
        if uid not in self. whitelist:
            self.get_logger().warn(f'UID {uid} not in whitelist.')
            self._publish_auth('FAIL', 'UID_NOT_FOUND')
            return

        record = self.whitelist[uid]
        if password == record['password']:
            self.get_logger().info(f'Auth OK for UID={uid}, user={record["name"]}')
            self._publish_auth('OK', 'OK')
        else:
            self.get_logger(). warn(f'Wrong password for UID={uid}')
            self._publish_auth('FAIL', 'WRONG_PASSWORD')

    def logout_callback(self, msg):
        """收到登出請求"""
        self.get_logger(). info('Logout request received.')
        self. last_uid = None
        self._publish_auth('LOGOUT', 'USER_LOGOUT')

    def _publish_auth(self, result, reason):
        """發布認證結果到 /auth_result & /auth_state"""
        # 給 Arduino 的 topic
        msg_result = String()
        msg_result.data = result
        self.auth_result_pub. publish(msg_result)

        # 給 Web 的狀態
        msg_state = String()
        uid_str = self.last_uid if self.last_uid else "NONE"
        msg_state.data = f'AUTH_{result},REASON={reason},UID={uid_str}'
        self.auth_state_pub.publish(msg_state)


def main(args=None):
    rclpy.init(args=args)
    node = AuthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node. destroy_node()
        rclpy. shutdown()


if __name__ == '__main__':
    main()