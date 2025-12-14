#!/usr/bin/env python3
import json
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class SupervisorNode(Node):
    """Node that handles high-privilege supervisor commands.

    Supported commands (case-insensitive):
      - BLOCK_SYSTEM: stop the motor and block logins.
      - UNBLOCK_SYSTEM: clear the block state.
      - ADD_TEMP_USER <uid> <password>: register a temporary user with a timeout.
      - SUPER_PWM <value>: directly set the motor PWM (0-255) without login.
    """

    def __init__(self):
        super().__init__('supervisor_node')

        self.temp_user_timeout_sec = self.declare_parameter(
            'temp_user_timeout_sec', 600
        ).get_parameter_value().integer_value
        self.status_period_sec = self.declare_parameter(
            'status_period_sec', 1.0
        ).get_parameter_value().double_value

        self.blocked = False
        self.temp_users: Dict[str, float] = {}
        self.last_pwm = 0
        self.last_action = 'INIT'

        self.block_pub = self.create_publisher(Bool, 'supervisor_block', 10)
        self.motor_override_pub = self.create_publisher(Int32, 'supervisor_motor_override', 10)
        self.temp_user_pub = self.create_publisher(String, 'temporary_user', 10)
        self.status_pub = self.create_publisher(String, 'supervisor_status', 10)

        self.cmd_sub = self.create_subscription(
            String, 'supervisor_cmd', self.command_callback, 10
        )

        self.status_timer = self.create_timer(self.status_period_sec, self.publish_status)

        self.get_logger().info('Supervisor node started.')

    # -------------------------
    # Command handling
    # -------------------------
    def command_callback(self, msg: String):
        raw = msg.data.strip()
        upper = raw.upper()

        if upper.startswith('BLOCK_SYSTEM'):
            self._handle_block(True)
            return

        if upper.startswith('UNBLOCK_SYSTEM'):
            self._handle_block(False)
            return

        if upper.startswith('ADD_TEMP_USER'):
            self._handle_add_temp_user(raw[len('ADD_TEMP_USER'):].strip())
            return

        if upper.startswith('SUPER_PWM'):
            self._handle_super_pwm(raw[len('SUPER_PWM'):].strip())
            return

        self.get_logger().warn(f'Unknown supervisor command: {raw}')
        self.last_action = f'UNKNOWN:{raw}'

    def _handle_block(self, should_block: bool):
        self.blocked = should_block
        msg = Bool()
        msg.data = should_block
        self.block_pub.publish(msg)

        if should_block:
            self.last_pwm = 0
            self._publish_motor_override(0)
            self.last_action = 'BLOCKED_SYSTEM'
            self.get_logger().info('System blocked: motor stopped and logins disabled.')
        else:
            self.last_action = 'UNBLOCKED_SYSTEM'
            self.get_logger().info('System unblocked: logins allowed.')

    def _handle_add_temp_user(self, payload: str):
        # payload may be "UID PASSWORD" or "UID,PASSWORD"
        normalized = payload.replace(',', ' ').split()
        if len(normalized) < 2:
            self.get_logger().error(
                'ADD_TEMP_USER requires UID and PASSWORD separated by space or comma.'
            )
            self.last_action = 'ADD_TEMP_USER_FAILED'
            return

        uid, password = normalized[0], normalized[1]
        expire_at = time.time() + self.temp_user_timeout_sec
        self.temp_users[uid] = expire_at

        msg = String()
        # msg.data = json.dumps({
        #     'uid': uid,
        #     'password': password,
        #     'expire_at': expire_at,
        # })
        msg.data = f"{uid}:{password}"
        self.temp_user_pub.publish(msg)

        self.last_action = f'ADDED_TEMP_USER:{uid}'
        self.get_logger().info(
            f'Registered temporary user {uid} valid for {self.temp_user_timeout_sec}s.'
        )

    def _handle_super_pwm(self, payload: str):
        try:
            value = int(payload.split()[0])
        except (ValueError, IndexError):
            self.get_logger().error('SUPER_PWM requires an integer value (0-255).')
            self.last_action = 'SUPER_PWM_FAILED'
            return

        if value < 0:
            value = 0
        if value > 255:
            value = 255

        self.last_pwm = value
        self._publish_motor_override(value)
        self.last_action = f'SUPER_PWM:{value}'
        self.get_logger().info(f'Supervisor set motor PWM to {value}.')

    def _publish_motor_override(self, value: int):
        msg = Int32()
        msg.data = value
        self.motor_override_pub.publish(msg)

    # -------------------------
    # Status publishing
    # -------------------------
    def publish_status(self):
        self._cleanup_temp_users()
        status = {
            'blocked': self.blocked,
            'active_temp_users': list(self.temp_users.keys()),
            'last_pwm': self.last_pwm,
            'last_action': self.last_action,
            'timestamp': time.time(),
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _cleanup_temp_users(self):
        now = time.time()
        expired = [uid for uid, ts in self.temp_users.items() if ts < now]
        for uid in expired:
            del self.temp_users[uid]
            self.get_logger().info(f'Removed expired temporary user {uid}.')


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
