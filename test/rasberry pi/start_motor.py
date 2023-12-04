from typing import Set
import rclpy
import threading
import subprocess
import os

from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import Sync4SetPosition
from geometry_msgs.msg import Twist

import socket
import time


class MinimalPublisher(Node):

    def __init__(self):
        #make publisher
        super().__init__('minimal_publisher')
        self.positionPublisher_ = self.create_publisher(Sync4SetPosition, 'sync4_set_position', 10)

        #parameters
        #開放時の値
        self.initPos1 = 3000
        self.initPos2 = 3000 
        self.initPos3 = 1000
        self.initPos4 = 1000 
        #ロック時の値
        self.lockPos1 = 2000
        self.lockPos2 = 2000 
        self.lockPos3 = 2000
        self.lockPos4 = 2000 
        #完全ロック時の値
        self.comLockPos1 = 1000
        self.comLockPos2 = 1000 
        self.comLockPos3 = 3000
        self.comLockPos4 = 3000 

        dynamixel_msg = Sync4SetPosition() #make msg
        dynamixel_msg.id1 = 10
        dynamixel_msg.id2 = 11
        dynamixel_msg.id3 = 12
        dynamixel_msg.id4 = 13


def LockStatus(self):
    dynamixel_msg = Sync4SetPosition()
    dynamixel_msg.id1 = 10
    dynamixel_msg.id2 = 11
    dynamixel_msg.id3 = 12
    dynamixel_msg.id4 = 13
    #全部解放
    dynamixel_msg.position1 = self.initPos1
    dynamixel_msg.position2 = self.initPos2
    dynamixel_msg.position3 = self.initPos3
    dynamixel_msg.position4 = self.initPos4
    self.positionPublisher_.publish(dynamixel_msg)
    self.positionPublisher_.publish(dynamixel_msg)
    time.sleep(5)
    self.positionPublisher_.publish(dynamixel_msg)
    self.get_logger().info('id: "%s"' % dynamixel_msg.id1)
    self.get_logger().info('position: "%s"' % dynamixel_msg.position1)
    self.get_logger().info('id: "%s"' % dynamixel_msg.id2)
    self.get_logger().info('position: "%s"' % dynamixel_msg.position2)
    self.get_logger().info('id: "%s"' % dynamixel_msg.id3)
    self.get_logger().info('position: "%s"' % dynamixel_msg.position3)
    self.get_logger().info('id: "%s"' % dynamixel_msg.id4)
    self.get_logger().info('position: "%s"' % dynamixel_msg.position4)

def motorControl():
    # 送信先のIPアドレスとポート番号
    send_ip = "192.168.179.2"  # ESP32のIPアドレス 192.168.179.3
    send_port = 5002

    # ソケットを作成
    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 送信するデータ
    message = "0"

    # データを送信
    send_socket.sendto(message.encode(), (send_ip, send_port))
    print("send message 0")

    # ソケットを閉じる（通常は実際のアプリケーションでは適切なタイミングで閉じるべきです）
    send_socket.close()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    LockStatus(minimal_publisher)
    time.sleep(5)
    motorControl()

    # rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()