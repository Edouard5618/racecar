#!/usr/bin/env python

import rospy
import socket
import threading
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)
        self.broadcast_ip        = "255.255.255.255"
        self.listen_ip           = "192.168.137.4"

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.remote_request_loop)
        self.pos_track_thread = threading.Thread(target=self.pos_track_loop)

        print("ROSMonitor started.")

    def remote_request_loop(self):
        # Init your socket here :
        request_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # IPv4, TCP
        request_socket.bind((self.listen_ip, self.remote_request_port))
        request_socket.listen(1)
        try:
            conn, addr = request_socket.accept()
            print('Remote Request connected by', addr)
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                print("Received data: " + data.decode())
                if data is "RPOS":
                    conn.sendall((str(self.pos) + str(self.id)).encode())
                elif data is "OBSF":
                    conn.sendall(str(self.obstacle).encode())
                elif data is "RBID":
                    conn.sendall(str(self.id).encode())
                elif data is "EXIT":
                    break
        finally:
            conn.close()
            request_socket.close()
        

                
            
                        
                    

    def pos_track_loop(self):
        # Init your socket here :
        # request_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP
        # request_socket.bind((self.broadcast_ip, self.pos_broadcast_port))
        # request_socket.listen(1)
        # while True:
        #     conn, addr = request_socket.accept()
        #     with conn:
        #         print('Pos track connected by', addr)
        #         while True:   
        #             conn.sendall((str(self.pos) + str(self.id)).encode())
        #             time.sleep(1)  # Diffusion toutes les 1 seconde


    def odom_callback(self, msg):
        rospy.loginfo("odom_callback: ")
        rospy.loginfo(msg)

        # Transform the Odometry message into a tuple (x, y, yaw):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation))

        # Send the position to the remote client:
        # self.rr_socket.send(...)
        
        pass

    def quaternion_to_yaw(quat):
        # Uses TF transforms to convert a quaternion to a rotation angle around Z.
        # Usage with an Odometry message: 
        #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def laser_callback(self, msg):
        rospy.loginfo("laser_callback: " + str((msg.ranges)))

        if min(msg.ranges) < 1000: # 1m
            self.obstacle = True
        else:
            self.obstacle = False

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


