# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
import numpy as np
from transforms3d import quaternions,euler
from math import degrees, radians

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_x =0.1 # +ve is forward
cameratobody_y =0 # +ve is left
cameratobody_z =0 # +ve is up 

# Camera Topic for desired setpoint
camera_setpoint_topic="/camera_setpoint"

# Setpoint Topic to publish to
setpoint_topic="/mavros/setpoint_position/local"
local_position_topic="/mavros/local_position/pose"

# Rear Thruster Topic
thruster_output_topic="/thruster/pwm"

# Update rate
rate = 1/10 #10 times every second

class AlignmentController(Node):

    def __init__(self):
        super().__init__('alignment_controller')

        self.camera_setpoint = PoseStamped()
        self.subscription = self.create_subscription(
            PoseStamped,
            camera_setpoint_topic,
            self.camera_listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.local_position = PoseStamped()
        self.local_position_subscription = self.create_subscription(
            PoseStamped,
            local_position_topic,
            self.local_position_listener_callback,
            10)
        self.local_position_subscription  # prevent unused variable warning
        self.eul_deg=np.array([0,0,0])
        self.rad_deg=np.array([0,0,0,0])

        self.setpoint_publisher_ = self.create_publisher(PoseStamped, setpoint_topic, 10)
        # self.rate = self.create_timer(rate, self.timer_callback)
        self.thruster_publisher_ = self.create_publisher(PoseStamped, thruster_output_topic, 10)

        # Controller variables
        self.kp_x = 0.5
        self.kd_x = 0.125
        self.kp_y = 0.25
        self.kd_y = 0.0625
        self.kp_z = 0.2
        self.kd_z = 0.8
        self.error_past = np.array([0,0,0])
        self.feedback = np.array([self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z]) # FOR REFERENCE ONLY, DELETE LATER

    def camera_listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.camera_setpoint = msg

    def local_position_listener_callback(self, msg):
        self.local_setpoint = msg

        # if self.camera_setpoint.pose.position.z != 0:
        #     self.controller(self.camera_setpoint,self.local_position) # caculate for a new controller input everytime the UAV moves
        # else:
        #     print("Zero-z setpoint rejected")
        self.controller(self.camera_setpoint,self.local_position)

    # Called only for debugging
    def timer_callback(self):
        msg = PoseStamped()
        # msg.data = 'Hello World: %d' % "a"
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        msg.pose.position.x= 0.0
        msg.pose.position.y= 0.0
        msg.pose.position.z= 1.2
        msg.pose.orientation.z = -0.88
        msg.pose.orientation.w = -0.468
        # self.setpoint_publisher_.publish(msg)

    def controller(self, setpoint, current):

        current_yaw=euler.quat2euler([current.pose.orientation.w,current.pose.orientation.x,current.pose.orientation.y,current.pose.orientation.z]) #wxyz default
        setpoint_yaw=euler.quat2euler([setpoint.pose.orientation.w,setpoint.pose.orientation.x,setpoint.pose.orientation.y,setpoint.pose.orientation.z]) #wxyz default

        #Perform transformation of camera setpoint wrt to body
        self.setpoint.pose.position.x=self.setpoint.pose.position.x-cameratobody_x
        self.setpoint.pose.position.y=self.setpoint.pose.position.x-cameratobody_y
        self.setpoint.pose.position.z=self.setpoint.pose.position.x-cameratobody_z

        #Jog the UAV towards the setpoint
        if abs(self.setpoint.pose.position.x - self.current.pose.position.x) < 0.2 and abs(self.setpoint.pose.position.y-self.current.pose.position.y) < 0.2 and degrees(abs(setpoint_yaw[2]-current_yaw[2])) < 10:
            self.thruster_publisher_.publish(10) # Change to output PWM
        #Else, move towards setpoint with controller
        else:
            self.alignment_controller(self.camera_setpoint,self.local_position)

    def alignment_controller(self, setpoint, current):
        # Error = Setpoint - Feedback
        self.error = np.subtract(np.array([setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z]),np.array([current.pose.position.x, current.pose.position.y, current.pose.position.z]) )
        # Derivative error = Error - error_past
        self.derivative_error = self.error - self.error_past

        input_x = ((self.kp_x * self.error[0] * 1) + (self.kd_x * self.derivative_error[0]))
        input_y = ((self.kp_y * self.error[1] * 1) + (self.kd_y * self.derivative_error[1]))
        input_z = (self.kp_z * self.error[2] * 1) + (self.kd_z * self.derivative_error[2])

        msg = PoseStamped()
        msg.pose.position.x= input_x
        msg.pose.position.y= input_y
        msg.pose.position.z= input_z
        msg.pose.orientation.w = 1
        self.setpoint_publisher_.publish(msg)

        self.error_past = self.error
        

def main(args=None):
    rclpy.init(args=args)

    alightment_controller = AlignmentController()

    rclpy.spin(alightment_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    alightment_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
