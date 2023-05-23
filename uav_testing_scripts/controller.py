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
from std_msgs.msg import String
import numpy as np

# For alignment of camera_frame to drone_frame(CG)
bodywidth_x =0
bodywidth_y =0
bodywidth_z =0

# Camera Topic for desired setpoint
camera_setpoint_topic="/camera_setpoint"

# Setpoint Topic to publish to
setpoint_topic="/mavros/setpoint_position/local"
local_position_topic="/mavros/local_position/pose"

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
        # self.eul_deg=rad2deg(quat2eul(rb(i).Quaternion,"XYZ"))
        # self.eul_rad= quat2eul(rb(i).Quaternion,"XYZ")
        self.caculate_offset(self.camera_setpoint,self.local_position) # caculate for a new controller input everytime the UAV moves

    def timer_callback(self):
        msg = PoseStamped()
        # msg.data = 'Hello World: %d' % "a"
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        msg.pose.position.x= 0
        msg.pose.position.y= 0
        msg.pose.position.z= 0
        msg.pose.orientation.w = 1
        self.setpoint_publisher_.publish(msg)

    def caculate_offset(self, setpoint, current):
        # Error = Setpoint - Feedback
        self.error = np.subtract(np.array([setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z]),np.array([current.pose.position.x, current.pose.position.y, current.pose.position.z]) )
        # Derivative error = Error - error_past
        self.derivative_error = self.error - self.error_past

        input_z = (self.kp_z * self.error[2] * 1) + (self.kd_z * self.derivative_error[2])
        input_x = ((self.kp_x * self.error[0] * 1) + (self.kd_x * self.derivative_error[0]))
        input_y = ((self.kp_y * self.error[1] * 1) + (self.kd_y * self.derivative_error[1]))

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
