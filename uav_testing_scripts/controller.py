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

# For alignment of camera_frame to drone_frame(CG)
bodywidth_x =0
bodywidth_y =0
bodywidth_z =0

# Setpoint Topic to publish to
setpoint_topic="/uav1/mavros/setpoint_position/local"
local_position_topic="/uav1/mavros/local_position/pose"

# Update rate
rate = 1/10 #10 times every second

class AlignmentController(Node):

    def __init__(self):
        super().__init__('alignment_controller')

        self.camera_setpoint = PoseStamped()
        self.subscription = self.create_subscription(
            PoseStamped,
            'topic',
            self.camera_listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.local_setpoint = PoseStamped()
        self.local_setpoint_subscription = self.create_subscription(
            PoseStamped,
            local_position_topic,
            self.camera_listener_callback,
            10)
        self.local_setpoint_subscription  # prevent unused variable warning

        self.setpoint_publisher_ = self.create_publisher(PoseStamped, setpoint_topic, 10)
        self.rate = self.create_timer(rate, self.timer_callback)

    def camera_listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.camera_setpoint = msg

    def local_position_listener_callback(self, msg):
        self.local_setpoint = msg

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

    def caculate_offset(self, desired, current):
        desired = self.camera_setpoint
        current = self.local_setpoint


        

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
