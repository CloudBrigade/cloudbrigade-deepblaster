#################################################################################
#   Copyright Cloud Brigade, ScratchSpace, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
deepblaster_ctrl_node.py

This module decides the action messages (servo control messages specifically angle
and trigger) to be sent out using the detection deltas from object_detection_node.

The node defines:
    detection_delta_subscriber: A subscriber to the /deepblaster_object_detection_pkg/object_detection_delta
                                published by the deepblaster_object_detection_pkg with the normalized delta
                                of the detected object position from the target (reference) position
                                with respect to x and y axes.
    The node defines:
    action_publisher: A publisher to publish the action (angle and trigger values).
"""
from __future__ import print_function, division, absolute_import
import time
import signal
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)

from deepblaster_interfaces_pkg.msg import (BlasterCtrlMsg)
from deepblaster_ctrl_pkg import (constants,
                                utils)

from robust_serial import write_order, Order, write_i8, write_i16, read_i8, read_order
from robust_serial.utils import open_serial_port


class DBBlasterCtrlNode(Node):
    """Node responsible for signalling the blaster control module to aim and fire the blaster
    """

    def __init__(self, qos_profile):
        """Create a DBBlasterCtrlNode.
        """
        super().__init__('deepblaster_ctrl_node')
        self.get_logger().info("deepblaster_ctrl_node started.")
        self.connect_to_controller()

        # Double buffer to hold the input deltas in x and y from Object Detection.
        self.targeting_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Create subscription to detection deltas from deepblaster_targeting_node.
        self.deepblaster_targeting_subscriber = \
            self.create_subscription(BlasterCtrlMsg,
                                     constants.DB_TARGETING_TOPIC,
                                     self.deepblaster_targeting_cb,
                                     qos_profile)

        ### @TODO # Creating publisher to publish action (servo angles, flywheel, and trigger).
        ### self.action_publisher = self.create_publisher(ServoCtrlMsg,
        ###                                               constants.ACTION_PUBLISH_TOPIC,
        ###                                               qos_profile)

        # Initializing the msg to be published.
        msg = BlasterCtrlMsg()
        msg.x_angle = constants.ActionValues.XDEFAULT
        msg.y_angle = constants.ActionValues.YDEFAULT
        msg.flywheel = constants.ActionValues.SAFE
        msg.trigger = constants.ActionValues.SAFE

        self.lock = threading.Lock()

        # Create a background Blaster transmit thread.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.action_transmit, args=(msg,))
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(f"Waiting for input data: {constants.DB_TARGETING_TOPIC}")

    def wait_for_thread(self):
        """Function which joins the created background thread.
        """
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread.
        """
        self.stop_thread = True

    def connect_to_controller(self):
        self.serial_file.close()

    def connect_to_controller(self):
        try:
            self.serial_file = open_serial_port(serial_port=constants.SERIAL_PORT, baudrate=115200, timeout=None)
            if(self.serial_file.isOpen() == False):
                self.get_logger().info("Serial port is closed, opening now...")
                self.serial_file.open()
        except Exception as ex:
            self.get_logger().error(f"Failed to connect to Blaster Controller: {ex}")
            self.serial_file.close()

        try:
            is_connected = False
            serial_timeout = 0
            # Initialize communication with Blaster Controller
            while not is_connected:
                serial_timeout += 1
                if (serial_timeout > 5):
                     self.serial_file.close()
                     raise Exception
                self.get_logger().info("Waiting for Blaster Controller")
                write_order(self.serial_file, Order.HELLO)
                bytes_array = bytearray(self.serial_file.read(1))
                if not bytes_array:
                    time.sleep(2)
                    continue
                byte = bytes_array[0]
                if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
                    is_connected = True
        except Exception as ex:
            self.get_logger().error(f"Blaster Controller not responding: {ex}")
            self.serial_file.close()

        self.get_logger().info("Blaster Controller Connected")

    def deepblaster_targeting_cb(self, deepblaster_targeting):
        """Call back for whenever targeting instructions
           are received from deepblaster_targeting_node.

        Args:
            deepblaster_targeting (BlasterCtrlMsg): Message containing the servo control degrees
                                           and flywheel/trigger commands
        """

        self.get_logger().info(f"Received Values: {deepblaster_targeting}")
        self.targeting_buffer.put(deepblaster_targeting)

    def action_transmit(self, msg):
        self.get_logger().info(f"action_transmit thread started")

        """Function which runs in a separate thread to read Blaster messages
           from double buffer, decides the action and sends it to Blaster Controller.

        Args:
            msg: deepblaster_targeting (BlasterCtrlMsg): Message containing the normalized
                 servo angles in x and y axes, and flywheel and trigger values respectively passed as a list.
        """
        try:
            while not self.stop_thread:
                # Get a new message to plan action on
                # deepblaster_interfaces_pkg.msg.BlasterCtrlMsg(x_angle=80.0, y_angle=80.0, flywheel=0.0, trigger=0.0)
                deepblaster_targeting = self.targeting_buffer.get()
                msg.x_angle, msg.y_angle, msg.flywheel, msg.trigger = deepblaster_targeting.x_angle, deepblaster_targeting.y_angle, deepblaster_targeting.flywheel, deepblaster_targeting.trigger
                self.get_logger().info(f"Received Values x_servo:{msg.x_angle}, y_servo:{msg.y_angle}, flywheel:{msg.flywheel}, trigger:{msg.trigger}")
                write_order(self.serial_file, Order.XSERVO)
                write_i16(self.serial_file, msg.x_angle)
                order = read_order(self.serial_file)
                self.get_logger().info("Ordered received: {order}")

                # @TODO Publish msg based on what was sent or received from Blaster Controller
                # self.action_publisher.publish(msg)

                # Sleep for a default amount of time before checking if new data is available.
                time.sleep(constants.DEFAULT_SLEEP)
                # If new data is not available within default time, gracefully run blind.
                while self.targeting_buffer.is_empty() and not self.stop_thread:
                    msg.x_angle, msg.y_angle, msg.flywheel, msg.trigger = constants.ActionValues.XDEFAULT, constants.ActionValues.YDEFAULT, constants.ActionValues.SAFE, constants.ActionValues.SAFE
                    # Sleep before checking if new data is available.
                    time.sleep(0.1)
        except Exception as ex:
            self.get_logger().error(f"Failed to transmit action to Blaster Controller: {ex}")
            # Center the Blaster
            msg.x_angle, msg.y_angle, msg.flywheel, msg.trigger = constants.ActionValues.XDEFAULT, constants.ActionValues.YDEFAULT, constants.ActionValues.SAFE, constants.ActionValues.SAFE
            # Destroy the ROS Node running in another thread as well.

def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

    try:
        deepblaster_ctrl_node = DBBlasterCtrlNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number.
                frame: the current stack frame (None or a frame object).
            """
            deepblaster_ctrl_node.get_logger().info("Signal Handler initiated")
            deepblaster_ctrl_node.thread_shutdown()
            deepblaster_ctrl_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.spin(deepblaster_ctrl_node, executor)
    except Exception as ex:
        deepblaster_ctrl_node.get_logger().error(f"Exception in DBBlasterCtrlNode: {ex}")
        deepblaster_ctrl_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    deepblaster_ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
