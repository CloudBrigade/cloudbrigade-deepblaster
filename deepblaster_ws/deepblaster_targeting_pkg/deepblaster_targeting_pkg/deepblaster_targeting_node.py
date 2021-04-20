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
deepblaster_targeting_node.py

This module decides the action messages (servo control messages, and blaster control messages)
to be sent out using the detection deltas from object_detection_node.

The node defines:
    detection_delta_subscriber: A subscriber to the /object_detection_pkg/object_detection_delta
                                published by the deepblaster_object_detection_pkg with the normalized delta
                                of the detected object position from the target (reference) position
                                with respect to x and y axes.
    action_publisher: A publisher to publish the actions (servo angles and speed, blaster flywheel
spinup, and trigger values).
"""
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

from deepblaster_interfaces_pkg.msg import (BlasterCtrlMsg,
                                     DetectionDeltaMsg)
from deepblaster_targeting_pkg import (constants,
                                utils)


class DBTargetingNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
       and trigger) to be sent out using the detection deltas from object_detection_node.
    """

    def __init__(self, qos_profile):
        """Create a DBTargetingNode.
        """
        super().__init__('deepblaster_targeting_node')
        self.get_logger().info("deepblaster_targeting_node started.")

        # Double buffer to hold the input deltas in x and y from Object Detection.
        self.delta_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Create subscription to detection deltas from object_detection_node.
        self.detection_delta_subscriber = \
            self.create_subscription(DetectionDeltaMsg,
                                     constants.OBJECT_DETECTION_DELTA_TOPIC,
                                     self.detection_delta_cb,
                                     qos_profile)

        # Creating publisher to publish action (angle and trigger).
        self.action_publisher = self.create_publisher(BlasterCtrlMsg,
                                                      constants.ACTION_PUBLISH_TOPIC,
                                                      qos_profile)

        # Initializing the msg to be published.
        msg = BlasterCtrlMsg()
        msg.x_angle = constants.ActionValues.XDEFAULT
        msg.y_angle = constants.ActionValues.YDEFAULT
        msg.flywheel = constants.ActionValues.SAFE
        msg.trigger = constants.ActionValues.SAFE

        self.lock = threading.Lock()

        # Create a background servo publish thread.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.action_publish, args=(msg,))
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(f"Waiting for input delta: {constants.OBJECT_DETECTION_DELTA_TOPIC}")

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

    def detection_delta_cb(self, detection_delta):
        """Call back for whenever detection delta for a perception
           is received from object_detection_node.

        Args:
            detection_delta (DetectionDeltaMsg): Message containing the normalized detection
                                                       delta in x and y axes respectively passed as
                                                       a list.
        """

        self.delta_buffer.put(detection_delta)

    def plan_action(self, delta):
        """Helper method to calculate action to be undertaken from the detection delta
           received from object_detection_node.

        Args:
        delta (list of floats): detection deltas in x and y axes respectively.

        Returns:
        (int): Action Space Category defined in constants.py
        """
        delta_x = delta[0]
        delta_y = delta[1]

        # Delta_Y could be used to determine when we are close enough to fire blaster
        if delta_y < constants.DeltaValueMap.FIRE_DELTA_Y:
            # Fire!
            return constants.ACTION_SPACE[7][constants.ActionSpaceKeys.CATEGORY]

        # elif delta_y >= constants.DeltaValueMap.FIRE_DELTA_Y:
        # Forward Bracket
        if delta_x < constants.DeltaValueMap.SLOW_LEFT_DELTA_X \
                and delta_x > constants.DeltaValueMap.FAST_LEFT_DELTA_X:
            # Slow Left
            return constants.ACTION_SPACE[2][constants.ActionSpaceKeys.CATEGORY]
        elif delta_x <= constants.DeltaValueMap.FAST_LEFT_DELTA_X:
            # Fast Left
            return constants.ACTION_SPACE[3][constants.ActionSpaceKeys.CATEGORY]
        elif delta_x > constants.DeltaValueMap.SLOW_RIGHT_DELTA_X \
                and delta_x < constants.DeltaValueMap.FAST_RIGHT_DELTA_X:
            # Slow Right
            return constants.ACTION_SPACE[4][constants.ActionSpaceKeys.CATEGORY]
        elif delta_x >= constants.DeltaValueMap.FAST_RIGHT_DELTA_X:
            # Fast Right
            return constants.ACTION_SPACE[5][constants.ActionSpaceKeys.CATEGORY]
        else:
            # No Action
            return constants.ACTION_SPACE[1][constants.ActionSpaceKeys.CATEGORY]

    def get_mapped_action(self, action_category):
        """Return the angle and trigger values to be published for servo.

        Args:
            action_category (int): Integer value corresponding to the action space category.

        Returns:
            x_angle (float): Angle value to be published to servo.
            y_angle (float): Angle value to be published to servo.
            flywheel (int): Trigger value to be published to relay.
            trigger (int): Trigger value to be published to relay.
        """
        action = constants.ACTION_SPACE[action_category][constants.ActionSpaceKeys.ACTION]
        self.get_logger().info(action)
        x_angle = constants.ACTION_SPACE[action_category][constants.ActionSpaceKeys.XANGLE]
        y_angle = constants.ACTION_SPACE[action_category][constants.ActionSpaceKeys.YANGLE]

        # if object is detected, we should spinup flywheels in preparation for firing
        # flywheel = constants.ACTION_SPACE[action_category][constants.ActionSpaceKeys.FLYWHEEL]
        # else
        flywheel = constants.ActionValues.SAFE

        # if object detection x_delta is near zero, and flywheels are spinning, fire!
        # trigger = constants.ACTION_SPACE[action_category][constants.ActionSpaceKeys.TRIGGER]
        # else
        trigger = constants.ActionValues.SAFE
        return x_angle, y_angle, flywheel, trigger

    def action_publish(self, msg):
        """Function which runs in a separate thread to read object detection delta
           from double buffer, decides the action and sends it to Blaster Control node.

        Args:
            msg: detection_delta (DetectionDeltaMsg): Message containing the normalized
                 detection delta in x and y axes respectively passed as a list.
        """
        try:
            while not self.stop_thread:
                # Get a new message to plan action on
                detection_delta = self.delta_buffer.get()
                action_category = self.plan_action(detection_delta.delta)
                msg.x_angle, msg.y_angle, msg.flywheel, msg.trigger = self.get_mapped_action(action_category)
                # Publish msg based on action planned and mapped from a new object detection.
                self.action_publisher.publish(msg)

                # Sleep for a default amount of time before checking if new data is available.
                time.sleep(constants.DEFAULT_SLEEP)
                # If new data is not available within default time, gracefully run blind.
                while self.delta_buffer.is_empty() and not self.stop_thread:
                    msg.x_angle, msg.y_angle, msg.flywheel, msg.trigger = self.get_mapped_action(action_category)
                    # @TODO Return x_angle value to 90 degrees
                    # msg.x_angle = msg.x_angle / 2
                    # Publish blind action
                    self.action_publisher.publish(msg)
                    # Sleep before checking if new data is available.
                    time.sleep(0.1)
        except Exception as ex:
            self.get_logger().error(f"Failed to publish action to blaster control topic: {ex}")
            # Stop the car
            msg.x_angle, msg.y_angle, msg.flywheel, msg.trigger = constants.ActionValues.XDEFAULT, constants.ActionValues.YDEFAULT, constants.ActionValues.SAFE, constants.ActionValues.SAFE
            self.action_publisher.publish(msg)
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

    try:
        deepblaster_targeting_node = DBTargetingNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number.
                frame: the current stack frame (None or a frame object).
            """
            deepblaster_targeting_node.get_logger().info("Signal Handler initiated")
            deepblaster_targeting_node.thread_shutdown()
            deepblaster_targeting_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.spin(deepblaster_targeting_node, executor)
    except Exception as ex:
        deepblaster_targeting_node.get_logger().error(f"Exception in DBTargetingNode: {ex}")
        deepblaster_targeting_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    deepblaster_targeting_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
