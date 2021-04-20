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

ACTION_PUBLISH_TOPIC = "deepblaster_ctrl"
#SET_MAX_SPEED_SERVICE_NAME = "set_max_speed"

DB_TARGETING_PKG_NS = "/deepblaster_targeting_pkg"
DB_TARGETING_TOPIC = f"{DB_TARGETING_PKG_NS}/deepblaster_target"

SERIAL_PORT = "/dev/ttyACM0"

class BlasterCtrlValueMap():
    """ Map of values for serial communication with Blaster Controller
    """
    HELLO = 0
    XSERVO = 1
    YSERVO = 2
    FLYWHEEL = 3
    TRIGGER = 4
    ALREADY_CONNECTED = 5
    ERROR = 6
    RECEIVED = 7
    STOP = 8
    MOTOR = 9

class ActionValues():
    """Class with the PWM values with respect to
       the possible actions that can be sent to servo, pertaining to
       the angle and throttle.
    """
    FAST_LEFT = 80
    SLOW_LEFT = 90
    FAST_RIGHT = 110
    SLOW_RIGHT = 120
    SPINUP = 1
    FIRE = 1
    SAFE = 0
    DEFAULT = 0
    XDEFAULT = 98
    YDEFAULT = 90

# Default value to sleep for in sec.
DEFAULT_SLEEP = 0.08
