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

ACTION_PUBLISH_TOPIC = "deepblaster_target"

OBJECT_DETECTION_PKG_NS = "/deepblaster_object_detection_pkg"
OBJECT_DETECTION_DELTA_TOPIC = f"{OBJECT_DETECTION_PKG_NS}/object_detection_delta"


class DeltaValueMap():
    """Class with the delta values mapping to action brackets
       Impact of the deltas on actions can be understood from the README.
       TODO: Link the README here.

       Experiment results with object placed at different positions wrt camera:

       Left/Right   Obj Delta      Range [0-1]
       ----------------------------------------
       45 cm      |              |
       Left       | delta_x      |   -0.33
       15 cm      |              |
       Left       | delta_x      |   -0.13
       0 cm       |              |
       Left/Right | delta_x      |      0
       15 cm      |              |
       Right      | delta_x      |    0.13
       45 cm      |              |
       Right      | delta_x      |    0.33

    """
    SHORT_RIGHT_DELTA_X = 0.13
    FAR_RIGHT_DELTA_X = 0.33
    SHORT_LEFT_DELTA_X = -0.13
    FAR_LEFT_DELTA_X = -0.33
    FIRE_DELTA_X = 0.0


class ActionSpaceKeys():
    """Class with keys for the action space.
    """
    ACTION = "action"
    XANGLE = "xangle"
    YANGLE = "yangle"
    FLYWHEEL = "flywheel"
    CATEGORY = "category"


class ActionValues():
    """Class with the PWM values with respect to
       the possible actions that can be sent to servo, pertaining to
       the angle and throttle.
    """
    FAR_LEFT = 68
    SHORT_LEFT = 88
    FAR_RIGHT = 108
    SHORT_RIGHT = 128
    SPINUP = 1
    FIRE = 1
    SAFE = 0
    DEFAULT = 0
    XDEFAULT = 98
    YDEFAULT = 90


# Action Space configuration.
ACTION_SPACE = {
    1: {
        ActionSpaceKeys.ACTION: "No Action",
        ActionSpaceKeys.XANGLE: ActionValues.XDEFAULT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.FLYWHEEL: ActionValues.SAFE,
        ActionSpaceKeys.CATEGORY: 1
    },
    2: {
        ActionSpaceKeys.ACTION: "Short Left",
        ActionSpaceKeys.XANGLE: ActionValues.SHORT_LEFT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.FLYWHEEL: ActionValues.SPINUP,
        ActionSpaceKeys.CATEGORY: 2
    },
    3: {
        ActionSpaceKeys.ACTION: "Far Left",
        ActionSpaceKeys.XANGLE: ActionValues.FAR_LEFT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.FLYWHEEL: ActionValues.SPINUP,
        ActionSpaceKeys.CATEGORY: 3
    },
    4: {
        ActionSpaceKeys.ACTION: "Short Right",
        ActionSpaceKeys.XANGLE: ActionValues.SHORT_RIGHT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.FLYWHEEL: ActionValues.SPINUP,
        ActionSpaceKeys.CATEGORY: 4
    },
    5: {
        ActionSpaceKeys.ACTION: "Far Right",
        ActionSpaceKeys.XANGLE: ActionValues.FAR_RIGHT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.FLYWHEEL: ActionValues.SPINUP,
        ActionSpaceKeys.CATEGORY: 5
    }
}

# Default value to sleep for in sec.
DEFAULT_SLEEP = 0.08
