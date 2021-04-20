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

                                       <-Reverse->  <-NoAct->  <-Forward->

       LR/        |   Ahead->        30 cm    60 cm   90 cm   120 cm  150 cm
       --------------------------------------------------------------------
       45 cm      |              |
       Left       | delta_x      |    N/A     -0.37   -0.33   -0.25   -0.22
                  | delta_y      |    N/A     -0.06   -0.11   -0.14   -0.15
       15 cm      |              |
       Left       | delta_x      |   -0.25    -0.20   -0.13   -0.12   -0.09
                  | delta_y      |    0.11    -0.06   -0.11   -0.14   -0.15
       0 cm       |              |
       Left/Right | delta_x      |       0        0       0       0       0
                  | delta_y      |    0.11    -0.06   -0.11   -0.14   -0.15
       15 cm      |              |
       Right      | delta_x      |    0.25     0.20    0.13    0.12    0.09
                  | delta_y      |    0.11    -0.06   -0.11   -0.14   -0.15
       45 cm      |              |
       Right      | delta_x      |    N/A      0.37    0.33    0.25    0.22
                  | delta_y      |    N/A     -0.06   -0.11   -0.14   -0.15

    """
    SLOW_RIGHT_DELTA_X = 0.13
    FAST_RIGHT_DELTA_X = 0.33
    SLOW_LEFT_DELTA_X = -0.13
    FAST_LEFT_DELTA_X = -0.33
    FIRE_DELTA_Y = 0.0


class ActionSpaceKeys():
    """Class with keys for the action space.
    """
    ACTION = "action"
    XANGLE = "xangle"
    YANGLE = "yangle"
    CATEGORY = "category"


class ActionValues():
    """Class with the PWM values with respect to
       the possible actions that can be sent to servo, pertaining to
       the angle and throttle.
    """
    FAST_LEFT = 68
    SLOW_LEFT = 88
    FAST_RIGHT = 108
    SLOW_RIGHT = 128
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
        ActionSpaceKeys.CATEGORY: 1
    },
    2: {
        ActionSpaceKeys.ACTION: "Slow Left",
        ActionSpaceKeys.XANGLE: ActionValues.SLOW_LEFT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.CATEGORY: 2
    },
    3: {
        ActionSpaceKeys.ACTION: "Fast Left",
        ActionSpaceKeys.XANGLE: ActionValues.FAST_LEFT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.CATEGORY: 3
    },
    4: {
        ActionSpaceKeys.ACTION: "Slow Right",
        ActionSpaceKeys.XANGLE: ActionValues.SLOW_RIGHT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.CATEGORY: 4
    },
    5: {
        ActionSpaceKeys.ACTION: "Fast Right",
        ActionSpaceKeys.XANGLE: ActionValues.FAST_RIGHT,
        ActionSpaceKeys.YANGLE: ActionValues.YDEFAULT,
        ActionSpaceKeys.CATEGORY: 5
    }
}

# Default value to sleep for in sec.
DEFAULT_SLEEP = 0.08
