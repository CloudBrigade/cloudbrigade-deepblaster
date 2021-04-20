# Custom message with blaster control angle and trigger values.
#
# This message is used to send blaster trigger and servo angle
# ratios with respect to the device calibration. It can also be used to
# send raw PWM values for angle.
int32 x_angle     # Angle ratio w.r.t rotate_max/rotate_min
                    # in calibration file or raw PWM value to be sent
                    # to the servo.
int32 y_angle     # Angle ratio w.r.t height_max/height_min
                    # in calibration file or raw PWM value to be sent
                    # to the servo.
int32 flywheel      # Value to power flywheels on or off
int32 trigger       # Value to engage trigger (feeder motor)
