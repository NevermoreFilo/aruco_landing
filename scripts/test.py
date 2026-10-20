"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time
import numpy as np

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0,  # canali 1-6 invariati
    1100,               # canale 7 = pitch verso il basso
    0                   # canale 8 invariato
)

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
master.mav.manual_control_send(
    master.target_system,
    500,
    -500,
    250,
    500,
    0)
print("Sent manual control command to the autopilot.")

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
buttons = 1 + 1 << 3 + 1 << 7
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    500, # 500 means neutral throttle
    0,
    buttons)

master.mav.landing_target_send(master.target_system, 0, 0, 0, 0, 0, 0, 0)