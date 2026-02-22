"""
boot.py  —  Runs before main.py and before USB enumeration.

Exposes a SINGLE USB CDC data port for Dynamixel traffic.  The REPL console
is disabled so no CircuitPython output interferes with the binary protocol.

On the host:
  /dev/ttyACM0  — Dynamixel data port (for turtlebot3_node)

To temporarily re-enable the REPL for debugging, comment out the
usb_cdc.enable() line below and power-cycle the board.

Disables auto-reload so main.py is not restarted by USB file writes.
"""

import usb_cdc
import supervisor

# Single data port, REPL console disabled.
# main.py uses usb_cdc.data for all Dynamixel traffic.
usb_cdc.enable(console=False, data=True)

# Prevent CircuitPython from restarting main.py when files are saved over USB
supervisor.runtime.autoreload = False
