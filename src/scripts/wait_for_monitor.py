"""Wait for Teensy USB enumeration before PlatformIO starts its monitor.

PlatformIO resolves monitor_port wildcards only once. An upload can finish
while USB is still restarting, causing a port prompt instead of a logger.
This hook only enumerates devices; it never opens or retains a serial port.
"""

from fnmatch import fnmatch
import time

from serial.tools import list_ports
from SCons.Script import COMMAND_LINE_TARGETS


def wait_for_monitor_port(pattern, timeout=30.0, settle=1.0):
    deadline = time.monotonic() + timeout
    candidate = None
    present_since = None
    while time.monotonic() < deadline:
        ports = [
            port.device
            for port in list_ports.comports()
            if port.vid == 0x16C0
            and port.pid == 0x0483
            and fnmatch(port.device, pattern)
        ]
        if len(ports) > 1:
            raise RuntimeError(
                "Multiple Teensy serial ports found. Set monitor_port to the "
                "robot's exact port in platformio.ini."
            )
        current = ports[0] if ports else None
        if current != candidate:
            candidate = current
            present_since = time.monotonic() if current else None
        if candidate and time.monotonic() - present_since >= settle:
            return candidate
        time.sleep(0.1)
    raise RuntimeError(
        "Teensy USB serial port did not reappear within %.0f seconds "
        "(monitor_port=%s). Check the USB connection and Teensy Loader "
        "upload result; the serial monitor has not started." % (timeout, pattern)
    )


def wait_after_upload(source, target, env):
    print("Waiting for Teensy USB serial before starting the log recorder...")
    try:
        port = wait_for_monitor_port(env.GetProjectOption("monitor_port"))
    except RuntimeError as error:
        print("Error: %s" % error)
        return 1
    print("Teensy USB serial ready: %s" % port)
    return 0


Import("env")
if {"upload", "monitor"}.issubset(COMMAND_LINE_TARGETS):
    env.AddPostAction("upload", wait_after_upload)
