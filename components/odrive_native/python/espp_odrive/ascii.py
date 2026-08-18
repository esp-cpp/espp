"""Thin helper for the ODrive *ASCII* protocol (separate from the native one).

This is deliberately minimal: it just formats and sends the documented
``r/w/p/v/f`` text lines over a serial port and reads back a line for the
commands that reply. It shares nothing with the binary native protocol; use it
only if you specifically want the ASCII interface.

    a = OdriveAscii("/dev/ttyUSB0")
    a.position(0, 3.14)                 # p 0 3.14 0 0
    a.velocity(0, 5.0)                  # v 0 5.0 0
    pos, vel = a.feedback(0)            # f 0  -> "<pos> <vel>"
    vbus = float(a.read("vbus_voltage"))
    a.write("axis0.controller.input_pos", 1.0)
"""


class OdriveAscii:
    def __init__(self, port: str, baudrate: int = 115200, serial_obj=None):
        if serial_obj is not None:
            self._serial = serial_obj
        else:
            import serial
            self._serial = serial.Serial(port, baudrate, timeout=1.0)

    def _send(self, line: str) -> None:
        self._serial.write((line + "\n").encode("ascii"))
        self._serial.flush()

    def _send_recv(self, line: str) -> str:
        self._send(line)
        return self._serial.readline().decode("ascii").strip()

    def read(self, name: str) -> str:
        """``r <name>`` -- returns the raw string the device replies with."""
        return self._send_recv("r " + name)

    def write(self, name: str, value) -> None:
        """``w <name> <value>``."""
        self._send("w %s %s" % (name, value))

    def position(self, motor: int, pos, vel_ff=0, torque_ff=0) -> None:
        """``p <motor> <pos> <vel_ff> <torque_ff>``."""
        self._send("p %d %s %s %s" % (motor, pos, vel_ff, torque_ff))

    def velocity(self, motor: int, vel, torque_ff=0) -> None:
        """``v <motor> <vel> <torque_ff>``."""
        self._send("v %d %s %s" % (motor, vel, torque_ff))

    def feedback(self, motor: int):
        """``f <motor>`` -- returns ``(pos, vel)`` as floats."""
        reply = self._send_recv("f %d" % motor)
        parts = reply.split()
        return (float(parts[0]), float(parts[1])) if len(parts) >= 2 else (None, None)

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            # Best-effort close: the port may already be gone (device
            # unplugged) or never fully opened; nothing useful to do on error.
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
