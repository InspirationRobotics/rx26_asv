"""actuator_core — ROS-free Pololu Maestro wire-protocol encoding.

The one piece of real logic in the Mission-3 actuator path is the Pololu compact
protocol byte packing (quarter-microsecond target split into two 7-bit bytes). It
is extracted here so it is unit-testable without a serial port or ROS
(Format best-practice #2); a wrong encoding drives the water cannon / launcher to
the wrong position, so it is worth pinning with vectors. `MaestroLink.set_pwm`
(actuator_node.py) is a one-line wrapper over `maestro_target_bytes`.
"""

MAESTRO_SET_TARGET = 0x84   # Pololu compact-protocol "Set Target" command byte


def maestro_target_bytes(channel: int, target_us: int) -> bytes:
    """Encode a Set-Target frame for one Maestro channel.

    The Maestro target is in *quarter*-microseconds, transmitted as two 7-bit
    bytes (low 7 bits first, then the next 7). Returns the 4-byte frame
    ``[0x84, channel, target_low7, target_high7]``.

    Args:
        channel: Maestro servo channel (0-based).
        target_us: pulse width in microseconds.
    Returns:
        The 4-byte compact-protocol frame.
    """
    target = int(target_us) * 4
    return bytes([MAESTRO_SET_TARGET, channel,
                  target & 0x7F, (target >> 7) & 0x7F])
