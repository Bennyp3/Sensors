"""
ble_uart.py -- a minimal Bluetooth Low Energy "serial port" for the Pico 2 W.

Uses the Nordic UART Service (NUS), the de-facto standard way to send lines
of text over BLE. The Pico is the peripheral: it advertises under a name,
the computer connects to it, and the Pico sends text with BLE notifications.

Copy this file to the Pico (next to Wearable_Main.py, or into /lib).
Needs MicroPython firmware for the Pico 2 W (RPI_PICO2_W), v1.24 or newer.

Usage:
    ble = BLEUART("Wearable")
    ble.send("hello")       # queue one line (a newline is added)
    ble.flush()             # call every loop pass: sends one packet

Lines are queued and sent one packet per flush() call, so sending never
stalls the 4 ms sensor loop. Long lines are split into packets; the
computer joins them back together at each newline.
"""

import bluetooth
import struct
from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_MTU_EXCHANGED = const(21)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

# Nordic UART Service UUIDs (TX = Pico -> computer, RX = computer -> Pico)
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
            _FLAG_READ | _FLAG_NOTIFY)
_UART_RX = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
            _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX))

_DEFAULT_CHUNK = 20     # bytes per packet before the MTU is negotiated
_MAX_CHUNK = 180        # cap on bytes per packet after MTU negotiation
_MAX_QUEUE = 4096       # drop new data if this much is waiting (link too slow)


def _adv_field(adv_type, value):
    return struct.pack("BB", len(value) + 1, adv_type) + value


class BLEUART:
    def __init__(self, name="Wearable"):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        try:
            self._ble.config(mtu=_MAX_CHUNK + 3)   # ask for bigger packets
        except (ValueError, OSError):
            pass                                    # not supported: use 20 B
        self._ble.irq(self._irq)
        ((self._tx, self._rx),) = self._ble.gatts_register_services((_UART_SERVICE,))

        self._conn = None
        self._chunk = _DEFAULT_CHUNK
        self._queue = bytearray()

        # advertising packet: flags + name; scan response: the service UUID
        self._adv = _adv_field(0x01, b"\x06") + _adv_field(0x09, name.encode())
        self._resp = _adv_field(0x07, bytes(_UART_UUID))
        self._advertise()

    def _advertise(self):
        self._ble.gap_advertise(100_000, adv_data=self._adv, resp_data=self._resp)

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            self._conn, _, _ = data
            self._chunk = _DEFAULT_CHUNK
            self._queue = bytearray()
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._conn = None
            self._queue = bytearray()
            self._advertise()                      # let the computer reconnect
        elif event == _IRQ_MTU_EXCHANGED:
            _, mtu = data
            self._chunk = max(_DEFAULT_CHUNK, min(mtu - 3, _MAX_CHUNK))

    def is_connected(self):
        return self._conn is not None

    def send(self, line):
        """Queue one line of text. Does nothing while no computer is connected."""
        if self._conn is None or len(self._queue) > _MAX_QUEUE:
            return
        self._queue.extend(line.encode())
        self._queue.extend(b"\n")

    def flush(self):
        """Send at most one packet from the queue. Call once per loop pass."""
        if self._conn is None or not self._queue:
            return
        packet = self._queue[:self._chunk]
        try:
            self._ble.gatts_notify(self._conn, self._tx, packet)
        except OSError:
            return                  # BLE buffers full: try again next pass
        self._queue = self._queue[self._chunk:]
