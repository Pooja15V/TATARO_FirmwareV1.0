#!/usr/bin/env python3
"""
TATARO Therapy — Button state publisher node.

Connects to the XIAO BLE device (see xarm_ros2/src/sensors/read_button_ble.py
for the confirmed-working reference implementation this mirrors) and
publishes a std_msgs/Bool on /tataro/button_pressed:

  True  — the physical switch is pressed
  False — the physical switch is released

Firmware with the raw button characteristic (BUTTON_UUID, "PRESSED" /
"RELEASED" on every debounced edge) drives the topic directly. Older
firmware without it falls back to the status characteristic, where only a
"GREEN" notification (start of the 5 s light sequence) counts as a press.

This node only reports raw hardware state. Edge detection ("ignore repeated
press notifications while the button remains pressed") is the therapy FSM's
job, not this node's — perception/monitoring and state management are kept
separate on purpose.
"""

import asyncio
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from bleak import BleakClient, BleakScanner
from dbus_fast import BusType
from dbus_fast.aio import MessageBus

DEVICE_NAME = "xiao-tataro-task1"
STATUS_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
BUTTON_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
PRESSED_STATUS_VALUE = "GREEN"
PRESSED_BUTTON_VALUE = "PRESSED"
BUTTON_TOPIC = '/tataro/button_pressed'


class ButtonMonitorNode(Node):
    def __init__(self):
        super().__init__('tataro_button_monitor')
        self._pub = self.create_publisher(Bool, BUTTON_TOPIC, 10)
        self._has_button_char = False
        self._client = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_ble_loop, daemon=True)
        self._thread.start()

    def _publish(self, pressed: bool) -> None:
        msg = Bool()
        msg.data = pressed
        self._pub.publish(msg)

    def _on_button_notify(self, _handle, data: bytearray) -> None:
        value = data.decode("utf-8")
        self.get_logger().info(f"BUTTON: {value}")
        self._publish(value == PRESSED_BUTTON_VALUE)

    def _on_status_notify(self, _handle, data: bytearray) -> None:
        value = data.decode("utf-8")
        self.get_logger().info(f"STATUS: {value}")
        if not self._has_button_char:
            self._publish(value == PRESSED_STATUS_VALUE)

    def _run_ble_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ble_main())

    async def _drop_stale_connection(self) -> None:
        # A connected peripheral stops advertising, so a BlueZ link left open
        # by a previous run (e.g. one killed with Ctrl+C) would make every
        # scan below miss the device forever. Disconnect any such link first.
        bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        try:
            intro = await bus.introspect("org.bluez", "/")
            root = bus.get_proxy_object("org.bluez", "/", intro)
            objects = await root.get_interface(
                "org.freedesktop.DBus.ObjectManager").call_get_managed_objects()
            for path, ifaces in objects.items():
                dev = ifaces.get("org.bluez.Device1")
                if (dev and dev.get("Name") and dev["Name"].value == DEVICE_NAME
                        and dev["Connected"].value):
                    self.get_logger().warn(
                        f"Dropping stale BlueZ connection to {DEVICE_NAME} ...")
                    obj = bus.get_proxy_object(
                        "org.bluez", path, await bus.introspect("org.bluez", path))
                    await obj.get_interface("org.bluez.Device1").call_disconnect()
                    await asyncio.sleep(1.0)
        finally:
            bus.disconnect()

    async def disconnect(self) -> None:
        if self._client is not None and self._client.is_connected:
            await self._client.disconnect()

    async def _find_device(self):
        # Mirrors read_button_ble.py's discover()-then-filter approach
        # exactly, rather than BleakScanner.find_device_by_name()'s
        # convenience wrapper -- kept identical to what is confirmed working
        # on this hardware.
        devices = await BleakScanner.discover()
        for device in devices:
            if device.name == DEVICE_NAME:
                return device
        return None

    async def _ble_main(self) -> None:
        while rclpy.ok():
            try:
                await self._drop_stale_connection()
                self.get_logger().info(f"Scanning for {DEVICE_NAME} ...")
                device = await self._find_device()
                if device is None:
                    self.get_logger().warn(f"{DEVICE_NAME} not found — retrying ...")
                    await asyncio.sleep(3.0)
                    continue

                async with BleakClient(device) as client:
                    self._client = client
                    self._has_button_char = (
                        client.services.get_characteristic(BUTTON_UUID) is not None
                    )
                    await client.start_notify(STATUS_UUID, self._on_status_notify)
                    if self._has_button_char:
                        await client.start_notify(BUTTON_UUID, self._on_button_notify)
                        initial = bytes(await client.read_gatt_char(BUTTON_UUID))
                        self._on_button_notify(None, bytearray(initial))
                        self.get_logger().info(
                            "Connected — publishing raw button state (PRESSED/RELEASED).")
                    else:
                        self.get_logger().warn(
                            "Connected — firmware has no button characteristic; "
                            "falling back to GREEN status as press.")
                    while rclpy.ok() and client.is_connected:
                        await asyncio.sleep(1.0)
            except Exception as exc:
                self.get_logger().warn(f"BLE connection lost ({exc}) — retrying ...")
                await asyncio.sleep(3.0)


def main():
    rclpy.init()
    node = ButtonMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # The BLE thread is a daemon and would die without leaving its
        # `async with`, leaving the BlueZ link open -- close it explicitly.
        try:
            asyncio.run_coroutine_threadsafe(
                node.disconnect(), node._loop).result(timeout=5.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
