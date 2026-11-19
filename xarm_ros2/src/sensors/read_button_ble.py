import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "xiao-tataro-task1"

STATUS_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"


def status_handler(sender, data):
    value = data.decode("utf-8")

    print(f"STATUS: {value}")

    # The firmware has no dedicated button characteristic — a button press
    # is reported indirectly via a "GREEN" status update.
    if value == "GREEN":
        print("BUTTON: PRESSED")


async def main():

    print("Scanning for XIAO...")

    devices = await BleakScanner.discover()

    xiao = None

    for device in devices:

        if device.name == DEVICE_NAME:

            xiao = device
            break

    if xiao is None:

        print("XIAO_RGB not found")
        return

    print("Found:")
    print(xiao)

    print("Connecting...")

    async with BleakClient(xiao) as client:

        print("Connected!")

        await client.start_notify(
            STATUS_UUID,
            status_handler
        )

        print("Waiting for button events...")
        print("Press Ctrl+C to exit.")

        while True:

            await asyncio.sleep(1)


asyncio.run(main())