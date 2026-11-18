from machine import Pin, I2C
import aioble
import asyncio
import bluetooth
import time

led = Pin("LED", Pin.OUT)
led.off()

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])

if 0x68 in devices:
    MPU = 0x68
elif 0x69 in devices:
    MPU = 0x69
else:
    raise RuntimeError("MPU-6050 not found. Check wiring.")

i2c.writeto_mem(MPU, 0x6B, b"\x00")
time.sleep_ms(100)

SERVICE_UUID = bluetooth.UUID(0xFFE0)
CHAR_UUID = bluetooth.UUID(0xFFE1)
DEVICE_NAMES = ("dsd tech", "hmsoft", "hm-10")

PROP_WRITE_NO_RESPONSE = 0x04
PROP_WRITE = 0x08
PROP_NOTIFY = 0x10

GYRO_DEADZONE = 1.5
SEND_INTERVAL_MS = 60

yaw = 0.0
yaw_offset = 0.0
gyro_bias_z = 0.0


def read_gyro_z():
    # MPU-6050 Z-axis gyro registers at the default ±250°/s range.
    data = i2c.readfrom_mem(MPU, 0x47, 2)
    value = int.from_bytes(data, "big")
    if value >= 32768:
        value -= 65536
    return value / 131.0


def calibrate_gyro_bias():
    global gyro_bias_z

    print("Keep the sensor still...")
    time.sleep_ms(1000)

    total = 0.0
    for _ in range(500):
        total += read_gyro_z()
        time.sleep_ms(5)

    gyro_bias_z = total / 500
    print("Gyroscope bias:", gyro_bias_z)


def zero_position():
    global yaw_offset
    yaw_offset = yaw
    print("HEAD POSITION ZEROED!")


async def update_motion():
    global yaw, gyro_bias_z

    last_time = time.ticks_us()
    still_count = 0

    while True:
        gz = read_gyro_z()
        now = time.ticks_us()
        dt = time.ticks_diff(now, last_time) / 1000000.0
        last_time = now

        rotation = gz - gyro_bias_z

        if abs(rotation) < 0.8:
            still_count = min(still_count + 1, 101)
        else:
            still_count = 0

        if still_count > 100:
            gyro_bias_z = gyro_bias_z * 0.999 + gz * 0.001
            rotation = gz - gyro_bias_z

        if abs(rotation) >= GYRO_DEADZONE and 0 < dt < 0.2:
            yaw += rotation * dt

        await asyncio.sleep_ms(10)


async def receive_commands(characteristic):
    buffer = b""

    while True:
        buffer += await characteristic.notified()

        while b"\n" in buffer:
            command, buffer = buffer.split(b"\n", 1)
            if command.strip().upper() == b"ZERO":
                zero_position()

        if len(buffer) > 100:
            buffer = b""


async def send_motion(characteristic, connection):
    use_response = not bool(
        characteristic.properties & PROP_WRITE_NO_RESPONSE
    )

    while connection.is_connected():
        message = "Y:{:.1f}\n".format(yaw - yaw_offset)
        await characteristic.write(message.encode(), response=use_response)
        await asyncio.sleep_ms(SEND_INTERVAL_MS)


async def find_hm10():
    print("Scanning for Bluetooth devices...")

    async with aioble.scan(
        10000, interval_us=30000, window_us=30000, active=True
    ) as scanner:
        async for result in scanner:
            name = result.name() or "Unknown"
            print("Found:", name, result.device.addr)

            if any(label in name.lower() for label in DEVICE_NAMES):
                return result.device

    return None


async def main():
    calibrate_gyro_bias()
    zero_position()
    motion_task = asyncio.create_task(update_motion())

    try:
        while True:
            receive_task = None

            try:
                target = await find_hm10()
                if target is None:
                    print("HM-10 not found. Retrying...")
                    await asyncio.sleep_ms(2000)
                    continue

                print("Connecting to HM-10...")
                connection = await target.connect(timeout_ms=10000)

                async with connection:
                    service = await connection.service(SERVICE_UUID)
                    if service is None:
                        raise RuntimeError("FFE0 service not found")

                    characteristic = await service.characteristic(CHAR_UUID)
                    if characteristic is None:
                        raise RuntimeError("FFE1 characteristic not found")

                    props = characteristic.properties
                    if not props & (PROP_WRITE | PROP_WRITE_NO_RESPONSE):
                        raise RuntimeError("FFE1 does not support writing")

                    if props & PROP_NOTIFY:
                        try:
                            await characteristic.subscribe(notify=True)
                            receive_task = asyncio.create_task(
                                receive_commands(characteristic)
                            )
                            print("ZERO button ready.")
                        except Exception as error:
                            print("ZERO reception unavailable:", repr(error))
                    else:
                        print("ZERO reception unavailable on FFE1.")

                    led.on()
                    print("BLUETOOTH CONNECTED!")

                    try:
                        await send_motion(characteristic, connection)
                    finally:
                        if receive_task is not None:
                            receive_task.cancel()
                            await asyncio.sleep_ms(0)

            except Exception as error:
                print("Bluetooth error:", repr(error))

            finally:
                led.off()

            await asyncio.sleep_ms(2000)

    finally:
        motion_task.cancel()
        led.off()


asyncio.run(main())