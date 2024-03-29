import asyncio

from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    # 端口使用`ls /dev/serial/by-id`进行查看
    # await drone.connect(system_address="serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:57600")
    # 模拟仿真使用下面的
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone is connected")
            break

    async for is_ready in drone.telemetry.health():
        if is_ready.is_armable:
            print("Drone is ready to arm")
            break
        else:
            print("Drone not ready to arm, please check!")

    print("Arming...")
    await drone.action.arm()
    print("Drone is armed!")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
