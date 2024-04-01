import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to be connected...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone is connected")
            break

    async for is_ready in drone.telemetry.health():
        if is_ready.is_armable:
            print("Drone is ready to arm")
            break

    print("Arming...")
    await drone.action.arm()

    print("Takeoff...")
    await drone.action.set_takeoff_altitude(2)
    await drone.action.takeoff()

    # First, set the target position (2m above the initial point)
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))

    # Then start offboard mode.
    print("-- Starting offboard")
    try:
        await drone.offboard.start()

        # We want to fly the square at an altitude of 2 m above the initial point.
        # Each side of the square is 2 m.
        # The corners of the square are defined in the NED frame (north, east, down).
        print("-- Go to the north west corner")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 2.0, -2.0, 90.0))
        await asyncio.sleep(10)

        print("-- Go to the north east corner")
        await drone.offboard.set_position_ned(PositionNedYaw(2.0, 2.0, -2.0, 180.0))
        await asyncio.sleep(10)

        print("-- Go to the south east corner")
        await drone.offboard.set_position_ned(PositionNedYaw(2.0, 0.0, -2.0, -90.0))
        await asyncio.sleep(10)

        print("-- Go to the south west corner")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 180))
        await asyncio.sleep(10)

    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")

    # Finally we can land.
    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
