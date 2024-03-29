import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import threading
import keyboard
# pip install keyboard

# The current position of the drone
position = [0.0, 0.0, -2.0]


def move_forward(e):
    position[0] += 1.0


def move_backward(e):
    position[0] -= 1.0


def move_left(e):
    position[1] -= 1.0


def move_right(e):
    position[1] += 1.0


def handle_keyboard_input():
    keyboard.on_press_key('w', move_forward)
    keyboard.on_press_key('s', move_backward)
    keyboard.on_press_key('a', move_left)
    keyboard.on_press_key('d', move_right)


async def control_drone():
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

        # Keep the script running and listen to keyboard input
        while True:
            global position
            await drone.offboard.set_position_ned(PositionNedYaw(position[0], position[1], -2.0, 0))

            await asyncio.sleep(1)

    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")


if __name__ == "__main__":
    threading.Thread(target=handle_keyboard_input).start()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(control_drone())