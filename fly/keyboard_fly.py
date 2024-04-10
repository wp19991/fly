import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import threading
import keyboard

# The current position of the drone
position = [0, 0, -4.0, True]
step_size_m = 0.2
response_time_s = 0.2


def move_up(e):
    position[2] -= step_size_m


def move_down(e):
    if position[2] <= 0.0:
        position[2] += step_size_m


def move_forward(e):
    position[0] += step_size_m


def move_backward(e):
    position[0] -= step_size_m


def move_left(e):
    position[1] -= step_size_m


def move_right(e):
    position[1] += step_size_m


def move_q(e):
    position[3] = False


def handle_keyboard_input():
    keyboard.on_press_key('u', move_up)
    keyboard.on_press_key('i', move_down)
    keyboard.on_press_key('w', move_forward)
    keyboard.on_press_key('s', move_backward)
    keyboard.on_press_key('a', move_left)
    keyboard.on_press_key('d', move_right)
    keyboard.on_press_key('q', move_q)


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

    # First, set the target position (4m above the initial point)
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -4.0, 0.0))

    # Then start offboard mode.
    print("-- Starting offboard")

    try:
        await drone.offboard.start()

        # Keep the script running and listen to keyboard input
        while True:
            global position
            await drone.offboard.set_position_ned(PositionNedYaw(position[0], position[1], position[2], 0))
            await asyncio.sleep(response_time_s)
            print("x:{: <4},y:{: <4},z:{: <4}".format(position[0].__round__(2), position[1].__round__(2),
                                                      position[2].__round__(2)))
            # stop
            if not position[3]:
                break

    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
    except KeyboardInterrupt:
        print("User interrupted, stopping...")
    finally:
        await drone.action.land()
        await asyncio.sleep(15)
        # Stop offboard mode.
        print("Stopping offboard mode...")
        await drone.offboard.stop()
        # Disarm the drone.
        print("Disarming...")
        await drone.action.disarm()


if __name__ == "__main__":
    threading.Thread(target=handle_keyboard_input).start()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(control_drone())
