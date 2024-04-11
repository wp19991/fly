import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
import threading
import keyboard

# The current velocity of the drone
VelocityBodyYawSpeed = [0.0, 0.0, 0.0, 0.0]
is_running = True
step_size_m_s = 0.5  # 1 m/s step size for velocity control
response_time_s = 0.2


def move_up(e):
    VelocityBodyYawSpeed[2] -= step_size_m_s


def move_down(e):
    VelocityBodyYawSpeed[2] += step_size_m_s


def move_forward(e):
    VelocityBodyYawSpeed[0] += step_size_m_s


def move_backward(e):
    VelocityBodyYawSpeed[0] -= step_size_m_s


def move_left(e):
    VelocityBodyYawSpeed[1] -= step_size_m_s


def move_right(e):
    VelocityBodyYawSpeed[1] += step_size_m_s


def move_yaw_left(e):
    VelocityBodyYawSpeed[1] -= step_size_m_s


def move_yaw_right(e):
    VelocityBodyYawSpeed[1] += step_size_m_s


def move_exit(e):
    global is_running
    is_running = False


def handle_keyboard_input():
    keyboard.on_press_key('u', move_up)
    keyboard.on_press_key('i', move_down)
    keyboard.on_press_key('w', move_forward)
    keyboard.on_press_key('s', move_backward)
    keyboard.on_press_key('a', move_left)
    keyboard.on_press_key('d', move_right)
    keyboard.on_press_key('q', move_yaw_left)
    keyboard.on_press_key('e', move_yaw_right)
    keyboard.on_press_key('z', move_exit)


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
    global VelocityBodyYawSpeed
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*VelocityBodyYawSpeed))
    # Now use set_velocity_ned instead of set_position_ned
    try:
        await drone.offboard.start()

        # Keep the script running and listen to keyboard input
        while True:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*VelocityBodyYawSpeed))
            await asyncio.sleep(response_time_s)
            print(" x:{: <4},y:{: <4}z:{: <4}".format(
                *map(lambda x: round(x, 2), VelocityBodyYawSpeed)))

            # stop
            if not is_running:
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
