#!/usr/bin/env python3

import asyncio
import json

from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Get the list of parameters
    all_params = await drone.param.get_all_params()

    data_dict_list = []
    # Iterate through all int parameters
    for param in all_params.int_params:
        print(f"{param.name}: {param.value}")
        data_dict_list.append({"name": param.name, "value": param.value})

    for param in all_params.float_params:
        print(f"{param.name}: {param.value}")
        data_dict_list.append({"name": param.name, "value": param.value})
    with open("params.json", "w", encoding='utf-8') as file:
        file.write(json.dumps(data_dict_list, ensure_ascii=False, indent=4))


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
