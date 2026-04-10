import asyncio
import argparse
from tdmclient import ClientAsync


async def run_thymio(robot_addr=None, robot_port=None, use_ws=False, use_zeroconf=True):
    kwargs = {}
    if robot_addr:
        kwargs["tdm_addr"] = robot_addr
    if robot_port:
        kwargs["tdm_port"] = robot_port
    if use_ws:
        kwargs["tdm_ws"] = True
    if use_zeroconf:
        kwargs["zeroconf"] = True

    client = ClientAsync(**kwargs)
    try:
        node = await asyncio.wait_for(client.wait_for_node(), timeout=10.0)
    except asyncio.TimeoutError:
        print("Timeout: could not connect to Thymio")
        return

    await node.lock()
    print(f"Connected to {node.id}")

    await node.set_variables({
        "motor.left.target": [0],
        "motor.right.target": [0],
    })
    await node.stop()
    await node.compile("call sound.system(1)", load=True)
    await node.run()
    await asyncio.sleep(0.5)

    await node.unlock()
    print("Reset done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reset Thymio (stop motors, beep)")
    parser.add_argument("--addr", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--ws", action="store_true")
    parser.add_argument("--no-zeroconf", action="store_true")
    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
