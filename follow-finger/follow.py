import asyncio
import argparse
from tdmclient import ClientAsync

# Sensor layout on Thymio II:
# Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
# Rear:   [5] [6]              (both on the back)

# Configuration
TURN_SPEED = 100
FORWARD_SPEED = 100
REVERSE_SPEED = 100
DETECTION_THRESHOLD = 50
CENTERING_THRESHOLD = 50
TARGET_DISTANCE = 3000
TOO_CLOSE_THRESHOLD = 4000
REAR_OBSTACLE_THRESHOLD = 1000


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
        print("ERROR: Timeout waiting for robot. Is it powered on and in range?")
        return

    await node.lock()

    print(f"Connected to {node.id}")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            await node.wait_for_variables({"prox.horizontal"})
            sensors = node.v.prox.horizontal

            rear_blocked = sensors[5] > REAR_OBSTACLE_THRESHOLD or sensors[6] > REAR_OBSTACLE_THRESHOLD

            front_sensors = sensors[:5]
            max_val = max(front_sensors)
            target_index = front_sensors.index(max_val) if max_val > DETECTION_THRESHOLD else -1

            left_motor = 0
            right_motor = 0

            if max_val > DETECTION_THRESHOLD:
                center_value = sensors[2]

                if center_value > TOO_CLOSE_THRESHOLD:
                    # Too close - reverse if rear clear
                    if not rear_blocked:
                        left_motor = -REVERSE_SPEED
                        right_motor = -REVERSE_SPEED
                elif center_value > TARGET_DISTANCE:
                    # A bit close - stop
                    left_motor = 0
                    right_motor = 0
                else:
                    # Good distance - move forward and center
                    if target_index < 2:
                        # Object on left - turn left
                        left_motor = -TURN_SPEED
                        right_motor = TURN_SPEED
                    elif target_index > 2:
                        # Object on right - turn right
                        left_motor = TURN_SPEED
                        right_motor = -TURN_SPEED
                    else:
                        # Sensor 2 is strongest - move forward
                        left_motor = FORWARD_SPEED
                        right_motor = FORWARD_SPEED

            await node.set_variables({
                "motor.left.target": [left_motor],
                "motor.right.target": [right_motor]
            })

            await client.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        try:
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })
        except:
            pass
        try:
            await node.unlock()
        except:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Thymio finger-following robot")
    parser.add_argument("--addr", type=str, default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")

    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
