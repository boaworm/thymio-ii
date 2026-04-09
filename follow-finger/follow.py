import asyncio
import argparse
from tdmclient import ClientAsync

# Sensor layout on Thymio II:
# Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
# Rear:   [5] [6]              (both on the back)

# Configuration
MAX_SPEED = 150
ROTATE_SPEED = 60      # Slower speed for rotation to avoid overshooting
DETECTION_THRESHOLD = 50
TARGET_DISTANCE = 3000
TOO_CLOSE_THRESHOLD = 4000
REAR_OBSTACLE_THRESHOLD = 1000
ROTATE_THRESHOLD = 2000  # Above this, rotate in place instead of just turning
FINE_THRESHOLD = 200   # Difference below which we consider ourselves centered

# Control gain
SPEED_GAIN = 0.1       # How speed adjusts based on distance error


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

            left_motor = 0
            right_motor = 0

            if max_val > DETECTION_THRESHOLD:
                center_value = sensors[2]

                # Calculate forward speed based on distance to target
                if center_value > TOO_CLOSE_THRESHOLD:
                    # Too close - reverse if rear clear
                    if not rear_blocked:
                        speed = -MAX_SPEED
                    else:
                        speed = 0
                else:
                    # Move forward, slower when closer to target
                    distance_error = TARGET_DISTANCE - center_value
                    speed = max(0, min(MAX_SPEED, distance_error * SPEED_GAIN))

                # Check if we're mostly centered (sensor 2 is strongest)
                if sensors[2] >= sensors[0] and sensors[2] >= sensors[1] and \
                   sensors[2] >= sensors[3] and sensors[2] >= sensors[4]:
                    # Mostly centered - check if fine adjustment needed
                    fine_diff = sensors[1] - sensors[3]
                    if abs(fine_diff) < FINE_THRESHOLD:
                        # Well centered - just move forward
                        left_motor = int(speed)
                        right_motor = int(speed)
                    else:
                        # Slightly off - apply small correction
                        correction = fine_diff * 0.005
                        left_motor = int(speed - correction)
                        right_motor = int(speed + correction)
                else:
                    # Not centered - calculate left/right imbalance
                    # Sensors 0,1 are on the left; sensors 3,4 are on the right
                    left_sum = sensors[0] + sensors[1]
                    right_sum = sensors[3] + sensors[4]

                    if left_sum > right_sum:
                        # Finger is on the left - turn left
                        if left_sum > ROTATE_THRESHOLD:
                            # Rotate in place (slower speed)
                            left_motor = -ROTATE_SPEED
                            right_motor = ROTATE_SPEED
                        else:
                            # Both forward, right faster - scale by error
                            diff = left_sum - right_sum
                            left_motor = int(speed - diff * 0.005)
                            right_motor = int(speed + diff * 0.005)
                    else:
                        # Finger is on the right - turn right
                        if right_sum > ROTATE_THRESHOLD:
                            # Rotate in place (slower speed)
                            left_motor = ROTATE_SPEED
                            right_motor = -ROTATE_SPEED
                        else:
                            # Both forward, left faster - scale by error
                            diff = right_sum - left_sum
                            left_motor = int(speed + diff * 0.005)
                            right_motor = int(speed - diff * 0.005)

            await node.set_variables({
                "motor.left.target": [left_motor],
                "motor.right.target": [right_motor]
            })

            await client.sleep(0.3)

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
