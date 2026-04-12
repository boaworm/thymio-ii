import asyncio
import argparse
from tdmclient import ClientAsync
from tdmclient.client import DisconnectedError

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


class ThymioFollower:
    """Thymio finger follower using listener-based sensor updates."""

    def __init__(self, node):
        self.node = node
        self.running = False
        self.sensors = None
        self._stop_event = asyncio.Event()

    async def connect(self):
        """Wait for node and set up sensor listening."""
        await self.node.lock()
        # Enable variable watching
        await self.node.watch(variables=True)
        # Add listener for sensor updates
        self.node.add_variables_changed_listener(self._on_sensors_changed)
        # Wait for initial sensor data
        await self.node.wait_for_variables({"prox.horizontal"})
        print(f"Connected to {self.node.id}")
        print("Press Ctrl+C to stop\n")

    def _on_sensors_changed(self, node, variables):
        """Callback when sensor variables change."""
        if "prox.horizontal" in variables:
            self.sensors = list(variables["prox.horizontal"])
            if self.running:
                self._process_sensors()

    def _process_sensors(self):
        """Calculate motor speeds from current sensors."""
        if self.sensors is None:
            return

        rear_blocked = (self.sensors[5] > REAR_OBSTACLE_THRESHOLD or
                       self.sensors[6] > REAR_OBSTACLE_THRESHOLD)
        max_val = max(self.sensors[:5])

        left_motor = 0
        right_motor = 0

        if max_val > DETECTION_THRESHOLD:
            center_value = self.sensors[2]

            if center_value > TOO_CLOSE_THRESHOLD:
                if not rear_blocked:
                    speed = -MAX_SPEED
                else:
                    speed = 0
            else:
                distance_error = TARGET_DISTANCE - center_value
                speed = max(0, min(MAX_SPEED, distance_error * SPEED_GAIN))

            if (self.sensors[2] >= self.sensors[0] and
                self.sensors[2] >= self.sensors[1] and
                self.sensors[2] >= self.sensors[3] and
                self.sensors[2] >= self.sensors[4]):
                fine_diff = self.sensors[1] - self.sensors[3]
                if abs(fine_diff) < FINE_THRESHOLD:
                    left_motor = int(speed)
                    right_motor = int(speed)
                else:
                    correction = fine_diff * 0.005
                    left_motor = int(speed - correction)
                    right_motor = int(speed + correction)
            else:
                left_sum = self.sensors[0] + self.sensors[1]
                right_sum = self.sensors[3] + self.sensors[4]

                if left_sum > right_sum:
                    if left_sum > ROTATE_THRESHOLD:
                        left_motor = -ROTATE_SPEED
                        right_motor = ROTATE_SPEED
                    else:
                        diff = left_sum - right_sum
                        left_motor = int(speed - diff * 0.005)
                        right_motor = int(speed + diff * 0.005)
                else:
                    if right_sum > ROTATE_THRESHOLD:
                        left_motor = ROTATE_SPEED
                        right_motor = -ROTATE_SPEED
                    else:
                        diff = right_sum - left_sum
                        left_motor = int(speed + diff * 0.005)
                        right_motor = int(speed - diff * 0.005)

        asyncio.create_task(self._set_motors(left_motor, right_motor))

    async def _set_motors(self, left, right):
        """Set motor speeds."""
        try:
            await self.node.set_variables({
                "motor.left.target": [left],
                "motor.right.target": [right]
            })
        except Exception:
            pass

    async def stop(self):
        """Stop motors and clean up."""
        try:
            await self.node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })
        except Exception:
            pass
        try:
            await self.node.unlock()
        except Exception:
            pass


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
    follower = None

    try:
        node = await asyncio.wait_for(client.wait_for_node(), timeout=10.0)
        follower = ThymioFollower(node)
        await follower.connect()
        follower.running = True

        # Keep running until interrupted
        await follower._stop_event.wait()

    except asyncio.TimeoutError:
        print("ERROR: Timeout waiting for robot. Is it powered on and in range?")
    except KeyboardInterrupt:
        print("\nStopping...")
    except DisconnectedError:
        print("\nDisconnected from TDM")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if follower:
            await follower.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Thymio finger-following robot")
    parser.add_argument("--addr", type=str, default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")

    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
