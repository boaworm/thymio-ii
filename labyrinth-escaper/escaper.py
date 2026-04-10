import asyncio
import argparse
from tdmclient import ClientAsync

# Sensor layout on Thymio II:
# Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
# Rear:   [5] [6]              (both on the back)
# Scale: 0 = nothing, ~5000 = touching (higher = closer)

# Detection thresholds
CLEAR_THRESHOLD  = 80    # below this, sensor is effectively "clear"
NEAR_THRESHOLD   = 1200  # something nearby — start steering away
BLOCK_THRESHOLD  = 2800  # something in the way — stop & rotate in place

# Speeds
FORWARD_SPEED = 120
TURN_SPEED    = 110

# Steering gain when nudging around near obstacles
STEER_GAIN = 0.04

# Loop timing
LOOP_DT = 0.1   # 10 Hz

# Escape condition: accelerometer Y reads >= this value for several ticks in a
# row (robot has been tilted — "lifted to victory").
ACC_Y_THRESHOLD    = 2
ACC_CONFIRM_TICKS  = 4

# Safety: if we've been rotating in place for this long without finding a way
# out, nudge backwards briefly to unstick (dead-end recovery).
STUCK_ROTATE_TICKS = 40
REVERSE_TICKS      = 6


async def play_freq(node, freq, duration_60ths=12):
    await node.stop()
    await node.compile(f"call sound.freq({freq}, {duration_60ths})", load=True)
    await node.run()
    await asyncio.sleep(duration_60ths / 60 + 0.15)


async def set_motors(node, left, right):
    await node.set_variables({
        "motor.left.target":  [int(left)],
        "motor.right.target": [int(right)],
    })


async def phase1_startup(node):
    print("Phase 1: startup")
    for _ in range(3):
        await play_freq(node, 880)


async def phase2_escape(node, client):
    print("Phase 2: escaping")
    rotate_ticks = 0
    tilt_ticks = 0

    while True:
        await node.wait_for_variables({"prox.horizontal", "acc"})
        sensors = node.v.prox.horizontal
        acc = list(node.v.acc)
        front = list(sensors[:5])
        rear = list(sensors[5:7])

        # Escape when accelerometer Y is sustained >= threshold (robot tilted).
        if acc[1] >= ACC_Y_THRESHOLD:
            tilt_ticks += 1
        else:
            tilt_ticks = 0

        if tilt_ticks >= ACC_CONFIRM_TICKS:
            await set_motors(node, 0, 0)
            print(f"Phase 2 complete: escaped! acc={acc}")
            return

        max_front = max(front)
        left_sum  = front[0] + front[1]
        right_sum = front[3] + front[4]
        front_blocked = (
            front[1] > BLOCK_THRESHOLD or
            front[2] > BLOCK_THRESHOLD or
            front[3] > BLOCK_THRESHOLD
        )

        if front_blocked:
            # Wall directly ahead — rotate in place toward the freer side.
            rotate_ticks += 1

            if rotate_ticks > STUCK_ROTATE_TICKS:
                # Stuck spinning — back up briefly if the rear is clear.
                rear_blocked = max(rear) > NEAR_THRESHOLD
                if not rear_blocked:
                    left_motor  = -FORWARD_SPEED
                    right_motor = -FORWARD_SPEED
                    if rotate_ticks >= STUCK_ROTATE_TICKS + REVERSE_TICKS:
                        rotate_ticks = 0  # try rotating again after reversing
                else:
                    # Rear blocked too — keep spinning and hope for the best.
                    left_motor, right_motor = TURN_SPEED, -TURN_SPEED
            elif left_sum <= right_sum:
                # More obstacle on the right → turn left (away from it).
                left_motor  = -TURN_SPEED
                right_motor =  TURN_SPEED
            else:
                # More obstacle on the left → turn right.
                left_motor  =  TURN_SPEED
                right_motor = -TURN_SPEED

        elif max_front > NEAR_THRESHOLD:
            # Something nearby but not blocking — steer around it while moving.
            rotate_ticks = 0
            diff = left_sum - right_sum  # +: obstacle on left → bias right
            correction = diff * STEER_GAIN
            left_motor  = FORWARD_SPEED + correction
            right_motor = FORWARD_SPEED - correction

        else:
            # Clear (or faint) readings — just roll forward.
            rotate_ticks = 0
            left_motor  = FORWARD_SPEED
            right_motor = FORWARD_SPEED

        await set_motors(node, left_motor, right_motor)

        print(
            f"front={front} rear={rear} acc={acc} "
            f"tilt={tilt_ticks} rot={rotate_ticks} "
            f"L/R={int(left_motor)}/{int(right_motor)}"
        )

        await client.sleep(LOOP_DT)


async def phase3_celebrate(node):
    print("Phase 3: celebrate!")
    for freq, dur in [(523, 9), (659, 9), (784, 9), (1047, 24)]:
        await play_freq(node, freq, dur)


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
    print("Press Ctrl+C to stop\n")

    try:
        await phase1_startup(node)
        await phase2_escape(node, client)
        await phase3_celebrate(node)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        try:
            await set_motors(node, 0, 0)
        except Exception:
            pass
        try:
            await node.unlock()
        except Exception:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Thymio labyrinth escaper")
    parser.add_argument("--addr", default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")
    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
