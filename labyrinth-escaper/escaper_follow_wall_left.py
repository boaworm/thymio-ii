import asyncio
import argparse
from tdmclient import ClientAsync

# Sensor layout on Thymio II:
# Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
# Rear:   [5] [6]              (both on the back)
# Scale: 0 = nothing, ~5000 = touching (higher = closer)

# Wall detection thresholds
WALL_DETECT    = 200   # side wall is present (following it)
WALL_CLOSE     = 1500  # too close to the side wall — nudge away
BLOCK_THRESHOLD = 2800  # wall ahead — must turn

# Speeds
FORWARD_SPEED = 120
TURN_SPEED    = 110

# Proportional correction to maintain wall distance while cruising
WALL_GAIN = 0.03

# Loop timing
LOOP_DT = 0.1   # 10 Hz

# Escape condition: accelerometer Y >= threshold for N consecutive ticks.
ACC_Y_THRESHOLD   = 2
ACC_CONFIRM_TICKS = 4

# Stuck detection: if blocked for this many ticks, reverse briefly.
STUCK_TICKS   = 15   # ~1.5s of spinning without clearing
REVERSE_TICKS = 8    # ~0.8s of backing up


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
    """Left-hand rule: keep a wall on the left side."""
    print("Phase 2: escaping (left-hand wall follower)")
    tilt_ticks = 0
    blocked_ticks = 0

    while True:
        await node.wait_for_variables({"prox.horizontal", "acc"})
        sensors = node.v.prox.horizontal
        acc = list(node.v.acc)
        front = list(sensors[:5])

        # --- Victory check ---
        if acc[1] >= ACC_Y_THRESHOLD:
            tilt_ticks += 1
        else:
            tilt_ticks = 0
        if tilt_ticks >= ACC_CONFIRM_TICKS:
            await set_motors(node, 0, 0)
            print(f"Phase 2 complete: escaped! acc={acc}")
            return

        # --- Sensor sums ---
        left_sum  = front[0] + front[1]   # wall on the left
        right_sum = front[3] + front[4]   # wall on the right
        front_blocked = (
            front[1] > BLOCK_THRESHOLD or
            front[2] > BLOCK_THRESHOLD or
            front[3] > BLOCK_THRESHOLD
        )
        has_left_wall  = left_sum > WALL_DETECT
        left_too_close = left_sum > WALL_CLOSE

        # --- Left-hand rule ---
        # Priority: blocked ahead → turn right
        #           no wall on left but walls nearby → turn left to regain it
        #           no walls anywhere → drive forward to find walls
        #           wall on left → cruise forward with distance correction
        anything_nearby = max(front) > WALL_DETECT

        if front_blocked:
            blocked_ticks += 1

            if blocked_ticks > STUCK_TICKS:
                # Stuck spinning — back up to make room.
                rear = list(sensors[5:7])
                rear_blocked = max(rear) > WALL_CLOSE
                if not rear_blocked:
                    left_motor  = -FORWARD_SPEED
                    right_motor = -FORWARD_SPEED
                    state = "STUCK→reverse"
                    if blocked_ticks >= STUCK_TICKS + REVERSE_TICKS:
                        blocked_ticks = 0
                else:
                    left_motor  =  TURN_SPEED
                    right_motor = -TURN_SPEED
                    state = "STUCK→spin"
            else:
                # Wall ahead — turn right (away from our followed left wall).
                left_motor  =  TURN_SPEED
                right_motor = -TURN_SPEED
                state = "BLOCKED→right"

        else:
            blocked_ticks = 0

            if not has_left_wall and anything_nearby:
                # Lost the left wall but other walls nearby (corner) — turn left.
                left_motor  = -TURN_SPEED
                right_motor =  TURN_SPEED
                state = "CORNER→left"

            elif not has_left_wall:
                # Nothing in sensor range — drive forward to find walls.
                left_motor  = FORWARD_SPEED
                right_motor = FORWARD_SPEED
                state = "SEEK_WALL"

            elif left_too_close:
                # Too close to the left wall — steer right while moving forward.
                correction = (left_sum - WALL_CLOSE) * WALL_GAIN
                left_motor  = FORWARD_SPEED + correction
                right_motor = FORWARD_SPEED - correction
                state = "TOO_CLOSE→nudge_right"

            else:
                # Cruising with wall on left at a good distance.
                # Small proportional correction to hold distance.
                target = (WALL_DETECT + WALL_CLOSE) // 2
                error = left_sum - target
                correction = error * WALL_GAIN
                left_motor  = FORWARD_SPEED + correction
                right_motor = FORWARD_SPEED - correction
                state = "FOLLOW"

        await set_motors(node, left_motor, right_motor)

        print(
            f"front={front} L={left_sum} R={right_sum} "
            f"acc={acc} tilt={tilt_ticks} "
            f"state={state} L/R={int(left_motor)}/{int(right_motor)}"
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
    parser = argparse.ArgumentParser(description="Thymio labyrinth escaper (left-hand rule)")
    parser.add_argument("--addr", default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")
    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
