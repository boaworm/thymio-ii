import asyncio
import argparse
import time
from tdmclient import ClientAsync
from tdmclient.client import DisconnectedError

# Sensor layout on Thymio II:
# Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
# Rear:   [5] [6]              (both on the back)
# Scale: 0 = nothing, ~5000 = touching (higher = closer)

# Wall detection thresholds
WALL_DETECT     = 200   # side wall is present (following it)
WALL_CLOSE      = 1500  # too close to the side wall — nudge away
BLOCK_THRESHOLD = 2800  # wall ahead — must turn

# Speeds
FORWARD_SPEED = 120
TURN_INNER    = 30   # wall-side wheel: slow forward arc
TURN_OUTER    = 120  # far-side wheel: drives the turn

# Proportional correction to maintain wall distance while cruising
WALL_GAIN = 0.03

# Loop timing
LOOP_DT = 0.2   # 5 Hz — slower to avoid flooding the wireless TDM link

# Escape condition: accelerometer Y >= threshold for N consecutive ticks.
ACC_Y_THRESHOLD   = 2
ACC_CONFIRM_TICKS = 4

# Stuck detection: if blocked for this many ticks, reverse briefly.
RETREAT_TICKS = 4
STUCK_TICKS   = 15
REVERSE_TICKS = 8

# Pledge rotation tracking.
# Estimated degrees the robot turns per tick per unit of (right_motor - left_motor).
# With wheel base ~95mm and motor units roughly mm/s:
#   deg/tick ≈ (1 / 95) * LOOP_DT * (180 / π) ≈ 0.060
# Tune this if the robot over- or under-counts rotations.
DEG_PER_SPEED_TICK = 0.060

# How close to 0° total rotation counts as "back to original heading".
ROTATION_ZERO_TOL = 30  # degrees


WATCHDOG_PROGRAM = """\
var watchdog = 0
timer.period[0] = 300
onevent timer0
  if watchdog > 0 then
    watchdog = watchdog - 1
  else
    motor.left.target = 0
    motor.right.target = 0
  end
"""


async def setup_watchdog(node):
    """Skip watchdog for stability."""
    await node.stop()
    print("Watchdog skipped")


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
    """Pledge algorithm: go straight, wall-follow on collision, break free
    when cumulative rotation returns to zero."""
    print("Phase 2: escaping (Pledge algorithm)")
    await setup_watchdog(node)
    t0 = time.monotonic()
    tilt_ticks = 0
    blocked_ticks = 0
    total_rotation = 0.0  # cumulative degrees; positive = clockwise (right)
    mode = "STRAIGHT"     # or "WALL_FOLLOW"

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
        left_sum  = front[0] + front[1]
        right_sum = front[3] + front[4]
        front_blocked = (
            front[1] > BLOCK_THRESHOLD or
            front[2] > BLOCK_THRESHOLD or
            front[3] > BLOCK_THRESHOLD
        )
        left_blocked = front[0] > BLOCK_THRESHOLD  # wall-side sensor dangerously close

        if mode == "STRAIGHT":
            # Drive forward in the original heading until we hit a wall.
            if front_blocked or left_blocked:
                blocked_ticks += 1
                if blocked_ticks <= RETREAT_TICKS:
                    left_motor  = -FORWARD_SPEED
                    right_motor = -FORWARD_SPEED
                    state = "RETREAT"
                else:
                    mode = "WALL_FOLLOW"
                    total_rotation = 0.0
                    left_motor  = TURN_OUTER
                    right_motor = TURN_INNER
                    state = "BLOCKED→start_follow"
            else:
                blocked_ticks = 0
                left_motor  = FORWARD_SPEED
                right_motor = FORWARD_SPEED
                state = "STRAIGHT"

        else:  # WALL_FOLLOW — left-hand rule
            has_left_wall  = left_sum > WALL_DETECT
            left_too_close = left_sum > WALL_CLOSE

            if front_blocked or left_blocked:
                blocked_ticks += 1

                if blocked_ticks <= RETREAT_TICKS:
                    left_motor  = -FORWARD_SPEED
                    right_motor = -FORWARD_SPEED
                    state = "WF:RETREAT"
                elif blocked_ticks > STUCK_TICKS + RETREAT_TICKS:
                    rear = list(sensors[5:7])
                    rear_blocked = max(rear) > 3500  # rear must be VERY close
                    if not rear_blocked:
                        left_motor  = -FORWARD_SPEED
                        right_motor = -FORWARD_SPEED
                        state = "WF:STUCK→reverse"
                        if blocked_ticks >= STUCK_TICKS + RETREAT_TICKS + REVERSE_TICKS:
                            blocked_ticks = 0
                    else:
                        left_motor  = TURN_OUTER
                        right_motor = TURN_INNER
                        state = "WF:STUCK→arc_right"
                else:
                    # Arc right — wall is on left, so left=fast right=slow.
                    left_motor  = TURN_OUTER
                    right_motor = TURN_INNER
                    state = "WF:BLOCKED→right"

            elif not has_left_wall:
                # Arc left to hug — wall side (left) slow, far side (right) fast.
                blocked_ticks = 0
                left_motor  = TURN_INNER
                right_motor = TURN_OUTER
                state = "WF:HUG→left"

            elif left_too_close:
                blocked_ticks = 0
                correction = (left_sum - WALL_CLOSE) * WALL_GAIN
                left_motor  = FORWARD_SPEED + correction
                right_motor = FORWARD_SPEED - correction
                state = "WF:TOO_CLOSE→nudge"

            else:
                blocked_ticks = 0
                target = (WALL_DETECT + WALL_CLOSE) // 2
                error = left_sum - target
                correction = error * WALL_GAIN
                left_motor  = FORWARD_SPEED + correction
                right_motor = FORWARD_SPEED - correction
                state = "WF:FOLLOW"

            # Accumulate rotation: positive = clockwise.
            delta_deg = (right_motor - left_motor) * DEG_PER_SPEED_TICK
            total_rotation += delta_deg

            # Pledge exit: if rotation has returned to ~0 and path ahead is clear,
            # resume going straight.
            if (abs(total_rotation) < ROTATION_ZERO_TOL and
                    not front_blocked and not has_left_wall):
                mode = "STRAIGHT"
                state = "PLEDGE→straight"

        await set_motors(node, left_motor, right_motor)

        t = time.monotonic() - t0
        print(
            f"[{t:6.1f}s] front={front} L={left_sum} R={right_sum} "
            f"acc={acc} tilt={tilt_ticks} rot={total_rotation:+.0f}° "
            f"mode={state} L/R={int(left_motor)}/{int(right_motor)}"
        )

        await client.sleep(LOOP_DT)


async def phase3_celebrate(node):
    print("Phase 3: celebrate!")
    for freq, dur in [(523, 9), (659, 9), (784, 9), (1047, 24)]:
        await play_freq(node, freq, dur)


async def connect(kwargs):
    while True:
        client = ClientAsync(**kwargs)
        try:
            node = await asyncio.wait_for(client.wait_for_node(), timeout=10.0)
            await node.lock()
            print(f"Connected to {node.id}")
            return client, node
        except (asyncio.TimeoutError, Exception) as e:
            print(f"Connection failed ({e}), retrying in 2s...")
            await asyncio.sleep(2)


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

    client, node = await connect(kwargs)
    print("Press Ctrl+C to stop\n")

    startup_done = False
    try:
        while True:
            try:
                if not startup_done:
                    await phase1_startup(node)
                    startup_done = True
                await phase2_escape(node, client)
                await phase3_celebrate(node)
                return
            except DisconnectedError as e:
                print(f"\nTDM disconnected ({e}) — reconnecting...")
                client, node = await connect(kwargs)
                startup_done = True
            except Exception as e:
                print(f"\nError ({e}) — reconnecting...")
                client, node = await connect(kwargs)
                startup_done = True
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            await set_motors(node, 0, 0)
        except Exception:
            pass
        try:
            await node.unlock()
        except Exception:
            pass
        try:
            client.close()
        except Exception:
            pass
        print("Cleaned up.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Thymio labyrinth escaper (Pledge algorithm)")
    parser.add_argument("--addr", default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")
    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
