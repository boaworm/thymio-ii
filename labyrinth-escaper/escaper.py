import asyncio
import argparse
import time
from tdmclient import ClientAsync
from tdmclient.client import DisconnectedError

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
LOOP_DT = 0.2   # 5 Hz — slower to avoid flooding the wireless TDM link

# Escape condition: accelerometer Y reads >= this value for several ticks in a
# row (robot has been tilted — "lifted to victory").
ACC_Y_THRESHOLD    = 2
ACC_CONFIRM_TICKS  = 4

# Safety: if we've been rotating in place for this long without finding a way
# out, nudge backwards briefly to unstick (dead-end recovery).
RETREAT_TICKS      = 4    # reverse before turning to pull front away from wall
STUCK_ROTATE_TICKS = 40
REVERSE_TICKS      = 6


# On-board watchdog: stops motors if Python stops refreshing within ~1s.
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
    """Load a watchdog program that stops motors on disconnect."""
    await node.stop()
    await node.compile(WATCHDOG_PROGRAM, load=True)
    await node.run()
    print("Watchdog loaded")


async def play_freq(node, freq, duration_60ths=12):
    await node.stop()
    await node.compile(f"call sound.freq({freq}, {duration_60ths})", load=True)
    await node.run()
    await asyncio.sleep(duration_60ths / 60 + 0.15)


async def set_motors(node, left, right):
    await node.set_variables({
        "motor.left.target":  [int(left)],
        "motor.right.target": [int(right)],
        "watchdog": [10],
    })


async def phase1_startup(node):
    print("Phase 1: startup")
    for _ in range(3):
        await play_freq(node, 880)


async def phase2_escape(node, client):
    print("Phase 2: escaping")
    await setup_watchdog(node)
    t0 = time.monotonic()
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
            front[0] > BLOCK_THRESHOLD or
            front[1] > BLOCK_THRESHOLD or
            front[2] > BLOCK_THRESHOLD or
            front[3] > BLOCK_THRESHOLD or
            front[4] > BLOCK_THRESHOLD
        )

        if front_blocked:
            rotate_ticks += 1

            if rotate_ticks <= RETREAT_TICKS:
                # Phase 1: reverse to pull front away from wall.
                left_motor  = -FORWARD_SPEED
                right_motor = -FORWARD_SPEED
            elif rotate_ticks > STUCK_ROTATE_TICKS + RETREAT_TICKS:
                # Stuck — extended reverse if rear clear.
                rear_blocked = max(rear) > NEAR_THRESHOLD
                if not rear_blocked:
                    left_motor  = -FORWARD_SPEED
                    right_motor = -FORWARD_SPEED
                    if rotate_ticks >= STUCK_ROTATE_TICKS + RETREAT_TICKS + REVERSE_TICKS:
                        rotate_ticks = 0
                else:
                    left_motor, right_motor = TURN_SPEED, -TURN_SPEED
            elif left_sum <= right_sum:
                # Phase 2: rotate toward freer side.
                left_motor  = -TURN_SPEED
                right_motor =  TURN_SPEED
            else:
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

        t = time.monotonic() - t0
        print(
            f"[{t:6.1f}s] front={front} rear={rear} acc={acc} "
            f"tilt={tilt_ticks} rot={rotate_ticks} "
            f"L/R={int(left_motor)}/{int(right_motor)}"
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
            except DisconnectedError:
                print("\nTDM disconnected — reconnecting...")
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
    parser = argparse.ArgumentParser(description="Thymio labyrinth escaper")
    parser.add_argument("--addr", default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")
    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
