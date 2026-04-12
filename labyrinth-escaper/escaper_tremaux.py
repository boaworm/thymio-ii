import asyncio
import argparse
import math
import time
from tdmclient import ClientAsync
from tdmclient.client import DisconnectedError

# Sensor layout on Thymio II:
# Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
# Rear:   [5] [6]              (both on the back)
# Scale: 0 = nothing, ~5000 = touching (higher = closer)

# Wall detection thresholds
WALL_DETECT     = 200   # side wall is present
WALL_CLOSE      = 1500  # too close to the side wall
BLOCK_THRESHOLD = 2800  # wall ahead — must turn

# Speeds
FORWARD_SPEED = 120
TURN_SPEED    = 110

# Proportional correction for centering in corridors
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

# Odometry parameters (tunable — run straight and measure actual distance)
WHEEL_BASE  = 95.0   # mm between wheels
SPEED_SCALE = 0.4    # mm/s per motor speed unit

# Grid cell size for visit map.  Roughly robot-width so each corridor segment
# is one cell wide.
CELL_SIZE = 60  # mm

# How many ticks to rotate ~90° in place.
# With TURN_SPEED on both wheels (opposite), measure and adjust.
TICKS_PER_90 = 12


# ---------------------------------------------------------------------------
# Odometry tracker
# ---------------------------------------------------------------------------

class OdometryTracker:
    def __init__(self):
        self.x = 0.0        # mm
        self.y = 0.0        # mm
        self.heading = 0.0   # radians, 0 = initial forward

    def update(self, left_speed, right_speed, dt):
        lmm = left_speed * SPEED_SCALE
        rmm = right_speed * SPEED_SCALE
        v = (lmm + rmm) / 2.0
        omega = (rmm - lmm) / WHEEL_BASE
        self.heading += omega * dt
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt

    def grid_cell(self):
        return (round(self.x / CELL_SIZE), round(self.y / CELL_SIZE))

    def cell_ahead(self, direction_offset=0.0):
        """Grid cell ~1 cell ahead in a direction relative to current heading.
        direction_offset: 0=forward, -pi/2=left, +pi/2=right."""
        angle = self.heading + direction_offset
        nx = self.x + CELL_SIZE * math.cos(angle)
        ny = self.y + CELL_SIZE * math.sin(angle)
        return (round(nx / CELL_SIZE), round(ny / CELL_SIZE))


# ---------------------------------------------------------------------------
# Visit map
# ---------------------------------------------------------------------------

class VisitMap:
    def __init__(self):
        self.counts = {}

    def visit(self, cell):
        self.counts[cell] = self.counts.get(cell, 0) + 1

    def get(self, cell):
        return self.counts.get(cell, 0)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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


async def phase3_celebrate(node):
    print("Phase 3: celebrate!")
    for freq, dur in [(523, 9), (659, 9), (784, 9), (1047, 24)]:
        await play_freq(node, freq, dur)


# ---------------------------------------------------------------------------
# Rotate in place by a given number of ticks (+ticks = clockwise/right)
# ---------------------------------------------------------------------------

async def rotate_ticks(node, client, ticks, odo):
    """Rotate in place. Positive ticks = turn right, negative = turn left.
    Updates odometry while spinning."""
    if ticks == 0:
        return
    if ticks > 0:
        left_m, right_m = TURN_SPEED, -TURN_SPEED
    else:
        left_m, right_m = -TURN_SPEED, TURN_SPEED
        ticks = -ticks

    await set_motors(node, left_m, right_m)
    for _ in range(ticks):
        await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
        ls = node.v.motor.left.speed
        rs = node.v.motor.right.speed
        odo.update(ls, rs, LOOP_DT)
        await client.sleep(LOOP_DT)
    await set_motors(node, 0, 0)
    await client.sleep(0.05)


# ---------------------------------------------------------------------------
# Main escape loop — Tremaux's algorithm
# ---------------------------------------------------------------------------

async def phase2_escape(node, client):
    """Tremaux's algorithm: track visited cells, prefer unvisited paths."""
    print("Phase 2: escaping (Tremaux's algorithm)")
    await setup_watchdog(node)
    t0 = time.monotonic()

    odo = OdometryTracker()
    visits = VisitMap()
    tilt_ticks = 0
    blocked_ticks = 0
    mode = "CORRIDOR"  # CORRIDOR | JUNCTION | DEAD_END

    while True:
        await node.wait_for_variables({
            "prox.horizontal", "acc",
            "motor.left.speed", "motor.right.speed",
        })
        sensors = node.v.prox.horizontal
        acc = list(node.v.acc)
        front = list(sensors[:5])
        ls = node.v.motor.left.speed
        rs = node.v.motor.right.speed

        # --- Update odometry & visit map ---
        odo.update(ls, rs, LOOP_DT)
        cell = odo.grid_cell()
        visits.visit(cell)

        # --- Victory check ---
        if acc[1] >= ACC_Y_THRESHOLD:
            tilt_ticks += 1
        else:
            tilt_ticks = 0
        if tilt_ticks >= ACC_CONFIRM_TICKS:
            await set_motors(node, 0, 0)
            print(f"Phase 2 complete: escaped! acc={acc}")
            return

        # --- Sensor analysis ---
        left_sum  = front[0] + front[1]
        right_sum = front[3] + front[4]
        front_blocked = (
            front[0] > BLOCK_THRESHOLD or
            front[1] > BLOCK_THRESHOLD or
            front[2] > BLOCK_THRESHOLD or
            front[3] > BLOCK_THRESHOLD or
            front[4] > BLOCK_THRESHOLD
        )
        left_open  = left_sum < WALL_DETECT
        right_open = right_sum < WALL_DETECT
        front_open = not front_blocked

        open_count = sum([left_open, front_open, right_open])

        # --- Classify situation ---
        if not front_open and not left_open and not right_open:
            mode = "DEAD_END"
        elif open_count >= 2:
            mode = "JUNCTION"
        else:
            mode = "CORRIDOR"

        # --- Act based on mode ---
        if mode == "DEAD_END":
            blocked_ticks += 1

            if blocked_ticks <= RETREAT_TICKS:
                # Phase 1: reverse to pull front away from wall.
                left_motor = -FORWARD_SPEED
                right_motor = -FORWARD_SPEED
                state = "DEAD_END→retreat"
            elif blocked_ticks > STUCK_TICKS + RETREAT_TICKS:
                rear = list(sensors[5:7])
                rear_blocked = max(rear) > 3500  # rear must be VERY close
                if not rear_blocked:
                    left_motor = -FORWARD_SPEED
                    right_motor = -FORWARD_SPEED
                    state = "DEAD_END→reverse"
                    if blocked_ticks >= STUCK_TICKS + RETREAT_TICKS + REVERSE_TICKS:
                        blocked_ticks = 0
                else:
                    left_motor = TURN_SPEED
                    right_motor = -TURN_SPEED
                    state = "DEAD_END→spin"
            else:
                # Phase 2: turn toward the less-visited side
                cell_left = odo.cell_ahead(-math.pi / 2)
                cell_right = odo.cell_ahead(math.pi / 2)
                if visits.get(cell_left) <= visits.get(cell_right):
                    left_motor = -TURN_SPEED
                    right_motor = TURN_SPEED
                    state = "DEAD_END→left"
                else:
                    left_motor = TURN_SPEED
                    right_motor = -TURN_SPEED
                    state = "DEAD_END→right"

        elif mode == "JUNCTION":
            blocked_ticks = 0

            # Build list of (visit_count, direction_name, motor_pair)
            options = []
            if front_open:
                c = odo.cell_ahead(0)
                options.append((visits.get(c), "fwd", FORWARD_SPEED, FORWARD_SPEED))
            if left_open:
                c = odo.cell_ahead(-math.pi / 2)
                options.append((visits.get(c), "left", -TURN_SPEED, TURN_SPEED))
            if right_open:
                c = odo.cell_ahead(math.pi / 2)
                options.append((visits.get(c), "right", TURN_SPEED, -TURN_SPEED))

            # Sort: lowest visits first, break ties by preferring forward
            priority = {"fwd": 0, "left": 1, "right": 2}
            options.sort(key=lambda o: (o[0], priority.get(o[1], 9)))

            best = options[0]
            left_motor = best[2]
            right_motor = best[3]
            state = f"JUNCTION→{best[1]}(v={best[0]})"

        else:  # CORRIDOR
            blocked_ticks = 0

            if front_blocked:
                # Shouldn't normally happen in CORRIDOR (caught by DEAD_END)
                # but handle gracefully: turn toward less-visited side
                cell_left = odo.cell_ahead(-math.pi / 2)
                cell_right = odo.cell_ahead(math.pi / 2)
                if visits.get(cell_left) <= visits.get(cell_right):
                    left_motor = -TURN_SPEED
                    right_motor = TURN_SPEED
                    state = "CORRIDOR→turn_left"
                else:
                    left_motor = TURN_SPEED
                    right_motor = -TURN_SPEED
                    state = "CORRIDOR→turn_right"

            elif left_sum > WALL_CLOSE or right_sum > WALL_CLOSE:
                # Too close to a wall — steer away
                diff = left_sum - right_sum
                correction = diff * WALL_GAIN
                left_motor = FORWARD_SPEED + correction
                right_motor = FORWARD_SPEED - correction
                state = "CORRIDOR→nudge"

            elif left_sum > WALL_DETECT or right_sum > WALL_DETECT:
                # Wall on one or both sides — stay centered
                diff = left_sum - right_sum
                correction = diff * WALL_GAIN
                left_motor = FORWARD_SPEED + correction
                right_motor = FORWARD_SPEED - correction
                state = "CORRIDOR→follow"

            else:
                # Open ground — drive forward
                left_motor = FORWARD_SPEED
                right_motor = FORWARD_SPEED
                state = "CORRIDOR→forward"

        await set_motors(node, left_motor, right_motor)

        v = visits.get(cell)
        t = time.monotonic() - t0
        print(
            f"[{t:6.1f}s] front={front} L={left_sum} R={right_sum} "
            f"pos=({odo.x:.0f},{odo.y:.0f}) cell={cell} v={v} "
            f"hdg={math.degrees(odo.heading):.0f}° "
            f"state={state} L/R={int(left_motor)}/{int(right_motor)}"
        )

        await client.sleep(LOOP_DT)


# ---------------------------------------------------------------------------
# Connection & main
# ---------------------------------------------------------------------------

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
    parser = argparse.ArgumentParser(description="Thymio labyrinth escaper (Tremaux's algorithm)")
    parser.add_argument("--addr", default=None, help="Robot address")
    parser.add_argument("--port", type=int, default=None, help="Robot port")
    parser.add_argument("--ws", action="store_true", help="Use WebSocket")
    parser.add_argument("--no-zeroconf", action="store_true", help="Disable zeroconf")
    args = parser.parse_args()
    asyncio.run(run_thymio(args.addr, args.port, args.ws, not args.no_zeroconf))
