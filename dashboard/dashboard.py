import asyncio
from tdmclient import ClientAsync

async def run_dashboard():
    client = ClientAsync()
    node = await client.wait_for_node()
    await node.lock()
    await node.watch(variables=True)

    print("--- Thymio Sensor Dashboard ---")
    print("Press Ctrl+C to exit")

    try:
        while True:
            # Wait for fresh sensor data
            await node.wait_for_variables()
            
            # 1. Clear terminal screen (or move cursor to top)
            # This makes it look like a static table updating in real-time
            print("\033[H\033[J", end="") 

            print(f"{'Sensor Name':<20} | {'Value':<15}")
            print("-" * 40)

            # 2. Horizontal Proximity (All 7 sensors)
            for i, val in enumerate(node.v.prox.horizontal):
                name = f"Prox Horizontal [{i}]"
                print(f"{name:<20} | {val:<15}")

            print("-" * 40)

            # 3. Ground Proximity
            for i, val in enumerate(node.v.prox.ground.reflected):
                name = f"Prox Ground [{i}]"
                print(f"{name:<20} | {val:<15}")

            print("-" * 40)

            # 4. Accelerometer
            acc = node.v.acc
            print(f"{'Acc X':<20} | {acc[0]:<15}")
            print(f"{'Acc Y':<20} | {acc[1]:<15}")
            print(f"{'Acc Z':<20} | {acc[2]:<15}")

            # 5. Buttons
            btns = [
                ("Forward", node.v.button.forward),
                ("Backward", node.v.button.backward),
                ("Left", node.v.button.left),
                ("Right", node.v.button.right),
                ("Center", node.v.button.center)
            ]
            print("-" * 40)
            for name, val in btns:
                status = "PRESSED" if val == 1 else "OFF"
                print(f"{'Button ' + name:<20} | {status:<15}")

            # Update rate: 10Hz
            await client.sleep(0.1)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        await node.unlock()
        print("\nDashboard Closed.")

if __name__ == "__main__":
    asyncio.run(run_dashboard())