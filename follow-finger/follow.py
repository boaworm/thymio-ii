import asyncio
from tdmclient import ClientAsync

async def run_thymio():
    # Create a client and connect to the local TDM
    client = ClientAsync()
    
    # Wait for a robot to be found
    node = await client.wait_for_node()
    
    # Lock the robot so only this script controls it
    await node.lock()
    
    print(f"Connected to {node.id}")

    # Main loop
    while True:
        # 1. Get the latest sensor data
        await node.wait_for_variables({"prox.horizontal"})
        
        # 2. Access the array (Python lists start at index 0)
        sensors = node.v.prox.horizontal
        
        # 3. Logic: If center sensor > 1000, move forward
        if sensors[2] > 1000:
            speed = 200
        else:
            speed = 0
            
        # 4. Send the speed to the motors
        v = {
            "motor.left.target": [speed],
            "motor.right.target": [speed]
        }
        await node.set_variables(v)
        
        # Small sleep to let the robot breathe
        await client.sleep(0.1)


# Replace ClientAsync.run(run_thymio()) with:
if __name__ == "__main__":
    asyncio.run(run_thymio())