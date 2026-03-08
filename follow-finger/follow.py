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
        
        speed = 0
        v = {
            "motor.left.target": [speed],
            "motor.right.target": [speed]
        }

        ## First we find the sensor with the highest value (closest object)
        max_val = 0
        max_sensor_index = -1
        move_forward = False
        move_backwards = False

        for i, val in enumerate(node.v.prox.horizontal):
            if val > max_val:
                max_val = val
                max_sensor_index = i

                ## Any value is greater than 3500, we need to move back 
                if val > 3500:
                    move_forward = False
                    move_backwards = True
                elif val > 0 and move_backwards == False:
                    move_forward = True

    
        ## At this point, we know that
        if move_backwards:
            speed = 50
            v["motor.left.target"] = [-speed]
            v["motor.right.target"] = [-speed]
        elif move_forward:
            speed = 50

            if max_sensor_index == 0 or max_sensor_index == 1:
                v["motor.left.target"] = [-speed]
                v["motor.right.target"] = [speed]
            elif max_sensor_index == 3 or max_sensor_index == 4:
                v["motor.left.target"] = [speed]
                v["motor.right.target"] = [-speed]
            else: 
                ## Middle sensor is closest. 
                ## We may still need some finetuning
                sensor_1 = node.v.prox.horizontal[1]
                sensor_3 = node.v.prox.horizontal[3]

                if abs(sensor_1 - sensor_3) > 50:
                    # Need some micro adjustments 
                    if sensor_1 > sensor_3:
                        # If left sensor is much stronger, turn left
                        v["motor.left.target"] = [-speed]
                        v["motor.right.target"] = [speed]
                    else:
                        # If right sensor is much stronger, turn right
                        v["motor.left.target"] = [speed]
                        v["motor.right.target"] = [-speed]
                else:
                    # Otherwise, move forward or backwards
                    if sensors[2] > 0:
                        if sensors[2] > 4000:
                            speed = -speed # Move backward if too close
                        elif sensors[2] > 3500:
                            ## Stay still 
                            speed = 0

                    # Send the same speed to both motors
                    v = {
                        "motor.left.target": [speed],
                        "motor.right.target": [speed]
                    }
                        


        # Execute movement
        await node.set_variables(v)
        
        # Small sleep to let the robot breathe
        await client.sleep(0.1)


# Replace ClientAsync.run(run_thymio()) with:
if __name__ == "__main__":
    asyncio.run(run_thymio())