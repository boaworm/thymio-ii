import asyncio
from tdmclient import ClientAsync

async def main():
    client = ClientAsync()
    node = await client.wait_for_node()
    await node.lock()

    print(f"Connected to {node.id}")

    # Play a cheerful fanfare using system sounds
    # Sound 2 = success, Sound 3 = error (lower), Sound 4 = targeted
    sounds = [2, 2, 2, 3, 4]
    durations = [0.2, 0.2, 0.2, 0.3, 0.5]

    for sound, duration in zip(sounds, durations):
        program = f"call sound.system({sound})"
        await node.compile(program, load=True)
        await node.run()
        print(f"Playing sound.system({sound})")
        await asyncio.sleep(duration)

    await node.stop()
    await node.unlock()
    print("Done!")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
