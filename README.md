# thymo-demo

## Purpose
This project is for experimenting with the Thymio II robot. Each subdirectory contains a robotic program that can be used to control and run the Thymio II robot.

## Project Structure

### follow-finger/
Follower following program for the Thymio II robot.

**Usage:**
```bash
cd follow-finger && python -m uvicorn main.py --host 127.0.0.1
```

### thymio_env/
Thymo II robot environment configuration and setup files.

## Requirements

- Python 3.x with required dependencies listed in `requirements.txt`
```