import socket
import time
from gpiozero import DistanceSensor, DigitalOutputDevice, LED
import threading
import json
import os

# ========== Global Variables ==========
production_quantity = 0  # Total production quantity
box_count = 0  # Items in the current box
total_boxes = 0  # Total number of bad boxes
should_stop = False  # Force stop flag
lock = threading.Lock()  # Synchronization lock
current_stage = ""
# Save file path
FORCE_QUIT_DIR = "/home/juj/final_ws/Force_quit"
FORCE_QUIT_FILE = os.path.join(FORCE_QUIT_DIR, "state.json")

sensor = DistanceSensor(echo=24, trigger=23, max_distance=5)  # Ultrasonic sensor initialization

# Motor specifications
DEGREE_PER_STEP = 5.625 / 64  # Degree per step for 28BYJ-48

# Define GPIO pins
StepPins = [DigitalOutputDevice(8), DigitalOutputDevice(9),
            DigitalOutputDevice(10), DigitalOutputDevice(11)]

# Step counter
g_nStepCounter = 0

# Single coil stepping sequence
StepCount = 4
Seq = [[0, 0, 0, 1],
       [0, 0, 1, 0],
       [0, 1, 0, 0],
       [1, 0, 0, 0]]

# Relay
Relay = LED(3)

# Step_8825
CONST_PIN_MOT_DIR_8825 = DigitalOutputDevice(20)
CONST_PIN_MOT_STEP_8825 = DigitalOutputDevice(21)
CONST_MOT_SPR_8825 = 48
CONST_MOT_DELAY_8825 = 0.0208
CONST_MOT_DEGREE_8825 = 7.5

# ========== Functions ==========

def create_udp_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('192.168.47.149', 10000))
    sock.setblocking(False)
    return sock

def check_terminal_command(sock):
    try:
        data, addr = sock.recvfrom(1024)
        if data:
            reply = "replly command "+data.decode('utf-8').strip() 
            sock.send(reply)
            return data.decode('utf-8').strip()  # Return the received command
    except BlockingIOError:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
    return None  # Return None if no data is received


def save_state():
    """Save the current state to a file."""
    state = {
        "production_quantity": production_quantity,
        "box_count": box_count,
        "total_boxes": total_boxes,
        "current_stage": current_stage
    }
    if not os.path.exists(FORCE_QUIT_DIR):
        os.makedirs(FORCE_QUIT_DIR)
    with open(FORCE_QUIT_FILE, "w") as file:
        json.dump(state, file)
    print(f"[Raspberry Pi] State saved successfully: {FORCE_QUIT_FILE}")

def load_state():
    """Load the saved state."""
    global production_quantity, box_count, total_boxes, current_stage
    if os.path.exists(FORCE_QUIT_FILE):
        with open(FORCE_QUIT_FILE, "r") as file:
            state = json.load(file)
        production_quantity = state.get("production_quantity", 0)
        box_count = state.get("box_count", 0)
        total_boxes = state.get("total_boxes", 0)
        current_stage = state.get("current_stage", "Initializing")
        print(f"[Raspberry Pi] Previous state loaded: {state}")
    else:
        print("[Raspberry Pi] No previous state file found. Starting a new task.")
        current_stage = "Initializing"


def handle_auto_mode(sock):
    """
    Handle automatic production mode.
    """
    global should_stop
    global production_quantity
    receive_task(sock)  # Receive task first
    print("[Auto Mode] Starting automatic production...")

    while production_quantity > 0:
        if should_stop:
            save_state()
            print("[Auto Mode] Stopping production...")
            return

        print(f"[Raspberry Pi] Remaining production quantity: {production_quantity}")

        dispatch_item()
        start_conveyor_belt()

        if monitor_distance():
            result = capture_and_send_to_server(sock)
            if result == "Good":
                process_good_item()
            elif result == "Bad":
                process_bad_item()

            production_quantity -= 1  # Decrease production quantity after processing

    print("[Auto Mode] Production process completed.")
    save_state()  # Save state after completion


def handle_part_mode(sock):
    print("Part mode activated. Waiting for 'quantity_get','start_cv', 'stop_cv', 'good_process', 'bad_process', 'dp_item', 'monitor_distance', 'exit' command...")
    while True:
        command = check_terminal_command(sock)  # Call the reusable function
        if command:
            if command.lower() == "quantity_get":
                print("Executing quantity_get in part mode.")
            elif command.lower() == "start_cv":
                start_conveyor_belt()
                print("start running convayor part mode.")
            elif command.lower() == "stop_cv":
                stop_conveyor_belt()
                print("stop running convayor part mode.")
            elif command.lower() == "good_process":
                process_good_item()
                print("good item processing part mode.")
            elif command.lower() == "bad_process":
                process_bad_item()
                print("bad item processing part mode.")
            elif command.lower() == "dp_item":
                dispatch_item()
                print("dispatching item part mode.")
            elif command.lower() == "motor_control":
                rotate_motor()
                print("monitoring distance part mode.")            
            elif command.lower() == "monitor_distance":
                monitor_distance()
                print("monitoring distance part mode.")
            elif command.lower() == "exit":
                print("Exiting part mode.")
                break
            else:
                print("wrong_input")

def receive_task(sock):
    """
    Receive production task request via socket.
    Input the production quantity and set it to production_quantity.
    """
    global production_quantity
    print("[Task Request] Waiting for production quantity...")
    try:
        while True:
            task_input = check_terminal_command(sock)  # Receive command from socket
            if task_input:
                if task_input.isdigit() and int(task_input) > 0:
                    production_quantity = int(task_input)
                    print(f"[Raspberry Pi] Production quantity set to: {production_quantity}")
                    break  # Exit the loop once valid input is received
                else:
                    print("[Error] Please enter a valid integer.")
    except KeyboardInterrupt:
        print("[Raspberry Pi] Task request input interrupted.")

def mode_setting(sock):
    print("UDP Server Start (local port: 10000)")
    while True:
        command = check_terminal_command(sock)  # Call the reusable function
        if command:
            if command.lower() == "auto":
                print("Mode set to auto.")
                handle_auto_mode(sock)
            elif command.lower() == "part":
                print("Mode set to part.")
                handle_part_mode(sock)
            elif command.lower() == "exit":
                print("Exiting mode setting.")
                break
            else:
                print(f"[Error] Unrecognized command: {command}")

def rotate_motor(sock):
    print("UDP Server Start (local port: 10000)")
    while True:
        command = check_terminal_command(sock)  # Call the reusable function
        if command:
            try:
                angle = float(command)  
                print(f"Rotating motor by {angle} degrees...")
                cmd_rotate_motor_28BYJ_48(angle) 
            except ValueError:
                print(f"[Error] '{command}' is not a valid number. Please enter a valid number.")
                continue 
        
def cmd_rotate_motor_28BYJ_48(angle):
    global g_nStepCounter
    steps = int(abs(angle) / DEGREE_PER_STEP)
    direction = 1 if angle > 0 else -1

    print(f"Rotating {angle} degrees ({steps} steps)")
    for _ in range(steps):
        for pin in range(4):
            if Seq[g_nStepCounter][pin] != 0:
                StepPins[pin].on()
            else:
                StepPins[pin].off()
        g_nStepCounter += direction
        if g_nStepCounter >= StepCount:
            g_nStepCounter = 0
        elif g_nStepCounter < 0:
            g_nStepCounter = StepCount - 1
        time.sleep(0.0019)
    print(f"Rotation of {angle} degrees completed!")

def cmd_org_28BYJ_48(sock):
    global g_nStepCounter
    
    while True:
        command = check_terminal_command(sock)  # Use socket to receive commands
        if command:
            if command.upper() == "P":
                print("[Simulation] Server response: Good")
                return "Good"
            elif command.upper() == "D":
                print("[Simulation] Server response: Bad")
                return "Bad"
            else:
                print("[Error] Invalid input. Enter P or D.")



    print(f"Rotating {angle} degrees ({steps} steps)")
    for _ in range(steps):
        for pin in range(4):
            if Seq[g_nStepCounter][pin] != 0:
                StepPins[pin].on()
            else:
                StepPins[pin].off()
        g_nStepCounter += direction
        if g_nStepCounter >= StepCount:
            g_nStepCounter = 0
        elif g_nStepCounter < 0:
            g_nStepCounter = StepCount - 1
        time.sleep(0.0019)

def cmd_rotate_motor_8825(angle):

    steps = int(abs(angle) / CONST_MOT_DEGREE_8825)
    direction = 1 if angle > 0 else -1
    step_counter = 0 if direction == 1 else StepCount - 1

    if direction == 1:
        CONST_PIN_MOT_DIR_8825.on()    
    else:
        CONST_PIN_MOT_DIR_8825.off()

    print(f"Rotating {angle} degrees ({steps} steps)")
    for _ in range(steps):
        CONST_PIN_MOT_STEP_8825.on()
        time.sleep(CONST_MOT_DELAY_8825)
        CONST_PIN_MOT_STEP_8825.off()
        time.sleep(CONST_MOT_DELAY_8825)
        
    print(f"Rotation of {angle} degrees completed!")


def cmd_org_8825(rps):
    CONST_PIN_MOT_DIR_8825.off()
    CONST_PIN_MOT_STEP_8825.on()
    time.sleep(CONST_MOT_DELAY_8825)
    CONST_PIN_MOT_STEP_8825.off()
    time.sleep(CONST_MOT_DELAY_8825)


def dispatch_item():
    """
    Dispatch an item: Rotate 180° and return.
    """
    print("[Raspberry Pi] Starting item dispatch")
    cmd_rotate_motor_8825(180)
    time.sleep(0.5)
    cmd_rotate_motor_8825(-180)
    print("[Raspberry Pi] Item dispatch completed")

def start_conveyor_belt():
    """
    Start the conveyor belt (via gpiozero).
    """
    Relay.on()
    print("[Raspberry Pi] Conveyor belt started (relay ON)")

def stop_conveyor_belt():
    """
    Stop the conveyor belt (via gpiozero).
    """
    Relay.off()
    print("[Raspberry Pi] Conveyor belt stopped (relay OFF)")

def monitor_distance():
    """
    Measure distance using the ultrasonic sensor.
    Return True if an object is detected within the range (8cm - 9cm).
    """
    global detection_status
    try:
        while True:
            distance_cm = sensor.distance * 100  # Convert distance to cm
            if 8 <= distance_cm <= 9:
                print(f"[Sensor] Detected: {distance_cm:.1f} cm")
                time.sleep(1)  # Delay to simulate processing time
                stop_conveyor_belt()  # Stop the conveyor belt
                return True  # Detection successful
            else:
                time.sleep(0.5)  # Wait and retry detection
    except KeyboardInterrupt:
        print("Distance monitoring stopped.")
        return False

def capture_and_send_to_server(sock):
    """
    Capture an image and send it to the server (simulation input).
    Receive commands (P or D) from a socket.
    """
    while True:
        command = check_terminal_command(sock)  # Use socket to receive commands
        if command:
            if command.upper() == "P":
                print("[Simulation] Server response: Good")
                return "Good"
            elif command.upper() == "D":
                print("[Simulation] Server response: Bad")
                return "Bad"
            else:
                print("[Error] Invalid input. Enter P or D.")

def process_bad_item():
    """Logic for handling defective items."""
    print("[Simulation] Starting defective item processing")
    cmd_rotate_motor_28BYJ_48(90)
    start_conveyor_belt()
    time.sleep(4)
    stop_conveyor_belt()
    cmd_rotate_motor_28BYJ_48(-90)
    print("[Simulation] Defective item processing completed.")

def process_good_item():
    """Logic for handling good items."""
    print("[Simulation] Starting good item processing")
    start_conveyor_belt()
    time.sleep(5)
    stop_conveyor_belt()
    print("[Simulation] Good item processed")

def reset_all_motors():
    """
    Reset all motors (via gpiozero).
    """
    print("[Raspberry Pi] Resetting all motors...")
    Relay.off()
    CONST_PIN_MOT_DIR_8825.off()
    CONST_PIN_MOT_STEP_8825.off()  # Disable motor
    print("[Raspberry Pi] All motors reset completed.")

def cleanup_gpio():
    """
    Clean up gpiozero resources.
    """
    Relay.close()
    CONST_PIN_MOT_STEP_8825.close()
    CONST_PIN_MOT_DIR_8825.close()
    print("[Raspberry Pi] GPIO resources cleaned up.")

def main():
    sock = create_udp_socket() # Create the UDP socket 
    try: mode_setting(sock) # Start mode setting 
    except KeyboardInterrupt: 
        print("Program interrupted.") 
    finally: cleanup_gpio() # Cleanup GPIO resources


if __name__ == "__main__":
    main()
