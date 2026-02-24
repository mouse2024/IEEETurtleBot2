import serial
import time

# --- Setup serial port ---
arduino_port = "/dev/ttyACM1"
baud_rate = 115200
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

time.sleep(2)  # wait for Arduino to reset

# --- Command functions ---
def move_motor(steps: int, direction: int):
    if direction not in (0, 1): # 0 = down, 1 = up
        raise ValueError("Direction must be 0 or +1.")
    if direction == 1:
        direction_byte = 0x01
        direction_str = "up"
    else:
        direction_byte = 0x00
        direction_str = "down"
    ser.write(bytes([0xFF, 0x01, steps, direction_byte]))
    ack = ser.read()
    if ack == b'\xAA':
        print(f"Motor moved {steps} steps {direction_str}")
    else:
        print("Error moving motor")

def turn_servos(direction: int):
    if direction not in (0,1):
        raise ValueError ("Direction must be 0 or +1");
    ser.write(bytes([0xFF, 0x03, direction, 0x00]))
    ack = ser.read()
    if ack == b'\xAA':
        print(f"Servos turned")
    else:
        print("Error turning servos")

def set_relay(on: bool):
    ser.write(bytes([0xFF, 0x02, 0x01 if on else 0x00, 0x00]))
    ack = ser.read()
    if ack == b'\xAA':
        print(f"Relay {'ON' if on else 'OFF'}")
    else:
        print("Error setting relay")

# --- Example Usage ---
move_motor(250, 1)  # move 1000 steps up
time.sleep(2)
move_motor(250, 0)  # move 20 steps down
#set_relay(True)     # turn relay ON
time.sleep(2)
#set_relay(False)    # turn relay OFF
turn_servos(0)
turn_servos(1)
