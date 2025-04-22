import can
import struct

can_interface = "can0"
bus = can.interface.Bus(channel=can_interface, bustype="socketcan")

def receive_can_messages():
    while True:
        message = bus.recv()
        if message is not None:
            process_can_message(message)

def process_can_message(message):
    """Decodes and prints received CAN messages."""
    data = message.data

    if message.arbitration_id == 0x036:
        ax, ay = struct.unpack('ff', data)
        print(f"Received 0x036 -> ax: {ax:.2f}, ay: {ay:.2f}")

    elif message.arbitration_id == 0x037:
        az, vx = struct.unpack('ff', data)
        print(f"Received 0x037 -> az: {az:.2f}, vx: {vx:.2f}")

    elif message.arbitration_id == 0x038:
        vy, vz = struct.unpack('ff', data)
        print(f"Received 0x038 -> vy: {vy:.2f}, vz: {vz:.2f}")

    elif message.arbitration_id == 0x039:
        depth = struct.unpack('f', data[:4])[0]
        button_state = bool(data[4])
        print(f"Received 0x039 -> Depth: {depth:.2f}m, Button: {'PRESSED' if button_state else 'RELEASED'}")

print("Listening for CAN messages...")
receive_can_messages()
