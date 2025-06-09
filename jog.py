import argparse
import sys
import termios
import tty
import time
import os
import glob

# Import the official Feetech servo SDK
try:
    from scservo_sdk import *  # Import all from SDK
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.packet_handler import PacketHandler
except ImportError:
    print("ERROR: feetech-servo-sdk not installed. Run: pip install feetech-servo-sdk")
    sys.exit(1)

SERVO_ID = 6
BAUDRATE = 1_000_000

HELP_MSG = """
Controls:
  ← / a : move -10 ticks
  → / d : move +10 ticks
  ↓ / s : move -100 ticks
  ↑ / w : move +100 ticks
  q      : quit and print min/max reached
  h/?    : show this help
"""

ARROW_MAP = {
    'D': -10,  # left
    'C': +10,  # right
    'B': -100, # down
    'A': +100, # up
}

# Feetech servo protocol constants
SCS_GOAL_POSITION_L = 42
SCS_PRESENT_POSITION_L = 56
SCS_TORQUE_ENABLE = 40
PROTOCOL_END = 0  # ST3215/STS series uses 0 (STS/SMS=0, SCS=1)


def find_available_serial_ports():
    """Find available serial ports for ST3215 communication"""
    patterns = [
        '/dev/ttyUSB*',        # Linux USB-serial
        '/dev/ttyACM*',        # Linux ACM devices  
        '/dev/tty.usbserial*', # macOS FTDI
        '/dev/tty.usbmodem*',  # macOS CDC/ACM
        'COM*'                 # Windows
    ]
    
    devices = []
    for pattern in patterns:
        devices.extend(glob.glob(pattern))
    
    return sorted(devices)


class KeyReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self):
        """Read a single key press"""
        ch = sys.stdin.read(1)
        
        # Handle escape sequences (arrow keys)
        if ch == '\x1b':
            ch = sys.stdin.read(1)
            if ch == '[':
                ch = sys.stdin.read(1)
                return ARROW_MAP.get(ch, ch)
        
        return ch


def main():
    parser = argparse.ArgumentParser(description='ST3215 Servo Jogging Utility')
    parser.add_argument('device', nargs='?', help='Serial device path')
    parser.add_argument('--id', type=int, default=SERVO_ID, help='Servo ID (default: 1)')
    args = parser.parse_args()

    device = args.device
    if not device:
        # Auto-detect serial port
        ports = find_available_serial_ports()
        if not ports:
            print("No serial ports found. Please specify a device.")
            return
        elif len(ports) == 1:
            device = ports[0]
            print(f"Auto-selected port: {device}")
        else:
            print("Multiple serial ports found:")
            for i, port in enumerate(ports):
                print(f"  {i+1}: {port}")
            print("Please specify one as an argument")
            return

    # Connect to servo using official SDK
    try:
        port_handler = PortHandler(device)
        packet_handler = PacketHandler(PROTOCOL_END)  # ST3215 uses protocol_end = 0

        print(f"Attempting to open port {device}...")
        if not port_handler.openPort():
            print(f"ERROR: Failed to open port {device}. Check connection and permissions.")
            return
        print(f"SUCCESS: Port {device} opened.")

        print(f"Attempting to set baud rate to {BAUDRATE} on {device}...")
        if not port_handler.setBaudRate(BAUDRATE):
            print(f"ERROR: Failed to set baud rate to {BAUDRATE} on {device}.")
            port_handler.closePort()
            return
        print(f"SUCCESS: Baud rate set to {BAUDRATE}.")

        time.sleep(0.1)  # Short delay after port setup

        # Test connection with ping
        print(f"Pinging servo ID {args.id} on {device}...")
        model_number, ping_result, ping_error = packet_handler.ping(port_handler, args.id)

        if ping_result != COMM_SUCCESS:
            error_message = packet_handler.getTxRxResult(ping_result)
            rx_error_message = packet_handler.getRxPacketError(ping_error) if ping_error else "N/A"
            print(f"ERROR: Servo ID {args.id} not responding on {device}.")
            print(f"       TX/RX Result: {error_message} (Code: {ping_result})")
            print(f"       RX Packet Error: {rx_error_message} (Code: {ping_error})")
            print(f"       Please check servo ID, wiring, power, and baud rate (currently {BAUDRATE}).")
            port_handler.closePort()
            return
        
        print(f"SUCCESS: Servo ID {args.id} responded. Model number: {model_number}")

        # Enable torque
        print(f"Enabling torque for servo ID {args.id}...")
        torque_result, torque_error = packet_handler.write1ByteTxRx(port_handler, args.id, SCS_TORQUE_ENABLE, 1)
        if torque_result != COMM_SUCCESS:
            error_message = packet_handler.getTxRxResult(torque_result)
            rx_error_message = packet_handler.getRxPacketError(torque_error) if torque_error else "N/A"
            print(f"ERROR: Failed to enable torque for servo ID {args.id}.")
            print(f"       TX/RX Result: {error_message} (Code: {torque_result})")
            print(f"       RX Packet Error: {rx_error_message} (Code: {torque_error})")
            port_handler.closePort()
            return
        
        print(f"SUCCESS: Torque enabled for servo ID {args.id}.")

    except Exception as e:
        print(f"ERROR: An unexpected error occurred during servo connection: {e}")
        if 'port_handler' in locals() and port_handler.is_open:
            port_handler.closePort()
        return

    # Get starting position
    print(f"Reading starting position for servo ID {args.id}...")
    current_pos, read_result, read_error = packet_handler.read2ByteTxRx(port_handler, args.id, SCS_PRESENT_POSITION_L)
    if read_result != COMM_SUCCESS:
        error_message = packet_handler.getTxRxResult(read_result)
        rx_error_message = packet_handler.getRxPacketError(read_error) if read_error else "N/A"
        print(f"ERROR: Failed to read starting position for servo ID {args.id}.")
        print(f"       TX/RX Result: {error_message} (Code: {read_result})")
        print(f"       RX Packet Error: {rx_error_message} (Code: {read_error})")
        # Try to disable torque before exiting
        packet_handler.write1ByteTxRx(port_handler, args.id, SCS_TORQUE_ENABLE, 0)
        port_handler.closePort()
        return
    
    print(f"SUCCESS: Starting position: {current_pos}")

    min_reached = current_pos
    max_reached = current_pos

    print(HELP_MSG)

    with KeyReader() as reader:
        while True:
            try:
                key = reader.read_key()
                
                if key in ('q', 'Q'):
                    break
                elif key in ('h', '?'):
                    print(HELP_MSG)
                    continue
                
                # Calculate movement
                delta = 0
                if key in ('a', 'A'):
                    delta = -10
                elif key in ('d', 'D'):
                    delta = 10
                elif key in ('s', 'S'):
                    delta = -100
                elif key in ('w', 'W'):
                    delta = 100
                elif isinstance(key, int):  # Arrow keys return deltas directly
                    delta = key
                else:
                    continue  # Unknown key
                
                # Move servo
                new_pos = max(0, min(4095, current_pos + delta))
                if new_pos == current_pos:
                    print(f"At limit (position {current_pos})")
                    continue
                
                result, error = packet_handler.write2ByteTxRx(port_handler, args.id, SCS_GOAL_POSITION_L, new_pos)
                if result == COMM_SUCCESS:
                    current_pos = new_pos
                    min_reached = min(min_reached, current_pos)
                    max_reached = max(max_reached, current_pos)
                    print(f"Position: {current_pos} (range: {min_reached}-{max_reached})")
                else:
                    print("Failed to move servo")
                    
                time.sleep(0.05)  # Small delay for smooth operation
                
            except KeyboardInterrupt:
                break

    print(f"\nFinal results:")
    print(f"Minimum position reached: {min_reached}")
    print(f"Maximum position reached: {max_reached}")
    print(f"Range: {max_reached - min_reached} ticks")
    print(f"\nAdd these constants to your server script:")
    print(f"SERVO_MIN_POS = {min_reached}")
    print(f"SERVO_MAX_POS = {max_reached}")
    
    # Clean up
    port_handler.closePort()


if __name__ == "__main__":
    main() 
