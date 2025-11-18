"""
EV Controller - Main Application (Terminal-Based)
Full production software for Raspberry Pi side
SVGS EV Team - Porsche Project

Usage:
    python3 ev_main.py [port] [baudrate]
    
Example:
    python3 ev_main.py /dev/ttyUSB0 115200
"""

import serial
import time
import sys
import os
import threading
import json
from datetime import datetime
from enum import Enum
from typing import Dict, Optional, Any
import queue


# ============================================================================
# PROTOCOL LAYER
# ============================================================================

class MessageType(Enum):
    """Message types for communication protocol"""
    # Commands (Pi -> STM32)
    SET_MAX_CURRENT = "SET_MAX_CURRENT"
    SET_CURRENT_LIMIT = "SET_CURRENT_LIMIT"
    EMERGENCY_STOP = "ESTOP"
    RESET_FAULT = "RESET_FAULT"
    
    # Queries (Pi -> STM32)
    GET_TELEMETRY = "GET_TELEM"
    GET_TEMP = "GET_TEMP"
    GET_STATUS = "GET_STATUS"
    GET_FAULTS = "GET_FAULTS"
    
    # Responses (STM32 -> Pi)
    DATA = "DATA"
    ACK = "ACK"
    NACK = "NACK"
    FAULT = "FAULT"


class EVProtocol:
    """Message protocol handler"""
    
    START_CHAR = '<'
    END_CHAR = '>'
    SEPARATOR = ':'
    PARAM_SEP = ';'
    VALUE_SEP = '='
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.rx_queue = queue.Queue()
        self.running = False
        self.rx_thread = None
        self.callbacks = {}
        
    def start(self):
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()
        
    def stop(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        self.serial.close()
        
    def _receive_loop(self):
        buffer = ""
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while self.START_CHAR in buffer and self.END_CHAR in buffer:
                        start = buffer.find(self.START_CHAR)
                        end = buffer.find(self.END_CHAR, start)
                        
                        if end > start:
                            message = buffer[start:end+1]
                            buffer = buffer[end+1:]
                            
                            parsed = self._parse_message(message)
                            if parsed:
                                self.rx_queue.put(parsed)
                                self._trigger_callback(parsed)
                        else:
                            break
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Protocol RX Error: {e}")
                time.sleep(0.1)
    
    def _parse_message(self, message: str) -> Optional[Dict[str, Any]]:
        try:
            message = message.strip().lstrip(self.START_CHAR).rstrip(self.END_CHAR)
            
            if self.SEPARATOR in message:
                msg_type, data_str = message.split(self.SEPARATOR, 1)
            else:
                msg_type = message
                data_str = ""
            
            data = {}
            if data_str:
                params = data_str.split(self.PARAM_SEP)
                for param in params:
                    if self.VALUE_SEP in param:
                        key, value = param.split(self.VALUE_SEP, 1)
                        try:
                            if '.' in value:
                                data[key] = float(value)
                            else:
                                data[key] = int(value)
                        except ValueError:
                            data[key] = value
                    else:
                        data[param] = True
            
            return {'type': msg_type, 'data': data, 'timestamp': time.time()}
        except Exception as e:
            return None
    
    def _build_message(self, msg_type: str, params: Optional[Dict[str, Any]] = None) -> str:
        message = self.START_CHAR + msg_type
        
        if params:
            param_strs = [f"{k}{self.VALUE_SEP}{v}" for k, v in params.items()]
            message += self.SEPARATOR + self.PARAM_SEP.join(param_strs)
        
        message += self.END_CHAR
        return message
    
    def send_message(self, msg_type: MessageType, params: Optional[Dict[str, Any]] = None) -> bool:
        try:
            message = self._build_message(msg_type.value, params)
            self.serial.write(message.encode('utf-8'))
            self.serial.flush()
            return True
        except Exception as e:
            print(f"Protocol TX Error: {e}")
            return False
    
    def register_callback(self, msg_type: str, callback):
        self.callbacks[msg_type] = callback
    
    def _trigger_callback(self, parsed_msg: Dict[str, Any]):
        msg_type = parsed_msg['type']
        if msg_type in self.callbacks:
            try:
                self.callbacks[msg_type](parsed_msg)
            except Exception as e:
                print(f"Callback error for {msg_type}: {e}")
    
    def get_message(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def wait_for_ack(self, command: str, timeout: float = 1.0) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.get_message(timeout=0.1)
            if msg and msg['type'] == 'ACK':
                if command in msg['data'] or msg['data'].get('ACK') == command:
                    return True
        return False


# ============================================================================
# DATA LOGGING
# ============================================================================

class DataLogger:
    """Handles logging of telemetry data to CSV files"""
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        self.log_file = None
        self.logging_enabled = False
        
        # Create logs directory
        os.makedirs(log_dir, exist_ok=True)
    
    def start_logging(self):
        """Start a new log file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"ev_log_{timestamp}.csv"
        filepath = os.path.join(self.log_dir, filename)
        
        self.log_file = open(filepath, 'w')
        # Write CSV header
        self.log_file.write("timestamp,rpm,temperature,current,voltage,battery_soc,throttle\n")
        self.logging_enabled = True
        print(f"📝 Logging started: {filepath}")
    
    def log_data(self, telemetry: Dict[str, Any], throttle: int):
        """Log a data point"""
        if self.logging_enabled and self.log_file:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            rpm = telemetry.get('RPM', 0)
            temp = telemetry.get('TEMP', 0)
            current = telemetry.get('CURRENT', 0)
            voltage = telemetry.get('VOLTAGE', 0)
            soc = telemetry.get('SOC', 0)
            
            line = f"{timestamp},{rpm},{temp},{current},{voltage},{soc},{throttle}\n"
            self.log_file.write(line)
            self.log_file.flush()
    
    def stop_logging(self):
        """Stop logging and close file"""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.logging_enabled = False
            print("📝 Logging stopped")


# ============================================================================
# CONFIGURATION MANAGER
# ============================================================================

class ConfigManager:
    """Manages system configuration"""
    
    def __init__(self, config_file: str = "ev_config.json"):
        self.config_file = config_file
        self.config = self.load_config()
    
    def load_config(self) -> Dict[str, Any]:
        """Load configuration from file"""
        default_config = {
            "max_current": 50.0,
            "current_limit": 50.0,
            "max_throttle": 100,
            "telemetry_interval": 0.5,
            "overheat_threshold": 80.0,
            "low_battery_threshold": 15.0,
            "emergency_stop_on_fault": True
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    loaded = json.load(f)
                    default_config.update(loaded)
                    print(f"⚙️  Configuration loaded from {self.config_file}")
            except Exception as e:
                print(f"⚠️  Error loading config: {e}, using defaults")
        else:
            self.save_config(default_config)
        
        return default_config
    
    def save_config(self, config: Dict[str, Any] = None):
        """Save configuration to file"""
        if config:
            self.config = config
        
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.config, f, indent=4)
            print(f"⚙️  Configuration saved to {self.config_file}")
        except Exception as e:
            print(f"⚠️  Error saving config: {e}")
    
    def get(self, key: str, default=None):
        return self.config.get(key, default)
    
    def set(self, key: str, value: Any):
        self.config[key] = value


# ============================================================================
# MAIN CONTROLLER
# ============================================================================

class EVController:
    """Main EV controller with all functionality"""
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.protocol = EVProtocol(port, baudrate)
        self.config = ConfigManager()
        self.logger = DataLogger()
        
        # State
        self.telemetry = {}
        self.faults = []
        self.connected = False
        self.last_telemetry_request = 0
        
        # Register callbacks
        self.protocol.register_callback('DATA', self._handle_telemetry)
        self.protocol.register_callback('FAULT', self._handle_fault)
        self.protocol.register_callback('ACK', self._handle_ack)
        self.protocol.register_callback('NACK', self._handle_nack)
        
        # Start protocol
        self.protocol.start()
        
        # Start telemetry request loop
        self.running = True
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        
        print("✅ EV Controller initialized")
    
    def _handle_telemetry(self, msg):
        """Handle incoming telemetry data"""
        self.telemetry.update(msg['data'])
        self.connected = True
        
        # Log data if enabled
        if self.logger.logging_enabled:
            throttle = self.telemetry.get('THROTTLE', 0)
            self.logger.log_data(self.telemetry, throttle)
        
        # Check for critical conditions
        self._check_safety_conditions()
    
    def _handle_fault(self, msg):
        """Handle fault messages"""
        fault = msg['data'].get('FAULT', 'UNKNOWN')
        if fault not in self.faults:
            self.faults.append(fault)
            print(f"\n⚠️  FAULT DETECTED: {fault}")
            
            if self.config.get('emergency_stop_on_fault'):
                print("🛑 Auto emergency stop triggered!")
                self.emergency_stop()
    
    def _handle_ack(self, msg):
        """Handle ACK messages"""
        pass  # Already handled by wait_for_ack
    
    def _handle_nack(self, msg):
        """Handle NACK messages"""
        cmd = msg['data'].get('CMD', 'UNKNOWN')
        reason = msg['data'].get('REASON', 'UNKNOWN')
        print(f"❌ NACK received: {cmd} - {reason}")
    
    def _check_safety_conditions(self):
        """Check for dangerous conditions"""
        temp = self.telemetry.get('TEMP', 0)
        soc = self.telemetry.get('SOC', 100)
        
        if temp > self.config.get('overheat_threshold', 80):
            if 'OVERHEAT' not in self.faults:
                self.faults.append('OVERHEAT')
                print(f"\n⚠️  WARNING: Temperature critical ({temp}°C)")
        
        if soc < self.config.get('low_battery_threshold', 15):
            if 'LOW_BATTERY' not in self.faults:
                self.faults.append('LOW_BATTERY')
                print(f"\n⚠️  WARNING: Battery low ({soc}%)")
    
    def _telemetry_loop(self):
        """Background thread to request telemetry periodically"""
        interval = self.config.get('telemetry_interval', 0.5)
        while self.running:
            self.protocol.send_message(MessageType.GET_TELEMETRY)
            time.sleep(interval)
    
    def set_max_throttle(self, max_throttle: int) -> bool:
        """Set maximum throttle limit (0-100%) - safety override"""
        max_throttle = max(0, min(100, max_throttle))
        success = self.protocol.send_message(MessageType.SET_MAX_CURRENT, {'MAX_THROTTLE': max_throttle})
        if success and self.protocol.wait_for_ack('SET_MAX_CURRENT', timeout=0.5):
            self.config.set('max_throttle', max_throttle)
            return True
        return False
    
    def set_current_limit(self, current: float) -> bool:
        """Set current limit in Amps"""
        success = self.protocol.send_message(MessageType.SET_CURRENT_LIMIT, {'LIMIT': current})
        if success and self.protocol.wait_for_ack('SET_CURRENT_LIMIT', timeout=0.5):
            self.config.set('current_limit', current)
            return True
        return False
    
    def emergency_stop(self) -> bool:
        """Trigger emergency stop"""
        return self.protocol.send_message(MessageType.EMERGENCY_STOP)
    
    def reset_faults(self) -> bool:
        """Reset all faults"""
        success = self.protocol.send_message(MessageType.RESET_FAULT)
        if success and self.protocol.wait_for_ack('RESET_FAULT', timeout=0.5):
            self.faults.clear()
            return True
        return False
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get latest telemetry data"""
        return self.telemetry.copy()
    
    def get_status(self) -> Dict[str, Any]:
        """Get complete system status"""
        return {
            'connected': self.connected,
            'faults': self.faults.copy(),
            'telemetry': self.telemetry.copy(),
            'config': self.config.config.copy()
        }
    
    def shutdown(self):
        """Clean shutdown"""
        print("\n🔌 Shutting down controller...")
        self.running = False
        self.emergency_stop()
        time.sleep(0.5)
        self.logger.stop_logging()
        self.config.save_config()
        self.protocol.stop()
        print("👋 Shutdown complete")


# ============================================================================
# TERMINAL INTERFACE
# ============================================================================

class TerminalInterface:
    """Terminal-based user interface"""
    
    def __init__(self, controller: EVController):
        self.controller = controller
        self.running = True
    
    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear' if os.name != 'nt' else 'cls')
    
    def print_header(self):
        """Print application header"""
        print("=" * 70)
        print("  🚗 PORSCHE EV CONTROLLER - SVGS EV Team")
        print("=" * 70)
    
    def print_status(self):
        """Print current status dashboard"""
        status = self.controller.get_status()
        telemetry = status['telemetry']
        
        # Connection status
        conn_status = "🟢 CONNECTED" if status['connected'] else "🔴 DISCONNECTED"
        print(f"\nStatus: {conn_status}")
        
        # Telemetry
        print(f"\n📊 TELEMETRY:")
        print(f"  Throttle:    {telemetry.get('THROTTLE', 0):3d}% (from pedal)")
        print(f"  RPM:         {telemetry.get('RPM', 0):7.1f}")
        print(f"  Current:     {telemetry.get('CURRENT', 0):6.1f} A")
        print(f"  Voltage:     {telemetry.get('VOLTAGE', 0):6.2f} V")
        print(f"  Temperature: {telemetry.get('TEMP', 0):6.1f} °C")
        print(f"  Battery SOC: {telemetry.get('SOC', 0):6.1f} %")
        
        # Power calculation
        power = telemetry.get('CURRENT', 0) * telemetry.get('VOLTAGE', 0)
        print(f"  Power:       {power:6.1f} W ({power/1000:.2f} kW)")
        
        # Faults
        if status['faults']:
            print(f"\n⚠️  FAULTS: {', '.join(status['faults'])}")
        else:
            print(f"\n✅ NO FAULTS")
        
        # Configuration
        config = status['config']
        print(f"\n⚙️  CONFIGURATION:")
        print(f"  Current Limit:   {config.get('current_limit', 0):.1f} A")
        print(f"  Max Throttle:    {config.get('max_throttle', 100)} %")
        print(f"  Logging:         {'ON' if self.controller.logger.logging_enabled else 'OFF'}")
    
    def print_menu(self):
        """Print command menu"""
        print("\n" + "-" * 70)
        print("COMMANDS:")
        print("  c [amps]   - Set current limit (e.g., 'c 40' for 40A)")
        print("  m [0-100]  - Set max throttle limit (safety override)")
        print("  e          - Emergency stop")
        print("  f          - Reset faults")
        print("  l          - Toggle data logging")
        print("  s          - Save configuration")
        print("  h          - Show this help")
        print("  q          - Quit")
        print("\nNOTE: Throttle is controlled by gas pedal (displayed above)")
        print("-" * 70)
    
    def run(self):
        """Main interface loop"""
        self.clear_screen()
        self.print_header()
        self.print_menu()
        
        print("\n💡 Waiting for STM32 connection...")
        
        # Wait for initial connection
        timeout = 10
        start = time.time()
        while not self.controller.connected and time.time() - start < timeout:
            time.sleep(0.1)
        
        if not self.controller.connected:
            print("⚠️  Warning: STM32 not responding. Check connection.")
            print("   Continuing anyway - commands will be sent but may not be confirmed.")
        else:
            print("✅ Connected to STM32!")
        
        time.sleep(1)
        
        # Display initial status
        self.clear_screen()
        self.print_header()
        self.print_status()
        self.print_menu()
        
        # Main loop - simple input mode (no auto-refresh while typing)
        try:
            while self.running:
                # Get user input (blocking)
                try:
                    command = input("\n> ").strip().lower()
                    
                    # Handle command
                    self.handle_command(command)
                    
                    # Refresh display after command
                    if self.running:  # Don't refresh if quitting
                        time.sleep(0.5)  # Brief pause to see result
                        self.clear_screen()
                        self.print_header()
                        self.print_status()
                        self.print_menu()
                    
                except EOFError:
                    break
                    
        except KeyboardInterrupt:
            print("\n\n⏹️  Interrupted by user")
        finally:
            self.controller.shutdown()
    
    def handle_command(self, command: str):
        """Handle user command"""
        parts = command.split()
        if not parts:
            return
        
        cmd = parts[0]
        
        try:
            if cmd == 'q':
                self.running = False
                return  # Don't show message
            
            elif cmd == 'm':
                if len(parts) < 2:
                    print("❌ Usage: m [0-100]")
                else:
                    max_throttle = int(parts[1])
                    if self.controller.set_max_throttle(max_throttle):
                        print(f"✅ Max throttle limit set to {max_throttle}%")
                    else:
                        print("❌ Failed to set max throttle")
            
            elif cmd == 'c':
                if len(parts) < 2:
                    print("❌ Usage: c [amps]")
                else:
                    current = float(parts[1])
                    if self.controller.set_current_limit(current):
                        print(f"✅ Current limit set to {current}A")
                    else:
                        print("❌ Failed to set current limit")
            
            elif cmd == 'e':
                if self.controller.emergency_stop():
                    print("🛑 EMERGENCY STOP ACTIVATED")
                else:
                    print("❌ Failed to send emergency stop")
            
            elif cmd == 'f':
                if self.controller.reset_faults():
                    print("✅ Faults reset")
                else:
                    print("❌ Failed to reset faults")
            
            elif cmd == 'l':
                if self.controller.logger.logging_enabled:
                    self.controller.logger.stop_logging()
                else:
                    self.controller.logger.start_logging()
            
            elif cmd == 's':
                self.controller.config.save_config()
            
            elif cmd == 'h':
                # Don't clear on help, just print
                pass
            
            elif cmd == '':
                # Empty command, just refresh
                pass
            
            else:
                print(f"❌ Unknown command: {cmd}. Type 'h' for help")
        
        except ValueError:
            print(f"❌ Invalid value. Check your input.")
        except Exception as e:
            print(f"❌ Error: {e}")


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point"""
    print("=" * 70)
    print("  🚗 PORSCHE EV CONTROLLER - Starting...")
    print("=" * 70)
    
    # Parse arguments
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print(f"\nPort: {port}")
    print(f"Baudrate: {baudrate}")
    print("\nPress Ctrl+C to exit at any time")
    print("=" * 70)
    
    time.sleep(2)
    
    try:
        # Initialize controller
        controller = EVController(port, baudrate)
        
        # Run interface
        interface = TerminalInterface(controller)
        interface.run()
        
    except serial.SerialException as e:
        print(f"\n❌ Serial Error: {e}")
        print(f"\nTroubleshooting:")
        print(f"  - Check that {port} exists: ls -l /dev/tty*")
        print(f"  - Check permissions: sudo usermod -a -G dialout $USER")
        print(f"  - Check if device is connected: dmesg | grep tty")
        sys.exit(1)
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()