"""
STM32 Simulator for Testing EV Protocol
Simulates the STM32 side so you can test the Raspberry Pi code
Run this in one terminal and your Pi code in another
"""

import serial
import time
import random
import threading
from typing import Dict, Any


class STM32Simulator:
    """Simulates STM32 responses for protocol testing"""
    
    START_CHAR = '<'
    END_CHAR = '>'
    SEPARATOR = ':'
    PARAM_SEP = ';'
    VALUE_SEP = '='
    
    def __init__(self, port: str, baudrate: int = 115200):
        """
        Initialize the simulator
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0')
            baudrate: Communication speed
        """
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        self.running = False
        
        # Simulated vehicle state
        self.state = {
            'speed': 0,
            'torque': 0,
            'rpm': 0,
            'temperature': 25.0,
            'current': 0.0,
            'voltage': 48.0,
            'battery_soc': 100.0,
            'max_current': 50.0,
            'regen_brake': 0,
            'faults': []
        }
        
        self.telemetry_interval = 1.0  # Send telemetry every N seconds
        self.last_telemetry_time = time.time()
        
        print("🔧 STM32 Simulator Started")
        print(f"📡 Port: {port} @ {baudrate} baud")
        print("=" * 50)
    
    def _parse_message(self, message: str) -> Dict[str, Any]:
        """Parse incoming message"""
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
            
            return {'type': msg_type, 'data': data}
        except Exception as e:
            print(f"❌ Parse error: {e}")
            return None
    
    def _build_message(self, msg_type: str, params: Dict[str, Any] = None) -> str:
        """Build outgoing message"""
        message = self.START_CHAR + msg_type
        
        if params:
            param_strs = [f"{k}{self.VALUE_SEP}{v}" for k, v in params.items()]
            message += self.SEPARATOR + self.PARAM_SEP.join(param_strs)
        
        message += self.END_CHAR
        return message
    
    def _send_message(self, msg_type: str, params: Dict[str, Any] = None):
        """Send message to Raspberry Pi"""
        message = self._build_message(msg_type, params)
        self.serial.write(message.encode('utf-8'))
        self.serial.flush()
        print(f"📤 Sent: {message}")
    
    def _send_ack(self, command: str):
        """Send ACK response"""
        self._send_message('ACK', {'ACK': command})
    
    def _send_nack(self, command: str, reason: str):
        """Send NACK response"""
        self._send_message('NACK', {'CMD': command, 'REASON': reason})
    
    def _send_telemetry(self):
        """Send telemetry data"""
        params = {
            'RPM': round(self.state['rpm'], 1),
            'TEMP': round(self.state['temperature'], 1),
            'CURRENT': round(self.state['current'], 1),
            'VOLTAGE': round(self.state['voltage'], 2),
            'SOC': round(self.state['battery_soc'], 1)
        }
        self._send_message('DATA', params)
    
    def _send_fault(self, fault_type: str):
        """Send fault message"""
        self._send_message('FAULT', {'FAULT': fault_type})
        self.state['faults'].append(fault_type)
    
    def _update_physics(self):
        """Simulate vehicle physics"""
        # Update RPM based on speed
        target_rpm = self.state['speed'] * 50  # 0-100% -> 0-5000 RPM
        self.state['rpm'] += (target_rpm - self.state['rpm']) * 0.1
        
        # Update current based on torque
        target_current = self.state['torque'] * 0.5  # 0-100% -> 0-50A
        self.state['current'] += (target_current - self.state['current']) * 0.1
        
        # Update temperature based on current
        heat_generation = self.state['current'] * 0.1
        cooling = (self.state['temperature'] - 25.0) * 0.05
        self.state['temperature'] += heat_generation - cooling
        
        # Add some noise
        self.state['temperature'] += random.uniform(-0.2, 0.2)
        self.state['rpm'] += random.uniform(-10, 10)
        self.state['current'] += random.uniform(-0.5, 0.5)
        
        # Discharge battery
        if self.state['current'] > 0:
            self.state['battery_soc'] -= self.state['current'] * 0.0001
            self.state['battery_soc'] = max(0, self.state['battery_soc'])
        
        # Voltage sag under load
        self.state['voltage'] = 48.0 - (self.state['current'] * 0.1)
        
        # Check for faults
        if self.state['temperature'] > 80 and 'OVERTEMP' not in self.state['faults']:
            self._send_fault('OVERTEMP')
        
        if self.state['current'] > self.state['max_current'] and 'OVERCURRENT' not in self.state['faults']:
            self._send_fault('OVERCURRENT')
        
        if self.state['battery_soc'] < 10 and 'LOW_BATTERY' not in self.state['faults']:
            self._send_fault('LOW_BATTERY')
    
    def _handle_command(self, msg: Dict[str, Any]):
        """Handle incoming command from Raspberry Pi"""
        if not msg:
            return
        
        msg_type = msg['type']
        data = msg['data']
        
        print(f"📥 Received: {msg_type} {data}")
        
        if msg_type == 'SET_SPEED':
            if 'SPEED' in data:
                self.state['speed'] = data['SPEED']
                self._send_ack('SET_SPEED')
                print(f"   ✓ Speed set to {self.state['speed']}%")
            else:
                self._send_nack('SET_SPEED', 'MISSING_PARAM')
        
        elif msg_type == 'SET_TORQUE':
            if 'TORQUE' in data:
                self.state['torque'] = data['TORQUE']
                self._send_ack('SET_TORQUE')
                print(f"   ✓ Torque set to {self.state['torque']}%")
            else:
                self._send_nack('SET_TORQUE', 'MISSING_PARAM')
        
        elif msg_type == 'SET_MAX_CURRENT':
            if 'CURRENT' in data:
                self.state['max_current'] = data['CURRENT']
                self._send_ack('SET_MAX_CURRENT')
                print(f"   ✓ Max current set to {self.state['max_current']}A")
            else:
                self._send_nack('SET_MAX_CURRENT', 'MISSING_PARAM')
        
        elif msg_type == 'SET_REGEN_BRAKE':
            if 'REGEN' in data:
                self.state['regen_brake'] = data['REGEN']
                self._send_ack('SET_REGEN_BRAKE')
                print(f"   ✓ Regen brake set to {self.state['regen_brake']}%")
            else:
                self._send_nack('SET_REGEN_BRAKE', 'MISSING_PARAM')
        
        elif msg_type == 'ESTOP':
            self.state['speed'] = 0
            self.state['torque'] = 0
            self.state['rpm'] = 0
            self.state['current'] = 0
            self._send_ack('ESTOP')
            print("   🛑 EMERGENCY STOP!")
        
        elif msg_type == 'RESET_FAULT':
            self.state['faults'].clear()
            self._send_ack('RESET_FAULT')
            print("   ✓ Faults cleared")
        
        elif msg_type == 'GET_TELEM':
            self._send_telemetry()
        
        elif msg_type == 'GET_TEMP':
            self._send_message('DATA', {'TEMP': round(self.state['temperature'], 1)})
        
        elif msg_type == 'GET_STATUS':
            self._send_message('DATA', {
                'SPEED': self.state['speed'],
                'TORQUE': self.state['torque'],
                'FAULTS': len(self.state['faults'])
            })
        
        elif msg_type == 'GET_FAULTS':
            fault_str = ','.join(self.state['faults']) if self.state['faults'] else 'NONE'
            self._send_message('DATA', {'FAULTS': fault_str})
        
        else:
            self._send_nack('UNKNOWN', 'INVALID_COMMAND')
            print(f"   ❌ Unknown command: {msg_type}")
    
    def _receive_loop(self):
        """Background thread to receive messages"""
        buffer = ""
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete messages
                    while self.START_CHAR in buffer and self.END_CHAR in buffer:
                        start = buffer.find(self.START_CHAR)
                        end = buffer.find(self.END_CHAR, start)
                        
                        if end > start:
                            message = buffer[start:end+1]
                            buffer = buffer[end+1:]
                            
                            parsed = self._parse_message(message)
                            self._handle_command(parsed)
                        else:
                            break
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"❌ Receive error: {e}")
                time.sleep(0.1)
    
    def _simulation_loop(self):
        """Background thread for physics simulation"""
        while self.running:
            self._update_physics()
            
            # Send periodic telemetry
            if time.time() - self.last_telemetry_time > self.telemetry_interval:
                self._send_telemetry()
                self.last_telemetry_time = time.time()
            
            time.sleep(0.1)
    
    def start(self):
        """Start the simulator"""
        self.running = True
        
        # Start receive thread
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()
        
        # Start simulation thread
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()
        
        print("✅ Simulator running. Press Ctrl+C to stop.")
        print("=" * 50)
    
    def stop(self):
        """Stop the simulator"""
        self.running = False
        self.serial.close()
        print("\n👋 Simulator stopped")
    
    def print_status(self):
        """Print current state"""
        print(f"\n📊 Current State:")
        print(f"   Speed: {self.state['speed']}%")
        print(f"   Torque: {self.state['torque']}%")
        print(f"   RPM: {self.state['rpm']:.1f}")
        print(f"   Temperature: {self.state['temperature']:.1f}°C")
        print(f"   Current: {self.state['current']:.1f}A")
        print(f"   Voltage: {self.state['voltage']:.2f}V")
        print(f"   Battery SOC: {self.state['battery_soc']:.1f}%")
        print(f"   Faults: {len(self.state['faults'])}")


if __name__ == "__main__":
    import sys
    
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    
    try:
        simulator = STM32Simulator(port)
        simulator.start()
        
        # Status update loop
        while True:
            time.sleep(5)
            simulator.print_status()
            
    except KeyboardInterrupt:
        print("\n⏹️  Stopping simulator...")
        simulator.stop()
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nUsage: python3 stm32_simulator.py [port]")
        print("Example: python3 stm32_simulator.py /dev/ttyUSB0")
