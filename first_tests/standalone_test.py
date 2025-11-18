"""
Standalone EV Protocol Test Script
Contains all code in one file - no imports needed!
"""

import serial
import time
import sys
from enum import Enum
from typing import Dict, Optional, Any
import threading
import queue


class MessageType(Enum):
    """Message types for communication protocol"""
    SET_SPEED = "SET_SPEED"
    SET_TORQUE = "SET_TORQUE"
    SET_MAX_CURRENT = "SET_MAX_CURRENT"
    SET_REGEN_BRAKE = "SET_REGEN_BRAKE"
    EMERGENCY_STOP = "ESTOP"
    RESET_FAULT = "RESET_FAULT"
    GET_TELEMETRY = "GET_TELEM"
    GET_TEMP = "GET_TEMP"
    GET_STATUS = "GET_STATUS"
    GET_FAULTS = "GET_FAULTS"
    DATA = "DATA"
    ACK = "ACK"
    NACK = "NACK"
    FAULT = "FAULT"


class EVProtocol:
    """Message protocol handler for EV control system"""
    
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
                print(f"Receive error: {e}")
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
            
            return {
                'type': msg_type,
                'data': data,
                'raw': message
            }
        except Exception as e:
            print(f"Parse error: {e} - Message: {message}")
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
            print(f"Send error: {e}")
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


class EVController:
    """High-level controller using the protocol"""
    
    def __init__(self, port: str):
        self.protocol = EVProtocol(port)
        self.telemetry = {}
        
        self.protocol.register_callback('DATA', self._handle_telemetry)
        self.protocol.register_callback('FAULT', self._handle_fault)
        
        self.protocol.start()
    
    def _handle_telemetry(self, msg):
        self.telemetry.update(msg['data'])
    
    def _handle_fault(self, msg):
        print(f"⚠️  FAULT: {msg['data']}")
    
    def set_speed(self, speed: int) -> bool:
        success = self.protocol.send_message(MessageType.SET_SPEED, {'SPEED': speed})
        if success:
            return self.protocol.wait_for_ack('SET_SPEED')
        return False
    
    def set_torque(self, torque: int) -> bool:
        success = self.protocol.send_message(MessageType.SET_TORQUE, {'TORQUE': torque})
        if success:
            return self.protocol.wait_for_ack('SET_TORQUE')
        return False
    
    def emergency_stop(self) -> bool:
        return self.protocol.send_message(MessageType.EMERGENCY_STOP)
    
    def request_telemetry(self):
        self.protocol.send_message(MessageType.GET_TELEMETRY)
    
    def get_telemetry(self) -> Dict[str, Any]:
        return self.telemetry.copy()
    
    def close(self):
        self.protocol.stop()


# ============================================================================
# TEST SCRIPT
# ============================================================================

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    
    print("=" * 60)
    print("🚗 EV Protocol Test Script (Standalone Version)")
    print("=" * 60)
    print(f"Port: {port}")
    print("\nMake sure the STM32 simulator is running in another terminal!")
    print("Run: python3 stm32_simulator.py [port]")
    print("=" * 60)
    
    input("\nPress Enter to start testing...")
    
    try:
        print("\n1️⃣  Initializing controller...")
        controller = EVController(port)
        time.sleep(1)
        print("   ✅ Controller initialized")
        
        print("\n2️⃣  Testing SET_SPEED command...")
        if controller.set_speed(30):
            print("   ✅ Speed set to 30% - ACK received")
        else:
            print("   ❌ Failed to set speed - no ACK")
        time.sleep(1)
        
        print("\n3️⃣  Testing SET_TORQUE command...")
        if controller.set_torque(50):
            print("   ✅ Torque set to 50% - ACK received")
        else:
            print("   ❌ Failed to set torque - no ACK")
        time.sleep(1)
        
        print("\n4️⃣  Testing GET_TELEMETRY command...")
        controller.request_telemetry()
        time.sleep(0.5)
        telemetry = controller.get_telemetry()
        if telemetry:
            print("   ✅ Telemetry received:")
            for key, value in telemetry.items():
                print(f"      {key}: {value}")
        else:
            print("   ⚠️  No telemetry data yet")
        
        print("\n5️⃣  Testing speed ramp (0% -> 70%)...")
        for speed in range(0, 71, 10):
            controller.set_speed(speed)
            time.sleep(0.3)
            print(f"   Speed: {speed}%")
        print("   ✅ Speed ramp complete")
        
        print("\n6️⃣  Monitoring telemetry for 10 seconds...")
        print("   (Watch RPM, temperature, and current increase)")
        for i in range(10):
            controller.request_telemetry()
            time.sleep(1)
            telemetry = controller.get_telemetry()
            if telemetry:
                rpm = telemetry.get('RPM', 0)
                temp = telemetry.get('TEMP', 0)
                current = telemetry.get('CURRENT', 0)
                print(f"   [{i+1}/10] RPM: {rpm:.1f} | Temp: {temp:.1f}°C | Current: {current:.1f}A")
        
        print("\n7️⃣  Testing EMERGENCY STOP...")
        if controller.emergency_stop():
            print("   ✅ Emergency stop sent")
        else:
            print("   ❌ Emergency stop failed")
        time.sleep(1)
        
        print("\n8️⃣  Verifying stop (checking telemetry)...")
        controller.request_telemetry()
        time.sleep(0.5)
        telemetry = controller.get_telemetry()
        if telemetry:
            rpm = telemetry.get('RPM', 0)
            if rpm < 10:
                print(f"   ✅ Vehicle stopped (RPM: {rpm:.1f})")
            else:
                print(f"   ⚠️  Vehicle still moving (RPM: {rpm:.1f})")
        
        print("\n9️⃣  Stress test - Rapid commands (50 messages)...")
        start_time = time.time()
        success_count = 0
        for i in range(50):
            speed = (i * 2) % 100
            if controller.protocol.send_message(MessageType.SET_SPEED, {'SPEED': speed}):
                success_count += 1
            time.sleep(0.05)
        elapsed = time.time() - start_time
        print(f"   ✅ Sent {success_count}/50 messages in {elapsed:.2f}s")
        print(f"   Rate: {success_count/elapsed:.1f} msg/sec")
        
        print("\n" + "=" * 60)
        print("✅ ALL TESTS COMPLETE!")
        print("=" * 60)
        print("\n📊 Final Telemetry:")
        controller.request_telemetry()
        time.sleep(0.5)
        telemetry = controller.get_telemetry()
        if telemetry:
            for key, value in sorted(telemetry.items()):
                print(f"   {key}: {value}")
        
        print("\n💡 Tips:")
        print("   - Watch the simulator terminal for sent/received messages")
        print("   - Telemetry updates every 1 second in simulator")
        print("   - Try triggering faults (high speed/torque)")
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔌 Closing connection...")
        controller.close()
        print("👋 Goodbye!")


if __name__ == "__main__":
    main()
