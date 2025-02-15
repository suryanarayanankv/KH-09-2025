import socket
import json
import numpy as np
from collections import deque
from datetime import datetime

class SignLanguageRecognizer:
    def __init__(self):
        # UDP Configuration
        self.PYTHON_IP = "0.0.0.0"
        self.PYTHON_PORT = 12345
        
        # Initialize UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.PYTHON_IP, self.PYTHON_PORT))
        
        # Buffer for gesture detection
        self.sensor_buffer = deque(maxlen=10)
        
        # Define ASL letter patterns (A-K)
        self.letter_patterns = {
            'A': {'primary_sensor': 0, 'acc_z': (0.8, 1.0), 'orientation': 'Up'},
            'B': {'primary_sensor': 0, 'acc_z': (0.8, 1.0), 'orientation': 'Away from you'},
            'C': {'primary_sensor': 0, 'acc_z': (0.6, 0.9), 'orientation': 'Away from you'},
            'D': {'primary_sensor': 2, 'acc_z': (0.9, 1.0), 'orientation': 'Up'},
            'E': {'primary_sensor': 0, 'acc_z': (0.7, 1.0), 'orientation': 'Away from you'},
            'F': {'primary_sensor': 1, 'acc_z': (0.6, 0.9), 'orientation': 'Away from you'},
            'G': {'primary_sensor': 2, 'acc_z': (0.9, 1.0), 'orientation': 'Away from you'},
            'H': {'primary_sensor': 2, 'acc_z': (0.9, 1.0), 'orientation': 'Away from you'},
            'I': {'primary_sensor': 5, 'acc_z': (0.9, 1.0), 'orientation': 'Up'},
            'J': {'primary_sensor': 5, 'acc_z': (0.9, 1.0), 'orientation': 'Moving'},
            'K': {'primary_sensor': 2, 'acc_z': (0.9, 1.0), 'orientation': 'Up'}
        }

    def parse_sensor_data(self, data_str):
        """Parse the sensor data string from ESP32"""
        try:
            # Split the data string into parts
            parts = data_str.split(': ')[1].split(', ')
            sensor_data = {}
            
            # Extract acceleration values
            for part in parts:
                if 'AccX=' in part:
                    sensor_data['acc_x'] = float(part.split('=')[1])
                elif 'AccY=' in part:
                    sensor_data['acc_y'] = float(part.split('=')[1])
                elif 'AccZ=' in part:
                    sensor_data['acc_z'] = float(part.split('=')[1])
                elif 'Orientation=' in part:
                    sensor_data['orientation'] = part.split('=')[1].split(',')[0]
                elif 'Quaternion=' in part:
                    quat = part.split('=')[1].strip('()').split(',')
                    sensor_data['quaternion'] = [float(q) for q in quat]
            
            return sensor_data
        except Exception as e:
            print(f"Error parsing sensor data: {e}")
            return None

    def detect_letter(self, sensor_data):
        """Detect which ASL letter is being shown"""
        best_match = None
        highest_confidence = 0

        for letter, pattern in self.letter_patterns.items():
            confidence = 0
            checks = 0
            
            # Check orientation
            if pattern['orientation'] == sensor_data['orientation']:
                confidence += 1
            checks += 1
            
            # Check acceleration thresholds
            if pattern['acc_z'][0] <= sensor_data['acc_z'] <= pattern['acc_z'][1]:
                confidence += 1
            checks += 1
            
            # Calculate confidence score
            score = confidence / checks if checks > 0 else 0
            
            if score > highest_confidence:
                highest_confidence = score
                best_match = letter

        return best_match if highest_confidence > 0.7 else None

    def run(self):
        """Main loop to receive and process sensor data"""
        print(f"Starting Sign Language Recognition System...")
        print(f"Waiting for data from ESP32 on port {self.PYTHON_PORT}...")
        
        try:
            while True:
                # Receive data from ESP32
                data, addr = self.udp_socket.recvfrom(1024)
                data_str = data.decode('utf-8')
                
                # Log raw data
                print(f"\nRaw data from {addr}:")
                print(data_str)
                
                # Parse sensor data
                sensor_data = self.parse_sensor_data(data_str)
                
                if sensor_data:
                    # Add to buffer for motion detection
                    self.sensor_buffer.append(sensor_data)
                    
                    # Detect letter
                    letter = self.detect_letter(sensor_data)
                    
                    if letter:
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        print(f"\n[{timestamp}] Detected Letter: {letter}")
                        print("Sensor Data:")
                        print(f"  Orientation: {sensor_data['orientation']}")
                        print(f"  Acceleration: X={sensor_data['acc_x']:.2f}, Y={sensor_data['acc_y']:.2f}, Z={sensor_data['acc_z']:.2f}")
                
        except KeyboardInterrupt:
            print("\nScript terminated by user.")
        finally:
            self.udp_socket.close()
            print("UDP socket closed.")

if __name__ == "__main__":
    recognizer = SignLanguageRecognizer()
    recognizer.run()