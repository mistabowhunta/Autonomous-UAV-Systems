import time
import math
import argparse
from datetime import datetime
from dronekit import connect, VehicleMode
from pymavlink import mavutil

class XP5FlightController:
    def __init__(self, connection_string, target_altitude=10):
        self.vehicle_name = "XP5"
        self.target_altitude = target_altitude
        
        # Flight profiles
        self.thrust_ascend = 0.50001
        self.thrust_hover = 0.5
        self.thrust_descend = 0.49999
        self.flight_mode = "GUIDED_NOGPS"
        self.failsafe_mode = "STABILIZE"

        # Encapsulated telemetry state: FS: forward sensor, LS: left sensor, RS: right sensor, BS: back sensor, ALT: down sensor, US: up sensor
        self.telemetry = {
            'FS': 0.0, 'LS': 0.0, 'RS': 0.0, 
            'BS': 0.0, 'ALT': 0.0, 'US': 0.0
        }

        print(f"Connecting to {self.vehicle_name} on {connection_string}...")
        self.vehicle = connect(connection_string, wait_ready=False, baud=57600)
        self.vehicle.wait_ready(True, timeout=60)
        
        self._setup_telemetry_listener()
        self._run_pre_flight_checks()

    def _setup_telemetry_listener(self):
        """Asynchronous listener for LiDAR distance sensors."""
        @self.vehicle.on_message('DISTANCE_SENSOR')
        def listener(vehicle, name, message):
            sensor_map = {0: 'FS', 1: 'LS', 2: 'RS', 3: 'BS', 4: 'ALT', 5: 'US'}
            if message.id in sensor_map:
                distance = float(message.current_distance)
                # Adjust downward sensor for ground clearance
                if message.id == 4:
                    distance -= 2.2 
                
                self.telemetry[sensor_map[message.id]] = distance
                
            # Optional: Log output (can be commented out for cleaner terminal during flight)
            # print(f"[{datetime.now()}] Telemetry Update: {self.telemetry}")

    def _run_pre_flight_checks(self):
        print(f"Autopilot Firmware version: {self.vehicle.version}")
        print(f"System status: {self.vehicle.system_status.state}")
        print(f"Is Armable?: {self.vehicle.is_armable}")

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        """Convert Euler angles (degrees) to Quaternions."""
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, 
                     yaw_rate=0.0, use_yaw_rate=False, thrust=0.5, duration=0):
        """Continuous attitude transmission for GUIDED_NOGPS mode."""
        self._send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate, thrust)
        
        start = time.time()
        while time.time() - start < duration:
            self._send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate, thrust)
            time.sleep(0.1)
            
        # Reset attitude
        self._send_attitude_target(0, 0, 0, 0, True, thrust)

    def _send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, 
                              yaw_rate=0.0, use_yaw_rate=False, thrust=0.5):
        if yaw_angle is None:
            yaw_angle = self.vehicle.attitude.yaw

        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, 1, 1,
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle),
            0, 0, math.radians(yaw_rate),
            thrust
        )
        self.vehicle.send_mavlink(msg)

    def execute_mission(self):
        """Primary flight sequence: Arm, Takeoff, Hover, Land."""
        # 1. Arming
        self.vehicle.wait_for_mode(self.flight_mode, 5)
        print(f"Flight mode set to: {self.flight_mode}")
        time.sleep(5)
        
        print(f"Arming {self.vehicle_name}...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        # 2. Takeoff
        print("Taking off!")
        t_end_ascend = time.time() + 15 
        
        while self.telemetry['ALT'] < self.target_altitude:
            self.set_attitude(thrust=self.thrust_ascend)
            time.sleep(0.1)
            
            # Failsafe overrides
            if self.telemetry['ALT'] >= self.target_altitude + 20:
                break
            if time.time() >= t_end_ascend:
                print("Ascent timeout reached.")
                break

        # 3. Hover
        print(f"Target altitude reached. Hovering for 5 seconds. Current ALT: {self.telemetry['ALT']}")
        self.set_attitude(thrust=self.thrust_hover, duration=5)

        # 4. Landing
        print("Initiating landing sequence...")
        self.vehicle.wait_for_mode('LAND', 10)
        
        t_end_land = time.time() + 15 
        while self.telemetry['ALT'] > 2:
            if time.time() >= t_end_land:
                print("Landing timeout reached.")
                break
            time.sleep(0.5)

        self.deinitialize()

    def deinitialize(self):
        print(f"Disarming {self.vehicle_name}...")
        self.vehicle.disarm(True, 10)
        time.sleep(2)

        self.vehicle.mode = VehicleMode(self.failsafe_mode)
        print(f"Flight mode reset to: {self.failsafe_mode}")

        self.vehicle.close()
        print("Mission Complete. Hardware safely decoupled.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='XP5 Autonomous Flight Controller')
    parser.add_argument('--connect', default='COM12', help='Vehicle connection target string.')
    args = parser.parse_args()

    # Initialize and run the UAV controller
    uav = XP5FlightController(connection_string=args.connect, target_altitude=10)
    
    try:
        uav.execute_mission()
    except KeyboardInterrupt:
        print("\nManual override triggered. Initiating emergency shutdown.")
        uav.deinitialize()
