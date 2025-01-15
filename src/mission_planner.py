import time
import math
import logging
from pathlib import Path
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import matplotlib.pyplot as plt
import yaml
import numpy as np

class DroneMissionPlanner:
    def __init__(self, config_path=None):
        self.logger = self._setup_logging()
        self.config = self._load_config(config_path)
        self.connection_string = self.config.get('connection_string', 'udp:127.0.0.1:14550')
        self.vehicle = None
        self.waypoints = self.config.get('default_waypoints', [])
        self.mission_started = False
        self.cruise_speed = self.config.get('cruise_speed', 5)  # m/s

    def _setup_logging(self):
        log_dir = Path('./logs')
        log_dir.mkdir(exist_ok=True)
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_dir / 'drone_mission.log'),
                logging.StreamHandler()
            ]
        )
        return logging.getLogger(__name__)

    def _load_config(self, config_path):
        try:
            with open(config_path, 'r') as file:
                self.logger.info(f"Loading config from {config_path}")
                return yaml.safe_load(file)
        except Exception as e:
            self.logger.error(f"Failed to load config: {e}")
            raise

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points using Haversine formula"""
        R = 6371000  # Earth's radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (math.sin(delta_phi/2) * math.sin(delta_phi/2) +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(delta_lambda/2) * math.sin(delta_lambda/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points"""
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        y = math.sin(delta_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        return math.degrees(math.atan2(y, x))

    def _get_perpendicular_point(self, lat1, lon1, lat2, lon2, distance):
        """Calculate a point perpendicular to the path at given distance"""
        bearing = self._calculate_bearing(lat1, lon1, lat2, lon2) + 90  # Add 90 degrees for perpendicular
        bearing_rad = math.radians(bearing)
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        
        angular_distance = distance / 6371000  # Earth's radius in meters
        
        lat2_rad = math.asin(
            math.sin(lat1_rad) * math.cos(angular_distance) +
            math.cos(lat1_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
        )
        
        lon2_rad = lon1_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat1_rad),
            math.cos(angular_distance) - math.sin(lat1_rad) * math.sin(lat2_rad)
        )
        
        return math.degrees(lat2_rad), math.degrees(lon2_rad)

    def connect_vehicle(self):
        try:
            self.logger.info("Connecting to vehicle...")
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=30)
            self.logger.info("Successfully connected to the vehicle.")
        except Exception as e:
            self.logger.error(f"Error connecting to vehicle: {e}")
            raise

    def _calculate_mission_stats(self):
        """Calculate total distance and estimated time for the mission"""
        total_distance = 0
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            distance = self._calculate_distance(wp1['lat'], wp1['lon'], wp2['lat'], wp2['lon'])
            alt_change = abs(wp2['alt'] - wp1['alt'])
            total_distance += math.sqrt(distance**2 + alt_change**2)
        
        estimated_time = total_distance / self.cruise_speed
        return total_distance, estimated_time

    def upload_mission(self):
        try:
            self.logger.info("Uploading mission commands...")
            cmds = self.vehicle.commands
            cmds.clear()
            
            # Add takeoff command
            takeoff_alt = self.waypoints[0]['alt']
            cmds.add(Command(
                0, 0, 0, 
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 
                self.waypoints[0]['lat'],
                self.waypoints[0]['lon'],
                takeoff_alt
            ))
            
            # Add waypoints
            for wp in self.waypoints:
                cmds.add(Command(
                    0, 0, 0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0,
                    wp['lat'], wp['lon'], wp['alt']
                ))
            
            # Add RTL command at the end
            cmds.add(Command(
                0, 0, 0, 
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0,
                0, 0, 0
            ))
            
            # Upload commands
            cmds.upload()
            self.logger.info("Flight plan received")
            time.sleep(2)  # Wait for the upload to complete
            
            # Verify upload
            self.vehicle.commands.download()
            self.vehicle.commands.wait_ready()
            
            if len(self.vehicle.commands) == len(self.waypoints) + 2:  # +2 for takeoff and RTL
                self.logger.info(f"Successfully uploaded {len(self.waypoints)} waypoints")
                return True
            else:
                self.logger.error("Mission upload verification failed")
                return False
                
        except Exception as e:
            self.logger.error(f"Error uploading mission: {str(e)}")
            return False

    def start_mission(self):
        self.logger.info("Starting mission in GUIDED mode...")

        # Set GUIDED mode and arm, Quad can't be armed in AUTO Mode
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.vehicle.mode.name == "GUIDED":
            self.logger.info("Waiting for GUIDED mode...")
            time.sleep(1)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.logger.info("Waiting for vehicle to arm...")
            time.sleep(1)

        # Takeoff to the specified altitude
        takeoff_altitude = self.config['takeoff_altitude']
        self.vehicle.simple_takeoff(takeoff_altitude)

        # Wait until the altitude is reached
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            self.logger.info(f"Current altitude: {current_altitude:.2f} m")
            if current_altitude >= takeoff_altitude * 0.95:
                self.logger.info("Reached target altitude.")
                break
            time.sleep(1)

        # Switch to AUTO mode
        self.logger.info("Switching to AUTO mode for the mission...")
        self.vehicle.mode = VehicleMode("AUTO")

        # Monitor mission progress
        prev_waypoint = 0
        mission_complete = False
        landing_phase = False

        while not mission_complete:
            next_wp = self.vehicle.commands.next
            current_altitude = self.vehicle.location.global_relative_frame.alt

            # Check if we're in landing phase
            if next_wp == len(self.waypoints) + 2:  # +2 for takeoff and land commands
                if not landing_phase:
                    self.logger.info("Entering landing phase...")
                    landing_phase = True

                # Check if we've landed
                if landing_phase and current_altitude < 0.2:
                    self.logger.info("Landing detected. Mission complete.")
                    mission_complete = True
                    break

            # Only update progress if we're not in landing phase
            if not landing_phase and next_wp != prev_waypoint:
                # Calculate remaining distance and ETA
                current_pos = self.vehicle.location.global_relative_frame
                remaining_distance = 0
                for i in range(next_wp - 1, len(self.waypoints)):
                    if i == next_wp - 1:
                        remaining_distance += self._calculate_distance(
                            current_pos.lat, current_pos.lon,
                            self.waypoints[i]['lat'], self.waypoints[i]['lon']
                        )
                    else:
                        remaining_distance += self._calculate_distance(
                            self.waypoints[i-1]['lat'], self.waypoints[i-1]['lon'],
                            self.waypoints[i]['lat'], self.waypoints[i]['lon']
                        )

                eta = remaining_distance / self.cruise_speed
                self.logger.info(f"Current Waypoint: {next_wp}")
                self.logger.info(f"Remaining distance: {remaining_distance:.2f} meters")
                self.logger.info(f"Estimated time to completion: {eta:.2f} seconds")
                prev_waypoint = next_wp

            time.sleep(1)

        # Wait for disarming
        while self.vehicle.armed:
            self.logger.info("Waiting for disarming...")
            time.sleep(1)

        self.logger.info("Mission completed and vehicle disarmed.")


    def plot_mission(self):
        lats = [wp['lat'] for wp in self.waypoints]
        lons = [wp['lon'] for wp in self.waypoints]
        
        # Calculate perpendicular waypoint for visualization
        lat1, lon1 = self.waypoints[9]['lat'], self.waypoints[9]['lon']
        lat2, lon2 = self.waypoints[10]['lat'], self.waypoints[10]['lon']
        perp_lat, perp_lon = self._get_perpendicular_point(lat2, lon2, lat1, lon1, 100)
        
        # Insert perpendicular point into the path
        lats.insert(10, perp_lat)
        lons.insert(10, perp_lon)

        plt.figure(figsize=(10, 6))
        plt.plot(lons, lats, marker='o', linestyle='-', color='b', label="Mission Path")
        plt.plot(lons[10], lats[10], marker='*', color='r', markersize=15, label="Perpendicular Waypoint")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("Mission Path with Perpendicular Waypoint")
        plt.grid()
        plt.legend()
        plt.savefig('./logs/mission_path.png')
        self.logger.info("Mission path plot saved as 'logs/mission_path.png'.")
        plt.close()