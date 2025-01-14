# src/mission_planner.py
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List
import threading
import logging
import sys
import yaml
import os
from pathlib import Path

class DroneMissionPlanner:
    def __init__(self, config_path=None):
        """
        Initialize the drone mission planner
        Args:
            config_path: Path to configuration YAML file
        """
        self.logger = self._setup_logging()
        self.config = self._load_config(config_path)
        self.connection_string = self.config.get('connection_string', 'udp:127.0.0.1:14550')
        self.vehicle = None
        self.waypoints = self.config.get('default_waypoints', [])
        self.mission_started = False
        self.mission_complete = False

    def _setup_logging(self):
        """Setup logging configuration"""
        # Get the project root directory (2 levels up from src/)
        project_root = Path(__file__).resolve().parent.parent.parent
        log_dir = project_root / 'logs'
        log_dir.mkdir(exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_dir / 'drone_mission.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        return logging.getLogger(__name__)

    def _load_config(self, config_path=None):
        """Load configuration from YAML file"""
        try:
            if config_path is None:
                # Get the project root directory (2 levels up from src/)
                project_root = Path(__file__).resolve().parent.parent.parent
                config_path = project_root / 'config' / 'default_mission.yaml'
            
            self.logger.info(f"Loading config from: {config_path}")
            
            if not config_path.exists():
                self.logger.error(f"Config file not found at: {config_path}")
                raise FileNotFoundError(f"Config file not found at: {config_path}")
                
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
                
        except Exception as e:
            self.logger.error(f"Error loading config: {str(e)}")
            self.logger.info("Using default configuration")
            return {
                'connection_string': 'udp:127.0.0.1:14550',
                'takeoff_altitude': 30,
                'cruise_speed': 10,
                'waypoint_acceptance_radius': 5,
                'default_waypoints': [
                    {'lat': 47.3977419, 'lon': 8.5455938, 'alt': 50},
                    {'lat': 47.3977419, 'lon': 8.5456938, 'alt': 60},
                    # ... rest of default waypoints ...
                ]
            }

    def connect_vehicle(self):
        """Connect to the vehicle with retry logic"""
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self.logger.info(f"Attempting to connect to vehicle (Attempt {attempt + 1}/{max_attempts})")
                self.vehicle = connect(self.connection_string, wait_ready=True, timeout=30)
                self.logger.info("Successfully connected to vehicle")
                return True
            except Exception as e:
                self.logger.error(f"Connection attempt {attempt + 1} failed: {str(e)}")
                if attempt < max_attempts - 1:
                    time.sleep(2)
                else:
                    self.logger.error("Failed to connect after maximum attempts")
                    return False

    def wait_for_armable(self, timeout=60):
        """Wait for vehicle to become armable"""
        start_time = time.time()
        self.logger.info("Waiting for vehicle to become armable...")
        
        while not self.vehicle.is_armable:
            if time.time() - start_time > timeout:
                self.logger.error("Timeout waiting for vehicle to become armable")
                return False
            time.sleep(1)
            
        self.logger.info("Vehicle is now armable")
        return True

    def upload_mission(self):
        """Upload mission commands to vehicle with verification"""
        try:
            self.logger.info("Uploading mission commands...")
            cmds = self.vehicle.commands
            cmds.clear()
            
            # Add takeoff command
            cmds.add(Command(
                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 
                self.waypoints[0]['lat'],
                self.waypoints[0]['lon'],
                self.waypoints[0]['alt']
            ))
            
            # Add waypoints
            for wp in self.waypoints:
                cmds.add(Command(
                    0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0,
                    wp['lat'], wp['lon'], wp['alt']
                ))
            
            cmds.upload()
            self.logger.info(f"Successfully uploaded {len(self.waypoints)} waypoints")
            
            # Verify upload
            cmds.download()
            cmds.wait_ready()
            if len(cmds) != len(self.waypoints) + 1:  # +1 for takeoff command
                self.logger.error("Mission upload verification failed")
                return False
                
            return True
            
        except Exception as e:
            self.logger.error(f"Error uploading mission: {str(e)}")
            return False

    def monitor_mission(self):
        """Monitor mission progress and print updates"""
        last_print_time = 0
        
        while not self.mission_complete and self.vehicle:
            current_time = time.time()
            
            # Update every 2 seconds
            if current_time - last_print_time >= 2:
                try:
                    if self.vehicle.commands.next <= len(self.waypoints):
                        current_wp = self.waypoints[self.vehicle.commands.next - 1]
                        
                        # Calculate remaining distance
                        remaining_dist = 0
                        for i in range(self.vehicle.commands.next - 1, len(self.waypoints) - 1):
                            remaining_dist += self.get_distance_between_points(
                                self.waypoints[i]['lat'], self.waypoints[i]['lon'],
                                self.waypoints[i + 1]['lat'], self.waypoints[i + 1]['lon']
                            )
                        
                        # Estimate time (assuming average speed of 10 m/s)
                        est_time = remaining_dist / 10.0
                        
                        self.logger.info(f"Current waypoint: {self.vehicle.commands.next}")
                        self.logger.info(f"Current mode: {self.vehicle.mode.name}")
                        self.logger.info(f"GPS: {self.vehicle.gps_0.fix_type}")
                        self.logger.info(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
                        self.logger.info(f"Remaining distance: {remaining_dist:.1f} meters")
                        self.logger.info(f"Estimated time: {est_time:.1f} seconds")
                        self.logger.info("------------------------")
                    
                    last_print_time = current_time
                except Exception as e:
                    self.logger.error(f"Error in monitor_mission: {str(e)}")
            
            time.sleep(0.1)

    def get_distance_between_points(self, lat1: float, lon1: float, 
                                  lat2: float, lon2: float) -> float:
        """Calculate distance between two points in meters"""
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

    def start_mission(self):
        """Start and monitor the mission with proper initialization checks"""
        if not self.connect_vehicle():
            return False
            
        if not self.wait_for_armable():
            return False
            
        if not self.upload_mission():
            return False
            
        try:
            self.logger.info("Starting mission...")
            
            # Ensure we're in GUIDED mode first
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(2)
            
            # Arm vehicle
            self.vehicle.armed = True
            while not self.vehicle.armed:
                self.logger.info("Waiting for vehicle to arm...")
                time.sleep(1)
            
            # Switch to AUTO mode after arming
            self.vehicle.mode = VehicleMode("AUTO")
            time.sleep(2)
            
            if self.vehicle.mode.name != "AUTO":
                self.logger.error("Failed to switch to AUTO mode")
                return False
            
            self.mission_started = True
            self.logger.info("Mission started successfully")
            
            # Start monitoring thread
            monitor_thread = threading.Thread(target=self.monitor_mission)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            # Wait for mission completion
            while not self.mission_complete:
                if not self.vehicle.armed:
                    self.logger.warning("Vehicle disarmed!")
                    break
                    
                if self.vehicle.mode.name != "AUTO":
                    self.logger.warning(f"Vehicle mode changed to {self.vehicle.mode.name}")
                    break
                    
                if self.vehicle.commands.next > len(self.waypoints):
                    self.logger.info("Mission complete!")
                    self.mission_complete = True
                    break
                    
                time.sleep(1)
            
            self.logger.info("Mission ended")
            self.vehicle.close()
            return True
            
        except Exception as e:
            self.logger.error(f"Error during mission execution: {str(e)}")
            if self.vehicle:
                self.vehicle.close()
            return False

    def plot_mission(self):
        """Plot the mission path in 2D"""
        lats = [wp['lat'] for wp in self.waypoints]
        lons = [wp['lon'] for wp in self.waypoints]
        
        plt.figure(figsize=(10, 10))
        plt.plot(lons, lats, 'b-', linewidth=2)
        plt.plot(lons, lats, 'r.', markersize=10)
        
        for i, (lon, lat) in enumerate(zip(lons, lats)):
            plt.annotate(f'WP{i+1}', (lon, lat), 
                        xytext=(5, 5), textcoords='offset points')
        
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('Mission Path')
        plt.grid(True)
        plt.show()