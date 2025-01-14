# examples/simple_mission.py
from pathlib import Path
from src.mission_planner import DroneMissionPlanner
import logging
import sys
import time
from dronekit import VehicleMode

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        stream=sys.stdout
    )
    return logging.getLogger(__name__)

def wait_for_mode_change(vehicle, mode, timeout=10):
    """Wait for vehicle mode to change with timeout"""
    start_time = time.time()
    while vehicle.mode.name != mode:
        if time.time() - start_time > timeout:
            return False
        time.sleep(0.1)
    return True

def run_simple_mission():
    """
    Run a simple mission using default configuration with enhanced status checking
    """
    logger = setup_logging()
    
    try:
        # Get the project root directory
        project_root = Path(__file__).resolve().parent.parent
        config_path = project_root / 'config' / 'default_mission.yaml'
        
        logger.info("Initializing mission planner...")
        planner = DroneMissionPlanner(config_path=config_path)
        
        # Plot initial mission
        logger.info("Plotting initial mission path...")
        planner.plot_mission()
        
        # Connect to vehicle
        logger.info("Connecting to vehicle...")
        if not planner.connect_vehicle():
            logger.error("Failed to connect to vehicle")
            return
        
        # Wait for GPS lock
        logger.info("Waiting for GPS lock...")
        while planner.vehicle.gps_0.fix_type < 3:
            logger.info(f"GPS fix: {planner.vehicle.gps_0.fix_type}")
            time.sleep(1)
        
        # Upload mission
        logger.info("Uploading mission...")
        if not planner.upload_mission():
            logger.error("Failed to upload mission")
            return
        
        # Change to GUIDED mode first
        logger.info("Changing to GUIDED mode...")
        planner.vehicle.mode = VehicleMode("GUIDED")
        if not wait_for_mode_change(planner.vehicle, "GUIDED"):
            logger.error("Failed to change to GUIDED mode")
            return
        
        # Arm vehicle
        logger.info("Arming vehicle...")
        planner.vehicle.armed = True
        start_time = time.time()
        while not planner.vehicle.armed:
            if time.time() - start_time > 10:
                logger.error("Failed to arm vehicle")
                return
            time.sleep(1)
        
        # Change to AUTO mode
        logger.info("Changing to AUTO mode...")
        planner.vehicle.mode = VehicleMode("AUTO")
        if not wait_for_mode_change(planner.vehicle, "AUTO"):
            logger.error("Failed to change to AUTO mode")
            return
        
        logger.info("Mission started!")
        
        # Monitor mission progress
        while True:
            # Log current status
            logger.info(f"Mode: {planner.vehicle.mode.name}")
            logger.info(f"Armed: {planner.vehicle.armed}")
            logger.info(f"Current waypoint: {planner.vehicle.commands.next}")
            logger.info(f"GPS: {planner.vehicle.gps_0.fix_type}")
            logger.info(f"Altitude: {planner.vehicle.location.global_relative_frame.alt}")
            logger.info("------------------------")
            
            # Check if mission is complete
            if planner.vehicle.commands.next > len(planner.waypoints):
                logger.info("Mission complete!")
                break
                
            # Check if mode changed
            if planner.vehicle.mode.name != "AUTO":
                logger.warning(f"Mode changed to {planner.vehicle.mode.name}")
                break
                
            # Check if vehicle disarmed
            if not planner.vehicle.armed:
                logger.warning("Vehicle disarmed!")
                break
                
            time.sleep(2)
        
        # Plot final mission path
        logger.info("Plotting final mission path...")
        planner.plot_mission()
        
    except KeyboardInterrupt:
        logger.info("Mission terminated by user")
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
    finally:
        if 'planner' in locals() and planner.vehicle:
            planner.vehicle.close()

if __name__ == "__main__":
    run_simple_mission()