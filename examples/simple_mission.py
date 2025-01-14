from src.mission_planner import DroneMissionPlanner

def main():
    planner = DroneMissionPlanner('./config/default_mission.yaml')   
    planner.connect_vehicle()
    planner.plot_mission()
    planner.upload_mission()
    planner.start_mission()
    

if __name__ == "__main__":
    main()