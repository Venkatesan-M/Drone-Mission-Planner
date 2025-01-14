import matplotlib.pyplot as plt

def plot_mission_path(waypoints):
    """
    Plot the mission path on a 2D map (latitude vs. longitude).
    Args:
        waypoints: List of waypoints, each containing latitude and longitude.
                   Example: [{'lat': 52.229, 'lon': 21.012}, ...]
    """
    # Extract latitudes and longitudes
    latitudes = [point['lat'] for point in waypoints]
    longitudes = [point['lon'] for point in waypoints]
    
    # Plot the path
    plt.figure(figsize=(10, 6))
    plt.plot(longitudes, latitudes, marker='o', linestyle='-', color='b', label="Mission Path")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Mission Path")
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_altitude_profile(waypoints):
    """
    Plot the altitude profile of the mission.
    Args:
        waypoints: List of waypoints, each containing altitude (and optionally lat/lon).
                   Example: [{'lat': 52.229, 'lon': 21.012, 'alt': 300}, ...]
    """
    # Extract altitudes
    altitudes = [point['alt'] for point in waypoints]
    waypoint_indices = list(range(1, len(waypoints) + 1))  # Waypoint indices for x-axis
    
    # Plot the altitude profile
    plt.figure(figsize=(10, 6))
    plt.plot(waypoint_indices, altitudes, marker='o', linestyle='-', color='g', label="Altitude Profile")
    plt.xlabel("Waypoint Index")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude Profile")
    plt.grid(True)
    plt.legend()
    plt.show()

