import math
import numpy as np

def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the distance between two points on the Earth's surface.
    Args:
        lat1, lon1: Latitude and longitude of the first point in degrees.
        lat2, lon2: Latitude and longitude of the second point in degrees.
    Returns:
        Distance in kilometers.
    """
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Radius of Earth in kilometers
    R = 6371.0
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the initial bearing (forward azimuth) between two points.
    Args:
        lat1, lon1: Latitude and longitude of the first point in degrees.
        lat2, lon2: Latitude and longitude of the second point in degrees.
    Returns:
        Bearing in degrees from north.
    """
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Difference in longitude
    dlon = lon2 - lon1
    
    # Bearing calculation
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    
    # Convert to degrees and normalize to 0-360
    initial_bearing = math.degrees(initial_bearing)
    return (initial_bearing + 360) % 360

