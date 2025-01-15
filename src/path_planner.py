import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Set
import heapq
import os
from datetime import datetime

class TimeAwarePathPlanner:
    def __init__(self, grid_size: int = 100):
        self.grid_size = grid_size
        # Create weight grid with 10% random high-weight points
        self.weights = np.zeros((grid_size + 1, grid_size + 1, grid_size + 1))
        random_points = np.random.random(self.weights.shape) < 0.1
        self.weights[random_points] = np.random.uniform(5, 10, np.sum(random_points))
        
        # Store occupied points with their times
        self.occupied_points = {}  # (x, y, z) -> set of times
        
    def manhattan_distance(self, start: Tuple[int, int, int], end: Tuple[int, int, int]) -> int:
        return sum(abs(a - b) for a, b in zip(start, end))
    
    def get_neighbors(self, point: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        x, y, z = point
        neighbors = []
        for dx, dy, dz in [(0,0,1), (0,0,-1), (0,1,0), (0,-1,0), (1,0,0), (-1,0,0)]:
            new_x, new_y, new_z = x + dx, y + dy, z + dz
            if (0 <= new_x <= self.grid_size and 
                0 <= new_y <= self.grid_size and 
                0 <= new_z <= self.grid_size):
                neighbors.append((new_x, new_y, new_z))
        return neighbors
    
    def is_point_free(self, point: Tuple[int, int, int], time: float) -> bool:
        """Check if a point is free at a specific time"""
        if point in self.occupied_points:
            # Check if the point is occupied at this time
            return time not in self.occupied_points[point]
        return True
    
    def mark_path_occupied(self, path: List[Tuple[int, int, int]], velocity: float):
        """Mark all points in the path as occupied at their respective times"""
        for i, point in enumerate(path):
            time = i / velocity  # Time to reach this point
            if point not in self.occupied_points:
                self.occupied_points[point] = set()
            self.occupied_points[point].add(time)
    
    def find_path(self, start: Tuple[int, int, int], end: Tuple[int, int, int], 
                  start_time: float, velocity: float) -> List[Tuple[int, int, int]]:
        """Find path from start to end avoiding occupied points at specific times"""
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, end)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            for neighbor in self.get_neighbors(current):
                # Calculate time at this point
                tentative_g = g_score[current] + 1 + self.weights[neighbor]
                time_at_point = start_time + tentative_g / velocity
                
                if not self.is_point_free(neighbor, time_at_point):
                    continue
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.manhattan_distance(neighbor, end)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found
    
    def plot_paths(self, paths: List[List[Tuple[int, int, int]]]):
        """Plot all paths in 3D and save to file"""
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        colors = ['b', 'r', 'g', 'c', 'm', 'y']
        
        for i, path in enumerate(paths):
            if path:
                xs, ys, zs = zip(*path)
                color = colors[i % len(colors)]
                ax.plot(xs, ys, zs, color=color, linewidth=2, label=f'Path {i+1}')
                ax.scatter(xs[0], ys[0], zs[0], color=color, marker='o', s=100, label=f'Start {i+1}')
                ax.scatter(xs[-1], ys[-1], zs[-1], color=color, marker='s', s=100, label=f'End {i+1}')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Paths with Time Constraints')
        ax.legend()
        
        # Save plot
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filename = f'logs/path_visualization.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Plot saved to: {filename}")

def main():
    # Example usage
    planner = TimeAwarePathPlanner()
    
    # Define paths to find
    path_requests = [
        ((0, 0, 0), (100, 100, 100)),
        ((0, 100, 0), (100, 0, 100)),
        ((50, 0, 0), (50, 100, 100))
    ]
    
    velocity = 2.0  # m/s
    paths = []
    start_time = 0.0
    
    print("Finding paths...")
    for i, (start, end) in enumerate(path_requests):
        print(f"Calculating path {i+1}...")
        path = planner.find_path(start, end, start_time, velocity)
        if path:
            paths.append(path)
            planner.mark_path_occupied(path, velocity)
            print(f"Path {i+1} found! Length: {len(path)} points")
        else:
            print(f"No valid path found for path {i+1}")
    
    if paths:
        print("\nPlotting paths...")
        planner.plot_paths(paths)
    
    return paths

if __name__ == "__main__":
    main()