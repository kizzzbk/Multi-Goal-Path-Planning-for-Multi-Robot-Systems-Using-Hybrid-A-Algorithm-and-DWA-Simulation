import math
import heapq
import random
from typing import List, Tuple, Optional, Dict
from collections import deque

class Map:
    def __init__(self, filename: str):
        self.grid = []
        self.height = 0
        self.width = 0
        self.load_map(filename)

    def load_map(self, filename: str):
        with open(filename, 'r') as f:
            lines = f.readlines()
        for line in lines:
            if line.startswith('type') or line.startswith('height') or line.startswith('width') or line.startswith('map'):
                continue
            self.grid.append(list(line.strip()))
        self.height = len(self.grid)
        self.width = len(self.grid[0]) if self.grid else 0

    def is_valid(self, x: int, y: int) -> bool:
        return 0 <= x < self.height and 0 <= y < self.width and self.grid[x][y] == '.'

    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int, float]]:
        neighbors = []
        directions = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.4), (-1, 1, 1.4), (1, -1, 1.4), (1, 1, 1.4)
        ]
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                if dx != 0 and dy != 0:
                    if not (self.is_valid(x + dx, y) and self.is_valid(x, y + dy)):
                        continue
                neighbors.append((nx, ny, cost))
        return neighbors

    def find_empty_cell_near(self, x: int, y: int) -> Optional[Tuple[int, int]]:
        if self.is_valid(x, y):
            return (x, y)
        visited = set()
        queue = deque([(x, y)])
        visited.add((x, y))
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while queue:
            cx, cy = queue.popleft()
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited and 0 <= nx < self.height and 0 <= ny < self.width:
                    if self.is_valid(nx, ny):
                        return (nx, ny)
                    queue.append((nx, ny))
                    visited.add((nx, ny))
        return None
    
    def getter_length_width(self) -> Tuple[int, int]:
        return self.height, self.width

class Robot:
    def __init__(self, id: int, energy: float, position: Tuple[int, int]):
        self.id = id
        self.energy = energy
        self.position = position
        self.assigned_goals = []
        self.road_map = []
        self.current_goal = (0,0)
        self.A_star_path = []
        self.astar_idx = 0
        self.replanning_times = 0
        self.count_stuck = 0
        self.counting_step = 0
        self.recorded_x, self.recorded_y = 0,0
        self.way_points = []
        self.static_ob_awareness = True
        self.history_positions = []
        self.flag_stuck = False
        self.FORWARDWEIGHT = 1
        self.OBSTACLEWEIGHT = 5

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.x, self.y, self.theta = 0,0,0
        self.traced_goal_idx = 0
        self.flag_finished = False

class Goal:
    def __init__(self, id: int, position: Tuple[int, int], energy_requirement: int):
        self.id = id
        self.position = position
        self.energy_requirement = energy_requirement
        self.is_charged = False

class ChargingStation:
    def __init__(self, position: Tuple[int, int]):
        self.position = position

class Simulation:
    def __init__(self, map_filename: str, robot_count: int, goal_count: int):
        self.map = Map(map_filename)
        self.robots = []
        self.goals = []
        self.charging_station = None
        self.log = []
        self.robot_count = robot_count
        self.goal_count = goal_count
        self.initialize()
    
    def getter_map_length_width(self) -> Tuple[int, int]:
        return self.map.getter_length_width()

    def initialize(self):
        initial_energy = math.ceil(math.sqrt(self.map.height**2 + self.map.width**2))
        center_x, center_y = self.map.height // 2, self.map.width // 2
        station_pos = self.map.find_empty_cell_near(center_x, center_y)
        if not station_pos:
            raise ValueError("No empty cell found for charging station")
        self.charging_station = ChargingStation(station_pos)
        self.log.append(f"[Map Info] Charging Station {station_pos}")
        robot_positions = []
        for i in range(self.robot_count): 
            pos = self.map.find_empty_cell_near(station_pos[0] + i, station_pos[1] + i)
            if not pos or pos in robot_positions or pos == station_pos:
                pos = self.map.find_empty_cell_near(station_pos[0] - i, station_pos[1] - i)
            if not pos:
                raise ValueError(f"No empty cell found for robot {i}")
            robot_positions.append(pos)
            self.robots.append(Robot(i, initial_energy, pos))
        robot_info = ", ".join(f"Robot {i} {robot_positions[i]}" for i in range(self.robot_count))
        self.log.append(f"[Map Info] {robot_info} initialized with {initial_energy}")
        used_positions = set(robot_positions + [station_pos])
        for i in range(self.goal_count):
            while True:
                x, y = random.randint(0, self.map.height - 1), random.randint(0, self.map.width - 1)
                if self.map.is_valid(x, y) and (x, y) not in used_positions:
                    self.goals.append(Goal(i, (x, y), random.randint(50, 100)))
                    used_positions.add((x, y))
                    break
        goal_info = ", ".join(f"Goal {g.id} {g.position}" for g in self.goals)
        self.log.append(f"[Map Info] {goal_info}")

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[float]:
        open_set = [(0, start)]
        g_score = {start: 0}
        f_score = {start: self.chebyshev_distance(start, goal)}
        came_from = {}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return g_score[current]
            for neighbor_x, neighbor_y, cost in self.map.get_neighbors(*current):
                neighbor = (neighbor_x, neighbor_y)
                tentative_g_score = g_score[current] + cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.chebyshev_distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return float(-1)

    def get_a_star_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Return the list of (row, col) waypoints from start to goal using A*."""
        open_set = [(0, start)]
        g_score = {start: 0}
        f_score = {start: self.chebyshev_distance(start, goal)}
        came_from = {}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Reverse to get start-to-goal order
            for neighbor_x, neighbor_y, cost in self.map.get_neighbors(*current):
                neighbor = (neighbor_x, neighbor_y)
                tentative_g_score = g_score[current] + cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.chebyshev_distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []  # Return empty list if no path found

    def chebyshev_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        return max(dx, dy) * 1.4

    def compute_effective_path_length(self, robot: Robot, goal: Goal) -> float:
        to_goal_cost = self.a_star(robot.position, goal.position)
        if to_goal_cost == float('inf'):
            return float('inf')
        to_station_cost = self.a_star(goal.position, self.charging_station.position)
        if to_station_cost == float('inf'):
            return float('inf')
        total_energy_needed = to_goal_cost + goal.energy_requirement + to_station_cost
        if robot.energy >= total_energy_needed:
            return to_goal_cost
        station_cost = self.a_star(robot.position, self.charging_station.position)
        if station_cost == float('inf'):
            return float('inf')
        return station_cost + to_goal_cost

    def find_nearest_goal(self, robot: Robot, available_goals: List[Goal]) -> Tuple[Optional[Goal], float]:
        min_distance = float('inf')
        nearest_goal = None
        for goal in available_goals:
            distance = self.compute_effective_path_length(robot, goal)
            if distance < min_distance:
                min_distance = distance
                nearest_goal = goal
        return nearest_goal, min_distance

    def assign_goals(self):
        uncharged_goals = [g for g in self.goals if not g.is_charged]
        if not uncharged_goals:
            return False
        available_robots = self.robots[:]
        assignments = {}
        while available_robots and uncharged_goals:
            robot_to_goal = {}
            for robot in available_robots:
                goal, distance = self.find_nearest_goal(robot, uncharged_goals)
                if goal:
                    robot_to_goal[robot] = (goal, distance)
            goal_to_robots = {}
            for robot, (goal, distance) in robot_to_goal.items():
                if goal not in goal_to_robots:
                    goal_to_robots[goal] = []
                goal_to_robots[goal].append((robot, distance))
            for goal, robots in goal_to_robots.items():
                if robots:
                    robots.sort(key=lambda x: (x[1], x[0].id))
                    assigned_robot, assigned_distance = robots[0]
                    assignments[assigned_robot] = goal
                    if len(robots) > 1:
                        for i in range(1, len(robots)):
                            other_robot, other_distance = robots[i]
                            if assigned_distance == other_distance:
                                self.log.append(f"[Robot] Robot {other_robot.id} conflict with Robot {assigned_robot.id} at goal {goal.id}. Robot {assigned_robot.id} was assigned to goal {goal.id} (path length: {assigned_distance:.1f} = {other_distance:.1f}, lowest ID).")
                            else:
                                self.log.append(f"[Robot] Robot {other_robot.id} conflict with Robot {assigned_robot.id} at goal {goal.id}. Robot {assigned_robot.id} was assigned to goal {goal.id} (path length: {assigned_distance:.1f} < {other_distance:.1f}).")
                    uncharged_goals.remove(goal)
                    available_robots.remove(assigned_robot)
        for robot, goal in assignments.items():
            to_goal_cost = self.a_star(robot.position, goal.position)
            to_station_cost = self.a_star(goal.position, self.charging_station.position)
            total_energy_needed = to_goal_cost + goal.energy_requirement + to_station_cost
            if robot.energy < total_energy_needed:
                station_cost = self.a_star(robot.position, self.charging_station.position)
                self.log.append(f"[Robot] Robot {robot.id} moved to charging station at {self.charging_station.position} with energy cost {station_cost:.1f}. Remaining energy: {robot.energy - station_cost:.1f}.")
                robot.position = self.charging_station.position
                robot.energy -= station_cost
                recharge_amount = total_energy_needed - robot.energy
                robot.energy += recharge_amount
                self.log.append(f"[Robot] Robot {robot.id} recharged at charging station to {robot.energy:.1f} units.")
                self.log.append(f"[Robot] Robot {robot.id} moved to goal {goal.id} at {goal.position} with energy cost {to_goal_cost:.1f}. Remaining energy: {robot.energy - to_goal_cost:.1f}.")
                robot.position = goal.position
                robot.energy -= to_goal_cost
            else:
                self.log.append(f"[Robot] Robot {robot.id} moved to goal {goal.id} at {goal.position} with energy cost {to_goal_cost:.1f}. Remaining energy: {robot.energy - to_goal_cost:.1f}.")
                robot.position = goal.position
                robot.energy -= to_goal_cost
                robot.road_map.append('station')
            self.log.append(f"[Goal] Goal {goal.id} requested {goal.energy_requirement} energy units, charged by Robot {robot.id}.")
            robot.energy -= goal.energy_requirement
            self.log.append(f"[Robot] Robot {robot.id} charged goal {goal.id}. Remaining energy: {robot.energy:.1f}.")
            goal.is_charged = True
            robot.assigned_goals.append(goal.id)
            robot.road_map.append(goal.position)
        return True

    def run(self):
        while self.assign_goals():
            pass
        for robot in self.robots:
            to_station_cost = self.a_star(robot.position, self.charging_station.position)
            if to_station_cost != float('inf'):
                self.log.append(f"[Robot] Robot {robot.id} returned to charging station at {self.charging_station.position} with energy cost {to_station_cost:.1f}. Remaining energy: {robot.energy - to_station_cost:.1f}.")
                robot.position = self.charging_station.position
                robot.energy -= to_station_cost
        with open('log.txt', 'w') as f:
            for entry in self.log:
                f.write(entry + '\n')
        # Construct the output as a list of lists of goal positions for each robot
        goal_positions_list = []
        for robot in self.robots:
            robot_goals = [self.charging_station.position if isinstance(goal, str) else goal for goal in robot.road_map]
            goal_positions_list.append(robot_goals)
        return goal_positions_list

if __name__ == '__main__':
    sim = Simulation('Boston_1_1024.map', robot_count=3, goal_count=10)
    assignments = sim.run()
    print("Goal assignments:", assignments)