import pygame, os, math, time, random, copy
from pygame.locals import *
import time
import pandas as pd 
from matplotlib import pyplot as plt 
import numpy as np
import random 
import time
 

from Multi_Robot_Allocation_Multi_Goals_3 import Simulation, Robot, Goal

# Set random seed
random.seed(34)

class Environment:
    def __init__(self, num_obstacles, sim, assignments):   
        # Set parameters for drawing 
        self.WIDTH = 1920  # Screen width in pixels
        self.HEIGHT = 1000  # Screen height in pixels
        self.size = [self.WIDTH, self.HEIGHT]
        # Compute scaling factor to fit map (512x512 meters) in screen
        
        # Define colors
        self.black = (20,20,40)
        self.lightblue = (0,120,255)
        self.darkblue = (0,40,160)
        self.red = (255,100,0)
        self.white = (255,255,255)
        self.blue = (0,0,255)
        self.grey = (70,70,70)
        self.green = (0,128,0)
        self.yellow = (255,255,0)  # Color for guidance path
        self.darkgrey = (50,50,50)  # Color for static obstacles
        # Screen center corresponds to (x, y) = (0, 0)
        self.u0 = self.WIDTH / 2    # Horizontal screen coordinate: u = u0 + k * x
        self.v0 = self.HEIGHT / 2   # Vertical screen coordinate: v = v0 - k * y
        # Configure parameters for visualization in pygame
        pygame.init()
        self.screen = pygame.display.set_mode(self.size)
        pygame.mouse.set_visible(1)
        
        self.success = False

        # Simulation time interval
        self.dt = 0.1
        self.STEPS_AHEAD_TO_PLAN = 10
        self.TAU = self.dt * self.STEPS_AHEAD_TO_PLAN

        self.ROBOT_SAFE_RADIUS = 20
        self.ROBOT_RADIUS = 5
        self.MAX_VEL_LINEAR = 60
        self.MAX_ACC_LINEAR = 20
        self.MAX_VEL_ANGULAR = 60
        self.MAX_ACC_ANGULAR = 20
        self.W = 2 * self.ROBOT_RADIUS

        # Initiate obstacles
        self.OBSTACLE_DRAW_RADIUS = 5  # Pixels (dynamic obstacles)
        self.OBSTACLE_RADIUS = 5  # Meters (for collision)
        self.OBSTACLE_MAX_VEL_LINEAR = 40
        self.OBSTACLE_MAX_VEL_ANGULAR = 40
        self.init_obstacle_num = num_obstacles 
        self.GOAL_RADIUS = 10  # Pixels
        # Guidance path parameters
        self.GUIDANCE_DOT_RADIUS = 1.5  # Pixels (adjustable)
        # Static obstacle parameters
        self.STATIC_OBSTACLE_SAFE_RADIUS = 5  # Pixels (adjustable)
        self.CHARGING_STATION_RADIUS = 7
        self.STATIC_OBSTACLE_RADIUS = 1
        # Simulation object for A* pathfinding and map access
        
                        
        # Define playfield corners in meters (not scaled)
        self.sim = sim
        self.map_height, self.map_width = sim.getter_map_length_width()
        padding = 100  # Pixel padding on each side
        self.k = min((self.WIDTH - 2 * padding) / self.map_width, 
                     (self.HEIGHT - 2 * padding) / self.map_height)  # Pixels per meter
        self.PLAYFIELDCORNERS = (-self.map_width / 2, -self.map_height / 2, self.map_width / 2, self.map_height / 2)
        self.view_radius = min(self.map_height, self.map_width) / 4
        
        self.robots = sim.robots
        for robot in self.robots: #Cập nhật lại init pos của robot
            robot.x, robot.y = self.grid_to_real(robot.position)
            robot.linear_vel, robot.angular_vel = 0.0, 0.0
            robot.flag_finished = False
            robot.theta = 0
            robot.traced_goal_idx = 0

        self.goals = sim.goals
        for goal in self.goals:
            goal.position = self.grid_to_real(goal.position)

        self.charging_station = sim.charging_station
        self.charging_station.position = self.grid_to_real(self.charging_station.position)
        self.robot_count = sim.robot_count
        self.goal_count = sim.goal_count
        self.goal_nb_reached = 1

        for i in range (len(assignments)):
            robot_road_map = []
            for goal in assignments[i]:
                real_x, real_y = self.grid_to_real(goal)
                robot_road_map.append([real_x, real_y])
            self.robots[i].road_map = robot_road_map
            self.robots[i].current_goal = robot_road_map[0] #Cho thêm vào reset  -----> Cho vao Environment
        self.sim_times = 0
        self.collision_times = 0
        self.avg_tg = 0
        self.avg_distance_travelled = 0
        self.tg_vec = []
        self.vl_vec = []
        self.vr_vec = []
        self.total_path_length = 0
        self.reset()

    def reset(self):
        self.obstacles = []
        for i in range(self.init_obstacle_num):
            pos_x, pos_y = self.find_valid_position()
            theta = 0
            v = random.gauss(0.0, self.OBSTACLE_MAX_VEL_LINEAR)
            w = random.gauss(0.0, self.OBSTACLE_MAX_VEL_ANGULAR)
            obstacle = [pos_x, pos_y, v, w, theta]
            self.obstacles.append(obstacle)

        self.sim = sim
        self.success = False
        self.robots = sim.robots
        for robot in self.robots: #Cập nhật lại init pos của robot
            robot.x, robot.y = self.grid_to_real(robot.position)
            robot.linear_vel, robot.angular_vel = 0.0, 0.0
            robot.flag_finished = False
            robot.theta = 0
            robot.traced_goal_idx = 0
            robot.replanning_times = 0
            robot.count_stuck = 0
            robot.wp_idx = 0
            robot.distance_travelled = 0
            robot.counting_step = 0
            robot.history_positions = []
        
        # self.goals = sim.goals
        # for goal in self.goals:
        #     goal.position = self.grid_to_real(goal.position)

        for i in range (len(assignments)):
            robot_road_map = []
            for goal in assignments[i]:
                real_x, real_y = self.grid_to_real(goal)
                robot_road_map.append([real_x, real_y])
            self.robots[i].road_map = robot_road_map
            self.robots[i].current_goal = robot_road_map[0]
            self.robots[i].A_star_path = self.get_a_star_path((self.robots[i].x, self.robots[i].y), self.robots[i].current_goal)
            self.robots[i].way_points = self.calculate_waypoints(self.robots[i].A_star_path) #Trả ra real position
            
        self.sim_over = False
        self.time_to_goal = 0.0
        self.total_path_length = 0
        
        self.history_vl_vec = []
        self.history_vr_vec = []

    #  2 hàm chuyển đổi giữa tọa độ grid và tọa độ thực tế meters
    def grid_to_real(self, position) -> tuple[float, float]:
        """Convert grid coordinates (row, col) to real-world coordinates (x, y) in meters."""
        real_x = position[1] - self.sim.map.width / 2
        real_y = self.sim.map.height / 2 - position[0]
        return (real_x, real_y)

    def real_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert real-world coordinates (x, y) in meters to grid coordinates (row, col)."""
        col = int(x + self.sim.map.width / 2)
        row = int(self.sim.map.height / 2 - y)
        return (row, col)

    def is_valid_position(self, x: float, y: float) -> bool:
        """Check if a position is valid (no overlap with static obstacles)."""
        center_row, center_col = self.real_to_grid(x, y)
        radius_meters = self.OBSTACLE_RADIUS  # 10 meters
        grid_span = int(radius_meters) + 1  # Check ~11 cells in each direction
        for dr in range(-grid_span, grid_span + 1):
            for dc in range(-grid_span, grid_span + 1):
                row, col = center_row + dr, center_col + dc
                if 0 <= row < self.sim.map.height and 0 <= col < self.sim.map.width and self.sim.map.grid[row][col] != '.':
                    # Convert grid cell to real-world coordinates
                    static_x, static_y = self.grid_to_real((row, col))
                    # Check if static obstacle is within obstacle radius
                    dist = math.sqrt((x - static_x) ** 2 + (y - static_y) ** 2)
                    if dist < self.OBSTACLE_RADIUS:
                        return False
        return True

    def find_valid_position(self, max_attempts: int = 100) -> tuple[float, float]:
        """Find a valid position that doesn't overlap with static obstacles."""
        attempts = 0
        while attempts < max_attempts:
            pos_x = random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2])
            pos_y = random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3])
            if self.is_valid_position(pos_x, pos_y):
                return (pos_x, pos_y)
            attempts += 1
        # Trong trường hợp map quá dày đặc triển khai thuật toán BFS để tìm vị trí hợp lệ (rất ít xảy ra)
        start_x, start_y = self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[1]
        queue = [(start_x, start_y)]
        visited = set([(start_x, start_y)])
        step_size = 1.0  # Step 1 meter at a time
        directions = [(0, step_size), (0, -step_size), (step_size, 0), (-step_size, 0)]
        while queue:
            x, y = queue.pop(0)
            if self.is_valid_position(x, y):
                return (x, y)
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and \
                   self.PLAYFIELDCORNERS[0] <= nx <= self.PLAYFIELDCORNERS[2] and \
                   self.PLAYFIELDCORNERS[1] <= ny <= self.PLAYFIELDCORNERS[3]:
                    queue.append((nx, ny))
                    visited.add((nx, ny))
        # If no valid position found, return bottom-left (should be rare)
        return (start_x, start_y)
    
    def calculate_Closest_Observing_Obstacle(self, robot_id, robot_position, dnm_obstacles, static_ob_awareness):
        closestdist = 100000.0
        grid_x, grid_y = self.real_to_grid(robot_position[0], robot_position[1])

        if static_ob_awareness:
            for row in range(-8,8):
                for col in range(-8,8):
                    point_x, point_y = grid_x + row, grid_y + col
                    if 0 <= point_x < self.sim.map.width and \
                        0 <= point_y < self.sim.map.height and \
                        (row, col) != (0,0) and self.sim.map.grid[point_x][point_y] != '.':

                        point_x, point_y = self.grid_to_real((point_x, point_y))
                        dx = point_x - robot_position[0] 
                        dy = point_y - robot_position[1] 
                        d = math.sqrt(dx**2+dy**2)
                        dist = d - self.ROBOT_RADIUS - self.STATIC_OBSTACLE_SAFE_RADIUS
                        if dist < closestdist:
                            closestdist = dist
                            closest_ob_x, closest_ob_y = point_x, point_y
        
        for (idx, obstacle) in enumerate(dnm_obstacles):
            dx = obstacle[0] - robot_position[0] 
            dy = obstacle[1] - robot_position[1]
            d = math.sqrt(dx**2+dy**2)
            dist = d - self.ROBOT_RADIUS - self.OBSTACLE_RADIUS
            if dist < closestdist:
                closestdist = dist
                closest_ob_x, closest_ob_y = obstacle[0], obstacle[1]

        # for robot in self.robots:
        #     if robot.id != robot_id:
        #         dx = robot.x - robot_position[0]
        #         dy = robot.y - robot_position[1]
        #         dist = math.sqrt(dx**2+dy**2)
        #         if dist < closestdist:
        #             closestdist = dist
        #             closest_ob_x, closest_ob_y = robot.x, robot.y

        # for goal in self.goals:
        #     dx = goal.position[0] - robot_position[0] - self.GOAL_RADIUS
        #     dy = goal.position[1] - robot_position[1] - self.GOAL_RADIUS
        #     dist = math.sqrt(dx**2+dy**2)
        #     if dist < closestdist:
        #         closestdist = dist
        #         closest_ob_x, closest_ob_y = goal.position[0], goal.position[1]

        # dx = self.charging_station.position[0] - robot_position[0] - self.CHARGING_STATION_RADIUS
        # dy = self.charging_station.position[1] - robot_position[1] - self.CHARGING_STATION_RADIUS
        # dist = math.sqrt(dx**2+dy**2)
        # if dist < closestdist:
        #     closestdist = dist
        #     closest_ob_x, closest_ob_y = self.charging_station.position[0], self.charging_station.position[1]

        return closestdist, closest_ob_x, closest_ob_y

    def predict_position(self, x, y, theta, vLpossible, vRpossible, delta_time):
        W = 2 * self.ROBOT_RADIUS
        if (round(vLpossible,3) == round(vRpossible,3)):
            new_x = x + vLpossible * delta_time * math.cos(theta)
            new_y = y + vLpossible * delta_time * math.sin(theta)
            new_theta = theta
            path = (0, vLpossible * delta_time)
        elif (round(vLpossible,3) == -round(vRpossible,3)):
            new_x = x
            new_y = y
            new_theta = theta + ((vRpossible - vLpossible) * delta_time / W)
            path = (1, 0)
        else:
            R = W / 2.0 * (vLpossible + vRpossible) / (vRpossible - vLpossible)
            deltatheta = (vRpossible - vLpossible) * delta_time / W
            new_x = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
            new_y = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
            new_theta = theta + deltatheta
            (cx, cy) = (x - R * math.sin(theta), y + R * math.cos(theta))
            Rabs = abs(R)
            ((tlx, tly), (Rx, Ry)) = ((int(self.u0 + self.k * (cx - Rabs)), int(self.v0 - self.k * (cy + Rabs))), (int(self.k * (2 * Rabs)), int(self.k * (2 * Rabs))))
            if (R > 0):
                start_angle = theta - math.pi/2.0
            else:
                start_angle = theta + math.pi/2.0
            stop_angle = start_angle + deltatheta
            path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle)
        return (new_x, new_y, new_theta, path)

    def planning(self, robot, goal, dnm_obsatcles, static_ob_awareness, FORWARDWEIGHT, OBSTACLEWEIGHT, predict_time):
        # obstacles_copy = copy.deepcopy(self.obstacles)
        # for _ in range(self.STEPS_AHEAD_TO_PLAN):
        #     self.move_obstacles(self.obstacles)

        vLpossiblearray = (robot.linear_vel - self.MAX_ACC_LINEAR * self.dt, robot.linear_vel, robot.linear_vel + self.MAX_ACC_LINEAR * self.dt)
        vRpossiblearray = (robot.angular_vel - self.MAX_ACC_ANGULAR * self.dt, robot.angular_vel, robot.angular_vel + self.MAX_ACC_ANGULAR * self.dt)

        bestBenefit = -10000000
        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                if (vLpossible <= self.MAX_VEL_LINEAR and vRpossible <= self.MAX_VEL_ANGULAR and vLpossible >= -self.MAX_VEL_LINEAR and vRpossible >= -self.MAX_VEL_ANGULAR):
                    new_x, new_y, _, _ = self.predict_position(robot.x, robot.y, robot.theta, vLpossible, vRpossible, predict_time)
                    distanceToObstacle, _, _ = self.calculate_Closest_Observing_Obstacle(robot.id, (new_x, new_y), dnm_obsatcles, static_ob_awareness) 
                    if (distanceToObstacle < self.ROBOT_SAFE_RADIUS):
                        dis_to_safe_radius = (self.ROBOT_SAFE_RADIUS - distanceToObstacle)
                    else:
                        dis_to_safe_radius = 0.0

                    obstacleCost = OBSTACLEWEIGHT * dis_to_safe_radius

                    # if robot.flag_stuck:
                    robot_current_position = self.real_to_grid(robot.x, robot.y)
                    goal_position = self.real_to_grid(goal[0], goal[1])
                    robot_new_position = self.real_to_grid(new_x, new_y)

                    previousTargetDistance_grid = sim.a_star(robot_current_position, goal_position)
                    newTargetDistance_grid = sim.a_star(robot_new_position, goal_position)

                    distanceForward = previousTargetDistance_grid - newTargetDistance_grid
                    distanceBenefit = FORWARDWEIGHT * distanceForward
                        
                    # previousTargetDistance = math.sqrt((robot.x - goal[0])**2 + (robot.y - goal[1])**2)
                    # newTargetDistance = math.sqrt((new_x - goal[0])**2 + (new_y - goal[1])**2)
                    # distanceForward = previousTargetDistance - newTargetDistance
                    # distanceBenefit = self.FORWARDWEIGHT * distanceForward

                    benefit = distanceBenefit - obstacleCost
                    if (benefit > bestBenefit):
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        bestBenefit = benefit

        # self.obstacles = obstacles_copy 
        return vLchosen, vRchosen

    def move_obstacles(self, obstacles):
        for (i, obstacle) in enumerate(obstacles):
            # Store original position and velocities
            old_x, old_y = obstacle[0], obstacle[1]
            old_vx, old_vy = obstacle[2], obstacle[3]
            
            # Update position based on velocities
            obstacles[i][0] += obstacles[i][2] * self.dt  # x += vx * dt
            obstacles[i][1] += obstacles[i][3] * self.dt  # y += vy * dt
            
            # Bounce off map boundaries (in meters)
            if obstacles[i][0] < self.PLAYFIELDCORNERS[0]:
                obstacles[i][0] = self.PLAYFIELDCORNERS[0]
                obstacles[i][2] = -obstacles[i][2]
            if obstacles[i][0] > self.PLAYFIELDCORNERS[2]:
                obstacles[i][0] = self.PLAYFIELDCORNERS[2]
                obstacles[i][2] = -obstacles[i][2]
            if obstacles[i][1] < self.PLAYFIELDCORNERS[1]:
                obstacles[i][1] = self.PLAYFIELDCORNERS[1]
                obstacles[i][3] = -obstacles[i][3]
            if obstacles[i][1] > self.PLAYFIELDCORNERS[3]:
                obstacles[i][1] = self.PLAYFIELDCORNERS[3]
                obstacles[i][3] = -obstacles[i][3]
            
            # Check for collision with static obstacles
            new_x, new_y = obstacles[i][0], obstacles[i][1]
            # Convert new position to grid coordinates
            center_row, center_col = self.real_to_grid(new_x, new_y)
            # Define a grid region to check (covering OBSTACLE_RADIUS = 10 meters)
            radius_meters = self.OBSTACLE_RADIUS  # 10 meters
            grid_span = int(radius_meters) + 1  # Check ~11 cells in each direction
            collision_detected = False
            closest_static_x, closest_static_y = new_x, new_y  # For push-back 
            
            for dr in range(-grid_span, grid_span + 1): 
                for dc in range(-grid_span, grid_span + 1):
                    row, col = center_row + dr, center_col + dc
                    # Check if grid cell is a wall using precomputed wall set
                    if 0 <= row < self.sim.map.height and 0 <= col < self.sim.map.width and self.sim.map.grid[row][col] != '.':
                        # Convert grid cell to real-world coordinates
                        static_x, static_y = self.grid_to_real((row, col))
                        # Compute distance between dynamic obstacle center and static obstacle
                        dist = math.sqrt((new_x - static_x) ** 2 + (new_y - static_y) ** 2)
                        if dist < self.OBSTACLE_RADIUS:
                            # Collision detected
                            collision_detected = True
                            # Update closest static obstacle for push-back
                            if dist < math.sqrt((new_x - closest_static_x) ** 2 + (new_y - closest_static_y) ** 2):
                                closest_static_x, closest_static_y = static_x, static_y
            
            if collision_detected:
                # Negate both velocities for a strong bounce
                obstacles[i][2] = -old_vx   # Damp velocity by 20%
                obstacles[i][3] = -old_vy
                # Push back to avoid sticking
                dx = new_x - closest_static_x
                dy = new_y - closest_static_y
                dist = math.sqrt(dx ** 2 + dy ** 2)
                if dist > 0:  # Avoid division by zero
                    # Move obstacle to just outside the collision radius
                    push_factor = (self.OBSTACLE_RADIUS + 0.1) / dist  # Small buffer
                    obstacles[i][0] = closest_static_x + dx * push_factor
                    obstacles[i][1] = closest_static_y + dy * push_factor
                else:
                    # Rare case: obstacle exactly at static obstacle center
                    obstacles[i][0] = old_x
                    obstacles[i][1] = old_y

    def draw_obstacles(self):
        for obstacle in self.obstacles:
            x, y = obstacle[0], obstacle[1]
            pygame.draw.circle(self.screen, self.lightblue, 
                                (int(self.u0 + self.k * x), 
                                int(self.v0 - self.k * y)), 
                                self.OBSTACLE_DRAW_RADIUS, 0)

    def draw_static_obstacles(self, fov_bounds):
        x_min, x_max, y_min, y_max = fov_bounds
        # Convert FOV bounds to grid coordinates
        row_min, col_min = self.real_to_grid(x_min, y_max)  # Top-left
        row_max, col_max = self.real_to_grid(x_max, y_min)  # Bottom-right
        # Ensure bounds are within map
        row_min = max(0, min(row_min, self.sim.map.height - 1))
        row_max = max(0, min(row_max, self.sim.map.height - 1))
        col_min = max(0, min(col_min, self.sim.map.width - 1))
        col_max = max(0, min(col_max, self.sim.map.width - 1))

        # for row in range(row_min, row_max + 1):
        #     for col in range(col_min, col_max + 1):
        for row in range(self.sim.map.height):
            for col in range(self.sim.map.width):
                if self.sim.map.grid[row][col] != '.':
                    x, y = self.grid_to_real((row, col))
                    pygame.draw.circle(self.screen, self.darkgrey, 
                                     (int(self.u0 + self.k * x), int(self.v0 - self.k * y)), 
                                     self.STATIC_OBSTACLE_RADIUS, 0)
            
    def draw_goal(self): #---> Lấy từng position của đối tượng robot để vẽ
        color = self.red
        for goal in self.goals:
            pygame.draw.circle(self.screen, color, 
                              (int(self.u0 + self.k * goal.position[0]), int(self.v0 - self.k * goal.position[1])), 
                              self.GOAL_RADIUS, 2)
            
    def draw_station(self):
        pygame.draw.circle(self.screen, self.green, 
                              (int(self.u0 + self.k * self.charging_station.position[0]), int(self.v0 - self.k * self.charging_station.position[1])), 
                              self.CHARGING_STATION_RADIUS, 0)

    def draw_robot(self):
        color = self.white
        radius = self.ROBOT_RADIUS #* self.k  # Scale robot radius to pixels

        for robot in self.robots:
            position = (robot.x, robot.y) 
            pygame.draw.circle(self.screen, color, 
                            (int(self.u0 + self.k * position[0]), int(self.v0 - self.k * position[1])), 
                            int(radius), 3)
            # wlx = robot.x - (self.W/2.0) * math.sin(robot.theta)
            # wly = robot.y + (self.W/2.0) * math.cos(robot.theta)
            # ulx = self.u0 + self.k * wlx
            # vlx = self.v0 - self.k * wly
            # WHEELBLOB = 0.04 * self.k  # Scale wheel blob
            # pygame.draw.circle(self.screen, self.blue, (int(ulx), int(vlx)), int(WHEELBLOB))
            # wrx = robot.x + (self.W/2.0) * math.sin(self.agent.theta)
            # wry = robot.y - (self.W/2.0) * math.cos(self.agent.theta)
            # urx = self.u0 + self.k * wrx
            # vrx = self.v0 - self.k * wry
            # pygame.draw.circle(self.screen, self.blue, (int(urx), int(vrx)), int(WHEELBLOB))

    def draw_history_trajectory(self):
        """ Draw trajectory of moving robot """
        color = self.grey
        for robot in self.robots:
            for pos in robot.history_positions:
                pygame.draw.circle(self.screen, color, 
                                (int(self.u0 + self.k * pos[0]), int(self.v0 - self.k * pos[1])), 
                                2, 0)
    
    # def draw_predicted_trajectory(self, predicted_path_to_draw):
    #     """ Draw predicted trajectory of robot """
    #     for path in predicted_path_to_draw:
    #         if path[0] == 0:  # Straight line
    #             straightpath = path[1]
    #             linestart = (self.u0 + self.k * self.agent.x, self.v0 - self.k * self.agent.y)
    #             lineend = (self.u0 + self.k * (self.agent.x + straightpath * math.cos(self.agent.theta)), 
    #                       self.v0 - self.k * (self.agent.y + straightpath * math.sin(self.agent.theta)))
    #             pygame.draw.line(self.screen, (0, 200, 0), linestart, lineend, 1)
    #         if path[0] == 1:  # Rotation, nothing to draw
    #             pass
    #         if path[0] == 2:  # General case: circular arc
    #             if (path[3] > path[2]):
    #                 startangle = path[2]
    #                 stopangle = path[3]
    #             else:
    #                 startangle = path[3]
    #                 stopangle = path[2]
    #             if (startangle < 0):
    #                 startangle += 2*math.pi
    #                 stopangle += 2*math.pi
    #             if (path[1][1][0] > 0 and path[1][0][0] > 0 and path[1][1][1] > 1):
    #                 pygame.draw.arc(self.screen, (0, 200, 0), path[1], startangle, stopangle, 1)

    def draw_guidance_path(self):
        for robot in self.robots:
            for (x, y) in robot.A_star_path:
                pygame.draw.circle(self.screen, self.yellow, 
                                (int(self.u0 + self.k * x), int(self.v0 - self.k * y)), 
                                self.GUIDANCE_DOT_RADIUS, 0)
                
    def draw_way_points(self):
        for robot in self.robots:
            for (x, y) in robot.way_points:
                pygame.draw.circle(self.screen, self.red, 
                                (int(self.u0 + self.k * x), int(self.v0 - self.k * y)), 
                                self.GUIDANCE_DOT_RADIUS, 0)       
    
    def get_a_star_path(self, robot_position, goal): #Procedure
        start_pos = self.real_to_grid(robot_position[0], robot_position[1])
        goal_pos = self.real_to_grid(goal[0], goal[1])
        grid_a_star_path = self.sim.get_a_star_path(start_pos, goal_pos)
        real_a_star_path = [self.grid_to_real((row, col)) for (row, col) in grid_a_star_path]   
        
        return real_a_star_path   
    
    def calculate_waypoints(self, A_star_path):
        grid_a_star_path = [self.real_to_grid(x, y) for (x, y) in A_star_path]  

        grid_waypoints = grid_a_star_path[::20]
        if grid_a_star_path[-1] != grid_waypoints[-1]:  
            grid_waypoints.append(grid_a_star_path[-1])

        real_waypoints = [self.grid_to_real((row, col)) for (row, col) in grid_waypoints]   

        return real_waypoints

    def draw_frame(self):
        Eventlist = pygame.event.get()
        self.screen.fill(self.black)
        fov_bounds = (0,0,0,0) 
        fov_bounds = (self.charging_station.position[0] - self.view_radius, self.charging_station.position[0]  + self.view_radius,
                    self.charging_station.position[1] - self.view_radius, self.charging_station.position[1]  + self.view_radius)
        # self.draw_static_obstacles(fov_bounds)
        self.draw_goal()
        self.draw_obstacles()
        self.draw_history_trajectory()
        self.draw_guidance_path()
        self.draw_way_points()
        self.draw_robot()
        self.draw_station()
        pygame.display.flip()

    def check_collsion(self):
        for obstacle in self.obstacles:
            for robot in self.robots:
                dist = math.sqrt((obstacle[0] - robot.x)**2 + (obstacle[1] - robot.y)**2)
                if dist < self.ROBOT_RADIUS + self.OBSTACLE_RADIUS:
                    self.collision_times += 1
                    return True
        return False

    def run(self):
        if self.sim_over == True:
            self.reset()
        while self.sim_over == False:
            self.time_to_goal += self.dt
            obstacles_copy = copy.deepcopy(self.obstacles)
            for _ in range(self.STEPS_AHEAD_TO_PLAN):
                self.move_obstacles(self.obstacles)

            for robot in self.robots:
                print(1)
                if robot.flag_finished == False:
                    dist_to_goal = math.sqrt((robot.x - robot.current_goal[0])**2 + (robot.y - robot.current_goal[1])**2)
                    if round(dist_to_goal, 3) < 25:
                        print(2)
                        robot.traced_goal_idx += 1
                        robot.wp_idx = 0
                        # with open("Hybrid.txt", "a") as log_file:
                        #     log_file.write(f"\nGoal {self.goal_nb_reached} reached.\n\n")
                        #     self.goal_nb_reached += 1

                        if robot.traced_goal_idx  == len(robot.road_map):
                            robot.linear_vel, robot.angular_vel = 0.0, 0.0
                            robot.flag_finished = True
                        else:
                            robot.current_goal = robot.road_map[robot.traced_goal_idx] 
                            robot.A_star_path = self.get_a_star_path((robot.x, robot.y), robot.current_goal)
                            robot.way_points = self.calculate_waypoints(robot.A_star_path) #Trả ra real position
                    
                    else:
                        print(3)
                        dist_to_trace = math.sqrt((robot.x - robot.way_points[robot.wp_idx][0])**2 + (robot.y - robot.way_points[robot.wp_idx][1])**2)                                                                                                  
                        if round(dist_to_trace, 3) < 25 and robot.wp_idx < len(robot.way_points) - 1:  
                            print(4)
                            robot.wp_idx += 1
                        
                        vLchosen, vRchosen = self.planning(robot, robot.way_points[robot.wp_idx], self.obstacles, robot.static_ob_awareness, robot.FORWARDWEIGHT, robot.OBSTACLEWEIGHT, self.TAU)
                        robot.linear_vel, robot.angular_vel = vLchosen, vRchosen
                        robot.x, robot.y, robot.theta, _ = self.predict_position(robot.x, robot.y, robot.theta, robot.linear_vel, robot.angular_vel, self.dt) 
                        
                        robot.counting_step += 1

                        robot.history_positions.append((robot.x, robot.y))
                        path_length = (robot.linear_vel + robot.angular_vel) * self.dt / 2
                        robot.distance_travelled += path_length
                        self.total_path_length += path_length
 
            self.obstacles = obstacles_copy 
            self.move_obstacles(self.obstacles) 
             
            # self.history_vl_vec.append(vLchosen)
            # self.history_vr_vec.append(vRchosen)
            print(5)
            self.draw_frame()  #---> draw_robot

            if self.check_collsion() == True: 
                self.sim_over = True
                self.sim_times += 1
                print('#{} \t Failure \t [tg:  None] \t  [total collision times:{}]'.format(self.sim_times, self.collision_times))
                break

            if all(robot.flag_finished for robot in self.robots):
                self.sim_over = True
                self.success = True
                self.sim_times += 1
                print('#{} \t Success \t [tg:  None] \t  [total collision times:{}]'.format(self.sim_times, self.collision_times))
                break

if __name__ == '__main__':
    dir_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(dir_path, "Boston_1_1024.map")
    sim = Simulation(file_path, robot_count=5, goal_count=10)
    
    start_time = time.time()
    assignments = sim.run()
    end_time = time.time()        
    elapsed_time = end_time - start_time
    with open("Hybrid.txt", "a") as log_file:
        log_file.write(f"\n\nPlanning Time: {elapsed_time:.2f} seconds.\n")

    env = Environment(50, sim, assignments)
    while env.sim_times < 100:
        start_time = time.time()
        env.run()
        end_time = time.time()        
        elapsed_time = end_time - start_time
        with open("Hybrid.txt", "a") as log_file:
            log_file.write(f"\nExecution Time: {elapsed_time:.2f} seconds.\n")
            log_file.write(f"Total Path Length: {env.total_path_length:.2f} meters.\n")
            log_file.write(f"Average Path Length {env.robot_count} robots: {(env.total_path_length/env.robot_count):.2f} meters/robot.\n")
            if env.success:
                log_file.write(f"Result: Success.\n")
            else:
                log_file.write(f"Result: Failure.\n")
            log_file.write(f"Restart.\n")

    tg_vec_sorted = sorted(env.tg_vec)
    tg_75th = tg_vec_sorted[int(0.75*len(tg_vec_sorted))-1]
    tg_90th = tg_vec_sorted[int(0.90*len(tg_vec_sorted))-1]
    res_str1 = '[Collision Rate: {}/{}={:.2f}%] \n'.format(env.collision_times, env.sim_times, env.collision_times/env.sim_times*100)
    # res_str2 = '[Average time to goal: {:.2f} secs] \t [tg_75th: {:.2f} secs] \t [tg_90th: {:.2f} secs] \n'.format(env.avg_tg, tg_75th, tg_90th)
    # to_txt = res_str1 + res_str2 + res_str3
    # print("\n" + "* "*30 + "\n")
    # print(to_txt)
    # print("\n" + "* "*30 + "\n")
    # with open('static_goal_new_dwa_result{}.txt'.format(env.init_obstacle_num), 'w') as f:
    #     f.write(to_txt)
    tg_file = pd.DataFrame(data=env.tg_vec, columns=["tg"])
    tg_file.to_csv("static_goal_new_dwa_tg{}.csv".format(env.init_obstacle_num))
    vel_vec = [[env.vl_vec[idx], env.vr_vec[idx]] for idx in range(len(env.vl_vec))]
    vel_file = pd.DataFrame(data=vel_vec, columns=["vl", "vr"])
    vel_file.to_csv("static_goal_new_dwa_vel{}.csv".format(env.init_obstacle_num))
    plt.figure()
    plt.plot(list(range(len(env.tg_vec))), env.tg_vec, 'bo', list(range(len(env.tg_vec))), env.tg_vec, 'k')
    plt.title("time to goal")
    plt.xlabel("simulation times")
    plt.ylabel("tg(sec)")
    plt.figure()
    plt.subplot(211)
    # plt.plot(list(range(len(env.vl_vec))), env.vl_vec, 'r', label="linear velocity")
    plt.xlabel("simulation steps")
    plt.ylabel("vl(m/s)")
    plt.legend(loc="upper right")
    plt.subplot(212)
    # plt.plot(list(range(len(env.vr_vec))), env.vr_vec, 'g', label="angular velocity")
    plt.xlabel("simulation steps")
    plt.ylabel("vr(rad/s)")
    plt.legend(loc="upper right")
    plt.show()
