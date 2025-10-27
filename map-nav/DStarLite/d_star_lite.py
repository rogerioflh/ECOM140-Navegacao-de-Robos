import math
import matplotlib.pyplot as plt
import random
import numpy as np
from typing import List, Tuple, Optional, Set, Dict, Any
import json 
import time 


SHOW_ANIMATION: bool = True
ANIMATION_PAUSE_TIME: float = 0.08
STATUS_MESSAGE_PAUSE_TIME: float = 1.5
P_CREATE_RANDOM_OBSTACLE: float = 0.0
MAP_FILENAME: str = "simulation_map.pgm" 
LOG_FILENAME: str = "simulation_log.json" 

class Node:
    """Represents a node in the grid graph."""
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost

def add_coordinates(node1: Node, node2: Node) -> Node:
    """Adds the coordinates and costs of two nodes."""
    return Node(node1.x + node2.x, node1.y + node2.y, node1.cost + node2.cost)

def compare_coordinates(node1: Node, node2: Node) -> bool:
    """Checks if two nodes have the same coordinates."""
    return node1.x == node2.x and node1.y == node2.y



class DStarLite:
    """Implements the D* Lite path planning algorithm with logging."""

    motions = [
        Node(1, 0, 1), Node(0, 1, 1), Node(-1, 0, 1), Node(0, -1, 1),
        Node(1, 1, math.sqrt(2)), Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)), Node(-1, -1, math.sqrt(2))
    ]

    def __init__(self,
                 static_obstacle_x: List[int],
                 static_obstacle_y: List[int],
                 robot_radius: int = 1):
        """Initializes the D* Lite planner."""
        if not static_obstacle_x or not static_obstacle_y:
            raise ValueError("Obstacle lists cannot be empty.")
        self.robot_radius: int = robot_radius

        
        self.x_min_world: int = int(min(static_obstacle_x))
        self.y_min_world: int = int(min(static_obstacle_y))
        
        self.x_max_world: int = int(max(static_obstacle_x))
        self.y_max_world: int = int(max(static_obstacle_y))
        
        self.grid_width: int = self.x_max_world - self.x_min_world + 1
        self.grid_height: int = self.y_max_world - self.y_min_world + 1

        original_obstacles_grid: List[Node] = [
            Node(x - self.x_min_world, y - self.y_min_world)
            for x, y in zip(static_obstacle_x, static_obstacle_y)
        ]

        
        print("Inflating static obstacles...")
        self.inflated_static_obstacles_xy: Set[Tuple[int, int]] = self._inflate_obstacles(
            {(obs.x, obs.y) for obs in original_obstacles_grid}
        )
        print(f"Original static obstacles: {len(original_obstacles_grid)}, "
              f"Inflated static cells: {len(self.inflated_static_obstacles_xy)}")

        
        self.start: Node = Node(0, 0)
        self.goal: Node = Node(0, 0)
        self.U: list = []
        self.km: float = 0.0

        self.rhs: np.ndarray = self.create_grid(float("inf"))
        self.g: np.ndarray = self.create_grid(float("inf"))

        self.inflated_dynamic_obstacles_xy: Set[Tuple[int, int]] = set()
        self.original_dynamic_obstacles_xy: Set[Tuple[int, int]] = set()

        
        self.status_text_handle: Optional[plt.Text] = None
        if SHOW_ANIMATION:
            self.dynamic_obstacles_plot_x: List[int] = []
            self.dynamic_obstacles_plot_y: List[int] = []

        
        self.log_data: List[Dict[str, Any]] = []

        self.initialized: bool = False

    def _inflate_obstacles(self, original_coords: Set[Tuple[int, int]]) -> Set[Tuple[int, int]]:
        """Inflates a set of obstacle coordinates by the robot radius."""
        inflated_coords = set()
        if self.robot_radius == 0:
            return original_coords
        for obs_x, obs_y in original_coords:
            for dx in range(-self.robot_radius, self.robot_radius + 1):
                for dy in range(-self.robot_radius, self.robot_radius + 1):
                    inflated_coords.add((obs_x + dx, obs_y + dy))
        return inflated_coords

    def create_grid(self, val: float) -> np.ndarray:
        """Creates a grid initialized with a specific value."""
        
        return np.full((self.grid_width, self.grid_height), val)

    def is_obstacle(self, node: Node) -> bool:
        """Checks if a given node corresponds to an inflated obstacle cell."""
        coords = (node.x, node.y)
        
        if not (0 <= node.x < self.grid_width and 0 <= node.y < self.grid_height):
             return True 
        return coords in self.inflated_static_obstacles_xy or \
               coords in self.inflated_dynamic_obstacles_xy

    def c(self, node1: Node, node2: Node) -> float:
        """Calculates the cost of moving from node1 to node2."""
        if self.is_obstacle(node2): return math.inf
        motion_vector = Node(node2.x - node1.x, node2.y - node1.y)
        for motion in self.motions:
            if compare_coordinates(motion, motion_vector): return motion.cost
        return math.inf

    def h(self, s: Node) -> float:
        """Calculates the heuristic cost (Octile distance simplified)."""
        dx = abs(s.x - self.start.x)
        dy = abs(s.y - self.start.y)
        return max(dx, dy)

    def calculate_key(self, s: Node) -> Tuple[float, float]:
        """Calculates the priority key for a node 's'."""
        if not self.is_valid(s): return (float("inf"), float("inf"))
        min_g_rhs = min(self.g[s.x, s.y], self.rhs[s.x, s.y])
        return (min_g_rhs + self.h(s) + self.km, min_g_rhs)

    def is_valid(self, node: Node) -> bool:
        """Checks if a node's coordinates are within the grid boundaries."""
        
        return 0 <= node.x < self.grid_width and 0 <= node.y < self.grid_height

    def get_neighbours(self, u: Node) -> List[Node]:
        """Gets all valid neighbouring nodes of a given node 'u'."""
        neighbours = []
        for motion in self.motions:
            neighbour = add_coordinates(u, motion)
            if self.is_valid(neighbour):
                neighbours.append(neighbour)
        return neighbours

    def pred(self, u: Node) -> List[Node]:
        """Returns the predecessors of node 'u'."""
        return self.get_neighbours(u)

    def succ(self, u: Node) -> List[Node]:
        """Returns the successors of node 'u'."""
        return self.get_neighbours(u)

    def initialize(self, start_world: Node, goal_world: Node) -> None:
        """Initializes or resets the D* Lite algorithm state."""
        self.start.x = start_world.x - self.x_min_world
        self.start.y = start_world.y - self.y_min_world
        self.goal.x = goal_world.x - self.x_min_world
        self.goal.y = goal_world.y - self.y_min_world

        self.U = []
        self.km = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.inflated_dynamic_obstacles_xy = set()
        self.original_dynamic_obstacles_xy = set()
        self.log_data = [] 

        if SHOW_ANIMATION:
            self.dynamic_obstacles_plot_x = []
            self.dynamic_obstacles_plot_y = []
            if self.status_text_handle:
                try: self.status_text_handle.remove()
                except ValueError: pass
                self.status_text_handle = None

        if not self.is_valid(self.goal):
             raise ValueError("Goal position is out of bounds or inside an obstacle.")
        if not self.is_valid(self.start):
             raise ValueError("Start position is out of bounds or inside an obstacle.")
        if self.is_obstacle(self.goal):
             raise ValueError("Goal position is inside an obstacle.")
        if self.is_obstacle(self.start):
             raise ValueError("Start position is inside an obstacle.")

        self.rhs[self.goal.x, self.goal.y] = 0
        self.U.append((self.goal, self.calculate_key(self.goal)))
        self.U.sort(key=lambda x: x[1])

        self.initialized = True
        print('D* Lite Initialized.')
        
        self._log_state(state="INITIALIZING", pose_node=self.start)


    def update_vertex(self, u: Node) -> None:
        """Updates the rhs value of a node and its priority queue status."""
        if not self.is_valid(u): return

        if not compare_coordinates(u, self.goal):
            min_succ_cost = float("inf")
            for sprime in self.succ(u):
                 if self.is_valid(sprime): 
                      min_succ_cost = min(min_succ_cost,
                                          self.c(u, sprime) + self.g[sprime.x, sprime.y])
            self.rhs[u.x, u.y] = min_succ_cost

        
        u_in_queue = False
        for i, (node, _) in enumerate(self.U):
            if compare_coordinates(u, node):
                self.U.pop(i)
                u_in_queue = True
                break

        
        if self.g[u.x, u.y] != self.rhs[u.x, u.y]:
            self.U.append((u, self.calculate_key(u)))
            self.U.sort(key=lambda x: x[1])

    def compare_keys(self, key1: Tuple[float, float], key2: Tuple[float, float]) -> bool:
        """Compares two keys lexicographically. Returns True if key1 < key2."""
        if key1[0] < key2[0]: return True
        if key1[0] == key2[0] and key1[1] < key2[1]: return True
        return False

    def compute_shortest_path(self) -> None:
        """Repeatedly processes nodes from the priority queue."""
        self.U.sort(key=lambda x: x[1])
        start_key = self.calculate_key(self.start) 

        while (self.U and
               (self.compare_keys(self.U[0][1], start_key) or
                self.rhs[self.start.x, self.start.y] != self.g[self.start.x, self.start.y])):

            k_old = self.U[0][1]
            u = self.U.pop(0)[0]

            u_key = self.calculate_key(u) 

            if self.compare_keys(k_old, u_key):
                self.U.append((u, u_key))
            elif self.g[u.x, u.y] > self.rhs[u.x, u.y]:
                self.g[u.x, u.y] = self.rhs[u.x, u.y]
                for s in self.pred(u): self.update_vertex(s)
            else:
                self.g[u.x, u.y] = float("inf")
                for s in self.pred(u) + [u]: self.update_vertex(s)
            
            
            self.U.sort(key=lambda x: x[1])
            start_key = self.calculate_key(self.start) 


    def detect_changes(self) -> List[Node]:
        """Detects new dynamic obstacles, inflates, updates state, handles viz."""
        newly_inflated_obstacle_nodes = []
        if not hasattr(self, 'spoofed_obstacles') or not self.spoofed_obstacles:
            return newly_inflated_obstacle_nodes 

        if self.spoofed_obstacles:
            original_spoofed_nodes = self.spoofed_obstacles.pop(0)
            new_obstacle_plotted_this_step = False

            for spoofed_node in original_spoofed_nodes:
                if compare_coordinates(spoofed_node, self.start) or \
                   compare_coordinates(spoofed_node, self.goal): continue

                for dx in range(-self.robot_radius, self.robot_radius + 1):
                    for dy in range(-self.robot_radius, self.robot_radius + 1):
                        inflated_x = spoofed_node.x + dx
                        inflated_y = spoofed_node.y + dy
                        inflated_coords = (inflated_x, inflated_y)

                        if inflated_coords in self.inflated_static_obstacles_xy: continue

                        if inflated_coords not in self.inflated_dynamic_obstacles_xy:
                            
                            if 0 <= inflated_x < self.grid_width and 0 <= inflated_y < self.grid_height:
                                self.inflated_dynamic_obstacles_xy.add(inflated_coords)
                                newly_inflated_obstacle_nodes.append(Node(inflated_x, inflated_y))

                original_coords = (spoofed_node.x, spoofed_node.y)
                if original_coords not in self.original_dynamic_obstacles_xy:
                    self.original_dynamic_obstacles_xy.add(original_coords)
                    if SHOW_ANIMATION:
                        plot_x = spoofed_node.x + self.x_min_world
                        plot_y = spoofed_node.y + self.y_min_world
                        if hasattr(self, 'dynamic_obstacles_plot_x'): 
                            self.dynamic_obstacles_plot_x.append(plot_x)
                            self.dynamic_obstacles_plot_y.append(plot_y)
                            plt.plot(plot_x, plot_y, "sk", markersize=10)
                            plt.pause(0.05)
                            new_obstacle_plotted_this_step = True

        return newly_inflated_obstacle_nodes


    def compute_current_path(self) -> List[Node]:
        """Extracts the current best path from start to goal."""
        path = []
        current_node = Node(self.start.x, self.start.y)
        count = 0 
        max_count = self.grid_width * self.grid_height

        while not compare_coordinates(current_node, self.goal) and count < max_count:
            path.append(current_node)
            count += 1
            min_cost = float("inf")
            next_node = None
            possible_successors = self.succ(current_node)

            if not possible_successors:
                 print(f"Error: No successors found from node ({current_node.x}, {current_node.y})")
                 return []

            for sprime in possible_successors:
                 if not self.is_valid(sprime): continue 
                 cost = self.c(current_node, sprime) + self.g[sprime.x, sprime.y]
                 if cost < min_cost:
                    min_cost = cost
                    next_node = sprime

            if next_node is None or min_cost == float("inf"):
                 print(f"Error: Could not find valid successor with finite cost from ({current_node.x}, {current_node.y})")
                 return []

            current_node = next_node
        
        if count >= max_count:
             print("Error: Path computation exceeded maximum length.")
             return []

        if compare_coordinates(current_node, self.goal):
             path.append(self.goal)
        else: 
             print("Warning: Path computation finished without reaching goal.")
             return []
             
        return path


    def compare_paths(self, path1: List[Node], path2: List[Node]) -> bool:
        """Compares two paths to see if they are identical."""
        if len(path1) != len(path2): return False
        for node1, node2 in zip(path1, path2):
            if not compare_coordinates(node1, node2): return False
        return True

    def display_path(self, path: List[Node], color: str, alpha: float = 1.0) -> Optional[plt.Line2D]:
        """Plots a given path on the matplotlib figure."""
        if not path or not SHOW_ANIMATION: return None
        px = [(node.x + self.x_min_world) for node in path]
        py = [(node.y + self.y_min_world) for node in path]
        try:
             drawing = plt.plot(px, py, color, alpha=alpha)
             plt.pause(ANIMATION_PAUSE_TIME / 2)
             return drawing[0]
        except Exception as e:
             print(f"Error during plotting: {e}")
             return None


    def _log_state(self, state: str, pose_node: Node, prev_pose_node: Optional[Node] = None) -> None:
        """Logs the current robot state, pose, and velocity command."""
        ts = time.time()
        
        
        pose = {
            "x": pose_node.x + self.x_min_world,
            "y": pose_node.y + self.y_min_world
            
        }

        
        if prev_pose_node and state == "MOVING":
            
            dx_grid = pose_node.x - prev_pose_node.x
            dy_grid = pose_node.y - prev_pose_node.y
            
            linear_x = float(dx_grid)
            linear_y = float(dy_grid)
        else:
            linear_x = 0.0
            linear_y = 0.0

        twist = {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": 0.0 
        }

        log_entry = {
            "timestamp": ts,
            "state": state,
            "pose": pose,
            "twist": twist
        }
        self.log_data.append(log_entry)

    def save_log_data(self, filename: str) -> None:
        """Saves the accumulated log data to a JSON file."""
        try:
            with open(filename, 'w') as f:
                json.dump(self.log_data, f, indent=4)
            print(f"Simulation log saved to '{filename}'")
        except IOError as e:
            print(f"Error saving log file '{filename}': {e}")


    def run_planning(self,
                     start_world: Node,
                     goal_world: Node,
                     dynamic_obstacle_x_schedule: List[List[int]],
                     dynamic_obstacle_y_schedule: List[List[int]],
                     log_filename: str = "simulation_log.json", 
                     robot_marker: Optional[plt.Line2D] = None
                     ) -> Tuple[bool, List[int], List[int]]:
        """Executes the main D* Lite planning and navigation loop with logging."""
        self.spoofed_obstacles = [
            [Node(x - self.x_min_world, y - self.y_min_world)
             for x, y in zip(rowx, rowy) if self.is_valid(Node(x - self.x_min_world, y - self.y_min_world))] 
            for rowx, rowy in zip(dynamic_obstacle_x_schedule, dynamic_obstacle_y_schedule)
        ]

        path_x_history = []
        path_y_history = []

        try:
             self.initialize(start_world, goal_world)
        except ValueError as e:
             print(f"Initialization Error: {e}")
             self._log_state(state="ERROR_INITIALIZATION", pose_node=Node(start_world.x, start_world.y)) 
             self.save_log_data(log_filename)
             return False, [], []


        last_node_for_km = self.start
        self.compute_shortest_path()

        path_x_history.append(self.start.x + self.x_min_world)
        path_y_history.append(self.start.y + self.y_min_world)

        current_path_vis = []
        previous_path_vis = []
        previous_path_image = None
        current_path_image = None

        if SHOW_ANIMATION:
            current_path_vis = self.compute_current_path()
            previous_path_vis = current_path_vis.copy()
            previous_path_image = self.display_path(previous_path_vis, ".c", alpha=0.3)
            current_path_image = self.display_path(current_path_vis, ".c")

        
        step_count = 0
        max_steps = self.grid_width * self.grid_height * 2 

        while not compare_coordinates(self.goal, self.start) and step_count < max_steps:
            step_count += 1
            if self.g[self.start.x, self.start.y] == float("inf"):
                print("No path possible to the goal.")
                self._log_state(state="PATH_IMPOSSIBLE", pose_node=self.start)
                self.save_log_data(log_filename)
                return False, path_x_history, path_y_history

            
            prev_start_node = Node(self.start.x, self.start.y)

            
            next_start_node = min(self.succ(self.start),
                                  key=lambda sprime:
                                  self.c(self.start, sprime) + self.g[sprime.x, sprime.y])
            self.start = next_start_node

            path_x_history.append(self.start.x + self.x_min_world)
            path_y_history.append(self.start.y + self.y_min_world)

            
            self._log_state(state="MOVING", pose_node=self.start, prev_pose_node=prev_start_node)


            if SHOW_ANIMATION:
                if current_path_vis: current_path_vis.pop(0)
                if robot_marker:
                    robot_marker.set_data([self.start.x + self.x_min_world],
                                          [self.start.y + self.y_min_world])
                plt.plot(path_x_history, path_y_history, "-r", alpha=0.4)
                plt.pause(ANIMATION_PAUSE_TIME)

            
            changed_inflated_vertices = self.detect_changes()
            if changed_inflated_vertices:
                print("--- Map change detected! Recalculating path... ---")
                
                self._log_state(state="REPLANNING_DETECTED", pose_node=self.start)

                if SHOW_ANIMATION and self.status_text_handle:
                    try: self.status_text_handle.remove(); self.status_text_handle = None
                    except ValueError: pass
                if SHOW_ANIMATION:
                    self.status_text_handle = plt.text(
                        0.1, 0.9, "ALERTA: Obstáculo detectado! Recalculando rota...",
                        transform=plt.gca().transAxes, fontsize=12, color='red',
                        fontweight='bold', bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.5')
                    )
                    plt.pause(STATUS_MESSAGE_PAUSE_TIME)

                self.km += self.h(last_node_for_km)
                last_node_for_km = self.start

                for u_changed in changed_inflated_vertices:
                    for s_pred in self.pred(u_changed): self.update_vertex(s_pred)
                    self.update_vertex(u_changed)

                self.compute_shortest_path()
                
                self._log_state(state="REPLANNING_COMPLETE", pose_node=self.start)


                if SHOW_ANIMATION:
                    new_path_vis = self.compute_current_path()
                    if not self.compare_paths(current_path_vis, new_path_vis):
                        if current_path_image: current_path_image.remove()
                        if previous_path_image: previous_path_image.remove()
                        previous_path_vis = current_path_vis.copy()
                        current_path_vis = new_path_vis.copy()
                        previous_path_image = self.display_path(previous_path_vis, ".c", alpha=0.3)
                        current_path_image = self.display_path(current_path_vis, ".c")

                        print("--- Path successfully recalculated! ---")
                        if self.status_text_handle:
                            try: self.status_text_handle.remove(); self.status_text_handle = None
                            except ValueError: pass
                        self.status_text_handle = plt.text(
                            0.1, 0.9, "Rota modificada com sucesso!",
                            transform=plt.gca().transAxes, fontsize=12, color='green',
                            fontweight='bold', bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.5')
                        )
                        plt.pause(ANIMATION_PAUSE_TIME)

        
        if step_count >= max_steps:
             print("Simulation stopped: Maximum steps reached.")
             self._log_state(state="ERROR_MAX_STEPS", pose_node=self.start)
             self.save_log_data(log_filename)
             return False, path_x_history, path_y_history

        
        if SHOW_ANIMATION and self.status_text_handle:
            try: self.status_text_handle.remove(); self.status_text_handle = None
            except ValueError: pass

        print("Goal reached!")
        self._log_state(state="GOAL_REACHED", pose_node=self.start)
        self.save_log_data(log_filename) 
        return True, path_x_history, path_y_history




def setup_environment() -> Tuple[List[int], List[int], int, int, int, int]:
    """Defines the static obstacles and start/goal points."""
    start_x, start_y = 5, 5
    goal_x, goal_y = 45, 45
    grid_size = 50 

    static_obstacle_x, static_obstacle_y = [], []
    
    for i in range(grid_size):
        static_obstacle_x.extend([i, 0, i, grid_size - 1])
        static_obstacle_y.extend([0, i, grid_size - 1, i])

    
    for i in range(0, 30): static_obstacle_x.append(15); static_obstacle_y.append(i)
    for i in range(10, 40): static_obstacle_x.append(30); static_obstacle_y.append(i)
    for i in range(0, 30): static_obstacle_x.append(40); static_obstacle_y.append(grid_size - 1 - i)

    unique_obstacles = set(zip(static_obstacle_x, static_obstacle_y))
    static_obstacle_x = [x for x,y in unique_obstacles]
    static_obstacle_y = [y for x,y in unique_obstacles]

    return static_obstacle_x, static_obstacle_y, start_x, start_y, goal_x, goal_y

def setup_dynamic_obstacles() -> Tuple[List[List[int]], List[List[int]]]:
    """Defines the schedule for dynamic obstacles."""
    dynamic_obstacle_x_schedule = [[] for _ in range(15)]
    dynamic_obstacle_y_schedule = [[] for _ in range(15)]
    wall_x = list(range(15, 31)); wall_y = [15] * len(wall_x)
    dynamic_obstacle_x_schedule.append(wall_x)
    dynamic_obstacle_y_schedule.append(wall_y)
    return dynamic_obstacle_x_schedule, dynamic_obstacle_y_schedule


def save_map_as_pgm(filename: str,
                    width: int, height: int,
                    obstacles_xy: Set[Tuple[int, int]],
                    x_min_grid: int = 0, y_min_grid: int = 0) -> None:
    """Saves the static obstacle map as a PGM file.

    Args:
        filename: The name of the file to save (e.g., "map.pgm").
        width: The width of the grid.
        height: The height of the grid.
        obstacles_xy: A set of (x, y) tuples representing obstacle grid coordinates.
        x_min_grid: The minimum x grid coordinate (usually 0).
        y_min_grid: The minimum y grid coordinate (usually 0).
    """
    print(f"Generating PGM map '{filename}'...")
    
    grid_map = np.full((height, width), 255, dtype=np.uint8)

    for obs_x, obs_y in obstacles_xy:
        
        
        pgm_y = height - 1 - obs_y
        if 0 <= obs_x < width and 0 <= pgm_y < height:
            grid_map[pgm_y, obs_x] = 0 

    try:
        with open(filename, 'w') as f:
            f.write("P2\n") 
            f.write(f"{width} {height}\n")
            f.write("255\n") 
            for row in grid_map:
                f.write(" ".join(map(str, row)) + "\n")
        print("PGM map saved successfully.")
    except IOError as e:
        print(f"Error saving PGM map '{filename}': {e}")


def main() -> None:
    """Sets up and runs the D* Lite simulation, saves map and logs."""
    print("Starting D* Lite Simulation...")

    static_ox, static_oy, sx, sy, gx, gy = setup_environment()
    dyn_ox_schedule, dyn_oy_schedule = setup_dynamic_obstacles()

    
    
    
    all_x = static_ox + [sx, gx]
    all_y = static_oy + [sy, gy]
    world_x_min = min(all_x)
    world_y_min = min(all_y)
    world_x_max = max(all_x)
    world_y_max = max(all_y)

    map_width = world_x_max - world_x_min + 1 + 2 
    map_height = world_y_max - world_y_min + 1 + 2 
    
    
    map_obstacles_xy = set(
        (x - world_x_min + 1, y - world_y_min + 1) 
        for x, y in zip(static_ox, static_oy)
    )
    
    save_map_as_pgm(MAP_FILENAME, map_width, map_height, map_obstacles_xy)
    
    
    

    
    robot_marker = None
    if SHOW_ANIMATION:
        plt.figure(figsize=(10, 10))
        plt.plot(static_ox, static_oy, "sk", markersize=10) 
        plt.plot(sx, sy, "og", markersize=10)
        plt.plot(gx, gy, "xb", markersize=10)
        robot_marker, = plt.plot(sx, sy, "or", markersize=20)
        plt.grid(True)
        plt.axis("equal") 
        
        plt.xlim(world_x_min - 2, world_x_max + 2)
        plt.ylim(world_y_min - 2, world_y_max + 2)
        plt.title("D* Lite Path Planning Simulation")
        plt.pause(ANIMATION_PAUSE_TIME)

    
    ROBOT_RADIUS_CELLS = 1
    dstar_lite = DStarLite(static_ox, static_oy, robot_radius=ROBOT_RADIUS_CELLS)

    success, path_x, path_y = dstar_lite.run_planning(
        start_world=Node(x=sx, y=sy),
        goal_world=Node(x=gx, y=gy),
        dynamic_obstacle_x_schedule=dyn_ox_schedule,
        dynamic_obstacle_y_schedule=dyn_oy_schedule,
        log_filename=LOG_FILENAME, 
        robot_marker=robot_marker
    )

    if success: print("Simulation completed successfully.")
    else: print("Simulation failed or goal unreachable.")

    if SHOW_ANIMATION:
        plt.plot(path_x, path_y, "-r", alpha=0.6)
        print("Close the plot window to exit.")
        plt.show()

if __name__ == "__main__":
    main()