# agent.py
from collections import deque
import heapq
import math


class GreedyGridAgent:
    """A simple agent that tries to move around systematically to clear the grid."""

    def __init__(self):
        self.actions_pool = ['Up', 'Down', 'Left', 'Right']

    def sense_and_act(self, percept: dict) -> str:
        # If standing directly on food, or just wander / move towards coordinates
        pos = percept['agent_pos']
        # Simple heuristic or fallback random sweep
        return random.choice(self.actions_pool)

class SearchAgent:
    def __init__(self, start_pos=(0,0)):
        self.estimated_pos = tuple(start_pos)
        self.plan = []
        self.active_algo = 'AStar'

    def manhattan_distance(self, pos, goal):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x1 - x2) + abs(y1 - y2)

    def euclidean_distance(self, pos, goal):
        x1, y1 = pos
        x2, y2 = goal

        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def _get_closest_food(self, current_pos, food_list):
        """Finds the food pellet with the shortest Manhattan distance."""
        return min(
            food_list,
            key=lambda f: abs(f[0] - current_pos[0]) + abs(f[1] - current_pos[1])
        )

    def sense_and_act(self, percept: dict) -> str:
        # Update current position directly from percept if provided, else fallback to tracking
        current_pos = tuple(percept.get('agent_pos', self.estimated_pos))

        # Check if current plan is empty
        if not self.plan:
            all_food = percept['all_food']
            if not all_food:
                return 'Stay'  # No food left

            # Find closest food target
            target_food = self._get_closest_food(current_pos, all_food)

            # Generate path plan using the active search algorithm
            grid_size = percept['grid_size']
            walls = set(percept['walls'])

            if self.active_algo == 'BFS':
                self.plan = self.bfs_search(current_pos, target_food, grid_size, walls)
            elif self.active_algo == 'DFS':
                self.plan = self.dfs_search(current_pos, target_food, grid_size, walls)
            elif self.active_algo == 'UCS':
                self.plan = self.ucs_search(current_pos, target_food, grid_size, walls)
            elif self.active_algo == 'AStar':
                self.plan = self.astar_search(
                    current_pos,
                    target_food,
                    walls,
                    grid_size,
                    heuristic_type='manhattan'
                )

        # Turn toward the next planned direction, then move once aligned.
        # Keep the step in the plan until the move has actually been issued.
        if self.plan:
            desired_direction = self.plan[0]
            action = self.direction_to_action(desired_direction, percept['facing'])
            if action == 'move_forward':
                self.plan.pop(0)
            return action

        # No route is available to the selected food pellet.
        return 'Stay'

    def direction_to_action(self, desired_direction, current_facing):
        directions = ['Up', 'Right', 'Down', 'Left']

        current_index = directions.index(current_facing)
        desired_index = directions.index(desired_direction)

        difference = (desired_index - current_index) % 4

        if difference == 0:
            return 'move_forward'
        elif difference == 1:
            return 'turn_right'
        elif difference == 3:
            return 'turn_left'
        else:
            # Opposite direction
            return 'turn_right'

    def get_neighbors(self, state, grid_size, walls):
        x, y = state
        width, height = grid_size

        neighbors = []

        moves = [
            ('Up', (0, 1)),
            ('Down', (0, -1)),
            ('Left', (-1, 0)),
        ('Right', (1, 0))
        ]

        for action, (dx, dy) in moves:
            new_state = (x + dx, y + dy)

            nx, ny = new_state

            if (
                0 <= nx < width
                and 0 <= ny < height
                and new_state not in walls
            ):
                neighbors.append((new_state, action))

        return neighbors

    def bfs_search(self, start, goal, grid_size, walls):

        queue = deque()
        queue.append((start, []))

        reached = {start}

        while queue:

            current, path = queue.popleft()

            if current == goal:
                return path

            for next_state, action in self.get_neighbors(current, grid_size, walls):

                if next_state not in reached:
                    reached.add(next_state)

                    queue.append((next_state, path + [action]))
        return []
    

    def dfs_search(self, start, goal, grid_size, walls):

        stack = [(start, [])]
        reached = {start}

        while stack :

            current, path = stack.pop()

            if current == goal:
                return path

            for next_state, action in self.get_neighbors(current, grid_size, walls):

                if next_state not in reached:
                    reached.add(next_state)
                    stack.append((next_state, path + [action]))

        return []

    def ucs_search(self, start, goal, grid_size, walls):

        frontier = []

        heapq.heappush(
            frontier,
            (0, start, [])
        )

        reached = {start: 0}

        while frontier:

            cost, current, path = heapq.heappop(frontier)

            if current == goal:
                return path

            for next_state, action in self.get_neighbors(
                current, grid_size, walls
            ):

                new_cost = cost + 1

                if (
                    next_state not in reached
                    or new_cost < reached[next_state]
                ):

                    reached[next_state] = new_cost

                    heapq.heappush(
                        frontier,
                        (
                            new_cost,
                            next_state,
                            path + [action]
                        )
                    )

        return []

    def astar_search(
        self,
        start_pos,
        goal_pos,
        walls,
        grid_size,
        heuristic_type='manhattan'
    ):
        frontier = []

        # Calculate heuristic for the starting position
        if heuristic_type == 'euclidean':
            h_cost = self.euclidean_distance(start_pos, goal_pos)
        else:
            h_cost = self.manhattan_distance(start_pos, goal_pos)

        # A* starts with g(n) = 0
        g_cost = 0

        # f(n) = g(n) + h(n)
        f_cost = g_cost + h_cost

        # Tuple format:
        # (f_cost, g_cost, current_pos, path_taken)
        heapq.heappush(
            frontier,
            (f_cost, g_cost, start_pos, [])
        )

        reached_states = set()

        while frontier: 

            f_cost, g_cost, current_pos, path_taken = heapq.heappop(frontier)

            #Goal reached
            if current_pos == goal_pos:
                return path_taken

            # Skip states that have already been explored
            if current_pos in reached_states:
                continue

            reached_states.add(current_pos)

            # Explore neighbouring cells
            for next_state, action in self.get_neighbors(
                current_pos,
                grid_size,
                walls
            ):

                if next_state in reached_states:
                    continue

                #cost to reach the neibhour
                new_g_cost = g_cost + 1

                #Calculate heuristic
                if heuristic_type == 'euclidean':
                    new_h_cost = self.euclidean_distance(next_state, goal_pos)
                else:
                    new_h_cost = self.manhattan_distance(next_state, goal_pos)

                # A* evaluation function
                new_f_cost = new_g_cost + new_h_cost

                # Add action to the path
                new_path = path_taken + [action]

                heapq.heappush(
                    frontier,
                    (
                        new_f_cost,
                        new_g_cost,
                        next_state,
                        new_path
                    )
                )

        # No path found
        return []
        
       


# Testing the SearchAgent with A* search
#if __name__ == "__main__":
#    agent = SearchAgent()
#
#    start = (0, 0)
#    goal = (4, 4)

#    walls = {
#        (1, 1),
#        (1, 2),
#        (2, 2),
#        (3, 2)
#    }

#    grid_size = (5, 5)

 #   print("Start:", start)
 #   print("Goal:", goal)
 #   print("Walls:", walls)

 #   print()
 #   print("Manhattan heuristic:")
 #   path = agent.astar_search(
 #       start,
 #       goal,
 #       walls,
 #       grid_size,
 #       heuristic_type='manhattan'
 #   )

 #   print("Path:", path)
 #   print("Path length:", len(path))

 #   print()
 #   print("Euclidean heuristic:")
 #   path = agent.astar_search(
 #       start,
 #       goal,
 #       walls,
 #       grid_size,
 #       heuristic_type='euclidean'
 #   )


#    print("Path:", path)
#    print("Path length:", len(path))




# if __name__ == "__main__":
#     agent = SearchAgent()
# 
#     print("Manhattan distance:", agent.manhattan_distance((0, 0), (3, 4)))
#     print("Euclidean distance:", agent.euclidean_distance((0, 0), (3, 4)))
    
    
if __name__ == "__main__":
    agent = SearchAgent(start_pos=(0, 0))

    percept = {
        'agent_pos': (0, 0),
        'facing': 'Right',
        'all_food': [(4, 4)],
        'remaining_food': 1,
        'grid_size': (5, 5),
        'walls': {
            (1, 1),
            (1, 2),
            (2, 2),
            (3, 2)
        }
    }

    print("Algorithm:", agent.active_algo)

    action = agent.sense_and_act(percept)

    print("First action:", action)
    print("Remaining plan:", agent.plan)
