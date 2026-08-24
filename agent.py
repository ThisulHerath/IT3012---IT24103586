# agent.py
from collections import deque
import heapq


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
        self.active_algo = 'UCS'

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

        # Execute next step in plan
        if self.plan:
            action = self.plan.pop(0)
            
            # Map directional actions directly to environment actions if needed
            # Assuming environment accepts direct movement commands ('Up', 'Down', 'Left', 'Right')
            return action

        return 'Stay'

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