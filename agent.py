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
        self.activate_algo = 'BFS'

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