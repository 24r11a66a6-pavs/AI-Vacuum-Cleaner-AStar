"""
astar.py - A* Search Algorithm Implementation for SmartSweep AI
Implements pure A* pathfinding from scratch using Python's heapq.
Supports 4-directional and 8-directional (with diagonal corner-cutting prevention) movement.
"""

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Dict, Generator, List, Optional, Set, Tuple

# Coordinate type alias: (row, col)
Coord = Tuple[int, int]


@dataclass
class PathResult:
    """Encapsulates the outcome and statistics of an A* pathfinding run."""
    path: List[Coord] = field(default_factory=list)
    cost: float = 0.0
    nodes_explored: int = 0
    execution_time_ms: float = 0.0
    explored_order: List[Coord] = field(default_factory=list)
    frontier_nodes: Set[Coord] = field(default_factory=set)
    success: bool = False
    message: str = ""


def manhattan_distance(p1: Coord, p2: Coord) -> float:
    """
    Manhattan distance heuristic for 4-directional grid movement.
    Admissible and consistent when orthogonal step cost is 1.
    h(n) = |r1 - r2| + |c1 - c2|
    """
    return float(abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]))


def octile_distance(p1: Coord, p2: Coord) -> float:
    """
    Octile distance heuristic for 8-directional grid movement.
    Admissible and consistent when orthogonal cost is 1 and diagonal cost is sqrt(2).
    h(n) = max(dr, dc) + (sqrt(2) - 1) * min(dr, dc)
    """
    dr = abs(p1[0] - p2[0])
    dc = abs(p1[1] - p2[1])
    min_d = min(dr, dc)
    max_d = max(dr, dc)
    return float(max_d + (math.sqrt(2.0) - 1.0) * min_d)


def get_heuristic_function(allow_diagonal: bool):
    """Returns the appropriate admissible heuristic function."""
    return octile_distance if allow_diagonal else manhattan_distance


def get_neighbors(
    node: Coord,
    rows: int,
    cols: int,
    obstacles: Set[Coord],
    allow_diagonal: bool = False,
) -> List[Tuple[Coord, float]]:
    """
    Generates valid reachable neighbors and their transition costs.
    Prevents diagonal corner-cutting between obstacles.
    """
    r, c = node
    neighbors: List[Tuple[Coord, float]] = []

    # 4 Orthogonal directions: (dr, dc, cost)
    orthogonal = [
        (-1, 0, 1.0),  # Up
        (1, 0, 1.0),   # Down
        (0, -1, 1.0),  # Left
        (0, 1, 1.0),   # Right
    ]

    for dr, dc, cost in orthogonal:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            if (nr, nc) not in obstacles:
                neighbors.append(((nr, nc), cost))

    # 4 Diagonal directions with corner-cutting prevention
    if allow_diagonal:
        diag_cost = math.sqrt(2.0)
        diagonals = [
            (-1, -1),  # Up-Left
            (-1, 1),   # Up-Right
            (1, -1),   # Down-Left
            (1, 1),    # Down-Right
        ]
        for dr, dc in diagonals:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if (nr, nc) not in obstacles:
                    # Corner-cutting check:
                    # Robot cannot squeeze diagonally if either adjacent orthogonal cell is blocked
                    adj1 = (r + dr, c)
                    adj2 = (r, c + dc)
                    if adj1 not in obstacles and adj2 not in obstacles:
                        neighbors.append(((nr, nc), diag_cost))

    return neighbors


def reconstruct_path(came_from: Dict[Coord, Coord], current: Coord) -> List[Coord]:
    """Reconstructs the optimal path from start to goal by backtracking through came_from."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def a_star_search(
    rows: int,
    cols: int,
    start: Coord,
    goal: Coord,
    obstacles: Set[Coord],
    allow_diagonal: bool = False,
) -> PathResult:
    """
    Executes the A* pathfinding algorithm from start to goal.

    Evaluation function: f(n) = g(n) + h(n)
    - g(n): exact cost from start to node n
    - h(n): estimated admissible cost from node n to goal
    - f(n): total estimated path cost through n

    Returns a PathResult containing the path, cost, explored nodes, and metrics.
    """
    start_time = time.perf_counter()

    # Boundary and validation checks
    if not (0 <= start[0] < rows and 0 <= start[1] < cols):
        return PathResult(
            success=False,
            message=f"Start coordinates {start} are out of grid bounds ({rows}x{cols})."
        )
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return PathResult(
            success=False,
            message=f"Goal coordinates {goal} are out of grid bounds ({rows}x{cols})."
        )
    if start in obstacles:
        return PathResult(
            success=False,
            message=f"Start position {start} is inside an obstacle."
        )
    if goal in obstacles:
        return PathResult(
            success=False,
            message=f"Goal position {goal} is inside an obstacle."
        )

    # Trivial case: start equals goal
    if start == goal:
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        return PathResult(
            path=[start],
            cost=0.0,
            nodes_explored=1,
            execution_time_ms=elapsed_ms,
            explored_order=[start],
            frontier_nodes=set(),
            success=True,
            message="Start and goal are identical. Path cost is 0."
        )

    heuristic = get_heuristic_function(allow_diagonal)

    # Priority queue: entries are tuples of (f_score, h_score, tie_breaker, node)
    # Using h_score as secondary tie-breaker favors nodes closer to the goal
    open_heap: List[Tuple[float, float, int, Coord]] = []
    tie_counter = 0

    g_score: Dict[Coord, float] = {start: 0.0}
    f_score: Dict[Coord, float] = {start: heuristic(start, goal)}

    heapq.heappush(open_heap, (f_score[start], heuristic(start, goal), tie_counter, start))
    open_set_tracked: Set[Coord] = {start}
    closed_set: Set[Coord] = set()

    came_from: Dict[Coord, Coord] = {}
    explored_order: List[Coord] = []

    while open_heap:
        current_f, current_h, _, current = heapq.heappop(open_heap)

        # Ignore stale entries that were updated with a lower cost
        if current in closed_set:
            continue

        open_set_tracked.discard(current)
        closed_set.add(current)
        explored_order.append(current)

        # Check goal reached
        if current == goal:
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            final_path = reconstruct_path(came_from, current)
            return PathResult(
                path=final_path,
                cost=g_score[goal],
                nodes_explored=len(closed_set),
                execution_time_ms=elapsed_ms,
                explored_order=explored_order,
                frontier_nodes=open_set_tracked,
                success=True,
                message=f"Optimal path found with {len(final_path)} steps (Cost: {g_score[goal]:.2f})."
            )

        # Expand neighbors
        for neighbor, move_cost in get_neighbors(current, rows, cols, obstacles, allow_diagonal):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h_val = heuristic(neighbor, goal)
                f_val = tentative_g + h_val
                f_score[neighbor] = f_val

                tie_counter += 1
                heapq.heappush(open_heap, (f_val, h_val, tie_counter, neighbor))
                open_set_tracked.add(neighbor)

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    return PathResult(
        path=[],
        cost=float('inf'),
        nodes_explored=len(closed_set),
        execution_time_ms=elapsed_ms,
        explored_order=explored_order,
        frontier_nodes=open_set_tracked,
        success=False,
        message="No path found! Destination is completely unreachable or blocked by obstacles."
    )


def a_star_step_generator(
    rows: int,
    cols: int,
    start: Coord,
    goal: Coord,
    obstacles: Set[Coord],
    allow_diagonal: bool = False,
) -> Generator[Tuple[Optional[Coord], Set[Coord], Set[Coord], bool, Optional[PathResult]], None, None]:
    """
    Generator version of A* for interactive, step-by-step exploration visualization.
    Yields (current_node, open_set, closed_set, is_finished, path_result).
    """
    start_time = time.perf_counter()

    # Validation
    if not (0 <= start[0] < rows and 0 <= start[1] < cols) or not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        res = PathResult(success=False, message="Start or goal is outside the room grid.")
        yield (None, set(), set(), True, res)
        return

    if start in obstacles or goal in obstacles:
        res = PathResult(success=False, message="Start or goal is blocked by an obstacle.")
        yield (None, set(), set(), True, res)
        return

    if start == goal:
        res = PathResult(
            path=[start], cost=0.0, nodes_explored=1,
            execution_time_ms=0.0, explored_order=[start],
            success=True, message="Start is already at goal."
        )
        yield (start, set(), {start}, True, res)
        return

    heuristic = get_heuristic_function(allow_diagonal)
    open_heap: List[Tuple[float, float, int, Coord]] = []
    tie_counter = 0

    g_score: Dict[Coord, float] = {start: 0.0}
    f_score: Dict[Coord, float] = {start: heuristic(start, goal)}

    heapq.heappush(open_heap, (f_score[start], heuristic(start, goal), tie_counter, start))
    open_set_tracked: Set[Coord] = {start}
    closed_set: Set[Coord] = set()
    came_from: Dict[Coord, Coord] = {}
    explored_order: List[Coord] = []

    while open_heap:
        current_f, current_h, _, current = heapq.heappop(open_heap)

        if current in closed_set:
            continue

        open_set_tracked.discard(current)
        closed_set.add(current)
        explored_order.append(current)

        if current == goal:
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            final_path = reconstruct_path(came_from, current)
            res = PathResult(
                path=final_path,
                cost=g_score[goal],
                nodes_explored=len(closed_set),
                execution_time_ms=elapsed_ms,
                explored_order=explored_order,
                frontier_nodes=set(open_set_tracked),
                success=True,
                message=f"Optimal path found ({len(final_path)} steps, Cost: {g_score[goal]:.2f})."
            )
            yield (current, set(open_set_tracked), set(closed_set), True, res)
            return

        # Expand neighbors
        for neighbor, move_cost in get_neighbors(current, rows, cols, obstacles, allow_diagonal):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + move_cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h_val = heuristic(neighbor, goal)
                f_val = tentative_g + h_val
                f_score[neighbor] = f_val

                tie_counter += 1
                heapq.heappush(open_heap, (f_val, h_val, tie_counter, neighbor))
                open_set_tracked.add(neighbor)

        # Yield step state
        yield (current, set(open_set_tracked), set(closed_set), False, None)

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    res = PathResult(
        path=[], cost=float('inf'),
        nodes_explored=len(closed_set),
        execution_time_ms=elapsed_ms,
        explored_order=explored_order,
        frontier_nodes=set(open_set_tracked),
        success=False,
        message="No path found! Destination is completely unreachable."
    )
    yield (None, set(open_set_tracked), set(closed_set), True, res)


def a_star(
    grid_or_rows,
    start: Coord,
    goal: Coord,
    obstacles: Optional[Set[Coord]] = None,
    allow_diagonal: bool = False,
    return_sets: bool = False,
    **kwargs
) -> Tuple[List[Coord], Dict[str, Any]]:
    """
    Convenience and interoperability wrapper for a_star_search.
    Supports:
      a_star(grid_matrix, start, goal, allow_diagonal=..., return_sets=...)
    Returns (path, stats_dict).
    """
    from typing import Any
    if isinstance(grid_or_rows, list):
        grid = grid_or_rows
        rows = len(grid)
        cols = len(grid[0]) if rows > 0 else 0
        obs_set = set()
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] != 0:
                    obs_set.add((r, c))
    else:
        rows = int(grid_or_rows)
        cols = int(kwargs.get("cols", rows))
        obs_set = obstacles if obstacles is not None else set()

    res = a_star_search(
        rows=rows,
        cols=cols,
        start=start,
        goal=goal,
        obstacles=obs_set,
        allow_diagonal=allow_diagonal
    )

    stats = {
        "success": res.success,
        "path_cost": res.cost,
        "nodes_explored": res.nodes_explored,
        "execution_time_ms": res.execution_time_ms,
        "message": res.message,
    }
    if return_sets:
        stats["explored_nodes"] = set(res.explored_order)
        stats["frontier_nodes"] = set(res.frontier_nodes)

    return res.path, stats

