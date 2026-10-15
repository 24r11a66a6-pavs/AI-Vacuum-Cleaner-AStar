"""
Unit tests for A* Pathfinding Algorithm in SmartSweep AI.
Tests straight paths, obstacles, narrow corridors, blocked targets,
corner-cutting prevention, diagonal costs, heuristics, and optimality.
"""

import math
import unittest
from astar import a_star_search, manhattan_distance, octile_distance, PathResult


class TestAStar(unittest.TestCase):

    def test_straight_unobstructed_route(self):
        """Tests a basic straight line route with Manhattan distance."""
        res = a_star_search(
            rows=10, cols=10,
            start=(0, 0), goal=(0, 5),
            obstacles=set(), allow_diagonal=False
        )
        self.assertTrue(res.success)
        self.assertEqual(res.cost, 5.0)
        self.assertEqual(len(res.path), 6)
        self.assertEqual(res.path[0], (0, 0))
        self.assertEqual(res.path[-1], (0, 5))

    def test_route_around_obstacle(self):
        """Tests detouring around a solid wall obstacle."""
        # Horizontal wall in the middle: (2, 0) to (2, 3)
        obstacles = {(2, 0), (2, 1), (2, 2), (2, 3)}
        res = a_star_search(
            rows=5, cols=5,
            start=(1, 1), goal=(3, 1),
            obstacles=obstacles, allow_diagonal=False
        )
        self.assertTrue(res.success)
        # To get around (2, 1), robot must walk to column 4:
        # (1, 1) -> (1, 2) -> (1, 3) -> (1, 4) -> (2, 4) -> (3, 4) -> (3, 3) -> (3, 2) -> (3, 1)
        # Cost is 8 steps
        self.assertEqual(res.cost, 8.0)
        self.assertNotIn((2, 1), res.path)

    def test_narrow_passage(self):
        """Tests navigation through a 1-cell narrow corridor."""
        # Walls creating a single doorway at (2, 2)
        obstacles = {
            (2, 0), (2, 1), (2, 3), (2, 4)
        }
        res = a_star_search(
            rows=5, cols=5,
            start=(0, 2), goal=(4, 2),
            obstacles=obstacles, allow_diagonal=False
        )
        self.assertTrue(res.success)
        self.assertIn((2, 2), res.path)

    def test_blocked_destination_unreachable(self):
        """Tests that an unreachable goal returns failure cleanly without loops."""
        obstacles = {(0, 1), (1, 0), (1, 1)}  # Encircles (0, 0)
        res = a_star_search(
            rows=5, cols=5,
            start=(0, 0), goal=(4, 4),
            obstacles=obstacles, allow_diagonal=False
        )
        self.assertFalse(res.success)
        self.assertEqual(res.path, [])
        self.assertEqual(res.cost, float('inf'))
        self.assertIn("unreachable", res.message.lower())

    def test_identical_start_and_goal(self):
        """Tests start == goal returns a 0-cost single-node path."""
        res = a_star_search(
            rows=5, cols=5,
            start=(2, 2), goal=(2, 2),
            obstacles=set(), allow_diagonal=False
        )
        self.assertTrue(res.success)
        self.assertEqual(res.cost, 0.0)
        self.assertEqual(res.path, [(2, 2)])

    def test_invalid_start_and_goal_coordinates(self):
        """Tests out-of-bounds start or goal coords."""
        res1 = a_star_search(rows=5, cols=5, start=(-1, 2), goal=(2, 2), obstacles=set())
        self.assertFalse(res1.success)

        res2 = a_star_search(rows=5, cols=5, start=(2, 2), goal=(5, 5), obstacles=set())
        self.assertFalse(res2.success)

    def test_start_or_goal_inside_obstacle(self):
        """Tests that starting or ending on an obstacle fails gracefully."""
        res = a_star_search(rows=5, cols=5, start=(1, 1), goal=(3, 3), obstacles={(1, 1)})
        self.assertFalse(res.success)

    def test_diagonal_movement_costs(self):
        """Tests diagonal movement with cost sqrt(2)."""
        res = a_star_search(
            rows=5, cols=5,
            start=(0, 0), goal=(2, 2),
            obstacles=set(), allow_diagonal=True
        )
        self.assertTrue(res.success)
        # 2 diagonal steps = 2 * sqrt(2)
        expected_cost = 2.0 * math.sqrt(2.0)
        self.assertAlmostEqual(res.cost, expected_cost, places=5)
        self.assertEqual(len(res.path), 3)

    def test_diagonal_corner_cutting_prevention(self):
        """
        Critical test: diagonal move from (0, 0) to (1, 1) MUST NOT be allowed
        if adjacent cells (0, 1) or (1, 0) are obstacles.
        """
        obstacles = {(0, 1), (1, 0)}
        res = a_star_search(
            rows=3, cols=3,
            start=(0, 0), goal=(1, 1),
            obstacles=obstacles, allow_diagonal=True
        )
        # Since (0, 1) and (1, 0) are blocked, the robot cannot cut through the diagonal pinch point!
        # And in a 3x3 grid with (0, 1) and (1, 0) blocked, (0, 0) has NO other neighbors!
        self.assertFalse(res.success)

    def test_corner_cutting_one_wall_prevention(self):
        """Diagonal corner cutting blocked even if only ONE orthogonal neighbor is obstacle."""
        # Moving from (1, 1) to (2, 2) when (1, 2) is obstacle
        # Robot should not be able to brush against the corner of (1, 2)
        obstacles = {(1, 2)}
        res = a_star_search(
            rows=4, cols=4,
            start=(1, 1), goal=(2, 2),
            obstacles=obstacles, allow_diagonal=True
        )
        self.assertTrue(res.success)
        # Since (1, 2) is blocked, diagonal (1,1)->(2,2) cannot be taken directly.
        # Instead, it can go (1, 1) -> (2, 1) [cost 1] -> (2, 2) [cost 1] = cost 2.0
        # or diagonal through other open cells.
        self.assertNotIn(((1, 1), (2, 2)), list(zip(res.path[:-1], res.path[1:])))

    def test_optimality_against_known_grid(self):
        """Verifies A* returns the optimal cost on a known maze layout."""
        obstacles = {(1, 0), (1, 1), (1, 2)}
        res = a_star_search(
            rows=4, cols=4,
            start=(0, 0), goal=(3, 1),
            obstacles=obstacles, allow_diagonal=False
        )
        self.assertTrue(res.success)
        self.assertEqual(res.cost, 8.0)

    def test_optimality_compared_to_dijkstra_reference(self):
        """
        Compares A* path cost against Dijkstra's algorithm (uniform-cost search)
        on random grid configurations to prove admissible heuristic yields exact optimal cost.
        """
        import heapq
        from astar import get_neighbors

        def dijkstra(rows, cols, start, goal, obstacles, allow_diagonal):
            pq = [(0.0, start)]
            dist = {start: 0.0}
            visited = set()
            while pq:
                d, u = heapq.heappop(pq)
                if u in visited:
                    continue
                visited.add(u)
                if u == goal:
                    return d
                for v, cost in get_neighbors(u, rows, cols, obstacles, allow_diagonal):
                    if d + cost < dist.get(v, float('inf')):
                        dist[v] = d + cost
                        heapq.heappush(pq, (d + cost, v))
            return float('inf')

        # Test on 5 different obstacle configurations
        import random
        for seed in range(5):
            rng = random.Random(seed + 42)
            rows, cols = 8, 8
            start = (0, 0)
            goal = (7, 7)
            # Generate random obstacles
            obs = set()
            for r in range(rows):
                for c in range(cols):
                    if (r, c) != start and (r, c) != goal and rng.random() < 0.20:
                        obs.add((r, c))

            for allow_diag in (False, True):
                dijkstra_cost = dijkstra(rows, cols, start, goal, obs, allow_diag)
                astar_res = a_star_search(rows, cols, start, goal, obs, allow_diag)

                if dijkstra_cost == float('inf'):
                    self.assertFalse(astar_res.success)
                else:
                    self.assertTrue(astar_res.success)
                    self.assertAlmostEqual(astar_res.cost, dijkstra_cost, places=5)


if __name__ == "__main__":
    unittest.main()
