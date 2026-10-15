"""
room.py - Room Environment and Grid Management for SmartSweep AI
Handles grid dimensions, start/goal positions, obstacles, dirt spots,
preset demo rooms, random room generation, and JSON persistence.
"""

import json
import random
from typing import Any, Dict, List, Optional, Set, Tuple

Coord = Tuple[int, int]


class RoomError(Exception):
    """Custom exception raised for room validation or load errors."""
    pass


class Room:
    """
    Represents the 2D grid environment for the vacuum robot.
    Coordinates are (row, col) with 0 <= row < rows and 0 <= col < cols.
    """

    MIN_DIMENSION = 5
    MAX_DIMENSION = 30

    def __init__(
        self,
        rows: int = 15,
        cols: int = 15,
        start: Optional[Coord] = None,
        goal: Optional[Coord] = None,
        allow_diagonal: bool = False
    ):
        self.set_dimensions(rows, cols)
        self.allow_diagonal = allow_diagonal
        self.obstacles: Set[Coord] = set()
        self.dirt_locations: Set[Coord] = set()
        self.cleaned_dirt: Set[Coord] = set()

        self.start: Coord = start if start is not None else (1, 1)
        self.goal: Coord = goal if goal is not None else (self.rows - 2, self.cols - 2)

        self._validate_positions()

    def set_dimensions(self, rows: int, cols: int):
        """Sets and validates grid dimensions."""
        if not (self.MIN_DIMENSION <= rows <= self.MAX_DIMENSION):
            raise RoomError(f"Rows must be between {self.MIN_DIMENSION} and {self.MAX_DIMENSION}, got {rows}.")
        if not (self.MIN_DIMENSION <= cols <= self.MAX_DIMENSION):
            raise RoomError(f"Columns must be between {self.MIN_DIMENSION} and {self.MAX_DIMENSION}, got {cols}.")
        self.rows = rows
        self.cols = cols

    def is_within_bounds(self, r: int, c: int) -> bool:
        """Checks if a cell coordinate is within grid bounds."""
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _validate_positions(self):
        """Ensures start and goal are inside bounds and not obstructed."""
        if not self.is_within_bounds(*self.start):
            self.start = (0, 0)
        if not self.is_within_bounds(*self.goal):
            self.goal = (self.rows - 1, self.cols - 1)
        self.obstacles.discard(self.start)
        self.obstacles.discard(self.goal)
        self.dirt_locations.discard(self.start)
        self.dirt_locations.discard(self.goal)

    def set_start(self, r: int, c: int) -> bool:
        """Sets start position if valid and not an obstacle."""
        if not self.is_within_bounds(r, c):
            return False
        coord = (r, c)
        if coord in self.obstacles or coord == self.goal:
            return False
        self.dirt_locations.discard(coord)
        self.cleaned_dirt.discard(coord)
        self.start = coord
        return True

    def set_goal(self, r: int, c: int) -> bool:
        """Sets goal position if valid and not an obstacle."""
        if not self.is_within_bounds(r, c):
            return False
        coord = (r, c)
        if coord in self.obstacles or coord == self.start:
            return False
        self.dirt_locations.discard(coord)
        self.cleaned_dirt.discard(coord)
        self.goal = coord
        return True

    def add_obstacle(self, r: int, c: int) -> bool:
        """Adds an obstacle cell if not at start or goal."""
        if not self.is_within_bounds(r, c):
            return False
        coord = (r, c)
        if coord == self.start or coord == self.goal:
            return False
        self.dirt_locations.discard(coord)
        self.cleaned_dirt.discard(coord)
        self.obstacles.add(coord)
        return True

    def remove_obstacle(self, r: int, c: int) -> bool:
        """Removes an obstacle cell."""
        coord = (r, c)
        if coord in self.obstacles:
            self.obstacles.remove(coord)
            return True
        return False

    def toggle_obstacle(self, r: int, c: int) -> bool:
        """Toggles an obstacle at the given coordinates."""
        if (r, c) in self.obstacles:
            return self.remove_obstacle(r, c)
        return self.add_obstacle(r, c)

    def add_dirt(self, r: int, c: int) -> bool:
        """Adds a dirt location if cell is free."""
        if not self.is_within_bounds(r, c):
            return False
        coord = (r, c)
        if coord in self.obstacles or coord == self.start or coord == self.goal:
            return False
        self.dirt_locations.add(coord)
        self.cleaned_dirt.discard(coord)
        return True

    def remove_dirt(self, r: int, c: int) -> bool:
        """Removes dirt from cell."""
        coord = (r, c)
        removed = False
        if coord in self.dirt_locations:
            self.dirt_locations.remove(coord)
            removed = True
        if coord in self.cleaned_dirt:
            self.cleaned_dirt.remove(coord)
            removed = True
        return removed

    def clear_cell(self, r: int, c: int):
        """Erases obstacle or dirt at (r, c)."""
        self.remove_obstacle(r, c)
        self.remove_dirt(r, c)

    def mark_cleaned(self, r: int, c: int):
        """Marks a dirt location as cleaned."""
        coord = (r, c)
        if coord in self.dirt_locations:
            self.dirt_locations.remove(coord)
            self.cleaned_dirt.add(coord)

    def reset_cleaning_state(self):
        """Restores cleaned spots back into active dirt locations."""
        self.dirt_locations.update(self.cleaned_dirt)
        self.cleaned_dirt.clear()

    def clear_room(self):
        """Clears all obstacles and dirt, preserving start and goal."""
        self.obstacles.clear()
        self.dirt_locations.clear()
        self.cleaned_dirt.clear()

    def load_demo_room(self):
        """
        Loads a curated 15x15 demo room designed for academic presentations.
        Includes furniture obstacles (sofa, dining table, divider wall with door)
        and realistic dirt locations requiring smart path navigation.
        """
        self.set_dimensions(15, 15)
        self.clear_room()
        self.start = (1, 1)
        self.goal = (13, 13)

        # 1. Living room sofa (Row 3, Cols 1 to 5)
        for c in range(1, 6):
            self.obstacles.add((3, c))
        # Sofa L-shape extension
        self.obstacles.add((4, 5))

        # 2. Central wall divider with a door opening
        # Wall from row 1 to 9 along col 8, with door gap at row 5
        for r in range(1, 9):
            if r != 5:  # Doorway at (5, 8)
                self.obstacles.add((r, 8))

        # 3. Dining table & chairs block (Rows 7-9, Cols 10-12)
        for r in range(7, 10):
            for c in range(10, 13):
                self.obstacles.add((r, c))

        # 4. TV cabinet / media center (Row 11, Cols 2 to 5)
        for c in range(2, 6):
            self.obstacles.add((11, c))

        # 5. Kitchen island / partition (Rows 10-12, Col 8)
        for r in range(10, 13):
            self.obstacles.add((r, 8))

        # Strategic dirt locations
        self.dirt_locations = {
            (2, 4),   # Near sofa
            (4, 9),   # Corridor right past the divider doorway
            (9, 6),   # Hallway before kitchen
            (12, 10), # Near dining area / goal approach
            (6, 2),   # Living room center
        }

    def generate_random(self, obstacle_density: float = 0.20, dirt_count: int = 5):
        """
        Generates random obstacles and dirt locations while ensuring:
        - Start and goal remain unobstructed.
        - Obstacle density is reasonable for demonstration.
        - Dirt is not placed on obstacles or start/goal.
        """
        self.clear_room()
        total_cells = self.rows * self.cols
        num_obstacles = int(total_cells * max(0.05, min(0.40, obstacle_density)))

        all_coords = [
            (r, c)
            for r in range(self.rows)
            for c in range(self.cols)
            if (r, c) != self.start and (r, c) != self.goal
        ]
        random.shuffle(all_coords)

        # Place obstacles
        obs_candidates = all_coords[:num_obstacles]
        self.obstacles = set(obs_candidates)

        # Place dirt from remaining free cells
        remaining_free = all_coords[num_obstacles:]
        actual_dirt_count = min(dirt_count, len(remaining_free))
        self.dirt_locations = set(remaining_free[:actual_dirt_count])

    def to_dict(self) -> Dict[str, Any]:
        """Serializes room state to a dictionary for JSON storage."""
        return {
            "version": "1.0",
            "rows": self.rows,
            "cols": self.cols,
            "start": list(self.start),
            "goal": list(self.goal),
            "allow_diagonal": self.allow_diagonal,
            "obstacles": [list(coord) for coord in sorted(self.obstacles)],
            "dirt_locations": [list(coord) for coord in sorted(self.dirt_locations)],
            "cleaned_dirt": [list(coord) for coord in sorted(self.cleaned_dirt)],
        }

    def from_dict(self, data: Dict[str, Any]):
        """Deserializes and validates room configuration from dictionary."""
        if not isinstance(data, dict):
            raise RoomError("Invalid room data format: expected JSON object.")

        if "rows" not in data or "cols" not in data:
            raise RoomError("Room file missing 'rows' or 'cols' fields.")

        try:
            rows = int(data["rows"])
            cols = int(data["cols"])
        except (ValueError, TypeError):
            raise RoomError("Invalid rows/cols value: must be integers.")

        self.set_dimensions(rows, cols)

        # Start and Goal
        if "start" not in data or "goal" not in data:
            raise RoomError("Missing 'start' or 'goal' in room data.")

        try:
            start_coord = (int(data["start"][0]), int(data["start"][1]))
            goal_coord = (int(data["goal"][0]), int(data["goal"][1]))
        except (IndexError, ValueError, TypeError):
            raise RoomError("Invalid start or goal coordinate format.")

        if not self.is_within_bounds(*start_coord):
            raise RoomError(f"Start coordinate {start_coord} is out of bounds ({rows}x{cols}).")
        if not self.is_within_bounds(*goal_coord):
            raise RoomError(f"Goal coordinate {goal_coord} is out of bounds ({rows}x{cols}).")

        self.start = start_coord
        self.goal = goal_coord
        self.allow_diagonal = bool(data.get("allow_diagonal", False))

        # Obstacles
        self.obstacles.clear()
        for obs in data.get("obstacles", []):
            try:
                r, c = int(obs[0]), int(obs[1])
                if self.is_within_bounds(r, c) and (r, c) != self.start and (r, c) != self.goal:
                    self.obstacles.add((r, c))
            except (IndexError, ValueError, TypeError):
                continue

        # Dirt
        self.dirt_locations.clear()
        for dirt in data.get("dirt_locations", []):
            try:
                r, c = int(dirt[0]), int(dirt[1])
                if (
                    self.is_within_bounds(r, c)
                    and (r, c) not in self.obstacles
                    and (r, c) != self.start
                    and (r, c) != self.goal
                ):
                    self.dirt_locations.add((r, c))
            except (IndexError, ValueError, TypeError):
                continue

        self.cleaned_dirt.clear()
        for cd in data.get("cleaned_dirt", []):
            try:
                r, c = int(cd[0]), int(cd[1])
                if self.is_within_bounds(r, c):
                    self.cleaned_dirt.add((r, c))
            except (IndexError, ValueError, TypeError):
                continue

    def save_to_json(self, filepath: str):
        """Saves current room configuration to a JSON file."""
        data = self.to_dict()
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def load_from_json(self, filepath: str):
        """Loads and parses room configuration from a JSON file."""
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            raise RoomError(f"File contains invalid JSON syntax: {e}")
        except OSError as e:
            raise RoomError(f"Failed to read file: {e}")

        self.from_dict(data)
