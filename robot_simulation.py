"""
robot_simulation.py - Robot Vacuum Simulation & Multi-Target Path Planning
Manages simulation states, multi-dirt nearest-target planning,
step execution, statistics tracking, and animation lifecycle.
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

from astar import Coord, PathResult, a_star_search
from room import Room


class SimState(Enum):
    READY = "Ready"
    CALCULATING = "Calculating"
    EXPLORING = "Exploring (Step-by-Step)"
    PATH_READY = "Path Ready"
    RUNNING = "Cleaning in Progress"
    PAUSED = "Paused"
    COMPLETED = "Completed"
    NO_PATH = "No Path Found"


class CleaningMode(Enum):
    SINGLE_DESTINATION = "Single Destination"
    SMART_CLEANING = "Smart Cleaning (Multi-Dirt)"


@dataclass
class SimulationStats:
    total_path_cost: float = 0.0
    total_steps: int = 0
    distance_traveled: float = 0.0
    steps_completed: int = 0
    nodes_explored_count: int = 0
    execution_time_ms: float = 0.0
    initial_dirt_count: int = 0
    dirt_cleaned_count: int = 0
    dirt_remaining_count: int = 0
    unreachable_dirt_count: int = 0
    progress_percentage: float = 0.0
    current_position: Coord = (0, 0)
    current_heading: Tuple[int, int] = (0, 1)  # Default facing Right


class RobotSimulation:
    """
    Orchestrates robot vacuum movement, multi-dirt route planning,
    and simulation execution.
    """

    def __init__(self, room: Room):
        self.room = room
        self.state = SimState.READY
        self.mode = CleaningMode.SMART_CLEANING

        # Navigation and Path data
        self.planned_path: List[Coord] = []
        self.segment_transitions: List[Tuple[int, str]] = []  # (index in path, label: "dirt" or "goal")
        self.path_step_costs: List[float] = []

        # Real-time tracking
        self.current_path_index: int = 0
        self.current_pos: Coord = self.room.start
        self.current_heading: Tuple[int, int] = (0, 1)  # (dr, dc)

        # Explored sets for visualization
        self.explored_nodes: Set[Coord] = set()
        self.frontier_nodes: Set[Coord] = set()
        self.explored_order: List[Coord] = []

        # Multi-dirt planning results
        self.unreachable_dirt: Set[Coord] = set()
        self.initial_dirt_set: Set[Coord] = set()

        # Performance metrics
        self.total_path_cost: float = 0.0
        self.distance_traveled: float = 0.0
        self.steps_completed: int = 0
        self.execution_time_ms: float = 0.0
        self.status_message: str = "Ready to plan route."

    def reset(self):
        """Resets the simulation to the initial start state."""
        self.state = SimState.READY
        self.current_pos = self.room.start
        self.current_heading = (0, 1)
        self.current_path_index = 0
        self.distance_traveled = 0.0
        self.steps_completed = 0
        self.room.reset_cleaning_state()
        self.unreachable_dirt.clear()
        self.status_message = "Simulation reset to start position."

    def invalidate_path(self):
        """Invalidates planned path when the user alters obstacles or endpoints."""
        self.planned_path.clear()
        self.path_step_costs.clear()
        self.segment_transitions.clear()
        self.explored_nodes.clear()
        self.frontier_nodes.clear()
        self.explored_order.clear()
        self.total_path_cost = 0.0
        self.reset()
        self.status_message = "Room modified. Please recalculate path."

    def calculate_plan(self, mode: Optional[CleaningMode] = None) -> bool:
        """
        Calculates path according to selected mode:
        - Mode 1: Single Destination (Start -> Goal)
        - Mode 2: Smart Cleaning (Start -> Nearest Dirt -> ... -> Goal)
        """
        if mode is not None:
            self.mode = mode

        self.reset()
        self.state = SimState.CALCULATING

        if self.mode == CleaningMode.SINGLE_DESTINATION:
            return self._plan_single_destination()
        else:
            return self._plan_smart_cleaning()

    def _plan_single_destination(self) -> bool:
        """Plans optimal route directly from Start to Goal."""
        res = a_star_search(
            rows=self.room.rows,
            cols=self.room.cols,
            start=self.room.start,
            goal=self.room.goal,
            obstacles=self.room.obstacles,
            allow_diagonal=self.room.allow_diagonal,
        )

        self.execution_time_ms = res.execution_time_ms
        self.explored_nodes = set(res.explored_order)
        self.frontier_nodes = res.frontier_nodes
        self.explored_order = res.explored_order

        if not res.success:
            self.state = SimState.NO_PATH
            self.planned_path = []
            self.path_step_costs = []
            self.total_path_cost = 0.0
            self.status_message = res.message
            return False

        self.planned_path = res.path
        self._calculate_step_costs()
        self.total_path_cost = res.cost
        self.segment_transitions = [(len(self.planned_path) - 1, "goal")]
        self.state = SimState.PATH_READY
        self.status_message = f"Optimal route calculated ({len(self.planned_path)} steps, Cost: {self.total_path_cost:.2f})."
        return True

    def _plan_smart_cleaning(self) -> bool:
        """
        Plans multi-target route visiting all reachable dirt locations using
        a greedy nearest-target strategy based on A* path cost, concluding at the Goal.
        """
        self.initial_dirt_set = set(self.room.dirt_locations)
        unvisited_dirt = set(self.room.dirt_locations)
        self.unreachable_dirt.clear()

        full_path: List[Coord] = []
        transitions: List[Tuple[int, str]] = []
        total_time_ms = 0.0
        current_pos = self.room.start
        all_explored: Set[Coord] = set()
        all_frontier: Set[Coord] = set()
        all_order: List[Coord] = []

        # Iterate until all reachable dirt locations are consumed
        while unvisited_dirt:
            best_dirt: Optional[Coord] = None
            best_res: Optional[PathResult] = None
            best_cost = float('inf')

            for dirt_candidate in unvisited_dirt:
                res = a_star_search(
                    rows=self.room.rows,
                    cols=self.room.cols,
                    start=current_pos,
                    goal=dirt_candidate,
                    obstacles=self.room.obstacles,
                    allow_diagonal=self.room.allow_diagonal,
                )
                total_time_ms += res.execution_time_ms
                all_explored.update(res.explored_order)
                all_frontier.update(res.frontier_nodes)
                all_order.extend(res.explored_order)

                if res.success and res.cost < best_cost:
                    best_cost = res.cost
                    best_dirt = dirt_candidate
                    best_res = res

            if best_dirt is None or best_res is None:
                # All remaining dirt in this set is unreachable
                self.unreachable_dirt.update(unvisited_dirt)
                break

            # Append segment path (avoiding duplicate junction node)
            if not full_path:
                full_path.extend(best_res.path)
            else:
                full_path.extend(best_res.path[1:])

            unvisited_dirt.remove(best_dirt)
            current_pos = best_dirt
            transitions.append((len(full_path) - 1, "dirt"))

        # Finally, plan from last location to final Goal
        goal_res = a_star_search(
            rows=self.room.rows,
            cols=self.room.cols,
            start=current_pos,
            goal=self.room.goal,
            obstacles=self.room.obstacles,
            allow_diagonal=self.room.allow_diagonal,
        )
        total_time_ms += goal_res.execution_time_ms
        all_explored.update(goal_res.explored_order)
        all_frontier.update(goal_res.frontier_nodes)
        all_order.extend(goal_res.explored_order)

        if not goal_res.success:
            # Reached dirt, but cannot reach final destination
            if not full_path:
                self.state = SimState.NO_PATH
                self.planned_path = []
                self.path_step_costs = []
                self.status_message = "Neither dirt nor final goal could be reached from start!"
                return False
            # Still allow cleaning reachable dirt even if final goal is blocked
            self.status_message = f"Reachable dirt planned, but Final Goal at {self.room.goal} is blocked!"
        else:
            if not full_path:
                full_path.extend(goal_res.path)
            else:
                full_path.extend(goal_res.path[1:])
            transitions.append((len(full_path) - 1, "goal"))

        self.planned_path = full_path
        self._calculate_step_costs()
        self.segment_transitions = transitions
        self.execution_time_ms = total_time_ms
        self.explored_nodes = all_explored
        self.frontier_nodes = all_frontier
        self.explored_order = all_order

        cleaned_count = len(self.initial_dirt_set) - len(self.unreachable_dirt)
        if self.unreachable_dirt:
            self.status_message = (
                f"Planned route for {cleaned_count} dirt locations. "
                f"Note: {len(self.unreachable_dirt)} dirt spot(s) are trapped/unreachable."
            )
        else:
            self.status_message = (
                f"Complete Smart Cleaning route planned ({len(self.planned_path)} steps, "
                f"{len(self.initial_dirt_set)} dirt spots, Cost: {self.total_path_cost:.2f})."
            )

        self.state = SimState.PATH_READY
        return True

    def _calculate_step_costs(self):
        """Computes incremental step costs along the planned path."""
        self.path_step_costs = [0.0]
        total = 0.0
        for i in range(1, len(self.planned_path)):
            p1 = self.planned_path[i - 1]
            p2 = self.planned_path[i]
            dr = abs(p1[0] - p2[0])
            dc = abs(p1[1] - p2[1])
            cost = math.sqrt(2.0) if (dr == 1 and dc == 1) else 1.0
            self.path_step_costs.append(cost)
            total += cost
        self.total_path_cost = total

    def step_forward(self) -> bool:
        """
        Executes one movement step along the planned path.
        Returns True if moved, False if simulation has concluded.
        """
        if not self.planned_path or self.current_path_index >= len(self.planned_path) - 1:
            self.state = SimState.COMPLETED
            self.status_message = "Cleaning mission completed successfully!"
            return False

        # Advance to next coordinate
        self.current_path_index += 1
        prev_pos = self.current_pos
        self.current_pos = self.planned_path[self.current_path_index]

        # Update heading
        dr = self.current_pos[0] - prev_pos[0]
        dc = self.current_pos[1] - prev_pos[1]
        if dr != 0 or dc != 0:
            self.current_heading = (dr, dc)

        # Update metrics
        step_cost = self.path_step_costs[self.current_path_index]
        self.distance_traveled += step_cost
        self.steps_completed += 1

        # Clean dirt upon arrival
        if self.current_pos in self.room.dirt_locations:
            self.room.mark_cleaned(*self.current_pos)

        # Check if reached destination
        if self.current_path_index == len(self.planned_path) - 1:
            self.state = SimState.COMPLETED
            self.status_message = "Goal reached! Cleaning cycle complete."
            return False

        return True

    def pause(self):
        """Pauses simulation."""
        if self.state == SimState.RUNNING:
            self.state = SimState.PAUSED
            self.status_message = "Simulation paused."

    def resume(self):
        """Resumes simulation."""
        if self.state in (SimState.PAUSED, SimState.PATH_READY):
            self.state = SimState.RUNNING
            self.status_message = "Cleaning in progress..."

    def get_stats(self) -> SimulationStats:
        """Returns consolidated live snapshot of simulation statistics."""
        initial_dirt = len(self.initial_dirt_set) if self.initial_dirt_set else len(self.room.dirt_locations) + len(self.room.cleaned_dirt)
        dirt_cleaned = len(self.room.cleaned_dirt)
        dirt_remaining = len(self.room.dirt_locations)

        # Progress calculation
        if self.mode == CleaningMode.SMART_CLEANING and initial_dirt > 0:
            # 80% weight on dirt cleaning, 20% on reaching final goal
            dirt_pct = (dirt_cleaned / initial_dirt) * 80.0
            nav_pct = 20.0 if (self.state == SimState.COMPLETED) else (
                (self.current_path_index / max(1, len(self.planned_path) - 1)) * 20.0
            )
            progress = min(100.0, dirt_pct + nav_pct)
        else:
            if len(self.planned_path) <= 1:
                progress = 100.0 if self.state == SimState.COMPLETED else 0.0
            else:
                progress = min(100.0, (self.current_path_index / (len(self.planned_path) - 1)) * 100.0)

        if self.state == SimState.COMPLETED:
            progress = 100.0

        return SimulationStats(
            total_path_cost=self.total_path_cost,
            total_steps=max(0, len(self.planned_path) - 1),
            distance_traveled=self.distance_traveled,
            steps_completed=self.steps_completed,
            nodes_explored_count=len(self.explored_nodes),
            execution_time_ms=self.execution_time_ms,
            initial_dirt_count=initial_dirt,
            dirt_cleaned_count=dirt_cleaned,
            dirt_remaining_count=dirt_remaining,
            unreachable_dirt_count=len(self.unreachable_dirt),
            progress_percentage=round(progress, 1),
            current_position=self.current_pos,
            current_heading=self.current_heading,
        )
