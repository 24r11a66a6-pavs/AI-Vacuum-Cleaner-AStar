"""
Unit tests for Robot Simulation and Multi-Dirt Path Planning.
Tests single destination mode, multi-dirt nearest target sequence,
unreachable dirt handling, step forward, pause, and metrics.
"""

import unittest
from room import Room
from robot_simulation import CleaningMode, RobotSimulation, SimState


class TestSimulation(unittest.TestCase):

    def setUp(self):
        self.room = Room(rows=10, cols=10, start=(0, 0), goal=(9, 9))
        self.sim = RobotSimulation(self.room)

    def test_single_destination_planning(self):
        success = self.sim.calculate_plan(mode=CleaningMode.SINGLE_DESTINATION)
        self.assertTrue(success)
        self.assertEqual(self.sim.state, SimState.PATH_READY)
        self.assertGreater(len(self.sim.planned_path), 0)
        self.assertEqual(self.sim.planned_path[0], (0, 0))
        self.assertEqual(self.sim.planned_path[-1], (9, 9))

    def test_multi_dirt_cleaning_planning(self):
        # Place dirt at (0, 3) and (3, 3)
        self.room.add_dirt(0, 3)
        self.room.add_dirt(3, 3)

        success = self.sim.calculate_plan(mode=CleaningMode.SMART_CLEANING)
        self.assertTrue(success)
        self.assertEqual(self.sim.state, SimState.PATH_READY)
        # Both dirt locations should be visited along the planned path
        self.assertIn((0, 3), self.sim.planned_path)
        self.assertIn((3, 3), self.sim.planned_path)
        self.assertEqual(self.sim.planned_path[-1], (9, 9))

    def test_unreachable_dirt_reported(self):
        # Dirt at (0, 3) is reachable
        self.room.add_dirt(0, 3)
        # Dirt at (5, 5) trapped inside wall box
        self.room.add_dirt(5, 5)
        self.room.add_obstacle(4, 5)
        self.room.add_obstacle(6, 5)
        self.room.add_obstacle(5, 4)
        self.room.add_obstacle(5, 6)

        success = self.sim.calculate_plan(mode=CleaningMode.SMART_CLEANING)
        self.assertTrue(success)
        self.assertIn((5, 5), self.sim.unreachable_dirt)
        self.assertIn("trapped/unreachable", self.sim.status_message.lower())

    def test_step_forward_movement_and_cleaning(self):
        # Set goal to (0, 2) so (0, 1) is guaranteed to be on the straight optimal path
        self.room.set_goal(0, 2)
        self.room.add_dirt(0, 1)
        self.sim.calculate_plan(mode=CleaningMode.SINGLE_DESTINATION)

        # Before step
        self.assertEqual(self.sim.current_pos, (0, 0))
        self.assertEqual(self.sim.steps_completed, 0)
        self.assertIn((0, 1), self.room.dirt_locations)

        # Step 1: moves to (0, 1)
        moved = self.sim.step_forward()
        self.assertTrue(moved)
        self.assertEqual(self.sim.current_pos, (0, 1))
        self.assertEqual(self.sim.steps_completed, 1)
        # Dirt should be cleaned
        self.assertIn((0, 1), self.room.cleaned_dirt)
        self.assertNotIn((0, 1), self.room.dirt_locations)

    def test_pause_resume_reset(self):
        self.sim.calculate_plan(mode=CleaningMode.SINGLE_DESTINATION)
        self.sim.resume()
        self.assertEqual(self.sim.state, SimState.RUNNING)

        self.sim.pause()
        self.assertEqual(self.sim.state, SimState.PAUSED)

        self.sim.step_forward()
        self.sim.reset()
        self.assertEqual(self.sim.state, SimState.READY)
        self.assertEqual(self.sim.current_pos, self.room.start)
        self.assertEqual(self.sim.steps_completed, 0)


if __name__ == "__main__":
    unittest.main()
