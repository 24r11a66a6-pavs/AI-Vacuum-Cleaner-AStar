"""
Unit tests for Room Management in SmartSweep AI.
Tests grid boundaries, adding/removing obstacles & dirt,
demo room layout, random room generation, and JSON save/load.
"""

import json
import os
import tempfile
import unittest
from room import Room, RoomError


class TestRoom(unittest.TestCase):

    def setUp(self):
        self.room = Room(rows=10, cols=10, start=(1, 1), goal=(8, 8))

    def test_initial_state(self):
        self.assertEqual(self.room.rows, 10)
        self.assertEqual(self.room.cols, 10)
        self.assertEqual(self.room.start, (1, 1))
        self.assertEqual(self.room.goal, (8, 8))
        self.assertEqual(len(self.room.obstacles), 0)
        self.assertEqual(len(self.room.dirt_locations), 0)

    def test_invalid_dimensions(self):
        with self.assertRaises(RoomError):
            Room(rows=2, cols=10)
        with self.assertRaises(RoomError):
            Room(rows=10, cols=35)

    def test_add_and_remove_obstacles(self):
        self.assertTrue(self.room.add_obstacle(3, 3))
        self.assertIn((3, 3), self.room.obstacles)

        # Cannot add obstacle on start or goal
        self.assertFalse(self.room.add_obstacle(1, 1))
        self.assertFalse(self.room.add_obstacle(8, 8))

        # Remove obstacle
        self.assertTrue(self.room.remove_obstacle(3, 3))
        self.assertNotIn((3, 3), self.room.obstacles)

    def test_add_and_clean_dirt(self):
        self.assertTrue(self.room.add_dirt(4, 4))
        self.assertIn((4, 4), self.room.dirt_locations)

        # Cannot place dirt on obstacle
        self.room.add_obstacle(5, 5)
        self.assertFalse(self.room.add_dirt(5, 5))

        # Mark cleaned
        self.room.mark_cleaned(4, 4)
        self.assertNotIn((4, 4), self.room.dirt_locations)
        self.assertIn((4, 4), self.room.cleaned_dirt)

        # Reset cleaning state restores dirt
        self.room.reset_cleaning_state()
        self.assertIn((4, 4), self.room.dirt_locations)
        self.assertEqual(len(self.room.cleaned_dirt), 0)

    def test_demo_room(self):
        self.room.load_demo_room()
        self.assertEqual(self.room.rows, 15)
        self.assertEqual(self.room.cols, 15)
        self.assertEqual(self.room.start, (1, 1))
        self.assertEqual(self.room.goal, (13, 13))
        self.assertGreater(len(self.room.obstacles), 10)
        self.assertGreaterEqual(len(self.room.dirt_locations), 3)
        self.assertNotIn(self.room.start, self.room.obstacles)
        self.assertNotIn(self.room.goal, self.room.obstacles)

    def test_random_room_generation(self):
        self.room.generate_random(obstacle_density=0.20, dirt_count=5)
        self.assertNotIn(self.room.start, self.room.obstacles)
        self.assertNotIn(self.room.goal, self.room.obstacles)
        self.assertNotIn(self.room.start, self.room.dirt_locations)
        self.assertNotIn(self.room.goal, self.room.dirt_locations)
        # Check no overlap between dirt and obstacles
        overlap = self.room.dirt_locations.intersection(self.room.obstacles)
        self.assertEqual(len(overlap), 0)

    def test_json_save_and_load(self):
        self.room.add_obstacle(2, 2)
        self.room.add_dirt(3, 3)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            self.room.save_to_json(tmp_path)
            loaded_room = Room(rows=5, cols=5)
            loaded_room.load_from_json(tmp_path)

            self.assertEqual(loaded_room.rows, 10)
            self.assertEqual(loaded_room.cols, 10)
            self.assertEqual(loaded_room.start, (1, 1))
            self.assertEqual(loaded_room.goal, (8, 8))
            self.assertIn((2, 2), loaded_room.obstacles)
            self.assertIn((3, 3), loaded_room.dirt_locations)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_invalid_json_handling(self):
        with tempfile.NamedTemporaryFile(suffix=".json", mode="w", delete=False) as tmp:
            tmp.write("{ invalid json content ]")
            tmp_path = tmp.name

        try:
            with self.assertRaises(RoomError):
                self.room.load_from_json(tmp_path)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)


if __name__ == "__main__":
    unittest.main()
