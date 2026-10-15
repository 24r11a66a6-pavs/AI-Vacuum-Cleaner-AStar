"""
Integration tests for GUI actions and simulation state transitions.
Tests path finding, cleaning execution, pause/resume, step forward,
reset, and room generation directly on the GUI controller.
"""

import unittest
import customtkinter as ctk
from gui import SmartSweepGUI
from robot_simulation import SimState


class TestGUIIntegration(unittest.TestCase):

    def setUp(self):
        self.root = ctk.CTk()
        self.app = SmartSweepGUI(self.root)
        self.root.update()

    def tearDown(self):
        self.app._stop_animation()
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def test_initial_demo_room_loaded(self):
        self.assertEqual(self.app.room.rows, 15)
        self.assertEqual(self.app.room.cols, 15)
        self.assertGreater(len(self.app.room.obstacles), 10)
        self.assertGreaterEqual(len(self.app.room.dirt_locations), 3)

    def test_find_optimal_path_demo_room(self):
        self.app.find_optimal_path()
        self.root.update()
        self.assertEqual(self.app.sim.state, SimState.PATH_READY)
        self.assertGreater(len(self.app.sim.planned_path), 0)
        self.assertGreater(self.app.sim.total_path_cost, 0.0)

    def test_start_and_pause_cleaning(self):
        self.app.find_optimal_path()
        # Start cleaning
        self.app.sim.resume()
        self.app.update_status_badge()
        self.assertEqual(self.app.sim.state, SimState.RUNNING)

        # Step forward
        moved = self.app.sim.step_forward()
        self.assertTrue(moved)
        self.assertEqual(self.app.sim.steps_completed, 1)

        # Pause
        self.app.pause_resume()
        self.assertEqual(self.app.sim.state, SimState.PAUSED)

        # Resume
        self.app.pause_resume()
        self.assertEqual(self.app.sim.state, SimState.RUNNING)

        # Reset
        self.app.reset_simulation()
        self.assertEqual(self.app.sim.state, SimState.READY)
        self.assertEqual(self.app.sim.steps_completed, 0)

    def test_step_forward_button(self):
        self.app.find_optimal_path()
        self.app.step_forward()
        self.root.update()
        self.assertEqual(self.app.sim.steps_completed, 1)

    def test_random_room_and_clear_room(self):
        self.app.generate_random_room()
        self.root.update()
        self.assertGreater(len(self.app.room.obstacles), 0)

        self.app.clear_room()
        self.root.update()
        self.assertEqual(len(self.app.room.obstacles), 0)
        self.assertEqual(len(self.app.room.dirt_locations), 0)

        # Reload demo
        self.app.load_demo_room()
        self.root.update()
        self.assertGreater(len(self.app.room.obstacles), 10)


if __name__ == "__main__":
    unittest.main()
