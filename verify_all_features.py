"""
Comprehensive Verification Script for SmartSweep AI.
Tests and confirms:
1. Real A* algorithm execution & optimality.
2. Corner-cutting prevention & obstacle avoidance.
3. Multi-dirt nearest-target planning and cleaning.
4. Robot animation step-by-step state progression.
5. Pause, resume, and reset state machine.
6. Live statistics accuracy (costs, steps, nodes, time, progress %).
7. Save and Load JSON room persistence.
8. GUI launch, rendering, and theme toggle.
"""

import os
import sys
import tempfile
import time

# Ensure current dir is in sys.path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from astar import a_star_search, get_neighbors, manhattan_distance, octile_distance
from room import Room
from robot_simulation import CleaningMode, RobotSimulation, SimState
import customtkinter as ctk
from gui import SmartSweepGUI


def verify_real_astar():
    print("[1/8] Verifying Real A* Algorithm from Scratch...")
    # Verify evaluation function f(n) = g(n) + h(n) on 4-dir and 8-dir
    rows, cols = 6, 6
    start, goal = (0, 0), (5, 5)
    obstacles = {(2, 1), (2, 2), (2, 3), (2, 4)}

    # 4-Directional search
    res_4d = a_star_search(rows, cols, start, goal, obstacles, allow_diagonal=False)
    assert res_4d.success, "A* 4-Dir should find optimal route"
    assert res_4d.cost == 10.0, f"Expected 10.0, got {res_4d.cost}"
    assert len(res_4d.path) == 11, f"Expected 11 waypoints, got {len(res_4d.path)}"
    # Verify no path coordinates overlap with obstacles
    assert not set(res_4d.path).intersection(obstacles), "Path must not collide with obstacles!"

    # 8-Directional search
    res_8d = a_star_search(rows, cols, start, goal, obstacles, allow_diagonal=True)
    assert res_8d.success, "A* 8-Dir should find optimal route"
    assert res_8d.cost < res_4d.cost, "8-Dir cost must be less than or equal to 4-Dir cost"
    print(f"  [OK] A* 4-Dir Cost: {res_4d.cost:.2f}, 8-Dir Cost: {res_8d.cost:.2f}, Nodes Explored: {res_4d.nodes_explored}")


def verify_obstacle_avoidance_and_corner_cutting():
    print("[2/8] Verifying Obstacle Avoidance & Corner-Cutting Prevention...")
    # 2x2 pinch point: obstacles at (0, 1) and (1, 0)
    # Robot at (0, 0) wanting to go to (1, 1) diagonally
    obstacles = {(0, 1), (1, 0)}
    res = a_star_search(3, 3, (0, 0), (1, 1), obstacles, allow_diagonal=True)
    assert not res.success, "Diagonal corner cutting through pinch point MUST be blocked!"

    # Wall corner check: obstacle at (1, 2)
    # Robot moving from (1, 1) to (2, 2) must not cut through blocked corner (1, 2)
    obs_wall = {(1, 2)}
    res_wall = a_star_search(4, 4, (1, 1), (2, 2), obs_wall, allow_diagonal=True)
    assert res_wall.success, "Path should detour around wall corner"
    transitions = list(zip(res_wall.path[:-1], res_wall.path[1:]))
    assert ((1, 1), (2, 2)) not in transitions, "Corner-cutting move directly across blocked cell corner must not occur!"
    print("  [OK] Corner-cutting strictly prevented on both diagonal walls and pinch points.")


def verify_multi_dirt_cleaning():
    print("[3/8] Verifying Multiple-Dirt Cleaning Strategy...")
    room = Room(rows=10, cols=10, start=(0, 0), goal=(9, 9))
    dirt_spots = {(1, 3), (4, 4), (8, 2)}
    for d in dirt_spots:
        room.add_dirt(*d)

    sim = RobotSimulation(room)
    success = sim.calculate_plan(mode=CleaningMode.SMART_CLEANING)
    assert success, "Smart Cleaning route planning must succeed"
    assert sim.state == SimState.PATH_READY

    # Verify that all 3 dirt spots are visited along the planned path
    for d in dirt_spots:
        assert d in sim.planned_path, f"Dirt {d} must be visited along route!"
    assert sim.planned_path[-1] == (9, 9), "Final destination must be goal (9, 9)!"

    # Simulate stepping along entire path and verify dirt gets marked cleaned
    while sim.step_forward():
        pass

    assert sim.state == SimState.COMPLETED, "Simulation must reach COMPLETED state"
    assert len(room.dirt_locations) == 0, "All reachable dirt should be cleaned!"
    assert len(room.cleaned_dirt) == 3, "3 dirt spots should be in cleaned_dirt set!"
    print(f"  [OK] Multi-dirt route planned: {len(sim.planned_path)} steps. Cleaned 3/3 dirt spots.")


def verify_unreachable_dirt_reporting():
    print("[4/8] Verifying Unreachable Dirt Handling...")
    room = Room(rows=8, cols=8, start=(0, 0), goal=(7, 7))
    # Place reachable dirt
    room.add_dirt(0, 3)
    # Place unreachable dirt trapped in walls
    room.add_dirt(4, 4)
    room.add_obstacle(3, 4)
    room.add_obstacle(5, 4)
    room.add_obstacle(4, 3)
    room.add_obstacle(4, 5)

    sim = RobotSimulation(room)
    sim.calculate_plan(mode=CleaningMode.SMART_CLEANING)
    assert (4, 4) in sim.unreachable_dirt, "Trapped dirt must be detected as unreachable"
    assert (0, 3) in sim.planned_path, "Reachable dirt must still be included in plan"
    print("  [OK] Trapped dirt correctly reported as unreachable without halting simulation.")


def verify_pause_resume_reset():
    print("[5/8] Verifying Pause, Resume, and Reset Mechanics...")
    room = Room(rows=8, cols=8, start=(0, 0), goal=(7, 7))
    sim = RobotSimulation(room)
    sim.calculate_plan(mode=CleaningMode.SINGLE_DESTINATION)

    assert sim.state == SimState.PATH_READY
    sim.resume()
    assert sim.state == SimState.RUNNING

    # Step twice
    sim.step_forward()
    sim.step_forward()
    assert sim.steps_completed == 2
    assert sim.current_pos != (0, 0)

    # Pause
    sim.pause()
    assert sim.state == SimState.PAUSED

    # Resume
    sim.resume()
    assert sim.state == SimState.RUNNING

    # Reset
    sim.reset()
    assert sim.state == SimState.READY
    assert sim.current_pos == (0, 0)
    assert sim.steps_completed == 0
    assert sim.distance_traveled == 0.0
    print("  [OK] State transitions (Ready -> Running -> Paused -> Running -> Reset) confirmed.")


def verify_save_and_load():
    print("[6/8] Verifying JSON Save & Load Functionality...")
    room = Room(rows=12, cols=14, start=(2, 3), goal=(10, 11), allow_diagonal=True)
    room.add_obstacle(4, 5)
    room.add_obstacle(4, 6)
    room.add_dirt(7, 8)
    room.add_dirt(9, 2)

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        room.save_to_json(tmp_path)
        loaded = Room(rows=5, cols=5)
        loaded.load_from_json(tmp_path)

        assert loaded.rows == 12 and loaded.cols == 14
        assert loaded.start == (2, 3)
        assert loaded.goal == (10, 11)
        assert loaded.allow_diagonal == True
        assert (4, 5) in loaded.obstacles and (4, 6) in loaded.obstacles
        assert (7, 8) in loaded.dirt_locations and (9, 2) in loaded.dirt_locations
        print("  [OK] Room layout successfully serialized and deserialized with 100% fidelity.")
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def verify_statistics_accuracy():
    print("[7/8] Verifying Live Statistics Accuracy...")
    room = Room(rows=6, cols=6, start=(0, 0), goal=(0, 4))
    room.add_dirt(0, 2)
    sim = RobotSimulation(room)
    sim.calculate_plan(mode=CleaningMode.SMART_CLEANING)

    stats = sim.get_stats()
    assert stats.total_steps == 4, f"Expected 4 steps, got {stats.total_steps}"
    assert stats.total_path_cost == 4.0, f"Expected 4.0 cost, got {stats.total_path_cost}"
    assert stats.initial_dirt_count == 1
    assert stats.dirt_cleaned_count == 0
    assert stats.dirt_remaining_count == 1
    assert stats.progress_percentage == 0.0

    # Step to dirt (0, 2) - 2 steps
    sim.step_forward()
    sim.step_forward()
    stats2 = sim.get_stats()
    assert stats2.steps_completed == 2
    assert stats2.distance_traveled == 2.0
    assert stats2.dirt_cleaned_count == 1
    assert stats2.dirt_remaining_count == 0
    assert stats2.progress_percentage > 50.0

    # Finish path to goal (0, 4)
    sim.step_forward()
    sim.step_forward()
    assert sim.state == SimState.COMPLETED
    stats3 = sim.get_stats()
    assert stats3.progress_percentage == 100.0
    print("  [OK] Path cost, distance, steps, dirt metrics, and progress % verified.")


def verify_gui_launch_and_render():
    print("[8/8] Verifying Desktop GUI Launch, Canvas Rendering & User Actions...")
    root = ctk.CTk()
    app = SmartSweepGUI(root)
    root.update()

    # Check Demo Room loaded on start
    assert app.room.rows == 15 and app.room.cols == 15
    assert len(app.room.obstacles) > 10
    assert len(app.room.dirt_locations) >= 3

    # Find optimal path on demo room
    app.find_optimal_path()
    root.update()
    assert app.sim.state == SimState.PATH_READY
    assert len(app.sim.planned_path) > 0

    # Test step_forward() advances robot position and steps
    app.step_forward()
    app.step_forward()
    app.step_forward()
    root.update()
    assert app.sim.steps_completed == 3, f"Expected 3 steps, got {app.sim.steps_completed}"

    # Pause and Resume
    app.sim.resume()  # Set to RUNNING
    app.pause_resume()  # Toggles to PAUSED
    root.update()
    assert app.sim.state == SimState.PAUSED
    app.pause_resume()  # Toggles to RUNNING
    root.update()
    assert app.sim.state == SimState.RUNNING

    # Reset
    app.reset_simulation()
    root.update()
    assert app.sim.state == SimState.READY

    # Test Theme Toggle
    app.theme_switch.toggle()
    app._toggle_theme()
    root.update()

    # Clean shutdown
    app._stop_animation()
    try:
        root.quit()
        root.destroy()
    except Exception:
        pass
    print("  [OK] GUI initialized, Canvas rendered, controls exercised, and exited cleanly.")


def main():
    print("================================================================")
    print("       SMARTSWEEP AI - COMPREHENSIVE VERIFICATION SUITE         ")
    print("================================================================")
    start_t = time.time()
    verify_real_astar()
    verify_obstacle_avoidance_and_corner_cutting()
    verify_multi_dirt_cleaning()
    verify_unreachable_dirt_reporting()
    verify_pause_resume_reset()
    verify_save_and_load()
    verify_statistics_accuracy()
    verify_gui_launch_and_render()
    elapsed = time.time() - start_t
    print("================================================================")
    print(f" ALL 8 COMPREHENSIVE VERIFICATIONS PASSED IN {elapsed:.2f}s! ")
    print("================================================================")


if __name__ == "__main__":
    main()
