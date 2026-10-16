import heapq
import html
import json
import math
import random
import time
from urllib.parse import quote, urlsplit, urlunsplit

import streamlit as st
import streamlit.components.v1 as components


# ============================================================
# SmartSweep AI — Intelligent Robot Vacuum Cleaner Path Planner
# ============================================================

st.set_page_config(
    page_title="SmartSweep AI",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)


# ----------------------------- Themes -----------------------

THEMES = {
    "Dark": {
        "bg": "#0b1020",
        "panel": "#111a2e",
        "panel2": "#17223a",
        "text": "#e8eefc",
        "muted": "#94a3b8",
        "grid": "#26334d",
        "wall": "#64748b",
        "start": "#22c55e",
        "goal": "#ef4444",
        "dirt": "#f97316",
        "clean": "#0f766e",
        "robot": "#38bdf8",
        "path": "#6366f1",
        "frontier": "#334155",
        "explored": "#1e293b",
        "border": "#26334d",
    },
    "Light": {
        "bg": "#f4f7fb",
        "panel": "#ffffff",
        "panel2": "#edf2f8",
        "text": "#172033",
        "muted": "#64748b",
        "grid": "#d8e1ed",
        "wall": "#475569",
        "start": "#16a34a",
        "goal": "#dc2626",
        "dirt": "#f97316",
        "clean": "#99f6e4",
        "robot": "#0284c7",
        "path": "#6366f1",
        "frontier": "#e2e8f0",
        "explored": "#f1f5f9",
        "border": "#d8e1ed",
    },
}


# ----------------------------- Room -------------------------


def new_room(rows=12, cols=12, diagonal=False):
    """Create a new empty room."""
    rows, cols = int(rows), int(cols)

    return {
        "rows": rows,
        "cols": cols,
        "start": (0, 0),
        "goal": (rows - 1, cols - 1),
        "obstacles": set(),
        "dirt": set(),
        "cleaned": set(),
        "diagonal": bool(diagonal),
    }


def demo_room(rows=12, cols=12, diagonal=False):
    """Create a demo room sized to the selected dimensions."""
    room = new_room(rows, cols, diagonal)

    wall_a = cols // 3
    wall_b = (2 * cols) // 3

    for r in range(2, rows - 2):
        if r % 5 != 0:
            p = (r, wall_a)
            if p not in (room["start"], room["goal"]):
                room["obstacles"].add(p)

    for r in range(1, rows - 1):
        if r % 4 != 0:
            p = (r, wall_b)
            if p not in (room["start"], room["goal"]):
                room["obstacles"].add(p)

    for c in range(2, cols - 2):
        if c % 4 != 0:
            p = (rows // 2, c)
            if p not in (room["start"], room["goal"]):
                room["obstacles"].add(p)

    dirt_candidates = [
        (rows // 3, cols // 4),
        (rows // 2, cols // 2),
        (rows - 3, cols // 3),
        (rows // 3, cols - 3),
        (rows - 2, cols // 2),
        (2, cols - 3),
    ]

    for p in dirt_candidates:
        if (
            0 <= p[0] < rows
            and 0 <= p[1] < cols
            and p not in room["obstacles"]
            and p not in (room["start"], room["goal"])
        ):
            room["dirt"].add(p)

    return room


def random_room(rows, cols, density, dirt_count, diagonal=False):
    """Generate a random room using the selected settings."""
    room = new_room(rows, cols, diagonal)

    cells = [
        (r, c)
        for r in range(rows)
        for c in range(cols)
        if (r, c) not in (room["start"], room["goal"])
    ]

    random.shuffle(cells)

    dirt_count = min(max(0, int(dirt_count)), len(cells))

    obstacle_count = min(
        int(len(cells) * float(density) / 100),
        len(cells) - dirt_count,
    )

    room["obstacles"] = set(cells[:obstacle_count])
    room["dirt"] = set(
        cells[obstacle_count:obstacle_count + dirt_count]
    )

    return room


def normalize_room(data):
    """Validate and normalize an imported room JSON object."""
    if not isinstance(data, dict):
        raise ValueError("The JSON file must contain a room object.")

    rows = int(data["rows"])
    cols = int(data["cols"])

    if not (5 <= rows <= 30 and 5 <= cols <= 30):
        raise ValueError("Rows and columns must be between 5 and 30.")

    room = new_room(
        rows,
        cols,
        data.get("diagonal", data.get("allow_diagonal", False)),
    )

    def parse_cell(value):
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError("Each cell must contain two coordinates.")

        r, c = int(value[0]), int(value[1])

        if not (0 <= r < rows and 0 <= c < cols):
            raise ValueError(f"Cell {(r, c)} is outside the room.")

        return (r, c)

    room["start"] = parse_cell(data.get("start", [0, 0]))
    room["goal"] = parse_cell(data.get("goal", [rows - 1, cols - 1]))

    if room["start"] == room["goal"]:
        raise ValueError("Start and Goal must be different cells.")

    room["obstacles"] = {
        parse_cell(p) for p in data.get("obstacles", [])
    }

    room["dirt"] = {
        parse_cell(p)
        for p in data.get("dirt", data.get("dirt_locations", []))
    }

    room["cleaned"] = {
        parse_cell(p)
        for p in data.get("cleaned", data.get("cleaned_dirt", []))
    }

    room["obstacles"].discard(room["start"])
    room["obstacles"].discard(room["goal"])

    room["dirt"].discard(room["start"])
    room["dirt"].discard(room["goal"])
    room["dirt"] -= room["obstacles"]

    room["cleaned"] -= room["obstacles"]
    room["cleaned"].discard(room["start"])
    room["cleaned"].discard(room["goal"])

    return room


# ------------------------- Session state --------------------


def reset_simulation(room):
    """Reset simulation state while retaining the supplied room."""
    st.session_state.room = room
    st.session_state.robot = room["start"]
    st.session_state.heading = (0, 1)

    st.session_state.path = []
    st.session_state.path_index = 0

    st.session_state.explored = set()
    st.session_state.frontier = set()
    st.session_state.unreachable = set()

    st.session_state.route_cost = 0.0
    st.session_state.search_ms = 0.0
    st.session_state.moves = 0

    st.session_state.planned = False
    st.session_state.running = False
    st.session_state.finished = False

    st.session_state.message = (
        "Room ready. Calculate a route to begin."
    )

    st.session_state.initial_dirt = len(room["dirt"])
    st.session_state.visited_dirt = set()


def invalidate_route(message="Room edited. Calculate a new route."):
    """Clear stale route information without replacing the room."""
    room = st.session_state.room

    st.session_state.robot = room["start"]
    st.session_state.heading = (0, 1)

    st.session_state.path = []
    st.session_state.path_index = 0

    st.session_state.explored = set()
    st.session_state.frontier = set()
    st.session_state.unreachable = set()

    st.session_state.route_cost = 0.0
    st.session_state.search_ms = 0.0
    st.session_state.moves = 0

    st.session_state.planned = False
    st.session_state.running = False
    st.session_state.finished = False

    st.session_state.initial_dirt = len(room["dirt"])
    st.session_state.visited_dirt = set()
    st.session_state.message = message


if "room" not in st.session_state:
    reset_simulation(demo_room())


defaults = {
    "mode": "Smart Cleaning",
    "show_path": True,
    "show_search": True,
    "theme_name": "Dark",
    "edit_tool": "Dirt",
    "density_setting": 12,
    "dirt_setting": 6,
    "speed": 0.25,
    "rows_setting": st.session_state.room["rows"],
    "cols_setting": st.session_state.room["cols"],
    "diagonal_setting": st.session_state.room["diagonal"],
    "_sync_grid_widgets": False,
    "_last_edit_event": None,
    "_last_edit_result": "",
}

for key, value in defaults.items():
    if key not in st.session_state:
        st.session_state[key] = value


# Sync widget values before creating sidebar widgets.
if st.session_state.get("_sync_grid_widgets", False):
    active_room = st.session_state.room

    st.session_state.rows_setting = active_room["rows"]
    st.session_state.cols_setting = active_room["cols"]
    st.session_state.diagonal_setting = active_room["diagonal"]

    st.session_state._sync_grid_widgets = False


# ------------------------------ CSS -------------------------


def apply_css(colors):
    st.markdown(
        f"""
        <style>
        .stApp {{
            background: {colors["bg"]};
            color: {colors["text"]};
        }}

        [data-testid="stSidebar"] {{
            background: {colors["panel"]};
            border-right: 1px solid {colors["border"]};
        }}

        [data-testid="stHeader"] {{
            background: transparent;
        }}

        .block-container {{
            max-width: 1450px;
            padding-top: 0.65rem;
            padding-bottom: 0.75rem;
        }}

        .ss-hero {{
            background: linear-gradient(
                120deg,
                {colors["panel"]},
                {colors["panel2"]}
            );
            border: 1px solid {colors["border"]};
            border-radius: 16px;
            padding: 15px 22px;
            margin-bottom: 10px;
        }}

        .ss-hero h1 {{
            margin: 0;
            color: {colors["text"]};
            font-size: 1.8rem;
        }}

        .ss-hero p {{
            margin: 5px 0 0 0;
            color: {colors["muted"]};
            font-size: 0.95rem;
        }}

        .ss-card {{
            background: {colors["panel"]};
            border: 1px solid {colors["border"]};
            border-radius: 12px;
            padding: 10px 13px;
            margin-bottom: 8px;
        }}

        .ss-muted {{
            color: {colors["muted"]};
            font-size: 0.88rem;
        }}

        div[data-testid="stMetric"] {{
            background: {colors["panel"]};
            border: 1px solid {colors["border"]};
            padding: 8px 11px;
            border-radius: 12px;
        }}

        div[data-testid="stMetricLabel"] {{
            font-size: 0.85rem;
        }}

        div.stButton > button {{
            border-radius: 9px;
            font-weight: 600;
        }}
        </style>
        """,
        unsafe_allow_html=True,
    )


# -------------------------- A* algorithm --------------------


def heuristic(a, b, diagonal=False):
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])

    if diagonal:
        return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)

    return dr + dc


def neighbours(room, cell):
    """Return traversable neighboring cells and their movement costs."""
    rows, cols = room["rows"], room["cols"]
    r, c = cell

    directions = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
    ]

    if room["diagonal"]:
        directions.extend([
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        ])

    result = []

    for dr, dc, cost in directions:
        nr, nc = r + dr, c + dc
        nxt = (nr, nc)

        if not (0 <= nr < rows and 0 <= nc < cols):
            continue

        if nxt in room["obstacles"]:
            continue

        # Prevent diagonal movement through obstacle corners.
        if dr != 0 and dc != 0:
            if (r + dr, c) in room["obstacles"]:
                continue

            if (r, c + dc) in room["obstacles"]:
                continue

        result.append((nxt, cost))

    return result


def astar(room, start, goal):
    """Return path, explored cells, frontier cells, and path cost."""
    if start in room["obstacles"] or goal in room["obstacles"]:
        return [], set(), set(), float("inf")

    if start == goal:
        return [start], {start}, set(), 0.0

    open_heap = []
    counter = 0

    heapq.heappush(
        open_heap,
        (heuristic(start, goal, room["diagonal"]), counter, start),
    )

    came_from = {}
    g_score = {start: 0.0}
    explored = set()
    queued = {start}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        queued.discard(current)

        if current in explored:
            continue

        if current == goal:
            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()

            return path, explored | {goal}, queued, g_score[goal]

        explored.add(current)

        for nxt, move_cost in neighbours(room, current):
            tentative = g_score[current] + move_cost

            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                counter += 1

                priority = tentative + heuristic(
                    nxt,
                    goal,
                    room["diagonal"],
                )

                heapq.heappush(
                    open_heap,
                    (priority, counter, nxt),
                )

                queued.add(nxt)

    return [], explored, queued, float("inf")


def calculate_route():
    """Calculate the Smart Cleaning or Single Destination route."""
    room = st.session_state.room
    started_at = time.perf_counter()

    current = st.session_state.robot
    full_path = [current]

    explored_all = set()
    frontier_all = set()
    unreachable = set()
    total_cost = 0.0

    if st.session_state.mode == "Single Destination":
        targets = [room["goal"]]

    else:
        remaining = set(room["dirt"]) - set(room["cleaned"])

        # Keep the existing nearest-reachable-dirt strategy.
        while remaining:
            best = None

            for target in remaining:
                path, explored, frontier, cost = astar(
                    room,
                    current,
                    target,
                )

                explored_all.update(explored)
                frontier_all.update(frontier)

                if path and (best is None or cost < best[0]):
                    best = (cost, target, path)

            if best is None:
                unreachable.update(remaining)
                break

            cost, target, path = best

            total_cost += cost
            full_path.extend(path[1:])
            current = target
            remaining.remove(target)

        targets = [room["goal"]]

    for target in targets:
        if current == target:
            continue

        path, explored, frontier, cost = astar(
            room,
            current,
            target,
        )

        explored_all.update(explored)
        frontier_all.update(frontier)

        if not path:
            unreachable.add(target)
            continue

        total_cost += cost
        full_path.extend(path[1:])
        current = target

    st.session_state.search_ms = (
        time.perf_counter() - started_at
    ) * 1000

    st.session_state.path = full_path if len(full_path) > 1 else []
    st.session_state.path_index = 0

    st.session_state.explored = explored_all
    st.session_state.frontier = frontier_all
    st.session_state.unreachable = unreachable

    st.session_state.route_cost = total_cost
    st.session_state.moves = 0

    st.session_state.planned = bool(st.session_state.path)
    st.session_state.running = False
    st.session_state.finished = False
    st.session_state.visited_dirt = set()

    if st.session_state.path:
        st.session_state.message = (
            f"Route calculated: {len(st.session_state.path) - 1} moves."
        )
    elif unreachable:
        st.session_state.message = (
            "No complete route found. Check obstacles and unreachable targets."
        )
    else:
        st.session_state.message = (
            "The robot is already at its destination."
        )


def move_one_step():
    """Move one step, clean dirt if reached, and update simulation state."""
    path = st.session_state.path

    if not path:
        st.session_state.running = False
        st.session_state.message = "Calculate a route before moving."
        return

    next_index = st.session_state.path_index + 1

    if next_index >= len(path):
        st.session_state.running = False
        st.session_state.finished = True
        st.session_state.message = "Route completed."
        return

    previous = st.session_state.robot
    current = path[next_index]

    st.session_state.heading = (
        current[0] - previous[0],
        current[1] - previous[1],
    )

    st.session_state.robot = current
    st.session_state.path_index = next_index
    st.session_state.moves += 1

    room = st.session_state.room

    if current in room["dirt"]:
        room["cleaned"].add(current)
        st.session_state.visited_dirt.add(current)

    if next_index >= len(path) - 1:
        st.session_state.running = False
        st.session_state.finished = True
        st.session_state.message = "Route completed."
    else:
        st.session_state.message = (
            f"Robot moved to row {current[0]}, column {current[1]}."
        )


# ----------------------- Cell editing -----------------------


def edit_cell(r, c, tool):
    """Edit the existing room without recreating it."""
    room = st.session_state.room
    p = (r, c)

    if not (0 <= r < room["rows"] and 0 <= c < room["cols"]):
        return False, "Selected cell is outside the room."

    if tool == "Dirt":
        if p in (room["start"], room["goal"]):
            return False, "Dirt cannot be placed on Start or Goal."

        if p in room["obstacles"]:
            return False, "Dirt cannot be placed on an obstacle."

        if p in room["dirt"]:
            room["dirt"].remove(p)
            room["cleaned"].discard(p)
            return True, f"Dirt removed from {p}."

        room["dirt"].add(p)
        room["cleaned"].discard(p)
        return True, f"Dirt added at {p}."

    if tool == "Obstacle":
        if p in (room["start"], room["goal"]):
            return False, "Obstacle cannot be placed on Start or Goal."

        if p in room["obstacles"]:
            room["obstacles"].remove(p)
            return True, f"Obstacle removed from {p}."

        room["obstacles"].add(p)
        room["dirt"].discard(p)
        room["cleaned"].discard(p)
        return True, f"Obstacle added at {p}."

    if tool == "Start":
        if p == room["goal"]:
            return False, "Start cannot overlap Goal."

        if p in room["obstacles"]:
            return False, "Start cannot be placed on an obstacle."

        room["start"] = p
        room["dirt"].discard(p)
        room["cleaned"].discard(p)
        return True, f"Start moved to {p}."

    if tool == "Goal":
        if p == room["start"]:
            return False, "Goal cannot overlap Start."

        if p in room["obstacles"]:
            return False, "Goal cannot be placed on an obstacle."

        room["goal"] = p
        room["dirt"].discard(p)
        room["cleaned"].discard(p)
        return True, f"Goal moved to {p}."

    if tool == "Erase":
        if p in (room["start"], room["goal"]):
            return False, "Erase does not remove Start or Goal."

        had_object = (
            p in room["obstacles"]
            or p in room["dirt"]
            or p in room["cleaned"]
        )

        room["obstacles"].discard(p)
        room["dirt"].discard(p)
        room["cleaned"].discard(p)

        if had_object:
            return True, f"Cell {p} cleared."

        return False, f"Cell {p} was already empty."

    return False, "Unknown editing tool."


# --------------------- Click event transport ----------------


def current_page_url():
    """
    Get the absolute URL of the current Streamlit page.

    Removing the old query string ensures each cell click creates a
    fresh URL with its own cell coordinates and event identifier.
    """
    try:
        current_url = st.context.url
        if not current_url:
            return ""
        parsed = urlsplit(current_url)

        return urlunsplit(
            (
                parsed.scheme,
                parsed.netloc,
                parsed.path,
                "",
                "",
            )
        )

    except (AttributeError, TypeError, ValueError):
        # Streamlit versions without st.context.url use a root-relative
        # fallback. Current Streamlit Community Cloud supports st.context.
        return ""


def process_cell_event():
    """
    Read a click event, apply one edit, then consume the event.

    The room is mutated before query parameters are cleared.
    """
    query = st.query_params
    raw_cell = query.get("cell")
    nonce = query.get("edit_nonce")

    if raw_cell is None:
        return

    if isinstance(raw_cell, list):
        raw_cell = raw_cell[0] if raw_cell else ""

    if isinstance(nonce, list):
        nonce = nonce[0] if nonce else ""

    event_id = f"{raw_cell}|{nonce}"

    # Ignore an already processed event.
    if event_id == st.session_state.get("_last_edit_event"):
        query.clear()
        st.rerun()

    try:
        pieces = str(raw_cell).split(",")

        if len(pieces) != 2:
            raise ValueError("Expected exactly two coordinates.")

        r = int(pieces[0].strip())
        c = int(pieces[1].strip())

        room = st.session_state.room

        if not (0 <= r < room["rows"] and 0 <= c < room["cols"]):
            raise ValueError("Coordinates are outside the room.")

        # Read the currently selected sidebar tool or query parameter fallback.
        tool_param = query.get("tool")
        if tool_param:
            if isinstance(tool_param, list):
                tool_param = tool_param[0]
            if tool_param in ["Dirt", "Obstacle", "Start", "Goal", "Erase"]:
                st.session_state.edit_tool = tool_param
                selected_tool = tool_param
            else:
                selected_tool = st.session_state.get("edit_tool", "Dirt")
        else:
            selected_tool = st.session_state.get("edit_tool", "Dirt")

        # Mutate the existing room.
        changed, message = edit_cell(r, c, selected_tool)

        st.session_state._last_edit_event = event_id
        st.session_state._last_edit_result = message

        if changed:
            # invalidate_route does not create or replace the room.
            invalidate_route(message)
        else:
            st.session_state.message = message

    except (TypeError, ValueError, IndexError) as exc:
        st.session_state.message = f"Cell edit failed: {exc}"

    # The edit has been attempted and its result saved before clearing.
    query.clear()

    # One rerun renders the modified room.
    st.rerun()


# Process cell clicks before rendering the sidebar and grid.
process_cell_event()


# -------------------------- SVG renderer --------------------


def render_grid_svg(interactive=True):
    """
    Render the existing SVG grid.

    Only the main Room Map is interactive. The Live Simulation copy
    is display-only to avoid duplicate click-event sources.
    """
    room = st.session_state.room
    rows, cols = room["rows"], room["cols"]
    colors = THEMES[st.session_state.theme_name]

    largest_dimension = max(rows, cols)

    # Preserve compact cell sizing.
    cell_size = max(14, min(32, 430 / largest_dimension))
    label_size = 22

    svg_width = label_size + cols * cell_size
    svg_height = label_size + rows * cell_size

    path_cells = (
        set(st.session_state.path)
        if st.session_state.show_path
        else set()
    )

    explored = (
        st.session_state.explored
        if st.session_state.show_search
        else set()
    )

    frontier = (
        st.session_state.frontier
        if st.session_state.show_search
        else set()
    )

    robot = st.session_state.robot
    start = room["start"]
    goal = room["goal"]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'viewBox="0 0 {svg_width:.2f} {svg_height:.2f}" '
        f'width="{svg_width:.2f}" height="{svg_height:.2f}" '
        f'style="display:block;width:100%;height:auto;'
        f'max-width:{svg_width:.2f}px;margin:0 auto" '
        f'role="img" aria-label="SmartSweep room grid">'
    ]

    # Column coordinate labels.
    for c in range(cols):
        x = label_size + c * cell_size + cell_size / 2

        parts.append(
            f'<text x="{x:.2f}" y="14" text-anchor="middle" '
            f'font-size="10" fill="{colors["muted"]}" '
            f'pointer-events="none">{c}</text>'
        )

    # Row coordinate labels.
    for r in range(rows):
        y = label_size + r * cell_size + cell_size / 2 + 3

        parts.append(
            f'<text x="10" y="{y:.2f}" text-anchor="middle" '
            f'font-size="10" fill="{colors["muted"]}" '
            f'pointer-events="none">{r}</text>'
        )

    # Create one cell for every room coordinate.
    base_url = current_page_url()
    current_tool = st.session_state.get("edit_tool", "Dirt")

    for r in range(rows):
        for c in range(cols):
            p = (r, c)

            x = label_size + c * cell_size
            y = label_size + r * cell_size

            fill = colors["panel"]

            if p in explored:
                fill = colors["explored"]

            if p in frontier:
                fill = colors["frontier"]

            if p in path_cells:
                fill = colors["path"]

            if p in room["cleaned"]:
                fill = colors["clean"]

            if p in room["dirt"] and p not in room["cleaned"]:
                fill = colors["dirt"]

            if p in room["obstacles"]:
                fill = colors["wall"]

            if p == start:
                fill = colors["start"]

            if p == goal:
                fill = colors["goal"]

            title = html.escape(f"Row {r}, column {c}")

            parts.append("<g>")

            if interactive:
                # Unique identifier per click, not per animation rerun.
                cell_query = quote(f"{r},{c}", safe="")
                tool_query = quote(current_tool, safe="")
                nonce = time.time_ns()

                query_string = (
                    f"cell={cell_query}&tool={tool_query}&edit_nonce={nonce}"
                )

                if base_url:
                    href = f"{base_url}?{query_string}"
                else:
                    href = f"?{query_string}"

                # Absolute page URL avoids ambiguity about the iframe base.
                parts.append(
                    f'<a href="{html.escape(href, quote=True)}" '
                    f'target="_top" aria-label="{title}" '
                    f'onclick="return onCellClick(event, {r}, {c})" '
                    f'style="cursor:pointer">'
                )

            parts.append(
                f'<rect x="{x + 1:.2f}" y="{y + 1:.2f}" '
                f'width="{cell_size - 2:.2f}" '
                f'height="{cell_size - 2:.2f}" '
                f'rx="4" fill="{fill}" '
                f'stroke="{colors["grid"]}" stroke-width="1"/>'
            )

            symbol = ""
            symbol_color = "#ffffff"

            if p in room["obstacles"]:
                symbol = "■"
            elif p == start:
                symbol = "S"
            elif p == goal:
                symbol = "G"
            elif p in room["dirt"] and p not in room["cleaned"]:
                symbol = "●"
                symbol_color = "#7c2d12"
            elif p in room["cleaned"]:
                symbol = "✓"
                symbol_color = "#064e3b"

            if symbol:
                parts.append(
                    f'<text x="{x + cell_size / 2:.2f}" '
                    f'y="{y + cell_size / 2 + 4:.2f}" '
                    f'text-anchor="middle" '
                    f'font-size="{cell_size * 0.42:.2f}" '
                    f'font-weight="700" fill="{symbol_color}" '
                    f'pointer-events="none">{symbol}</text>'
                )

            if interactive:
                parts.append("</a>")

            parts.append("</g>")

    # Draw the robot on top of the grid.
    rx = label_size + robot[1] * cell_size + cell_size / 2
    ry = label_size + robot[0] * cell_size + cell_size / 2
    robot_radius = cell_size * 0.31

    parts.append(
        '<g pointer-events="none">'
        f'<circle cx="{rx:.2f}" cy="{ry:.2f}" '
        f'r="{robot_radius:.2f}" '
        f'fill="{colors["robot"]}" '
        f'stroke="#ffffff" stroke-width="1.6"/>'
    )

    hr, hc = st.session_state.heading

    if hr == 0 and hc == 0:
        hr, hc = 0, 1

    dx = hc * robot_radius * 0.65
    dy = hr * robot_radius * 0.65

    parts.append(
        f'<line x1="{rx:.2f}" y1="{ry:.2f}" '
        f'x2="{rx + dx:.2f}" y2="{ry + dy:.2f}" '
        f'stroke="#082f49" stroke-width="2.5" '
        f'stroke-linecap="round"/>'
        '</g></svg>'
    )

    if interactive:
        parts.append(
            f"""<script>
function onCellClick(e, r, c) {{
    if (e) {{
        if (e.preventDefault) e.preventDefault();
        if (e.stopPropagation) e.stopPropagation();
    }}
    try {{
        const nonce = Date.now();
        const pWin = window.parent;
        const path = (pWin && pWin.location && pWin.location.pathname) ? pWin.location.pathname : window.location.pathname;
        const url = path + '?cell=' + encodeURIComponent(r + ',' + c) + '&tool=' + encodeURIComponent('{current_tool}') + '&edit_nonce=' + nonce;
        if (pWin && pWin.history && pWin.history.pushState && pWin.dispatchEvent) {{
            pWin.history.pushState({{}}, '', url);
            pWin.dispatchEvent(new PopStateEvent('popstate'));
            return false;
        }}
    }} catch (err) {{
        try {{
            const nonce = Date.now();
            const fallbackUrl = window.parent.location.pathname + '?cell=' + encodeURIComponent(r + ',' + c) + '&tool=' + encodeURIComponent('{current_tool}') + '&edit_nonce=' + nonce;
            window.parent.location.href = fallbackUrl;
            return false;
        }} catch (e2) {{}}
    }}
    return false;
}}
</script>"""
        )

    return "".join(parts), int(svg_height + 18)


# --------------------------- Header -------------------------


theme = THEMES[st.session_state.theme_name]
apply_css(theme)

st.markdown(
    """
    <div class="ss-hero">
        <h1>🤖 SmartSweep AI</h1>
        <p>
            Intelligent Robot Vacuum Cleaner Path Planner ·
            A* Search · Smart Cleaning
        </p>
    </div>
    """,
    unsafe_allow_html=True,
)


# --------------------------- Sidebar ------------------------


with st.sidebar:
    st.markdown("## ⚙️ Controls")
    st.markdown("### Room presets")

    preset_left, preset_right = st.columns(2)

    with preset_left:
        if st.button("Demo Room", use_container_width=True):
            rows = st.session_state.rows_setting
            cols = st.session_state.cols_setting
            diagonal = st.session_state.diagonal_setting

            reset_simulation(demo_room(rows, cols, diagonal))
            st.session_state._sync_grid_widgets = True
            st.rerun()

    with preset_right:
        if st.button("Random Room", use_container_width=True):
            rows = st.session_state.rows_setting
            cols = st.session_state.cols_setting
            density = st.session_state.density_setting
            dirt_count = st.session_state.dirt_setting
            diagonal = st.session_state.diagonal_setting

            reset_simulation(
                random_room(rows, cols, density, dirt_count, diagonal)
            )

            st.session_state._sync_grid_widgets = True
            st.rerun()

    if st.button("New Empty Room", use_container_width=True):
        reset_simulation(
            new_room(
                st.session_state.rows_setting,
                st.session_state.cols_setting,
                st.session_state.diagonal_setting,
            )
        )

        st.session_state._sync_grid_widgets = True
        st.rerun()

    st.markdown("### Grid size")

    rows = st.slider(
        "Rows",
        min_value=5,
        max_value=30,
        key="rows_setting",
    )

    cols = st.slider(
        "Columns",
        min_value=5,
        max_value=30,
        key="cols_setting",
    )

    diagonal = st.checkbox(
        "Allow diagonal movement",
        key="diagonal_setting",
    )

    if st.button("Apply Grid Settings", use_container_width=True):
        reset_simulation(new_room(rows, cols, diagonal))
        st.session_state.message = (
            f"Grid set to {rows}×{cols}. New empty room ready."
        )
        st.rerun()

    st.markdown("### Random Room settings")

    density = st.slider(
        "Obstacle density (%)",
        min_value=0,
        max_value=35,
        key="density_setting",
    )

    dirt_count = st.slider(
        "Dirt spots",
        min_value=0,
        max_value=25,
        key="dirt_setting",
    )

    st.caption(
        "Used when clicking 'Random Room' preset."
    )

    st.markdown("### Cell editing")

    st.radio(
        "Select tool",
        options=["Dirt", "Obstacle", "Start", "Goal", "Erase"],
        key="edit_tool",
    )

    st.caption("Select a tool, then click a cell in the grid.")

    st.markdown("### Planning options")

    st.selectbox(
        "Planning mode",
        options=["Smart Cleaning", "Single Destination"],
        key="mode",
    )

    st.checkbox("Show planned path", key="show_path")
    st.checkbox("Show search visualization", key="show_search")

    st.selectbox(
        "Appearance",
        options=["Dark", "Light"],
        key="theme_name",
    )

    st.slider(
        "Animation delay (seconds)",
        min_value=0.05,
        max_value=1.0,
        step=0.05,
        key="speed",
    )

    st.markdown("### Robot controls")

    if st.button(
        "Calculate Route",
        type="primary",
        use_container_width=True,
    ):
        calculate_route()
        st.rerun()

    run_col, pause_col = st.columns(2)

    with run_col:
        if st.button("▶ Run / Resume", use_container_width=True):
            if st.session_state.path:
                st.session_state.running = True
                st.session_state.finished = False
                st.session_state.message = "Robot is moving."
            else:
                st.session_state.message = (
                    "Calculate a route before running the robot."
                )

            st.rerun()

    with pause_col:
        if st.button("⏸ Pause", use_container_width=True):
            st.session_state.running = False
            st.session_state.message = "Simulation paused."
            st.rerun()

    step_col, reset_col = st.columns(2)

    with step_col:
        if st.button("Step Forward", use_container_width=True):
            st.session_state.running = False
            move_one_step()
            st.rerun()

    with reset_col:
        if st.button("Reset Simulation", use_container_width=True):
            room = st.session_state.room
            room["cleaned"] = set()

            reset_simulation(room)
            st.rerun()

    st.markdown("### Import / export")

    uploaded_json = st.file_uploader(
        "Import room JSON",
        type=["json"],
        key="room_json_upload",
    )

    if uploaded_json is not None:
        if st.button("Load JSON Room", use_container_width=True):
            try:
                data = json.load(uploaded_json)
                loaded_room = normalize_room(data)

                reset_simulation(loaded_room)

                st.session_state.rows_setting = loaded_room["rows"]
                st.session_state.cols_setting = loaded_room["cols"]
                st.session_state.diagonal_setting = loaded_room["diagonal"]
                st.session_state._sync_grid_widgets = False
                st.session_state.message = "Room loaded successfully."

                st.rerun()

            except (
                ValueError,
                TypeError,
                KeyError,
                json.JSONDecodeError,
            ) as exc:
                st.error(f"Could not load room: {exc}")

    room_export = st.session_state.room

    export_data = {
        "version": "1.0",
        "rows": room_export["rows"],
        "cols": room_export["cols"],
        "start": list(room_export["start"]),
        "goal": list(room_export["goal"]),
        "diagonal": room_export["diagonal"],
        "allow_diagonal": room_export["diagonal"],
        "obstacles": [
            list(p) for p in sorted(room_export["obstacles"])
        ],
        "dirt": [
            list(p) for p in sorted(room_export["dirt"])
        ],
        "cleaned": [
            list(p) for p in sorted(room_export["cleaned"])
        ],
    }

    st.download_button(
        "Download Room JSON",
        data=json.dumps(export_data, indent=2),
        file_name="smartsweep_room.json",
        mime="application/json",
        use_container_width=True,
    )


# ------------------------ Responsive grid -------------------


room = st.session_state.room

grid_title_col, status_col = st.columns([3, 1])

with grid_title_col:
    st.markdown("### Room map")
    st.caption(
        f'{room["rows"]} rows × {room["cols"]} columns · '
        f'{"8-directional" if room["diagonal"] else "4-directional"} movement'
    )

with status_col:
    if st.session_state.running:
        st.markdown("**Status:** Running")
    elif st.session_state.finished:
        st.markdown("**Status:** Completed")
    elif st.session_state.planned:
        st.markdown("**Status:** Route ready")
    else:
        st.markdown("**Status:** Ready")

st.markdown(
    """
    <div class="ss-card">
        <div class="ss-muted">
            <b>S</b> Start &nbsp;·&nbsp;
            <b>G</b> Goal &nbsp;·&nbsp;
            <b>●</b> Dirt &nbsp;·&nbsp;
            <b>✓</b> Cleaned &nbsp;·&nbsp;
            <b>■</b> Obstacle &nbsp;·&nbsp;
            <b>Blue circle</b> Robot
        </div>
    </div>
    """,
    unsafe_allow_html=True,
)

# The main Room Map is the only interactive grid.
svg, svg_height = render_grid_svg(interactive=True)

components.html(
    svg,
    height=svg_height,
    scrolling=False,
)


# --------------------- Live simulation ----------------------


@st.fragment(run_every=0.25)
def animation_tick():
    """Update the live simulation and metrics while running."""
    if st.session_state.running:
        move_one_step()

    live_room = st.session_state.room

    st.markdown("### Live simulation")

    # Display-only SVG: it cannot create competing edit events.
    live_svg, live_height = render_grid_svg(interactive=False)

    components.html(
        live_svg,
        height=live_height,
        scrolling=False,
    )

    total_dirt = len(live_room["dirt"])
    cleaned_count = len(live_room["cleaned"] & live_room["dirt"])

    if total_dirt == 0:
        progress = 100
    else:
        progress = int(100 * cleaned_count / total_dirt)

    m1, m2, m3, m4 = st.columns(4)

    m1.metric("Moves", st.session_state.moves)
    m2.metric("Dirt cleaned", f"{cleaned_count}/{total_dirt}")
    m3.metric("Route cost", f'{st.session_state.route_cost:.2f}')
    m4.metric("Search time", f'{st.session_state.search_ms:.2f} ms')

    st.progress(
        progress / 100,
        text=f"Cleaning progress: {progress}%",
    )

    st.info(st.session_state.message)

    if st.session_state.unreachable:
        st.warning(
            "Unreachable targets: "
            + ", ".join(
                str(p) for p in sorted(st.session_state.unreachable)
            )
        )


animation_tick()


# ----------------------- Route information ------------------


left_col, right_col = st.columns(2)

with left_col:
    st.markdown("### Route information")

    if st.session_state.path:
        st.write(f"**Route cells:** {len(st.session_state.path)}")

        st.write(
            "**Remaining steps:** "
            f"{max(0, len(st.session_state.path) - 1 - st.session_state.path_index)}"
        )

        st.write(f"**Robot position:** {st.session_state.robot}")

        with st.expander("View route coordinates"):
            st.code(
                " → ".join(
                    f"({r}, {c})"
                    for r, c in st.session_state.path
                ),
                language="text",
            )

    else:
        st.write("No route calculated yet.")
        st.caption("Choose a planning mode and click Calculate Route.")

with right_col:
    st.markdown("### Algorithm insights")

    st.write("**Algorithm:** A* Search")

    st.write(
        "**Heuristic:** Manhattan distance for four-directional movement; "
        "octile distance for diagonal movement."
    )

    st.write(
        "**Smart Cleaning:** Visits reachable dirt spots using a "
        "nearest-reachable-target strategy, then heads toward the Goal."
    )

    st.write(f'**Explored cells:** {len(st.session_state.explored)}')
    st.write(f'**Frontier cells:** {len(st.session_state.frontier)}')


with st.expander("How to use SmartSweep AI"):
    st.markdown(
        """
        1. Choose Demo Room, Random Room, or New Empty Room.
        2. Set rows and columns, then click Apply Grid Settings.
        3. Select Dirt, Obstacle, Start, Goal, or Erase.
        4. Click a cell in the main Room Map; edits are retained.
        5. Select Smart Cleaning or Single Destination.
        6. Click Calculate Route.
        7. Use Run / Resume, Pause, or Step Forward.
        8. Export the room to JSON or load a saved room.
        """
    )

st.caption(
    "SmartSweep AI · A* Pathfinding · Interactive Cleaning Robot Simulation"
)