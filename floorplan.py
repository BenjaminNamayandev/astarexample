#!/usr/bin/env python3
"""
Example: Pathfinding from an "outside" location to a "room number" inside the walls.
1) Read DXF -> parse walls (LINE, LWPOLYLINE, ARC) into Shapely
2) Build a 2D occupancy grid
3) User provides start coords + a room number
4) Script looks up the room's center coords
5) Run A* from start to room center
6) Save a PNG visualization of the grid + path.
"""

import sys
import math
import ezdxf
import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import LineString, Point
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid as PathGrid
from pathfinding.finder.a_star import AStarFinder

# ------------------------------------------------------------------
# CONFIG
# ------------------------------------------------------------------
CELL_SIZE = 0.5  # 50cm per grid cell (adjust as needed!)
# Example: We might have a dictionary mapping room numbers -> (x,y) for the center
ROOM_COORDS = {
    "101": (30.0, 20.0),
    "102": (35.0, 25.0),
    # add more rooms as needed
}
# If we can't find the room number, the script will exit.

# ------------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------------

def arc_to_linestring(arc_entity, num_segments=16):
    """Convert a DXF ARC to a rough LineString by sampling points along the arc."""
    center = arc_entity.dxf.center
    radius = arc_entity.dxf.radius
    start_angle = arc_entity.dxf.start_angle
    end_angle = arc_entity.dxf.end_angle

    points = []
    angle_step = (end_angle - start_angle) / num_segments
    for i in range(num_segments + 1):
        angle_deg = start_angle + i * angle_step
        angle_rad = math.radians(angle_deg)
        px = center[0] + radius * math.cos(angle_rad)
        py = center[1] + radius * math.sin(angle_rad)
        points.append((px, py))

    return LineString(points)

def entity_to_shapely(e):
    """Convert DXF LINE, LWPOLYLINE, or ARC to Shapely geometry."""
    if e.dxftype() == "LINE":
        start = e.dxf.start
        end   = e.dxf.end
        return LineString([start, end])
    elif e.dxftype() == "LWPOLYLINE":
        pts = [(p[0], p[1]) for p in e.lwpoints]
        return LineString(pts)
    elif e.dxftype() == "ARC":
        return arc_to_linestring(e)
    # Skip other types
    return None

def compute_shapely_bounds(geoms):
    """Compute overall bounding box from Shapely geometries."""
    min_x = float('inf')
    min_y = float('inf')
    max_x = float('-inf')
    max_y = float('-inf')

    for g in geoms:
        if g.is_empty:
            continue
        gxmin, gymin, gxmax, gymax = g.bounds
        min_x = min(min_x, gxmin)
        min_y = min(min_y, gymin)
        max_x = max(max_x, gxmax)
        max_y = max(max_y, gymax)

    return (min_x, min_y, max_x, max_y)

def world_to_grid(x, y, xmin, ymin, cell_size):
    """Convert real-world coords -> grid indices."""
    col = int((x - xmin) // cell_size)
    row = int((y - ymin) // cell_size)
    return (row, col)

def grid_to_world(row, col, xmin, ymin, cell_size):
    """Convert grid indices -> real-world coords."""
    wx = xmin + (col + 0.5) * cell_size
    wy = ymin + (row + 0.5) * cell_size
    return (wx, wy)

# Visualization helper
def visualize_grid_and_path(grid, path, out_png="path_result.png"):
    """
    grid: 2D numpy array (0=walkable, 1=blocked)
    path: list of (x=col, y=row) tuples
    Saves a PNG with the occupancy grid (white=walkable, black=blocked),
    and the path in red.
    """
    h, w = grid.shape
    plt.figure(figsize=(10, 10))
    plt.title("Occupancy Grid + A* Path")
    # Show blocked=1 as black, free=0 as white => use a gray_r colormap
    plt.imshow(grid, origin="lower", cmap="gray_r")

    if path:
        px = [p[0] for p in path]  # col
        py = [p[1] for p in path]  # row
        plt.plot(px, py, color="red", linewidth=2)

    plt.savefig(out_png, dpi=150)
    print(f"Saved visualization to '{out_png}'")
    # If you want an interactive window:
    # plt.show()
    plt.close()


# ------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------

def main():
    """
    Usage:
       python path_room.py <DXF_FILE> <start_x> <start_y> <room_number>

    Example:
       python path_room.py floorplan.dxf  10 5  101
    """
    if len(sys.argv) < 5:
        print("Usage: python path_room.py <dxf_file> <start_x> <start_y> <room_number>")
        sys.exit(1)

    dxf_file = sys.argv[1]
    start_x  = float(sys.argv[2])
    start_y  = float(sys.argv[3])
    room_num = sys.argv[4]

    if room_num not in ROOM_COORDS:
        print(f"Unknown room number '{room_num}'. Please update ROOM_COORDS dictionary.")
        sys.exit(1)
    # Look up room center
    room_x, room_y = ROOM_COORDS[room_num]

    print(f"Loading DXF: {dxf_file}")
    print(f"Start coords: ({start_x}, {start_y}), Destination Room#{room_num}: ({room_x}, {room_y})")

    # Read DXF
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()

    # Gather line/polylines/arcs
    raw_ents = list(msp.query("LINE LWPOLYLINE ARC"))
    if not raw_ents:
        print("No geometry found (LINE/LWPOLYLINE/ARC).")
        sys.exit(1)

    # Convert to shapely
    shapely_walls = []
    for e in raw_ents:
        geom = entity_to_shapely(e)
        if geom and not geom.is_empty:
            shapely_walls.append(geom)

    if not shapely_walls:
        print("No valid shapely geometry.")
        sys.exit(1)

    # Compute bounding box
    xmin, ymin, xmax, ymax = compute_shapely_bounds(shapely_walls)
    print(f"Bounding box => xmin={xmin}, ymin={ymin}, xmax={xmax}, ymax={ymax}")

    # Create occupancy grid
    width  = int((xmax - xmin)/CELL_SIZE) + 1
    height = int((ymax - ymin)/CELL_SIZE) + 1
    print(f"Creating grid {width}x{height}, cell_size={CELL_SIZE}")

    grid = np.zeros((height, width), dtype=np.int8)

    half_cell = CELL_SIZE * 0.5
    # Mark blocked cells
    for row in range(height):
        for col in range(width):
            cx = xmin + (col + 0.5)*CELL_SIZE
            cy = ymin + (row + 0.5)*CELL_SIZE
            pt = Point(cx, cy)
            blocked = False
            for wall in shapely_walls:
                # If the center of this cell is closer than half_cell => blocked
                if wall.distance(pt) < half_cell:
                    blocked = True
                    break
            if blocked:
                grid[row, col] = 1

    # Convert start/dest to grid coords
    start_r, start_c = world_to_grid(start_x, start_y, xmin, ymin, CELL_SIZE)
    dest_r,  dest_c  = world_to_grid(room_x,  room_y,  xmin, ymin, CELL_SIZE)

    # Bounds check
    if not (0 <= start_r < height and 0 <= start_c < width):
        print("Start point is outside the grid. Aborting.")
        sys.exit(1)
    if not (0 <= dest_r < height and 0 <= dest_c < width):
        print("Destination point is outside the grid. Aborting.")
        sys.exit(1)
    if grid[start_r, start_c] == 1:
        print("Start cell is blocked!")
        sys.exit(1)
    if grid[dest_r, dest_c] == 1:
        print("Destination cell is blocked!")
        sys.exit(1)

    print(f"Start grid coords: (row={start_r}, col={start_c})")
    print(f"Dest grid coords : (row={dest_r}, col={dest_c})")

    # Create the pathfinding grid
    path_grid = PathGrid(matrix=grid.tolist())
    start_node = path_grid.node(start_c, start_r)  # note x=col, y=row
    end_node   = path_grid.node(dest_c,  dest_r)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start_node, end_node, path_grid)

    if not path:
        print("No path found!")
        sys.exit(0)

    print(f"A* runs: {runs}, path length: {len(path)}")
    print("Path (grid coords):", path)

    # Optionally convert path to real-world coords for debugging
    real_path = []
    for (colx, rowy) in path:
        wx, wy = grid_to_world(rowy, colx, xmin, ymin, CELL_SIZE)
        real_path.append((wx, wy))
    print("Path (world coords):")
    for (wx, wy) in real_path:
        print(f"  ({wx:.2f}, {wy:.2f})")

    # Finally, visualize
    visualize_grid_and_path(grid, path, out_png="floorplan_path.png")
    print("Done.")

if __name__ == "__main__":
    main()
