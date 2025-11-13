"""A* pathfinding for the 7x7 labyrinth bitmask grid.

Grid format: grid[y][x] where each cell is an int bitmask with bits:
  N=1, E=2, S=4, W=8 (1 means wall present on that side).

Functions:
  astar(grid, start, goal) -> List[(x,y)] | None
  solve_ordered(grid, start, end, points) -> List[(x,y)] | None

The solver uses 4-neighbour movement (up/right/down/left) with cost=1
and Manhattan heuristic.

Example:
  path = solve_ordered(grid, (1,1), (5,3), [(2,4),(4,2)])
"""
from heapq import heappush, heappop
from typing import List, Tuple, Optional, Dict

N, E, S, W = 1, 2, 4, 8
DIRS = [ (0, -1, N), (1, 0, E), (0, 1, S), (-1, 0, W) ]

Coord = Tuple[int, int]

def in_bounds(x: int, y: int, size: int = 7) -> bool:
    return 0 <= x < size and 0 <= y < size

def neighbors_from(grid: List[List[int]], x: int, y: int) -> List[Coord]:
    """Return list of neighbor coords reachable from (x,y).

    A move is allowed if there is no wall on the current cell in that
    direction and the neighbour is in bounds. We don't require checking the
    neighbour's opposite wall because well-formed grids keep both in sync,
    but to be robust we also check the neighbour doesn't have the opposite
    wall set.
    """
    size = len(grid)
    res: List[Coord] = []
    cur = grid[y][x]
    for dx, dy, bit in DIRS:
        nx, ny = x + dx, y + dy
        if not in_bounds(nx, ny, size):
            continue
        # if current cell has wall in this direction, cannot go
        if cur & bit:
            continue
        # additionally check neighbour's opposite wall for robustness
        opp = {N: S, E: W, S: N, W: E}[bit]
        if grid[ny][nx] & opp:
            # neighbor blocks the way
            continue
        res.append((nx, ny))
    return res

def manhattan(a: Coord, b: Coord) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct(came_from: Dict[Coord, Coord], current: Coord) -> List[Coord]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def astar(grid: List[List[int]], start: Coord, goal: Coord) -> Optional[List[Coord]]:
    """Run A* from start to goal on the labyrinth grid.

    Returns a list of coordinates (x,y) including start and goal, or None if
    no path exists.
    """
    if start == goal:
        return [start]
    size = len(grid)
    open_set = []  # heap of (f_score, g_score, coord)
    g_score: Dict[Coord, int] = {start: 0}
    f0 = manhattan(start, goal)
    heappush(open_set, (f0, 0, start))
    came_from: Dict[Coord, Coord] = {}

    closed = set()

    while open_set:
        f, g, current = heappop(open_set)
        if current in closed:
            continue
        if current == goal:
            return reconstruct(came_from, current)
        closed.add(current)

        x, y = current
        for nb in neighbors_from(grid, x, y):
            if nb in closed:
                continue
            tentative_g = g + 1
            if tentative_g < g_score.get(nb, 10**9):
                came_from[nb] = current
                g_score[nb] = tentative_g
                fscore = tentative_g + manhattan(nb, goal)
                heappush(open_set, (fscore, tentative_g, nb))

    return None

def solve_ordered(grid: List[List[int]], start: Coord, end: Coord, points: List[Coord]) -> Optional[List[Coord]]:
    """Solve path that starts at `start`, visits every point in `points` in order,
    and ends at `end`.

    Returns the full concatenated path (list of coords) or None if any segment
    is unsolvable.
    """
    seq = [start] + list(points) + [end]
    full_path: List[Coord] = []
    for a, b in zip(seq, seq[1:]):
        segment = astar(grid, a, b)
        if segment is None:
            return None
        if full_path:
            # avoid duplicating the connecting node
            full_path.extend(segment[1:])
        else:
            full_path.extend(segment)
    return full_path

if __name__ == "__main__":
    # quick smoke test using labyrinth.json if present next to this file
    import json
    from pathlib import Path
    p = Path(__file__).with_name("labyrinth.json")
    if p.exists():
        data = json.loads(p.read_text(encoding="utf-8"))
        grid = [[int(x) for x in row] for row in data]
        start = (1,1)
        end = (5,3)
        points = []
        path = solve_ordered(grid, start, end, points)
        print("Path:", path)
    else:
        print("No labyrinth.json found for smoke test.")
