import json
from pathlib import Path
from typing import List, Optional

SIZE = 7
grid: List[List[int]] = []  # will hold the loaded 7x7 bitmask grid

def load_labyrinth(path: Optional[str] = None) -> List[List[int]]:
    """
    Load a SIZE x SIZE JSON array of integers (bitmasks) into the module-level `grid`.
    If path is None the function looks for 'labyrinth.json' next to this file.
    """
    p = Path(path) if path else Path(__file__).with_name("labyrinth.json")
    data = json.loads(p.read_text(encoding="utf-8"))
    if not (isinstance(data, list) and len(data) == SIZE and
            all(isinstance(row, list) and len(row) == SIZE for row in data)):
        raise ValueError(f"JSON must be a {SIZE}x{SIZE} array")
    g = [[int(cell) for cell in row] for row in data]
    global grid
    grid = g
    return grid

if __name__ == "__main__":
    grid = load_labyrinth()
    print(grid)