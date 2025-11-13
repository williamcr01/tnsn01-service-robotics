import sys
import json
import pygame
from pathlib import Path
from labyrinth import load_labyrinth, SIZE
from a_star import solve_ordered

# wall bits (must match your JSON scheme)
N, E, S, W = 1, 2, 4, 8
OPPOSITE = {N: S, E: W, S: N, W: E}

def draw_labyrinth(screen, grid, cell_size=64, margin=20, wall_color=(0,0,0), wall_thick=4, start_pos=None, end_pos=None, points=None, path=None, font=None):
    """Draw walls and optional start/end markers.

    start_pos and end_pos are tuples (x,y) or None.
    """
    size = len(grid)
    ox = oy = margin
    screen.fill((255,255,255))
    for y in range(size):
        for x in range(size):
            cx = ox + x * cell_size
            cy = oy + y * cell_size
            v = grid[y][x]
            if v & N:
                pygame.draw.line(screen, wall_color, (cx, cy), (cx + cell_size, cy), wall_thick)
            if v & E:
                pygame.draw.line(screen, wall_color, (cx + cell_size, cy), (cx + cell_size, cy + cell_size), wall_thick)
            if v & S:
                pygame.draw.line(screen, wall_color, (cx, cy + cell_size), (cx + cell_size, cy + cell_size), wall_thick)
            if v & W:
                pygame.draw.line(screen, wall_color, (cx, cy), (cx, cy + cell_size), wall_thick)

    # draw start/end markers on top
    marker_r = int(cell_size * 0.28)
    # if start and end are the same cell, draw two smaller markers side-by-side
    if start_pos is not None and end_pos is not None and start_pos == end_pos:
        sx, sy = start_pos
        cx = ox + sx * cell_size + cell_size // 2
        cy = oy + sy * cell_size + cell_size // 2
        offset = max(10, int(cell_size * 0.2))
        small_r = max(4, int(marker_r * 0.75))
        left_center = (cx - offset, cy)
        right_center = (cx + offset, cy)
        pygame.draw.circle(screen, (0, 180, 0), left_center, small_r)
        pygame.draw.circle(screen, (200, 0, 0), right_center, small_r)
        if font is not None:
            txt = font.render(f"{sx},{sy}", True, (0, 80, 80))
            tr = txt.get_rect()
            tr.centerx = cx
            tr.top = cy + small_r + 4
            screen.blit(txt, tr)
    else:
        if start_pos is not None:
            sx, sy = start_pos
            scx = ox + sx * cell_size + cell_size // 2
            scy = oy + sy * cell_size + cell_size // 2
            pygame.draw.circle(screen, (0, 180, 0), (scx, scy), marker_r)
            # draw coordinates below the marker if font is provided
            if font is not None:
                txt = font.render(f"{sx},{sy}", True, (0, 120, 0))
                tr = txt.get_rect()
                tr.centerx = scx
                tr.top = scy + marker_r + 4
                screen.blit(txt, tr)
        if end_pos is not None:
            ex, ey = end_pos
            ecx = ox + ex * cell_size + cell_size // 2
            ecy = oy + ey * cell_size + cell_size // 2
            pygame.draw.circle(screen, (200, 0, 0), (ecx, ecy), marker_r)
            if font is not None:
                txt = font.render(f"{ex},{ey}", True, (160, 0, 0))
                tr = txt.get_rect()
                tr.centerx = ecx
                tr.top = ecy + marker_r + 4
                screen.blit(txt, tr)
    # draw stop points (ordered list of (x,y)) if font provided
    if font is not None and points:
        for idx, (px, py) in enumerate(points, start=1):
            pcx = ox + px * cell_size + cell_size // 2
            pcy = oy + py * cell_size + cell_size // 2
            pygame.draw.circle(screen, (0, 120, 200), (pcx, pcy), max(6, marker_r - 4))
            # label pN
            label = f"p{idx} {px},{py}"
            txt = font.render(label, True, (0, 80, 140))
            tr = txt.get_rect()
            tr.centerx = pcx
            tr.top = pcy + marker_r + 4
            screen.blit(txt, tr)

    # draw path if provided (sequence of coords)
    if path:
        # draw a polyline between cell centers
        pts = []
        for (px, py) in path:
            cx = ox + px * cell_size + cell_size // 2
            cy = oy + py * cell_size + cell_size // 2
            pts.append((cx, cy))
        if len(pts) >= 2:
            pygame.draw.lines(screen, (255, 128, 0), False, pts, max(3, cell_size // 12))

# UI: simple save button
BUTTON_W = 80
BUTTON_H = 28

def draw_ui(screen, font, save_rect, load_rect, reset_rect, start_rect, end_rect, status_msg, status_timer, start_mode, end_mode, start_pos, end_pos, point_mode=False):
    """Draw Save, Load, Start, End, Point and Solve buttons and a small status indicator.

    status_msg is a short string (eg. 'Saved' or 'Loaded') and status_timer is a frame count.
    Returns (point_rect, solve_rect) for event handling.
    """
    # background area for UI (light)
    ui_x = save_rect.left - 8
    ui_w = (end_rect.right - ui_x) + 8
    pygame.draw.rect(screen, (245, 245, 245), (ui_x, 0, ui_w, screen.get_height()))

    # Save button
    pygame.draw.rect(screen, (220,220,220), save_rect)
    pygame.draw.rect(screen, (0,0,0), save_rect, 2)
    txt = font.render("Save", True, (0,0,0))
    txt_rect = txt.get_rect(center=save_rect.center)
    screen.blit(txt, txt_rect)

    # Load button
    pygame.draw.rect(screen, (220,220,220), load_rect)
    pygame.draw.rect(screen, (0,0,0), load_rect, 2)
    txt = font.render("Load", True, (0,0,0))
    txt_rect = txt.get_rect(center=load_rect.center)
    screen.blit(txt, txt_rect)

    # Reset button (below Load)
    pygame.draw.rect(screen, (240,240,240), reset_rect)
    pygame.draw.rect(screen, (0,0,0), reset_rect, 2)
    txt = font.render("Reset", True, (0,0,0))
    txt_rect = txt.get_rect(center=reset_rect.center)
    screen.blit(txt, txt_rect)

    # Start button
    col = (200, 255, 200) if start_mode else (240,240,240)
    pygame.draw.rect(screen, col, start_rect)
    pygame.draw.rect(screen, (0,0,0), start_rect, 2)
    txt = font.render("Start", True, (0,0,0))
    txt_rect = txt.get_rect(center=start_rect.center)
    screen.blit(txt, txt_rect)

    # End button
    col = (255, 200, 200) if end_mode else (240,240,240)
    pygame.draw.rect(screen, col, end_rect)
    pygame.draw.rect(screen, (0,0,0), end_rect, 2)
    txt = font.render("End", True, (0,0,0))
    txt_rect = txt.get_rect(center=end_rect.center)
    screen.blit(txt, txt_rect)

    # Point button (add/remove stop points)
    # place it below End button
    point_rect = pygame.Rect(end_rect.left, end_rect.bottom + 8, BUTTON_W, BUTTON_H)
    col = (200, 230, 255) if point_mode else (240,240,240)
    pygame.draw.rect(screen, col, point_rect)
    pygame.draw.rect(screen, (0,0,0), point_rect, 2)
    txt = font.render("Point", True, (0,0,0))
    txt_rect = txt.get_rect(center=point_rect.center)
    screen.blit(txt, txt_rect)

    # Solve button (place it below the Point button)
    solve_rect = pygame.Rect(point_rect.left, point_rect.bottom + 8, BUTTON_W, BUTTON_H)
    pygame.draw.rect(screen, (240, 240, 200), solve_rect)
    pygame.draw.rect(screen, (0,0,0), solve_rect, 2)
    txt = font.render("Solve", True, (0,0,0))
    txt_rect = txt.get_rect(center=solve_rect.center)
    screen.blit(txt, txt_rect)

    # status indicator (Saved/Loaded/etc.)
    if status_timer > 0 and status_msg:
        # choose color for common messages
        msg_text = str(status_msg)
        color = (0,128,0) if msg_text.lower().startswith("saved") else (0, 80, 160)
        msg = font.render(msg_text, True, color)
        mr = msg.get_rect()
        mr.midleft = (save_rect.left - 8 - mr.width//2, save_rect.centery)
        screen.blit(msg, mr)

    # legend moved to be drawn under markers in the grid (draw_labyrinth)
    # return rects so caller can use them for event handling
    return point_rect, solve_rect

def cell_from_pos(px: int, py: int, cell_size: int, margin: int):
    x = (px - margin) // cell_size
    y = (py - margin) // cell_size
    if 0 <= x < SIZE and 0 <= y < SIZE:
        local_x = (px - margin) - x * cell_size
        local_y = (py - margin) - y * cell_size
        return int(x), int(y), local_x, local_y
    return None

def edge_direction_from_local(local_x: int, local_y: int, cell_size: int):
    # choose the nearest edge
    dN = local_y
    dS = cell_size - local_y
    dW = local_x
    dE = cell_size - local_x
    m = min(dN, dS, dW, dE)
    if m == dN:
        return N
    if m == dS:
        return S
    if m == dW:
        return W
    return E

def toggle_wall_between(grid, x, y, direction):
    # toggle wall on (x,y) and the opposite on neighbor if neighbor exists
    if not (0 <= x < SIZE and 0 <= y < SIZE):
        return
    nx = x + {N:0, S:0, W:-1, E:1}[direction]
    ny = y + {N:-1, S:1, W:0, E:0}[direction]
    # toggle current
    if grid[y][x] & direction:
        grid[y][x] &= ~direction
    else:
        grid[y][x] |= direction
    # toggle neighbor opposite if in bounds
    if 0 <= nx < SIZE and 0 <= ny < SIZE:
        opp = OPPOSITE[direction]
        if grid[ny][nx] & opp:
            grid[ny][nx] &= ~opp
        else:
            grid[ny][nx] |= opp

def remove_wall_between_cells(grid, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 1 and dy == 0:
        grid[y1][x1] &= ~E; grid[y2][x2] &= ~W
    elif dx == -1 and dy == 0:
        grid[y1][x1] &= ~W; grid[y2][x2] &= ~E
    elif dx == 0 and dy == 1:
        grid[y1][x1] &= ~S; grid[y2][x2] &= ~N
    elif dx == 0 and dy == -1:
        grid[y1][x1] &= ~N; grid[y2][x2] &= ~S

def save_grid_json(grid, path: str | None = None):
    # kept for backward compatibility (saves grid only)
    p = Path(path) if path else Path(__file__).with_name("labyrinth.json")
    p.write_text(json.dumps(grid, indent=2), encoding="utf-8")
    print(f"Saved grid to {p}")


def save_state(path: str | None, grid, start, end, points):
    """Save full editor state to JSON. Backwards compatible with old grid-only files.

    Structure:
      { "grid": [[...],[...]], "start": [x,y] or null, "end": [x,y] or null, "points": [[x,y], ...] }
    """
    p = Path(path) if path else Path(__file__).with_name("labyrinth.json")
    obj = {
        "grid": grid,
        "start": [start[0], start[1]] if start is not None else None,
        "end": [end[0], end[1]] if end is not None else None,
        "points": [[x, y] for (x, y) in points],
    }
    p.write_text(json.dumps(obj, indent=2), encoding="utf-8")
    print(f"Saved state to {p}")


def load_state(path: str | None):
    """Load saved state. Returns tuple (grid, start, end, points).

    If the file contains a plain list (old format), it's treated as grid only.
    """
    p = Path(path) if path else Path(__file__).with_name("labyrinth.json")
    data = json.loads(p.read_text(encoding="utf-8"))
    # old format: list -> grid only
    if isinstance(data, list):
        grid = [[int(x) for x in row] for row in data]
        return grid, None, None, []
    # new object format
    grid = data.get("grid")
    if grid is None:
        raise ValueError("Saved file missing 'grid' key")
    grid = [[int(x) for x in row] for row in grid]
    start = None
    end = None
    points = []
    if data.get("start") is not None:
        sx, sy = data["start"]
        start = (int(sx), int(sy))
    if data.get("end") is not None:
        ex, ey = data["end"]
        end = (int(ex), int(ey))
    if data.get("points") is not None:
        points = [ (int(x), int(y)) for x,y in data.get("points") ]
    return grid, start, end, points

def main(json_path: str | None = None):
    # load saved state if present (grid + start/end/points), otherwise fall back to old loader
    p = Path(json_path) if json_path else Path(__file__).with_name("labyrinth.json")
    if p.exists():
        try:
            grid, start_pos, end_pos, points = load_state(json_path)
        except Exception:
            # fallback to grid-only loader if state file malformed
            grid = load_labyrinth(json_path)
            start_pos = None
            end_pos = None
            points = []
    else:
        grid = load_labyrinth(json_path)
        start_pos = None
        end_pos = None
        points = []
    cell_size = 64
    margin = 16
    # allocate extra width on the right for UI (save button)
    ui_width = BUTTON_W + margin * 2
    # window width: left margin + grid + ui area
    win_w = margin + SIZE * cell_size + ui_width
    win_h = margin*2 + SIZE * cell_size

    pygame.init()
    screen = pygame.display.set_mode((win_w, win_h))
    pygame.display.set_caption("Labyrinth Editor")
    clock = pygame.time.Clock()

    # UI setup: place the save/start/end buttons in the right UI column
    font = pygame.font.SysFont(None, 20)
    ui_left = margin + SIZE * cell_size + margin
    button_x = ui_left
    # vertical stack: Save at top, then Load, then Reset, then Start, then End
    save_y = margin + 8
    load_y = save_y + BUTTON_H + 8
    reset_y = load_y + BUTTON_H + 8
    start_y = reset_y + BUTTON_H + 8
    end_y = start_y + BUTTON_H + 8
    save_rect = pygame.Rect(button_x, save_y, BUTTON_W, BUTTON_H)
    load_rect = pygame.Rect(button_x, load_y, BUTTON_W, BUTTON_H)
    reset_rect = pygame.Rect(button_x, reset_y, BUTTON_W, BUTTON_H)
    start_rect = pygame.Rect(button_x, start_y, BUTTON_W, BUTTON_H)
    end_rect = pygame.Rect(button_x, end_y, BUTTON_W, BUTTON_H)
    # status message shown near the Save button (text and timer in frames)
    status_msg = ""
    status_timer = 0

    # marker states
    start_mode = False
    end_mode = False
    # point mode and list of stop points (ordered)
    point_mode = False
    # points already set by load_state or fallback above
    # current solution path (list of (x,y)) or None
    path = None

    dragging = False
    drag_start_cell = None

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_s:
                    save_state(json_path, grid, start_pos, end_pos, points)
                    status_msg = "Saved"
                    status_timer = 60
                elif ev.key == pygame.K_l:
                    p = Path(json_path) if json_path else Path(__file__).with_name("labyrinth.json")
                    if p.exists():
                        try:
                            grid, start_pos, end_pos, points = load_state(json_path)
                            path = None
                            status_msg = "Loaded"
                            status_timer = 60
                            print("Reloaded state")
                        except Exception as exc:
                            print("Failed to reload state:", exc)
                    else:
                        print("No saved file to load")
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:  # left-click
                    # check UI buttons first
                    if save_rect.collidepoint(ev.pos):
                        save_state(json_path, grid, start_pos, end_pos, points)
                        status_msg = "Saved"
                        status_timer = 60  # frames (~1s at 60fps)
                        continue
                    if start_rect.collidepoint(ev.pos):
                        start_mode = not start_mode
                        if start_mode:
                            end_mode = False
                        continue
                    if end_rect.collidepoint(ev.pos):
                        end_mode = not end_mode
                        if end_mode:
                            start_mode = False
                        continue
                    if load_rect.collidepoint(ev.pos):
                        p = Path(json_path) if json_path else Path(__file__).with_name("labyrinth.json")
                        if p.exists():
                            try:
                                grid, start_pos, end_pos, points = load_state(json_path)
                                path = None
                                # set the unified status message so the UI shows 'Loaded'
                                status_msg = "Loaded"
                                status_timer = 60
                                print("Loaded state")
                            except Exception as exc:
                                print("Failed to load state:", exc)
                        else:
                            print("No saved file to load")
                        continue
                    if reset_rect.collidepoint(ev.pos):
                        # reset grid to all walls and clear markers/points
                        grid = [[15 for _ in range(SIZE)] for _ in range(SIZE)]
                        start_pos = None
                        end_pos = None
                        points.clear()
                        path = None
                        status_msg = "Reset"
                        status_timer = 60
                        print("Grid reset to all walls; markers cleared")
                        continue

                    # check Point and Solve buttons (draw_ui also creates rects)
                    # we'll get the rects at draw time and compare here via variables
                    # compute here a local rect matching draw_ui placement
                    point_rect_local = pygame.Rect(end_rect.left, end_rect.bottom + 8, BUTTON_W, BUTTON_H)
                    solve_rect_local = pygame.Rect(point_rect_local.left, point_rect_local.bottom + 8, BUTTON_W, BUTTON_H)
                    if point_rect_local.collidepoint(ev.pos):
                        # toggle point mode (exclusive)
                        point_mode = not point_mode
                        if point_mode:
                            start_mode = False
                            end_mode = False
                        continue
                    if solve_rect_local.collidepoint(ev.pos):
                        # run solver if possible
                        if start_pos is None or end_pos is None:
                            print("Start and End must be set to solve")
                        else:
                            path = solve_ordered(grid, start_pos, end_pos, points)
                            if path is None:
                                print("No path found for current configuration")
                            else:
                                print(f"Path of length {len(path)} found")
                        continue

                    info = cell_from_pos(ev.pos[0], ev.pos[1], cell_size, margin)
                    if info:
                        x, y, lx, ly = info
                        # placing start/end marker if mode active
                        if start_mode:
                            start_pos = (x, y)
                            start_mode = False
                            path = None
                            continue
                        if end_mode:
                            end_pos = (x, y)
                            end_mode = False
                            path = None
                            continue
                        if point_mode:
                            # toggle a stop point at (x,y)
                            if (x, y) in points:
                                points.remove((x, y))
                            else:
                                points.append((x, y))
                            path = None
                            continue
                        direction = edge_direction_from_local(lx, ly, cell_size)
                        toggle_wall_between(grid, x, y, direction)
                        path = None
                    dragging = True
                    # record cell for drag carve
                    cell_info = cell_from_pos(ev.pos[0], ev.pos[1], cell_size, margin)
                    if cell_info:
                        drag_start_cell = (cell_info[0], cell_info[1])
                elif ev.button == 3:  # right-click: remove wall inside cell center to open all sides
                    info = cell_from_pos(ev.pos[0], ev.pos[1], cell_size, margin)
                    if info:
                        x, y, lx, ly = info
                        # remove all walls of cell and matching neighbors
                        for dir_ in (N, E, S, W):
                            if grid[y][x] & dir_:
                                toggle_wall_between(grid, x, y, dir_)
            elif ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1 and dragging:
                    # if dragged to neighboring cell, carve passage
                    cell_info = cell_from_pos(ev.pos[0], ev.pos[1], cell_size, margin)
                    if cell_info and drag_start_cell:
                        x2, y2 = cell_info[0], cell_info[1]
                        x1, y1 = drag_start_cell
                        if abs(x1 - x2) + abs(y1 - y2) == 1:
                            remove_wall_between_cells(grid, x1, y1, x2, y2)
                    dragging = False
                    drag_start_cell = None

        draw_labyrinth(screen, grid, cell_size=cell_size, margin=margin, start_pos=start_pos, end_pos=end_pos, points=points, path=path, font=font)
        point_rect, solve_rect = draw_ui(screen, font, save_rect, load_rect, reset_rect, start_rect, end_rect, status_msg, status_timer, start_mode, end_mode, start_pos, end_pos, point_mode=point_mode)
        if status_timer > 0:
            status_timer -= 1
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    return 0

if __name__ == "__main__":
    sys.exit(main())