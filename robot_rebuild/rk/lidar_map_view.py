"""Rasterize FPGA scan bins for the HDMI preview. Distances are in millimetres."""
from functools import lru_cache


MAP_SIZE = 672  # Near-native diagram at 720p; also scales uniformly on a 1080p desktop.
_CENTER = MAP_SIZE // 2
MAP_RADIUS = 300
_RINGS_MM = (0, 1000, 2000, 4000, 8000)


@lru_cache(maxsize=1)
def _polar_grid():
    import numpy as np

    dy, dx = np.ogrid[-_CENTER:MAP_SIZE-_CENTER, -_CENTER:MAP_SIZE-_CENTER]
    radius = np.hypot(dx, dy)
    disk = radius <= MAP_RADIUS
    # Row coordinates point down. Sensor zero is forward and angles grow clockwise.
    angle = np.degrees(np.arctan2(dx, -dy)) % 360
    sector = (angle // 10).astype(np.uint8)
    canvas = np.full((MAP_SIZE, MAP_SIZE, 3), (19, 24, 29), dtype=np.uint8)
    canvas[disk] = (62, 65, 68)
    grid = (np.abs(dx) <= 1) | (np.abs(dy) <= 1)
    for ring in (75, 150, 225, 300):
        grid = grid | (np.abs(radius - ring) <= 1)
    canvas[grid & disk] = (105, 110, 114)
    vehicle = (dy >= -18) & (dy <= 10) & (np.abs(dx) <= (dy + 18)*.43)
    return canvas, radius, sector, vehicle


def render_map(known_mask, ranges_mm):
    """Return packed RGB. Unmeasured space, including behind a hit, stays grey."""
    import numpy as np

    canvas, radius, sector, vehicle = _polar_grid()
    pixels = canvas.copy()
    if known_mask:
        radii = np.interp(ranges_mm, _RINGS_MM, (0, 75, 150, 225, 300))
        known = np.array([bool(known_mask & (1 << i)) for i in range(36)])
        measured = known[sector] & (radius <= radii[sector])
        green, blue = pixels[:, :, 1], pixels[:, :, 2]
        np.add(green, 23, out=green, where=measured)
        np.add(blue, 24, out=blue, where=measured)
    pixels[vehicle] = (114, 229, 252)
    if known_mask:
        dy, dx = np.ogrid[-5:6, -5:6]
        dot = dx*dx + dy*dy <= 25
        for i in np.flatnonzero(known):
            angle = np.radians(i*10 + 5)
            x = _CENTER + round(radii[i]*np.sin(angle))
            y = _CENTER - round(radii[i]*np.cos(angle))
            pixels[y-5:y+6, x-5:x+6][dot] = (255, 148, 56)
    return pixels.tobytes()
