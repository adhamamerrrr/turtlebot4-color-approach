"""Offline RGB color detection and fail-closed approach decisions. No robot I/O."""
from collections import deque
from dataclasses import asdict, dataclass
import argparse
import colorsys
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Target:
    center_x: float
    area_fraction: float


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float
    reason: str


def read_ppm(path):
    """Read portable P3 (ASCII RGB) images with comments, validating every pixel."""
    text = Path(path).read_text(encoding='ascii')
    tokens = ' '.join(line.split('#', 1)[0] for line in text.splitlines()).split()
    if not tokens or tokens[0] != 'P3' or len(tokens) < 4:
        raise ValueError('expected a P3 ASCII PPM image')
    w, h, maximum = map(int, tokens[1:4])
    if not (1 <= w <= 2000 and 1 <= h <= 2000 and 1 <= maximum <= 65535):
        raise ValueError('invalid image dimensions or channel maximum')
    values = list(map(int, tokens[4:]))
    if len(values) != w * h * 3 or any(v < 0 or v > maximum for v in values):
        raise ValueError('incorrect pixel count or invalid channel value')
    return [[tuple(v / maximum for v in values[(y*w+x)*3:(y*w+x+1)*3])
             for x in range(w)] for y in range(h)]


def detect_red(pixels, min_pixels=4):
    """Return largest 4-connected red component; hue wraps around 0/1."""
    if not pixels or not pixels[0] or min_pixels < 1:
        raise ValueError('nonempty rectangular pixels and positive min_pixels required')
    h, w = len(pixels), len(pixels[0])
    if any(len(row) != w for row in pixels):
        raise ValueError('pixels must be rectangular')
    mask = set()
    for y, row in enumerate(pixels):
        for x, rgb in enumerate(row):
            if len(rgb) != 3 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in rgb):
                raise ValueError('RGB channels must be finite values in [0,1]')
            hue, sat, val = colorsys.rgb_to_hsv(*rgb)
            if (hue <= 0.05 or hue >= 0.95) and sat >= 0.5 and val >= 0.25:
                mask.add((x, y))
    largest = []
    while mask:
        start = min(mask, key=lambda p: (p[1], p[0]))
        mask.remove(start)
        queue, component = deque([start]), []
        while queue:
            x, y = queue.popleft()
            component.append((x, y))
            for neighbor in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if neighbor in mask:
                    mask.remove(neighbor)
                    queue.append(neighbor)
        if len(component) > len(largest):
            largest = component
    if len(largest) < min_pixels:
        return None
    return Target((sum(x + 0.5 for x, _ in largest) / len(largest)) / w,
                  len(largest) / (w*h))


def decide(target, frame_age, clearance, *, max_age=0.5,
           stop_distance=0.45, stop_area=0.2, max_speed=0.12, max_turn=0.5):
    """Propose a bounded velocity; positive angular means turn left (ROS convention)."""
    values = [frame_age, clearance, max_age, stop_distance, stop_area, max_speed, max_turn]
    if any(not math.isfinite(v) for v in values) or frame_age < 0 or clearance < 0:
        return Command(0, 0, 'invalid_sensor_data')
    if max_age <= 0 or stop_distance <= 0 or not 0 < stop_area <= 1 or max_speed <= 0 or max_turn <= 0:
        raise ValueError('invalid controller configuration')
    if frame_age > max_age:
        return Command(0, 0, 'stale_camera')
    if clearance <= stop_distance:
        return Command(0, 0, 'obstacle')
    if target is None:
        return Command(0, 0, 'no_target')
    if (not math.isfinite(target.center_x) or not math.isfinite(target.area_fraction)
            or not 0 <= target.center_x <= 1 or not 0 < target.area_fraction <= 1):
        return Command(0, 0, 'invalid_target')
    if target.area_fraction >= stop_area:
        return Command(0, 0, 'target_large_enough')
    error = 2 * target.center_x - 1
    turn = max(-max_turn, min(max_turn, -error))
    if abs(error) > 0.15:
        return Command(0, turn, 'align')
    return Command(max_speed * (1 - target.area_fraction / stop_area), turn, 'approach')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--frame-age', type=float, required=True)
    parser.add_argument('--clearance', type=float, required=True)
    parser.add_argument('--min-pixels', type=int, default=4)
    args = parser.parse_args()
    try:
        target = detect_red(read_ppm(args.image), args.min_pixels)
        command = decide(target, args.frame_age, args.clearance)
    except (ValueError, OSError) as exc:
        parser.exit(2, f'Input rejected: {exc}\n')
    print(json.dumps({'target': asdict(target) if target else None,
                      'command': asdict(command)}, indent=2))


if __name__ == '__main__':
    main()
