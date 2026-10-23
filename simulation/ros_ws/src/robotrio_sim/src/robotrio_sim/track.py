"""Measure cross-track error against the rendered world, rather than world Y."""
import math
import xml.etree.ElementTree as ET


def load_track(path, x=0.0, y=0.0, yaw=0.0):
    model = ET.parse(path).getroot().find("world/model[@name='line_2m_30mm']")
    if model is None:
        raise ValueError('Missing line model in the launched world file')
    visuals = model.findall('link/visual')
    centers, ends = [], []
    for visual in visuals:
        px, py, _, _, _, angle = map(float, visual.findtext('pose').split())
        length = float(visual.findtext('geometry/box/size').split()[0])
        centers.append((px, py))
        ends.append(((px-length/2*math.cos(angle), py-length/2*math.sin(angle)),
                     (px+length/2*math.cos(angle), py+length/2*math.sin(angle))))
    if not centers:
        raise ValueError('Line model has no box visuals')
    # Joining visual centres avoids counting the tiny seam overlaps as progress.
    points = [ends[0][0]] + centers + [ends[-1][1]]
    points = [(x+px*math.cos(yaw)-py*math.sin(yaw),
               y+px*math.sin(yaw)+py*math.cos(yaw)) for px, py in points]
    return list(zip(points, points[1:]))


def nearest_track(x, y, segments):
    """Return distance [m], along-track position [m], local heading [degrees]."""
    best, travelled = None, 0.0
    for (ax, ay), (bx, by) in segments:
        dx, dy = bx-ax, by-ay
        length = math.hypot(dx, dy)
        if length <= 0:
            raise ValueError('Degenerate track segment')
        t = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/(length*length)))
        distance = math.hypot(x-ax-t*dx, y-ay-t*dy)
        candidate = (distance, travelled+t*length, math.degrees(math.atan2(dy, dx)))
        if best is None or distance < best[0]:
            best = candidate
        travelled += length
    if best is None:
        raise ValueError('Empty track')
    return best
