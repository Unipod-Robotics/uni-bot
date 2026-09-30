"""Occupancy grid utilities shared by the ground-truth mapper, mission generator and metrics.

Grid convention (same as nav2 map_server): cell (row, col) with row 0 at the TOP of the image;
world x = origin_x + (col + 0.5) * res, world y = origin_y + (rows - row - 0.5) * res.
Values: 0 free, 100 occupied, -1 unknown.
"""
import os

import numpy as np
import yaml

FREE, OCC, UNKNOWN = 0, 100, -1


class Grid:
    def __init__(self, data, res, origin):
        self.data = np.asarray(data, dtype=np.int8)
        self.res = float(res)
        self.origin = (float(origin[0]), float(origin[1]))

    @property
    def shape(self):
        return self.data.shape

    def world_to_cell(self, x, y):
        rows = self.data.shape[0]
        col = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int)
        row = rows - 1 - np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int)
        return row, col

    def cell_to_world(self, row, col):
        rows = self.data.shape[0]
        x = self.origin[0] + (np.asarray(col) + 0.5) * self.res
        y = self.origin[1] + (rows - np.asarray(row) - 0.5) * self.res
        return x, y

    def inside(self, row, col):
        return (row >= 0) & (col >= 0) & (row < self.data.shape[0]) & (col < self.data.shape[1])

    # ---- I/O in nav2 map_server format (trinary PGM + YAML)
    def save(self, yaml_path):
        img = np.full(self.data.shape, 205, dtype=np.uint8)
        img[self.data == FREE] = 254
        img[self.data == OCC] = 0
        pgm = os.path.splitext(yaml_path)[0] + '.pgm'
        with open(pgm, 'wb') as f:
            f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
            f.write(img.tobytes())
        with open(yaml_path, 'w') as f:
            yaml.safe_dump({'image': os.path.basename(pgm), 'mode': 'trinary',
                            'resolution': self.res,
                            'origin': [self.origin[0], self.origin[1], 0.0], 'negate': 0,
                            'occupied_thresh': 0.65, 'free_thresh': 0.25}, f,
                           default_flow_style=None)

    @staticmethod
    def load(yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
        img = _read_pgm(pgm)
        if meta.get('negate', 0):
            img = 255 - img
        occ_p = (255 - img.astype(float)) / 255.0
        data = np.full(img.shape, UNKNOWN, dtype=np.int8)
        data[occ_p > meta.get('occupied_thresh', 0.65)] = OCC
        data[occ_p < meta.get('free_thresh', 0.25)] = FREE
        return Grid(data, meta['resolution'], meta['origin'][:2])


def _read_pgm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    # Header: magic, optional comments, width height, maxval; then binary data.
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    pos += 1
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    img = np.frombuffer(raw[pos:pos + w * h], dtype=np.uint8).reshape(h, w)
    if maxval != 255:
        img = (img.astype(float) * 255 / maxval).astype(np.uint8)
    return img


def distance_to(mask, res):
    """Euclidean distance (m) from every cell to the nearest True cell in mask."""
    from scipy.ndimage import distance_transform_edt
    if not mask.any():
        return np.full(mask.shape, np.inf)
    return distance_transform_edt(~mask) * res


def connected_from(mask, row, col):
    """Cells of mask 8-connected to (row, col)."""
    from scipy.ndimage import label
    lab, _ = label(mask, structure=np.ones((3, 3)))
    if not (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1]) or lab[row, col] == 0:
        return np.zeros_like(mask)
    return lab == lab[row, col]


def burn_glass(grid, panes, pad=0.0):
    """Return a copy of grid with glass panes (missions/<world>.yaml conditions.glass) marked
    occupied. Glass is invisible to the LiDAR but it is a real obstacle, so it belongs in the
    true occupancy of the C1 condition and in mission planning for every condition."""
    import math
    g = Grid(grid.data.copy(), grid.res, grid.origin)
    for p in panes or []:
        half = p['length'] / 2 + pad
        n = max(2, int(2 * half / (grid.res / 2)))
        s = np.linspace(-half, half, n)
        t = np.linspace(-(0.01 / 2 + pad), 0.01 / 2 + pad, max(2, int((0.01 + 2 * pad) / (grid.res / 2))))
        ss, tt = np.meshgrid(s, t)
        x = p['x'] + ss * math.cos(p['yaw']) - tt * math.sin(p['yaw'])
        y = p['y'] + ss * math.sin(p['yaw']) + tt * math.cos(p['yaw'])
        r, c = g.world_to_cell(x.ravel(), y.ravel())
        ok = g.inside(r, c)
        g.data[r[ok], c[ok]] = OCC
    return g
