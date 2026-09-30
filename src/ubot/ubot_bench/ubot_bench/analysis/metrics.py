"""Per-trial metrics. Definitions are the ones in PROTOCOL.md, section "Metrics".

Trajectory (evo 1.31, planar SE(2) data stored as SE(3) with z = roll = pitch = 0):
  ate_rmse          RMSE of translation error after Umeyama SE(3) alignment (no scale)
  ate_anchor_rmse   RMSE after aligning only the first pose (how a robot actually experiences
                    drift: the estimate starts where it starts)
  rpe_t_1m, rpe_r_1m  mean relative translation (m) / rotation (deg) error over 1 m segments
  Estimates are associated with ground truth by timestamp (max 20 ms apart).

Map (slam_toolbox map vs ground-truth map at the scan plane, both 5 cm):
  The SLAM map is placed in the world with the SE(2) alignment of its own trajectory to ground
  truth (so map and trajectory errors are judged in one consistent frame).
  map_precision   fraction of mapped-occupied cells within TOL of a true occupied cell
  map_recall      fraction of true occupied cells *observable from the route* within TOL of a
                  mapped-occupied cell (observable = within the stack's range of a route pose and
                  in line of sight on the ground-truth map)
  map_f1          harmonic mean of the two
  wall_offset_mean  mean distance (m) from mapped-occupied cells to the nearest true occupied cell
  free_iou        IoU of free space over the observable region
"""
import math

import numpy as np

from ubot_bench.grid import FREE, OCC, Grid, distance_to

TOL = 0.10            # m, two cells
ASSOC_MAX_DIFF = 0.02 # s


def _read_tum(path):
    from evo.tools import file_interface
    return file_interface.read_tum_trajectory_file(path)


def trajectory_metrics(gt_path, est_path):
    """ATE / RPE of est against gt; None values if too few associated poses."""
    import copy

    from evo.core import metrics, sync
    from evo.core.metrics import PoseRelation, Unit
    out = {'n_assoc': 0}
    try:
        gt, est = _read_tum(gt_path), _read_tum(est_path)
        gt, est = sync.associate_trajectories(gt, est, max_diff=ASSOC_MAX_DIFF)
    except Exception as e:                                   # empty file, no overlap
        out['error'] = str(e)
        return out
    out['n_assoc'] = est.num_poses
    if est.num_poses < 10:
        return out
    est_u = copy.deepcopy(est)
    est_u.align(gt, correct_scale=False)
    ape = metrics.APE(PoseRelation.translation_part)
    ape.process_data((gt, est_u))
    out['ate_rmse'] = float(ape.get_statistic(metrics.StatisticsType.rmse))
    out['ate_max'] = float(ape.get_statistic(metrics.StatisticsType.max))
    est_a = copy.deepcopy(est)
    est_a.align_origin(gt)
    ape_a = metrics.APE(PoseRelation.translation_part)
    ape_a.process_data((gt, est_a))
    out['ate_anchor_rmse'] = float(ape_a.get_statistic(metrics.StatisticsType.rmse))
    out['ate_anchor_final'] = float(ape_a.error[-1])
    for rel, key, conv in ((PoseRelation.translation_part, 'rpe_t_1m', 1.0),
                           (PoseRelation.rotation_angle_deg, 'rpe_r_1m', 1.0)):
        try:
            rpe = metrics.RPE(rel, delta=1.0, delta_unit=Unit.meters, all_pairs=False)
            rpe.process_data((gt, est))
            out[key] = float(rpe.get_statistic(metrics.StatisticsType.mean)) * conv
        except Exception as e:
            out[key + '_error'] = str(e)
    out['gt_length_m'] = float(gt.path_length)
    # SE(2) alignment (yaw, tx, ty) of est into the gt frame, reused by the map metrics
    R = est_u.positions_xyz  # noqa: N806 (after alignment)
    out['_align'] = _se2_from_alignment(est.positions_xyz[:, :2], R[:, :2])
    return out


def _se2_from_alignment(src, dst):
    """Least-squares SE(2) taking src points to dst points."""
    cs, cd = src.mean(0), dst.mean(0)
    h = (src - cs).T @ (dst - cd)
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(yaw), math.sin(yaw)
    t = cd - np.array([[c, -s], [s, c]]) @ cs
    return [yaw, float(t[0]), float(t[1])]


def observable_mask(gt, route_xy, rng_max, step=5):
    """True cells visible (line of sight on the GT map, within rng_max) from route poses."""
    occ = gt.data == OCC
    seen = np.zeros(gt.shape, dtype=bool)
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    s = np.arange(0.0, rng_max, gt.res / 2)
    for x, y in route_xy[::step]:
        px = x + s[None, :] * np.cos(angles)[:, None]
        py = y + s[None, :] * np.sin(angles)[:, None]
        r, c = gt.world_to_cell(px, py)
        ok = gt.inside(r, c)
        r = np.where(ok, r, 0)
        c = np.where(ok, c, 0)
        hit = occ[r, c] & ok
        # cells up to and including the first occupied cell on each ray
        first = np.where(hit.any(1), hit.argmax(1), s.size - 1)
        keep = (np.arange(s.size)[None, :] <= first[:, None]) & ok
        seen[r[keep], c[keep]] = True
    return seen & (gt.data != -1)


def map_metrics(gt, est_map, align, observable):
    """Compare a SLAM map (its own frame) to the GT grid via SE(2) align (yaw, tx, ty)."""
    yaw, tx, ty = align
    c, s = math.cos(yaw), math.sin(yaw)
    rows, cols = np.nonzero(est_map.data == OCC)
    ex, ey = est_map.cell_to_world(rows, cols)
    wx, wy = c * ex - s * ey + tx, s * ex + c * ey + ty
    gt_occ = gt.data == OCC
    d_gt = distance_to(gt_occ, gt.res)
    r, cc = gt.world_to_cell(wx, wy)
    ins = gt.inside(r, cc)
    d_est_pts = np.where(ins, d_gt[np.where(ins, r, 0), np.where(ins, cc, 0)], np.inf)
    precision = float(np.mean(d_est_pts <= TOL)) if d_est_pts.size else 0.0
    wall_offset = float(np.mean(np.minimum(d_est_pts, 1.0))) if d_est_pts.size else math.nan
    est_occ_on_gt = np.zeros(gt.shape, dtype=bool)
    est_occ_on_gt[r[ins], cc[ins]] = True
    d_est = distance_to(est_occ_on_gt, gt.res)
    target = gt_occ & observable
    recall = float(np.mean(d_est[target] <= TOL)) if target.any() else math.nan
    f1 = 2 * precision * recall / (precision + recall) if precision + recall > 0 else 0.0
    # free-space IoU over the observable region: sample the SLAM map at every GT cell centre
    gr, gc = np.nonzero(observable)
    gx, gy = gt.cell_to_world(gr, gc)
    lx, ly = c * (gx - tx) + s * (gy - ty), -s * (gx - tx) + c * (gy - ty)
    er, ec = est_map.world_to_cell(lx, ly)
    eins = est_map.inside(er, ec)
    est_free = np.zeros(gr.size, dtype=bool)
    est_free[eins] = est_map.data[er[eins], ec[eins]] == FREE
    true_free = gt.data[gr, gc] == FREE
    inter = np.sum(est_free & true_free)
    union = np.sum(est_free | true_free)
    return {'map_precision': precision, 'map_recall': recall, 'map_f1': f1,
            'wall_offset_mean': wall_offset,
            'free_iou': float(inter / union) if union else math.nan,
            'map_occ_cells': int(rows.size)}


def load_map(path):
    return Grid.load(path)
