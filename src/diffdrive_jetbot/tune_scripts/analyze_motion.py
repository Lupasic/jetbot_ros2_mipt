#!/usr/bin/env python3
"""
Analyzer for motion_logger.py CSVs — objective Nav2 tuning metrics.

Usage (host side, no ROS needed):
  python3 analyze_motion.py baseline.csv                 # metrics for one run
  python3 analyze_motion.py baseline.csv tuned.csv ...   # side-by-side + deltas
  python3 analyze_motion.py --plots out_dir baseline.csv # also write PNGs

Per translation segment (between nav/goal and goal_reached):
  weave    — RMS / max lateral deviation from the segment chord [mm];
             wz sign flips per metre; cruise wz RMS [rad/s]
  pace     — duration [s]; average speed [m/s]; fraction of time at
             v > 0.8 * v_peak; mid-segment speed dips (v < 50% of peak)
  arrival  — settle time: from first entry within 5 cm of the endpoint
             until the segment ends [s]
Rotation segments (net yaw > 90 deg, small displacement) report duration only.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

SETTLE_RADIUS = 0.05    # m, "at goal" circle for settle-time metric
WZ_DEADBAND = 0.02      # rad/s, ignore sign flips inside this band
CRUISE_TRIM = 0.15      # fraction of segment duration trimmed at each end


def load_csv(path):
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(v) for v in row] for row in reader if row]
    data = {name: np.array([r[i] for r in rows]) for i, name in enumerate(header)}
    return data


def split_segments(d):
    """Yield (seg_id, index_mask) for every goal-active segment."""
    for seg_id in sorted(set(d['seg_id'].astype(int))):
        if seg_id < 0:
            continue
        mask = (d['seg_id'].astype(int) == seg_id) & (d['goal_active'] == 1)
        if mask.sum() >= 5:
            yield seg_id, mask


def segment_metrics(d, mask):
    t = d['t'][mask]
    x, y, yaw = d['x'][mask], d['y'][mask], d['yaw'][mask]
    v, w = d['v_odom'][mask], d['w_odom'][mask]

    duration = t[-1] - t[0]
    if duration <= 0:
        return None

    dx, dy = np.diff(x), np.diff(y)
    path_len = float(np.sum(np.hypot(dx, dy)))
    net_disp = float(math.hypot(x[-1] - x[0], y[-1] - y[0]))
    net_yaw = abs(math.remainder(yaw[-1] - yaw[0], 2 * math.pi))

    m = {'duration': duration, 'path_len': path_len}

    # Rotation-in-place segment: only duration is meaningful
    if net_disp < 0.10 and net_yaw > math.pi / 3:
        m['kind'] = 'rotate'
        return m
    m['kind'] = 'move'

    # ── weave ─────────────────────────────────────────────────────────────
    ux, uy = (x[-1] - x[0]) / max(net_disp, 1e-9), (y[-1] - y[0]) / max(net_disp, 1e-9)
    lateral = (x - x[0]) * (-uy) + (y - y[0]) * ux       # signed cross-track, m
    m['lat_rms_mm'] = float(np.sqrt(np.mean(lateral ** 2)) * 1000)
    m['lat_max_mm'] = float(np.max(np.abs(lateral)) * 1000)

    wd = np.where(np.abs(w) > WZ_DEADBAND, np.sign(w), 0.0)
    nz = wd[wd != 0]
    flips = int(np.sum(nz[1:] != nz[:-1])) if nz.size > 1 else 0
    m['wz_flips_per_m'] = flips / max(path_len, 1e-9)

    cruise = (t - t[0] > duration * CRUISE_TRIM) & (t - t[0] < duration * (1 - CRUISE_TRIM))
    m['wz_rms_cruise'] = float(np.sqrt(np.mean(w[cruise] ** 2))) if cruise.any() else 0.0

    # ── pace ──────────────────────────────────────────────────────────────
    m['avg_speed'] = path_len / duration
    v_peak = float(np.max(v)) if v.size else 0.0
    m['v_peak'] = v_peak
    if v_peak > 0.01:
        m['frac_at_speed'] = float(np.mean(v > 0.8 * v_peak))
        mid = cruise
        below = v[mid] < 0.5 * v_peak
        # count distinct dip episodes, not samples
        m['dips'] = int(np.sum(below[1:] & ~below[:-1]) + (1 if below.size and below[0] else 0))
    else:
        m['frac_at_speed'] = 0.0
        m['dips'] = 0

    # ── arrival ───────────────────────────────────────────────────────────
    dist_to_end = np.hypot(x - x[-1], y - y[-1])
    inside = np.nonzero(dist_to_end < SETTLE_RADIUS)[0]
    m['settle_s'] = float(t[-1] - t[inside[0]]) if inside.size else 0.0

    return m


MOVE_COLS = [
    ('duration', 's', '{:6.1f}'), ('avg_speed', 'm/s', '{:6.3f}'),
    ('frac_at_speed', '', '{:6.2f}'), ('dips', '', '{:6.0f}'),
    ('lat_rms_mm', 'mm', '{:6.1f}'), ('lat_max_mm', 'mm', '{:6.1f}'),
    ('wz_flips_per_m', '1/m', '{:6.1f}'), ('wz_rms_cruise', 'rad/s', '{:6.3f}'),
    ('settle_s', 's', '{:6.2f}'),
]


def run_summary(path):
    d = load_csv(path)
    moves, rotates = [], []
    for seg_id, mask in split_segments(d):
        m = segment_metrics(d, mask)
        if m is None:
            continue
        (moves if m['kind'] == 'move' else rotates).append((seg_id, m))
    agg = {}
    if moves:
        for key, _, _ in MOVE_COLS:
            agg[key] = float(np.mean([m[key] for _, m in moves]))
    if rotates:
        agg['rotate_duration'] = float(np.mean([m['duration'] for _, m in rotates]))
    return d, moves, rotates, agg


def print_report(paths):
    aggs = []
    for path in paths:
        d, moves, rotates, agg = run_summary(path)
        aggs.append((path, agg))
        print(f'\n=== {path} — {len(moves)} move segments, {len(rotates)} rotations ===')
        hdr = 'seg | ' + ' | '.join(f'{k}{("," + u) if u else ""}' for k, u, _ in MOVE_COLS)
        print(hdr)
        for seg_id, m in moves:
            print(f'{seg_id:3d} | ' + ' | '.join(fmt.format(m[k]) for k, _, fmt in MOVE_COLS))
        if moves:
            print('avg | ' + ' | '.join(fmt.format(agg[k]) for k, _, fmt in MOVE_COLS))
        if rotates:
            print(f'rotation avg duration: {agg["rotate_duration"]:.1f} s '
                  f'({len(rotates)} segments)')

    if len(aggs) > 1:
        base_path, base = aggs[0]
        print(f'\n=== deltas vs baseline ({base_path}) ===')
        for path, agg in aggs[1:]:
            parts = []
            for k, _, _ in MOVE_COLS:
                if k in agg and k in base and base[k]:
                    parts.append(f'{k}: {100 * (agg[k] - base[k]) / abs(base[k]):+.0f}%')
            print(f'{path}: ' + ', '.join(parts))


def save_plots(paths, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available — text metrics only', file=sys.stderr)
        return
    os.makedirs(out_dir, exist_ok=True)
    for path in paths:
        d, moves, _, _ = run_summary(path)
        tag = os.path.splitext(os.path.basename(path))[0]

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 12))
        for seg_id, _m in moves:
            mask = (d['seg_id'].astype(int) == seg_id) & (d['goal_active'] == 1)
            ax1.plot(d['x'][mask], d['y'][mask], label=f'seg {seg_id}')
        ax1.set_title(f'{tag}: XY per move segment')
        ax1.axis('equal')
        ax1.legend(fontsize=7)

        t0 = d['t'][0]
        ax2.plot(d['t'] - t0, d['vx_nav'], label='vx DWB', alpha=0.6)
        ax2.plot(d['t'] - t0, d['vx_smooth'], label='vx smoothed', alpha=0.6)
        ax2.plot(d['t'] - t0, d['v_odom'], label='v odom', lw=1.5)
        ax2.set_title('linear velocity: command chain vs actual')
        ax2.legend(fontsize=8)

        ax3.plot(d['t'] - t0, d['wz_nav'], label='wz DWB', alpha=0.6)
        ax3.plot(d['t'] - t0, d['wz_smooth'], label='wz smoothed', alpha=0.6)
        ax3.plot(d['t'] - t0, d['w_odom'], label='w odom', lw=1.5)
        ax3.set_title('angular velocity: command chain vs actual')
        ax3.legend(fontsize=8)

        out = os.path.join(out_dir, f'{tag}.png')
        fig.tight_layout()
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(f'plot: {out}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csvs', nargs='+')
    parser.add_argument('--plots', metavar='DIR', help='also write PNG plots to DIR')
    args = parser.parse_args()
    print_report(args.csvs)
    if args.plots:
        save_plots(args.csvs, args.plots)


if __name__ == '__main__':
    main()
