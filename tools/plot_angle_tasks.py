#!/usr/bin/env python3
# Codex-generated reusable four-panel plot, archived with owner authorization, 2026-10-25.
"""KEEP: explicitly retained by the owner; exclude from temporary-data cleanup.

Preserved plotting program for the owner-selected historical four-panel figure.

Usage: python3 tools/plot_angle_tasks.py --data /path/to/plot_data.csv.gz --output /tmp/angle_error_replot.png
Historical measurement data was removed at the owner's request. Supply new data explicitly.
It is a historical V5 result, not the final angle-task validation.
"""
import argparse
import csv
import gzip
import json
from pathlib import Path


def load_plot_data(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8', newline='') as stream:
        if not stream.readline().startswith('# Codex-generated'):
            raise ValueError('Missing plot-data provenance header')
        metadata = json.loads(stream.readline().removeprefix('# '))
        series = {}
        for row in csv.DictReader(stream):
            key = (row['task'], row['series'])
            x, y = series.setdefault(key, ([], []))
            x.append(float(row['time_s']))
            y.append(float(row['value']))
    return metadata, series


def plot(data_path, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    metadata, series = load_plot_data(data_path)
    with plt.rc_context(matplotlib.rcParamsDefault):
        fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
        for i, task in enumerate(('angle_hold', 'angle_sweep')):
            left, right = axes[i]
            left.plot(*series[task, 'truth_angle'], label='Actual truth', lw=1.1)
            left.plot(*series[task, 'reference_angle'], '--', label='Offline angle reference', lw=1.1)
            left.set_title(metadata.get('titles', {}).get(task, task))
            left.set_ylabel('North angle [deg]')
            left.legend()
            right.plot(*series[task, 'actual_position'], label='PX4 local North')
            right.plot(*series[task, 'reference_position'], '--', label=metadata.get('position_reference_label', 'Physical planned North'))
            right.set_ylabel('North position [m]')
            right.legend()
            if i == 0:
                left.axvspan(*metadata['hold_interval_s'], color='green', alpha=.12, label=f"{metadata.get('hold_target_deg', 10):g}-deg hold")
        for axis in axes.flat:
            axis.grid(alpha=.25)
            axis.set_xlabel('Time since command [s]')
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=150, metadata={'Software': 'Codex-generated; archived historical V5 plot'})
        plt.close(fig)
    return metadata


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True, help='Explicit plot-data path; no experiment data is bundled.')
    parser.add_argument('--output', type=Path, default=Path('/tmp/angle_error_replot.png'))
    args = parser.parse_args()
    if not args.data.is_file():
        parser.error('Plot-data file does not exist: ' + str(args.data))
    original = root/'images/dynamic_angle/angle_error.png'
    if args.output.resolve() in (original.resolve(), args.data.resolve()):
        parser.error('Choose another output path; the archived original/data are protected.')
    metadata = plot(args.data, args.output)
    print('Saved:', args.output)
    print(metadata['note'])


if __name__ == '__main__':
    main()
