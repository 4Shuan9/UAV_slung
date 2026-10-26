#!/usr/bin/env python3
# Codex-generated reusable plot: fresh maneuver, vertically stacked panels, 2026-10-25.
"""Plot maneuver angle and North-position measurements from an explicit data file.

python3 tools/plot_maneuver.py --data /path/to/plot_data.csv.gz --output /tmp/maneuver_replot.png
Historical measurement data was removed at the owner's request; the plotting program is preserved.
"""
import argparse
from pathlib import Path
from plot_angle_tasks import load_plot_data


def plot(data_path, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    meta, series = load_plot_data(data_path)
    with plt.rc_context(matplotlib.rcParamsDefault):
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                                 constrained_layout=True)
        ax, pos = axes
        ax.plot(*series['maneuver', 'truth_angle'], label='Actual truth', lw=1.6)
        ax.plot(*series['maneuver', 'reference_angle'], '--',
                label='Offline angle reference', lw=1.6)
        ax.axvspan(*meta['hold_interval_s'], color='green', alpha=.12)
        ax.set_title('maneuver: 20-deg hold for 1.5 s')
        ax.set_ylabel('North angle [deg]')
        pos.plot(*series['maneuver', 'actual_position'],
                 label='PX4 North (home-relative)', lw=1.6)
        pos.plot(*series['maneuver', 'reference_position'], '--',
                 label='Nominal North reference', lw=1.6)
        pos.set_title('North position')
        pos.set_ylabel('North position [m]')
        for axis in axes:
            axis.grid(alpha=.25)
            axis.legend(loc='upper right', fontsize=11)
            axis.tick_params(labelsize=11)
            axis.set_xlabel('Time since maneuver start [s]', fontsize=12)
            axis.xaxis.set_tick_params(labelbottom=True)
            axis.set_xlim(0, meta['duration_s'])
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=180, metadata={
            'Software': 'Codex-generated; fresh Gazebo/PX4 maneuver measurements',
            'Description': meta['note']})
        plt.close(fig)
    return meta


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True,
                        help='Explicit plot-data path; no experiment data is bundled.')
    parser.add_argument('--output', type=Path, default=Path('/tmp/maneuver_replot.png'))
    args = parser.parse_args()
    if not args.data.is_file():
        parser.error('Plot-data file does not exist: ' + str(args.data))
    protected = [args.data, *(root/'images/dynamic_angle'/name for name in
                 ('angle_error.png', 'angle_tracking.png', 'maneuver_tracking.png', 'maneuver_20deg.png'))]
    if args.output.resolve() in [p.resolve() for p in protected]:
        parser.error('Choose a separate output path; retained figures/data are protected.')
    meta = plot(args.data, args.output)
    print('Saved:', args.output)
    print(meta['note'])


if __name__ == '__main__':
    main()
