"""Draw separate histograms for the stages recorded by HGVD."""
import argparse
from collections import defaultdict
import csv
from math import ceil, sqrt
from pathlib import Path

import matplotlib

matplotlib.use('Agg')

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', type=Path)
    parser.add_argument('--output', type=Path, help='Output image (default: next to the CSV)')
    args = parser.parse_args()

    groups = defaultdict(list)
    try:
        with args.csv_file.expanduser().open(encoding='utf-8', newline='') as stream:
            rows = csv.DictReader(stream)
            if not {'stage', 'duration_ms'}.issubset(rows.fieldnames or []):
                parser.error('CSV must contain stage and duration_ms columns')
            for row in rows:
                groups[row['stage']].append(float(row['duration_ms']))
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    if not groups:
        parser.error('the CSV contains no timing samples')

    rows = ceil(len(groups) / 2)
    fig, axes = plt.subplots(rows, 2, figsize=(11, 3.15 * rows),
                             squeeze=False, constrained_layout=True)
    fig.suptitle('HGVD stage latency distributions')
    for ax, (stage, times) in zip(axes.flat, groups.items()):
        median, p95 = np.percentile(times, [50, 95])
        bins = min(40, max(1, ceil(sqrt(len(times)))))
        ax.hist(times, bins=bins, color='#35788f', edgecolor='white')
        ax.axvline(median, color='#b35229', linestyle='--', label=f'P50 {median:.3f} ms')
        ax.axvline(p95, color='#642b73', linestyle=':', label=f'P95 {p95:.3f} ms')
        ax.set(title=f"{stage.replace('_', ' ').title()}  (n={len(times)})",
               xlabel='Duration (ms)', ylabel='Samples')
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax.legend(loc='upper right', fontsize='small')
    for ax in list(axes.flat)[len(groups):]:
        ax.set_visible(False)
    output = (args.output or args.csv_file.with_suffix('.png')).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(output.resolve())


if __name__ == '__main__':
    main()
