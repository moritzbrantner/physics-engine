"""Plot completed solver-budget data. No interpolation/extrapolation or omitted quality failures."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import matplotlib.pyplot as plt

DEFAULT_PROFILES = ['4s-8v-2p', '4s-4v-2p', '4s-2v-2p', '2s-4v-1p', '2s-2v-1p', '1s-2v-1p']
TITLES = {
    'sustained-stack': 'Four-layer stack: speed versus stability',
    'mixed-impacts': 'Four-layer stack under mixed projectile impacts',
    'shallow-contact': 'Single-layer contact: a cheaper budget can remain stable',
}

def plot_report(report_path: Path, output_dir: Path, profiles: list[str] | None = None) -> list[Path]:
    report = json.loads(report_path.read_text())
    if report.get('schema') != 'physics-solver-budget/v1':
        raise ValueError('Expected a completed physics-solver-budget/v1 report')
    expected = len(report['counts']) * len(report['profiles']) * len(report['scenes']) * report['trials']
    if len(report['records']) != expected:
        raise ValueError('Cannot plot an incomplete matrix as completed evidence')
    selected = profiles or DEFAULT_PROFILES
    selected = [p for p in selected if any(x['id'] == p for x in report['profiles'])]
    if not selected:
        raise ValueError('None of the selected profiles is present')
    output_dir.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []
    for scene in report['scenes']:
        if scene not in TITLES:
            continue  # A sleeping control must not look like active object capacity.
        fig = plt.figure(figsize=(11.5, 6.5), dpi=150)
        ax = fig.add_axes((0.10, 0.17, 0.86, 0.71))
        for profile in selected:
            points, rejected = [], []
            for count in report['counts']:
                rows = [r for r in report['records'] if r['scene'] == scene and r['profile']['id'] == profile and r['boxes'] == count]
                samples = [r['timing']['active']['p95_ms'] for r in rows]
                p95 = max(samples) if samples and all(x is not None for x in samples) else math.nan
                points.append(p95)
                if not all(r['completed'] and r['quality']['passed'] and r['repeatable'] for r in rows):
                    rejected.append((count, p95))
            cfg = next(p for p in report['profiles'] if p['id'] == profile)
            label = f"{cfg['substeps']} / {cfg['velocity']} / {cfg['position']}"
            if profile == '4s-8v-2p':
                label += ' (current maxima)'
            ax.plot(report['counts'], points, marker='o', markersize=4, linewidth=1.5, label=label)
            for count, value in rejected:
                if math.isfinite(value):
                    ax.annotate('×', (count, value), ha='center', va='center', fontsize=15, fontweight='bold')
        ax.axhline(8, linestyle='--', linewidth=1, label='8 ms physics-only budget')
        ax.set_xscale('log', base=2)
        ax.set_yscale('log', base=2)
        ax.set_xticks(report['counts'], [str(n) for n in report['counts']])
        ticks = [.0625, .125, .25, .5, 1, 2, 4, 8, 16, 32, 64, 128]
        lo, hi = ax.get_ylim()
        ticks = [t for t in ticks if lo <= t <= hi]
        ax.set_yticks(ticks, [f'{t:g}' for t in ticks])
        ax.set_xlabel('Dynamic boxes — all kept awake, plus one fixed floor and any live projectiles')
        ax.set_ylabel('Worst repetition p95 physics-call time (ms)')
        ax.set_title(TITLES[scene], loc='left', fontsize=15, pad=16)
        ax.grid(True, which='major', alpha=.18)
        ax.legend(title='Substeps / max velocity passes / max position passes', fontsize=8.5, title_fontsize=9, ncol=2)
        fig.text(.10, .065, '× fails a quality/repeatability check; an unmarked circle passes. A faster failed case is not usable capacity.', fontsize=9)
        fig.text(.10, .04, f"{report['trials']} repetitions · timing window: seconds 1–{report['ticks']/60:g} · quality checked throughout the full replay", fontsize=8)
        fig.text(.10, .015, f"{report['environment']['cpu']} · release WASM · no rendering or observation code in timings", fontsize=8)
        dest = output_dir / f'{scene}.png'
        fig.savefig(dest)
        plt.close(fig)
        files.append(dest)
    return files

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--profiles', help='Comma-separated IDs; defaults to six illustrative profiles')
    args = parser.parse_args()
    plot_report(args.report, args.output, args.profiles.split(',') if args.profiles else None)
