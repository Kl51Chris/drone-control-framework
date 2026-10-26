"""Generate the focused second XY/Z landing diagnostic plot without dependencies."""
from __future__ import annotations

import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "calibration/runs/20261026T080839_447421Z_cascaded-xy"
OUT = ROOT / "calibration/stage5_analysis/second_xy_landing"


def main() -> None:
    with (RUN / "flight_telemetry.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    times = [float(row["elapsed_s"]) for row in rows]
    width, height, left, right = 1100, 700, 80, 20
    panels = [
        ("altitude [m]", ("z_m", "reference_z_m"), ("#2563eb", "#111827")),
        ("vertical velocity [m/s]", ("vz_m_s",), ("#059669",)),
        ("normalized thrust", ("command_thrust_normalized",), ("#7c3aed",)),
    ]
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
           '<rect width="100%" height="100%" fill="white"/>']
    transitions, prior = [], None
    for row in rows:
        if row["phase"] != prior:
            transitions.append((float(row["elapsed_s"]), row["phase"]))
            prior = row["phase"]
    duration = max(times[-1] - times[0], 1e-9)
    for index, (label, keys, colors) in enumerate(panels):
        top, panel_height = 35 + index * 215, 165
        bottom = top + panel_height
        values = [float(row[key]) for row in rows for key in keys]
        guide = .060 if index == 0 else (.425 if index == 2 else 0.0)
        values.append(guide)
        lo, hi = min(values), max(values)
        padding = max((hi-lo)*.08, 1e-3); lo -= padding; hi += padding
        x = lambda elapsed: left + (elapsed-times[0])/duration*(width-left-right)
        y = lambda value: bottom - (value-lo)/(hi-lo)*panel_height
        svg.extend([
            f'<rect x="{left}" y="{top}" width="{width-left-right}" height="{panel_height}" fill="none" stroke="#bbb"/>',
            f'<text x="8" y="{top+15}" font-family="sans-serif" font-size="12">{label}</text>',
            f'<text x="{left-8}" y="{top+10}" text-anchor="end" font-family="sans-serif" font-size="10">{hi:.3f}</text>',
            f'<text x="{left-8}" y="{bottom}" text-anchor="end" font-family="sans-serif" font-size="10">{lo:.3f}</text>',
        ])
        for key, color in zip(keys, colors):
            points = " ".join(f'{x(float(row["elapsed_s"])):.1f},{y(float(row[key])):.1f}' for row in rows)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>')
        svg.append(f'<line x1="{left}" x2="{width-right}" y1="{y(guide):.1f}" y2="{y(guide):.1f}" stroke="#dc2626" stroke-dasharray="5 4"/>')
        for elapsed, _ in transitions[1:]:
            svg.append(f'<line x1="{x(elapsed):.1f}" x2="{x(elapsed):.1f}" y1="{top}" y2="{bottom}" stroke="#bbb"/>')
        if index == 0:
            for elapsed, phase in transitions:
                xpos = x(elapsed)+3
                svg.append(f'<text x="{xpos:.1f}" y="{top+12}" font-family="sans-serif" font-size="9" transform="rotate(90 {xpos:.1f} {top+12})">{phase}</text>')
    svg.append(f'<text x="{width/2}" y="{height-8}" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed time [s]</text>')
    svg.append('</svg>')
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "altitude_velocity_thrust.svg").write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
