from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any


SATURATION_FIELDS = ("altitude_saturated", "roll_rate_saturated",
                     "pitch_rate_saturated", "yaw_rate_saturated")
CONTRIBUTIONS = ("altitude_p", "altitude_velocity", "altitude_integral",
                 "altitude_correction", "roll_error_rad", "pitch_error_rad",
                 "yaw_error_rad", "desired_roll_rad", "desired_pitch_rad",
                 "command_roll_rate_rad_s", "command_pitch_rate_rad_s",
                 "command_yaw_rate_rad_s", "command_thrust_normalized")


def _events(rows: list[dict[str, str]], field: str) -> int:
    return sum(int(row[field]) and (index == 0 or not int(rows[index-1][field]))
               for index, row in enumerate(rows))


def analyze_cascaded_flight(run_directory: str | Path) -> dict[str, Any]:
    run = Path(run_directory)
    metadata = json.loads((run/"metadata.json").read_text(encoding="utf-8"))
    with (run/"flight_telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    safety = metadata.get("safety_event")
    report: dict[str, Any] = {
        "schema_version": 1, "analysis": "stage5b-custom-cascaded-flight",
        "software_execution_status": metadata.get("status"),
        "termination_reason": metadata.get("termination_reason"),
        "sample_count": len(rows), "safety_event": safety,
        "shutdown": metadata.get("shutdown"),
    }
    if not rows:
        report.update({"flight_duration_s": 0.0, "metrics_available": False,
                       "first_detected_instability_or_safety_event": safety,
                       "controller_contributions_immediately_preceding_failure": []})
    else:
        floats=lambda field:[float(row[field]) for row in rows]
        error=floats("altitude_error_m"); thrust=floats("command_thrust_normalized")
        saturation={field.removesuffix("_saturated"):{
            "sample_count":sum(int(row[field]) for row in rows),
            "duration_s":sum(float(row["dt_s"]) for row in rows if int(row[field])),
            "event_count":_events(rows,field)} for field in SATURATION_FIELDS}
        first_sat=next(({"type":"command_saturation","elapsed_s":float(row["elapsed_s"]),
                         "fields":[field for field in SATURATION_FIELDS if int(row[field])]}
                        for row in rows if any(int(row[field]) for field in SATURATION_FIELDS)),None)
        preceding=[{"elapsed_s":float(row["elapsed_s"]),"phase":row["phase"],
                    **{field:float(row[field]) for field in CONTRIBUTIONS}} for row in rows[-5:]]
        report.update({
            "metrics_available":True,"flight_duration_s":float(rows[-1]["elapsed_s"]),
            "altitude_tracking":{"rmse_m":math.sqrt(sum(v*v for v in error)/len(error)),
                                 "maximum_absolute_error_m":max(abs(v) for v in error)},
            "maximum_attitude_deviation_rad":{"roll":max(abs(v) for v in floats("roll_rad")),
                "pitch":max(abs(v) for v in floats("pitch_rad")),"yaw_from_reference":max(abs(v) for v in floats("yaw_error_rad"))},
            "maximum_horizontal_displacement_m":max(floats("horizontal_displacement_m")),
            "maximum_horizontal_speed_m_s":max(floats("horizontal_speed_m_s")),
            "maximum_absolute_vertical_velocity_m_s":max(abs(v) for v in floats("vz_m_s")),
            "command_saturation":saturation,
            "thrust":{"minimum":min(thrust),"maximum":max(thrust),"mean":sum(thrust)/len(thrust)},
            "first_detected_instability_or_safety_event":safety or first_sat,
            "controller_contributions_immediately_preceding_failure": preceding if metadata.get("status") != "complete" else [],
        })
        _write_svg(run, rows)
    (run/"flight_analysis.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    return report


def _write_svg(run: Path, rows: list[dict[str, str]]) -> None:
    width,height,left,right=1000,520,70,20;t=[float(r["elapsed_s"]) for r in rows];duration=max(t[-1]-t[0],1e-9)
    panels=[("altitude [m]",("z_m","reference_z_m"),("#2563eb","#111827")),
            ("attitude [rad]",("roll_rad","pitch_rad","yaw_error_rad"),("#dc2626","#059669","#7c3aed")),
            ("thrust",("command_thrust_normalized",),("#111827",))]
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">','<rect width="100%" height="100%" fill="white"/>']
    for i,(label,keys,colors) in enumerate(panels):
        top=25+i*160;bottom=top+125;values=[float(r[k]) for r in rows for k in keys];lo,hi=min(values),max(values)
        if lo==hi:lo-=.5;hi+=.5
        svg += [f'<rect x="{left}" y="{top}" width="{width-left-right}" height="125" fill="none" stroke="#ccc"/>',f'<text x="5" y="{top+15}" font-family="sans-serif" font-size="11">{label}</text>']
        for key,color in zip(keys,colors):
            points=" ".join(f'{left+(float(r["elapsed_s"])-t[0])/duration*(width-left-right):.1f},{bottom-(float(r[key])-lo)/(hi-lo)*125:.1f}' for r in rows)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}"/>')
    svg.append('</svg>');(run/"flight_response.svg").write_text("\n".join(svg)+"\n",encoding="utf-8")
