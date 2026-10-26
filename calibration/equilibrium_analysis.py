from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import mean, median, stdev
from typing import Any


RAW_ZERO = 10001.0
RAW_FULL = 60000.0
STABLE_SETTLING_S = 1.0
STABLE_MAX_ABS_ALTITUDE_ERROR_M = 0.03
STABLE_MAX_ABS_VERTICAL_SPEED_M_S = 0.03
STABLE_MAX_THRUST_AGE_S = 0.05


def raw_to_toolkit_normalized(value: float) -> float:
    """Invert cflib send_setpoint_manual's 10001..60000 affine mapping."""
    return (value-RAW_ZERO)/(RAW_FULL-RAW_ZERO)


def _percentile(values: list[float], percentile: float) -> float:
    ordered=sorted(values); index=(len(ordered)-1)*percentile/100
    lower=math.floor(index); upper=math.ceil(index)
    if lower==upper:return ordered[lower]
    return ordered[lower]*(upper-index)+ordered[upper]*(index-lower)


def _pearson(left: list[float], right: list[float]) -> float | None:
    if len(left)<2:return None
    ml,mr=mean(left),mean(right)
    numerator=sum((a-ml)*(b-mr) for a,b in zip(left,right))
    denominator=math.sqrt(sum((a-ml)**2 for a in left)*sum((b-mr)**2 for b in right))
    return numerator/denominator if denominator else None


def _stats(values: list[float]) -> dict[str,float]:
    return {"mean":mean(values),"median":median(values),"standard_deviation":stdev(values) if len(values)>1 else 0.0,
            "p05":_percentile(values,5),"p25":_percentile(values,25),"p75":_percentile(values,75),
            "p95":_percentile(values,95),"minimum":min(values),"maximum":max(values)}


def analyze_equilibrium(run_directory: str|Path) -> dict[str,Any]:
    run=Path(run_directory)
    metadata=json.loads((run/"metadata.json").read_text(encoding="utf-8"))
    with (run/"flight_telemetry.csv").open(newline="",encoding="utf-8") as stream:
        all_rows=list(csv.DictReader(stream))
    hover=[row for row in all_rows if row["phase"]=="HOVER"]
    if not hover:raise ValueError("run contains no hover samples")
    hover_start=float(hover[0]["elapsed_s"])
    valid=[]; stable=[]
    for row in hover:
        try:
            raw=float(row["onboard_thrust_raw"]); age=float(row["onboard_thrust_age_s"])
            values={"elapsed_s":float(row["elapsed_s"]),"raw":raw,"age":age,
                    "error":float(row["reference_altitude_m"])-float(row["z_m"]),
                    "vz":float(row["vz_m_s"]),"tilt":math.hypot(float(row["roll_rad"]),float(row["pitch_rad"]))}
        except (ValueError,KeyError):continue
        if all(math.isfinite(value) for value in values.values()) and 0<raw<=65535 and 0<=age<=STABLE_MAX_THRUST_AGE_S:
            valid.append(values)
            if (values["elapsed_s"]-hover_start>=STABLE_SETTLING_S and
                abs(values["error"])<=STABLE_MAX_ABS_ALTITUDE_ERROR_M and
                abs(values["vz"])<=STABLE_MAX_ABS_VERTICAL_SPEED_M_S):stable.append(values)
    if not stable:raise ValueError("no hover samples meet stable-sample criteria")
    raw=[row["raw"] for row in stable]; normalized=[raw_to_toolkit_normalized(value) for value in raw]
    correlations={name:_pearson(raw,[row[key] for row in stable]) for name,key in
                  (("altitude_error_m","error"),("vertical_velocity_m_s","vz"),("tilt_magnitude_rad","tilt"))}
    report={
        "schema_version":1,"analysis":"stage-4b-equilibrium-thrust","run_directory":str(run),
        "selection":{
            "hover_samples":len(hover),"valid_thrust_samples":len(valid),"stable_samples":len(stable),
            "criteria":{"phase":"HOVER","minimum_time_in_hover_s":STABLE_SETTLING_S,
                        "maximum_absolute_altitude_error_m":STABLE_MAX_ABS_ALTITUDE_ERROR_M,
                        "maximum_absolute_vertical_speed_m_s":STABLE_MAX_ABS_VERTICAL_SPEED_M_S,
                        "maximum_thrust_sample_age_s":STABLE_MAX_THRUST_AGE_S}},
        "measured":{"onboard_thrust_available_metadata":metadata.get("onboard_thrust_available"),
                    "hover_thrust_coverage_fraction":len(valid)/len(hover),"stabilizer_thrust_raw":_stats(raw),
                    "pearson_correlation_raw_thrust":correlations},
        "inferred":{"toolkit_normalized_equivalent":_stats(normalized),
                    "conversion":"(stabilizer.thrust - 10001) / (60000 - 10001)",
                    "recommended_transfer_status":"conditional_candidate_not_automatically_transferable"},
        "assumptions_and_limits":[
            "The flashed firmware uses the same legacy control.thrust scale as the inspected official firmware source; the run did not record a firmware revision or build ID.",
            "stabilizer.thrust is the collective controller output before power distribution, per-motor mixing, capping, and battery compensation; it is not PWM, newtons, or per-motor thrust.",
            "The affine conversion is an interface-scale equivalence to this toolkit's manual command path, not a physical thrust calibration.",
            "A single three-second hover does not characterize battery voltage, mass, propellers, ground effect, estimator bias, or run-to-run variation.",
            "Correlations are zero-lag descriptive Pearson correlations; samples are serially correlated and do not establish causality."
        ],
        "source_provenance":{
            "recorded_cflib_version":metadata.get("cflib_version"),"recorded_firmware_version":None,
            "installed_cflib_commander":str(Path(__file__).parents[1]/".venv/lib/python3.10/site-packages/cflib/crazyflie/commander.py"),
            "firmware_stabilizer_source":"https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/stabilizer.c",
            "firmware_pid_source":"https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/controller/controller_pid.c",
            "firmware_commander_source":"https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c"}}
    (run/"equilibrium_thrust_report.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    _write_markdown(run,report);_write_svg(run,stable)
    return report


def _write_markdown(run:Path,report:dict[str,Any])->None:
    measured=report["measured"]["stabilizer_thrust_raw"]; inferred=report["inferred"]["toolkit_normalized_equivalent"]
    correlations=report["measured"]["pearson_correlation_raw_thrust"]
    fmt=lambda value:"unavailable" if value is None else f"{value:.6f}"
    text=f"""# Stage 4B equilibrium-thrust analysis

## Selection

- Hover samples: {report['selection']['hover_samples']}
- Valid thrust samples: {report['selection']['valid_thrust_samples']}
- Stable samples: {report['selection']['stable_samples']}
- Stable means at least 1.0 s into HOVER, |altitude error| <= 0.03 m, |vz| <= 0.03 m/s, and thrust age <= 0.05 s.

## Measured: `stabilizer.thrust`

| statistic | raw firmware units |
|---|---:|
| mean | {measured['mean']:.6f} |
| median | {measured['median']:.6f} |
| standard deviation | {measured['standard_deviation']:.6f} |
| p05 / p25 / p75 / p95 | {measured['p05']:.6f} / {measured['p25']:.6f} / {measured['p75']:.6f} / {measured['p95']:.6f} |

Zero-lag Pearson correlations with raw thrust: altitude error {fmt(correlations['altitude_error_m'])}, vertical velocity {fmt(correlations['vertical_velocity_m_s'])}, tilt magnitude {fmt(correlations['tilt_magnitude_rad'])}.

## Inferred: toolkit manual-command equivalent

Using `(raw - 10001) / 49999`: mean {inferred['mean']:.6f}, median {inferred['median']:.6f}, standard deviation {inferred['standard_deviation']:.6f}, p05/p95 {inferred['p05']:.6f}/{inferred['p95']:.6f}.

This is a conditional candidate for the independent controller, not an automatically transferable or hardware-validated parameter. It is an interface-scale conversion, not force or PWM.

## Firmware relationship and limitations

Official firmware logs `stabilizer.thrust` from `control.thrust`. The PID position controller writes its collective output to that field; power distribution subsequently mixes it into motors and applies battery compensation/capping. Installed cflib 0.1.34 maps toolkit normalized `n` through percent to `int(10001 + n * 49999)` for manual commands. The run did not record the flashed firmware build ID, battery voltage, per-motor outputs, or saturation flags.

Sources: [firmware stabilizer](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/stabilizer.c), [firmware PID controller](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/controller/controller_pid.c), [generic commander](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c), and the installed cflib commander listed in the JSON report.
"""
    (run/"equilibrium_thrust_report.md").write_text(text,encoding="utf-8")


def _write_svg(run:Path,rows:list[dict[str,float]])->None:
    series=[("raw thrust","raw","#111827"),("altitude error [m]","error","#2563eb"),("vertical velocity [m/s]","vz","#059669"),("tilt magnitude [rad]","tilt","#dc2626")]
    width,height,left,right=1000,720,85,25;t=[r["elapsed_s"] for r in rows];duration=max(t[-1]-t[0],1e-9)
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">','<rect width="100%" height="100%" fill="white"/>','<text x="20" y="20" font-family="sans-serif" font-size="16">Stage 4B stable-hover selection</text>']
    for index,(label,key,color) in enumerate(series):
        top=40+index*165;bottom=top+125;vals=[r[key] for r in rows];lo,hi=min(vals),max(vals)
        if lo==hi:lo,hi=lo-.5,hi+.5
        svg += [f'<rect x="{left}" y="{top}" width="{width-left-right}" height="125" fill="none" stroke="#ccc"/>',f'<text x="5" y="{top+15}" font-family="sans-serif" font-size="11">{label}</text>']
        points=" ".join(f'{left+(r["elapsed_s"]-t[0])/duration*(width-left-right):.1f},{bottom-(r[key]-lo)/(hi-lo)*125:.1f}' for r in rows)
        svg.append(f'<polyline points="{points}" fill="none" stroke="{color}"/>')
    svg.append('</svg>');(run/"equilibrium_thrust_analysis.svg").write_text("\n".join(svg)+"\n",encoding="utf-8")
