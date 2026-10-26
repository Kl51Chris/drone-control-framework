from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable

from .cascaded_pid import (
    AltitudeLoopConfig, AttitudeLoopConfig, CascadedPIDConfig,
    CascadedPIDController, CascadedReference, HorizontalLoopConfig,
)
from .state import AltitudeState


DEFAULT_VELOCITY_GAINS = (0.0, 0.02, 0.04, 0.06, 0.08)


def _controller_config(raw: dict[str, Any], velocity_kp: float) -> CascadedPIDConfig:
    config = CascadedPIDConfig(
        schema_version=raw["schema_version"], stage="5C",
        equilibrium_thrust=raw["equilibrium_thrust"], min_thrust=raw["min_thrust"],
        max_thrust=raw["max_thrust"], altitude=AltitudeLoopConfig(**raw["altitude"]),
        attitude=AttitudeLoopConfig(**raw["attitude"]),
        horizontal=HorizontalLoopConfig(**raw["horizontal"]),
        hardware_flight_authorized=False,
        powered_attitude_validated=raw["powered_attitude_validated"],
        horizontal_mapping_hardware_validated=raw["horizontal_mapping_hardware_validated"],
    )
    config.altitude.velocity_kp = velocity_kp
    config.altitude.velocity_feedback_enabled = velocity_kp > 0
    config.altitude.integral_ki = 0.0
    config.altitude.integral_enabled = False
    config.validate()
    return config


def _reference_velocity(phase: str, profile: dict[str, float]) -> float:
    if phase == "TAKEOFF":
        return profile["takeoff_rate_m_s"]
    if phase == "LANDING":
        return -profile["landing_rate_m_s"]
    return 0.0


def _smoothed_extrema(rows: list[dict[str, str]], phase: str) -> list[dict[str, float | str]]:
    selected = [row for row in rows if row["phase"] == phase]
    altitude = [float(row["z_m"]) for row in selected]
    if len(altitude) < 7:
        return []
    smooth = [sum(altitude[max(0, i-3):min(len(altitude), i+4)]) /
              len(altitude[max(0, i-3):min(len(altitude), i+4)]) for i in range(len(altitude))]
    extrema=[]
    for i in range(1,len(smooth)-1):
        kind = "peak" if smooth[i-1] < smooth[i] >= smooth[i+1] else (
            "trough" if smooth[i-1] > smooth[i] <= smooth[i+1] else "")
        if kind:
            extrema.append({"kind":kind,"elapsed_s":float(selected[i]["elapsed_s"]),
                            "smoothed_altitude_m":smooth[i]})
    return extrema


def _write_svg(path: Path, rows: list[dict[str, str]], replay: list[dict[str, Any]], gains: tuple[float,...]) -> None:
    width,height,left,right,top,bottom=1000,620,70,25,30,45
    times=[float(row["elapsed_s"]) for row in rows];tmax=max(times)
    altitude=[float(row["z_m"]) for row in rows];reference=[float(row["reference_z_m"]) for row in rows]
    vz=[float(row["vz_m_s"]) for row in rows]
    def points(values,y0,y1,vmin,vmax):
        return " ".join(f"{left+t/tmax*(width-left-right):.1f},{y1-(v-vmin)/(vmax-vmin)*(y1-y0):.1f}" for t,v in zip(times,values))
    colors=["#777","#2ca02c","#d62728","#9467bd","#8c564b"]
    lines=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<text x="70" y="18" font-family="sans-serif" font-size="15">Stage 5C recorded response and exogenous-state thrust replay</text>',
           f'<polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{points(altitude,35,245,min(altitude+reference),max(altitude+reference))}"/>',
           f'<polyline fill="none" stroke="#111" stroke-dasharray="5 4" points="{points(reference,35,245,min(altitude+reference),max(altitude+reference))}"/>',
           f'<polyline fill="none" stroke="#ff7f0e" stroke-width="1.5" points="{points(vz,270,430,min(vz),max(vz))}"/>']
    for color,gain in zip(colors,gains):
        values=[float(row[f"thrust_kv_{gain:.3f}"]) for row in replay]
        lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.3" points="{points(values,455,570,.38,.45)}"/>')
    lines += ['<text x="75" y="50" font-family="sans-serif" font-size="12">altitude (blue), reference (black dashed)</text>',
              '<text x="75" y="287" font-family="sans-serif" font-size="12">vertical velocity</text>',
              '<text x="75" y="472" font-family="sans-serif" font-size="12">predicted thrust; Kv=0/.02/.04/.06/.08</text>',
              f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#333"/>',
              f'<text x="{width/2}" y="610" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed seconds (0 to {tmax:.2f})</text>', '</svg>']
    path.write_text("\n".join(lines)+"\n",encoding="utf-8")


def analyze_altitude_pd(run_directory: str | Path, output_directory: str | Path,
                        velocity_gains: Iterable[float] = DEFAULT_VELOCITY_GAINS) -> dict[str, Any]:
    source,destination=Path(run_directory),Path(output_directory)
    metadata=json.loads((source/"metadata.json").read_text(encoding="utf-8"))
    with (source/"flight_telemetry.csv").open(newline="",encoding="utf-8") as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        raise ValueError("altitude PD analysis requires telemetry")
    gains=tuple(float(value) for value in velocity_gains)
    if any(value<0 or not math.isfinite(value) for value in gains):
        raise ValueError("velocity gains must be finite and nonnegative")
    controllers={gain:CascadedPIDController(_controller_config(metadata["controller_config"],gain)) for gain in gains}
    profile=metadata["flight_config"]["profile"]; replay=[]
    summaries={gain:{"thrust":[],"velocity":[],"saturated":0} for gain in gains}
    for row in rows:
        state=AltitudeState(float(row["z_m"]),float(row["vz_m_s"]),float(row["roll_rad"]),
            float(row["pitch_rad"]),float(row["yaw_rad"]),float(row["monotonic_s"]),"recorded_replay",
            float(row["x_m"]),float(row["y_m"]),float(row["vx_m_s"]),float(row["vy_m_s"]),
            float(row["p_rad_s"]),float(row["q_rad_s"]),float(row["r_rad_s"]))
        reference=CascadedReference(float(row["reference_z_m"]),
            vertical_velocity_m_s=_reference_velocity(row["phase"],profile),
            yaw_rad=float(row["reference_yaw_rad"]),position_x_m=float(row["reference_x_m"]),
            position_y_m=float(row["reference_y_m"]))
        output={"elapsed_s":float(row["elapsed_s"]),"phase":row["phase"],
            "reference_z_m":reference.altitude_m,"reference_vz_m_s":reference.vertical_velocity_m_s,
            "z_m":state.altitude_m,"vz_m_s":state.vertical_velocity_m_s}
        for gain,controller in controllers.items():
            result=controller.update(state,reference,float(row["dt_s"])); key=f"{gain:.3f}"
            output[f"altitude_velocity_kv_{key}"]=result.contributions["altitude_velocity"]
            output[f"thrust_kv_{key}"]=result.command.thrust
            output[f"saturated_kv_{key}"]=result.contributions["altitude_saturated"]
            summaries[gain]["thrust"].append(result.command.thrust)
            summaries[gain]["velocity"].append(result.contributions["altitude_velocity"])
            summaries[gain]["saturated"]+=result.contributions["altitude_saturated"]
        replay.append(output)
    extrema=_smoothed_extrema(rows,"HOVER")
    major=[item for item in extrema if item["smoothed_altitude_m"]>0.2 or item["smoothed_altitude_m"]<0.1]
    oscillation={"hover_smoothed_extrema":major,
        "estimate_quality":"limited: the constant-reference HOVER interval contains only one clear peak-to-trough half-cycle"}
    if len(major)>=2:
        pair=next(((a,b) for a,b in zip(major,major[1:]) if a["kind"]!=b["kind"]),None)
        if pair:
            oscillation.update(peak_to_trough_m=abs(pair[0]["smoothed_altitude_m"]-pair[1]["smoothed_altitude_m"]),
                half_amplitude_m=abs(pair[0]["smoothed_altitude_m"]-pair[1]["smoothed_altitude_m"])/2,
                estimated_period_s=2*abs(pair[1]["elapsed_s"]-pair[0]["elapsed_s"]))
    candidate_summary={}
    sample_count=len(rows)
    for gain,values in summaries.items():
        thrust=values["thrust"];velocity=values["velocity"]
        candidate_summary[f"{gain:.3f}"]={"minimum_thrust":min(thrust),"maximum_thrust":max(thrust),
            "mean_thrust":sum(thrust)/sample_count,"minimum_velocity_contribution":min(velocity),
            "maximum_velocity_contribution":max(velocity),"saturation_samples":values["saturated"],
            "saturation_duration_s":sum(float(row["dt_s"]) for row,out in zip(rows,replay)
                if int(out[f"saturated_kv_{gain:.3f}"]))}
    destination.mkdir(parents=True,exist_ok=True)
    with (destination/"altitude_pd_replay.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(replay[0]));writer.writeheader();writer.writerows(replay)
    _write_svg(destination/"altitude_pd_replay.svg",rows,replay,gains)
    report={"schema_version":1,"analysis":"stage5c-altitude-pd-exogenous-state-replay",
        "source_run":str(source),"sample_count":sample_count,"duration_s":float(rows[-1]["elapsed_s"]),
        "maximum_altitude_m":max(float(row["z_m"]) for row in rows),
        "maximum_absolute_vertical_velocity_m_s":max(abs(float(row["vz_m_s"])) for row in rows),
        "oscillation":oscillation,"candidate_velocity_gains":candidate_summary,
        "reference_velocity_semantics":{"TAKEOFF":profile["takeoff_rate_m_s"],"HOVER":0.0,
            "LANDING":-profile["landing_rate_m_s"],"SETTLE":0.0,"LANDING_CONFIRM":0.0},
        "interpretation":"Recorded states are exogenous. Replay compares controller signs, contributions, bounds, and saturation; it is not a closed-loop simulation and cannot predict the damped trajectory."}
    (destination/"altitude_pd_analysis.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    return report
