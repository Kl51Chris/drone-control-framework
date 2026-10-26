from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import random
from statistics import mean
from typing import Any


PULSE_PHASES = {"PITCH_POSITIVE", "PITCH_NEGATIVE", "NEUTRAL_BEFORE",
                "RECOVERY_POSITIVE", "RECOVERY_NEGATIVE"}


def _corr(x,y):
    if len(x)<3:return None
    xm,ym=mean(x),mean(y);den=math.sqrt(sum((v-xm)**2 for v in x)*sum((v-ym)**2 for v in y))
    return sum((a-xm)*(b-ym) for a,b in zip(x,y))/den if den else None


def _slope(x,y):
    xm,ym=mean(x),mean(y);den=sum((v-xm)**2 for v in x)
    return sum((a-xm)*(b-ym) for a,b in zip(x,y))/den if den else 0.0


def _local_acceleration(rows,half_window=.15):
    t=[float(r["elapsed_s"]) for r in rows];vx=[float(r["vx_m_s"]) for r in rows];vy=[float(r["vy_m_s"]) for r in rows]
    result=[]
    for i,center in enumerate(t):
        selected=[j for j,value in enumerate(t) if abs(value-center)<=half_window]
        if len(selected)<8:continue
        tx=[t[j] for j in selected];ax=_slope(tx,[vx[j] for j in selected]);ay=_slope(tx,[vy[j] for j in selected])
        yaw=float(rows[i]["yaw_rad"])
        result.append((i,math.cos(yaw)*ax+math.sin(yaw)*ay,-math.sin(yaw)*ax+math.cos(yaw)*ay))
    return result


def _best_lag(left,right,max_samples):
    candidates=[]
    for lag in range(max_samples+1):
        value=_corr(left[:len(left)-lag or None],right[lag:])
        if value is not None:candidates.append((abs(value),lag,value))
    return max(candidates) if candidates else (0,0,None)


def _bootstrap_slope(x,y,block=5,repetitions=1000):
    rng=random.Random(505);n=len(x);estimates=[]
    if n<20:return None
    for _ in range(repetitions):
        indices=[]
        while len(indices)<n:
            start=rng.randrange(max(1,n-block+1));indices.extend(range(start,min(start+block,n)))
        indices=indices[:n];estimates.append(_slope([x[i] for i in indices],[y[i] for i in indices]))
    estimates.sort();return estimates[int(.025*repetitions)],estimates[int(.975*repetitions)]


def analyze_pitch_response(run_directory: str | Path) -> dict[str,Any]:
    run=Path(run_directory);metadata=json.loads((run/"metadata.json").read_text())
    with (run/"flight_telemetry.csv").open(newline="") as stream:rows=list(csv.DictReader(stream))
    active=[r for r in rows if r.get("pitch_identification_phase") in PULSE_PHASES]
    report={"schema_version":1,"analysis":"pitch-reference-to-body-x-response",
        "source_run":str(run),"software_status":metadata.get("status"),"sample_count":len(rows),
        "identification_sample_count":len(active),"safety_event":metadata.get("safety_event")}
    counts={phase:sum(r.get("pitch_identification_phase")==phase for r in active) for phase in sorted(PULSE_PHASES)}
    report["phase_sample_counts"]=counts
    required=all(counts[p]>=10 for p in ("NEUTRAL_BEFORE","PITCH_POSITIVE","PITCH_NEGATIVE","RECOVERY_POSITIVE","RECOVERY_NEGATIVE"))
    if not required:
        report.update(result="INCONCLUSIVE",reason="required neutral, bipolar pulse, and recovery samples are incomplete")
        return _save(run,report)
    dt=mean(float(r["dt_s"]) for r in active);maximum_lag=max(1,round(.30/dt))
    command=[float(r["pitch_excitation_reference_rad"]) for r in active];pitch=[float(r["pitch_rad"]) for r in active]
    _,attitude_lag,attitude_corr=_best_lag(command,pitch,maximum_lag)
    grouped={phase:[] for phase in PULSE_PHASES};grouped_q={phase:[] for phase in PULSE_PHASES}
    for i in range(len(active)-attitude_lag):
        phase=active[i]["pitch_identification_phase"];grouped[phase].append(float(active[i+attitude_lag]["pitch_rad"]));grouped_q[phase].append(float(active[i+attitude_lag]["q_rad_s"]))
    neutral=mean(grouped["NEUTRAL_BEFORE"]);positive=mean(grouped["PITCH_POSITIVE"])-neutral;negative=mean(grouped["PITCH_NEGATIVE"])-neutral
    command_pass=positive>=.005 and negative<=-.005
    accelerations=_local_acceleration(active);lookup={i:(ax,ay) for i,ax,ay in accelerations}
    valid=[i for i in range(len(active)) if i in lookup];pitch_valid=[float(active[i]["pitch_rad"]) for i in valid];ax=[lookup[i][0] for i in valid]
    _,acceleration_lag,attitude_accel_corr=_best_lag(pitch_valid,ax,maximum_lag)
    x=pitch_valid[:len(pitch_valid)-acceleration_lag or None];y=ax[acceleration_lag:]
    physical_slope=_slope(x,y);interval=_bootstrap_slope(x,y)
    if interval and interval[0]>0:identified_sign=1
    elif interval and interval[1]<0:identified_sign=-1
    else:identified_sign=None
    pitch_saturation=sum(int(r.get("pitch_rate_saturated",0) or 0) for r in active)/len(active)
    initial_yaw=float(active[0]["yaw_rad"]);initial_vx=float(active[0]["vx_m_s"]);initial_vy=float(active[0]["vy_m_s"])
    initial_body_velocity={"x_m_s":math.cos(initial_yaw)*initial_vx+math.sin(initial_yaw)*initial_vy,
                           "y_m_s":-math.sin(initial_yaw)*initial_vx+math.cos(initial_yaw)*initial_vy}
    accepted=command_pass and identified_sign is not None and metadata.get("safety_event") is None and pitch_saturation<=.05
    report.update({
        "initial_velocity":{"world_x_m_s":initial_vx,"world_y_m_s":initial_vy,
                            "body":initial_body_velocity},
        "command_to_attitude":{"best_lag_s":attitude_lag*dt,"correlation":attitude_corr,
            "neutral_mean_pitch_rad":neutral,"positive_pulse_delta_pitch_rad":positive,
            "negative_pulse_delta_pitch_rad":negative,"mean_q_rad_s":{"positive":mean(grouped_q["PITCH_POSITIVE"]),"negative":mean(grouped_q["PITCH_NEGATIVE"])},
            "polarity_pass":command_pass},
        "attitude_to_body_x_acceleration":{"method":"centered 0.30 s local velocity regression, yaw rotation, lag scan, 5-sample moving-block bootstrap",
            "best_lag_s":acceleration_lag*dt,"correlation":attitude_accel_corr,
            "slope_m_s2_per_rad":physical_slope,"slope_95_percent_block_bootstrap":interval,
            "identified_pitch_mapping_sign":identified_sign},
        "pitch_rate_saturation_fraction":pitch_saturation,
        "result":"PASS" if accepted else "INCONCLUSIVE",
        "reason":"both command-to-attitude polarity and physical attitude-to-acceleration sign identified without a safety event" if accepted else "polarity response, physical acceleration confidence, saturation, or safety criteria were not all satisfied",
        "automatic_configuration_change":False})
    return _save(run,report)


def _save(run,report):
    (run/"pitch_response_analysis.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n")
    return report
