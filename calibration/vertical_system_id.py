from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def _load_intervals(path: Path, delay_samples: int) -> tuple[np.ndarray,np.ndarray]:
    metadata=json.loads((path/"metadata.json").read_text(encoding="utf-8"))
    ground=float(metadata["captured_reference"]["ground_z_m"])
    with (path/"flight_telemetry.csv").open(newline="",encoding="utf-8") as stream:
        rows=list(csv.DictReader(stream))
    design=[];increments=[]
    for index in range(delay_samples,len(rows)-1):
        first,last=rows[index],rows[index+1]
        dt=float(last["monotonic_s"])-float(first["monotonic_s"])
        if (first["phase"] not in {"TAKEOFF","HOVER"} or last["phase"] not in {"TAKEOFF","HOVER"}
                or min(float(first["z_m"]),float(last["z_m"]))-ground<.02
                or not .01<=dt<=.04):
            continue
        velocity_first=float(first["vz_m_s"]);velocity_last=float(last["vz_m_s"])
        thrust=float(rows[index-delay_samples]["command_thrust_normalized"])
        # Integral model over one telemetry interval. Thrust is zero-order held;
        # velocity uses trapezoidal integration to avoid differentiating vz.
        design.append((dt*(thrust-.425),-dt*.5*(velocity_first+velocity_last),dt))
        increments.append(velocity_last-velocity_first)
    return np.asarray(design),np.asarray(increments)


def _fit(groups: list[tuple[np.ndarray,np.ndarray]]) -> tuple[np.ndarray,float,float]:
    design=np.vstack([group[0] for group in groups]);response=np.concatenate([group[1] for group in groups])
    coefficients=np.linalg.lstsq(design,response,rcond=None)[0]
    residual=response-design@coefficients
    return coefficients,float(np.sqrt(np.mean(residual**2))),float(np.linalg.cond(design))


def _candidate_replay(path: Path, kp: float, kv: float, track_reference: bool) -> dict[str,Any]:
    metadata=json.loads((path/"metadata.json").read_text(encoding="utf-8"));profile=metadata["flight_config"]["profile"]
    with (path/"flight_telemetry.csv").open(newline="",encoding="utf-8") as stream: rows=list(csv.DictReader(stream))
    phases={};saturated=0
    for row in rows:
        reference_velocity=(profile["takeoff_rate_m_s"] if row["phase"]=="TAKEOFF" else
            -profile["landing_rate_m_s"] if row["phase"]=="LANDING" else 0.0) if track_reference else 0.0
        raw=kp*(float(row["reference_z_m"])-float(row["z_m"]))+kv*(reference_velocity-float(row["vz_m_s"]))
        correction=min(max(raw,-.045),.025);saturated+=int(correction!=raw)
        phases.setdefault(row["phase"],[]).append(.425+correction)
    return {"overall_thrust_range":[min(value for values in phases.values() for value in values),
                                     max(value for values in phases.values() for value in values)],
        "saturation_samples":saturated,
        "phase_thrust_ranges":{phase:[min(values),max(values)] for phase,values in phases.items()}}


def identify_vertical_dynamics(p_only_run: str | Path, pd_run: str | Path,
                               output_directory: str | Path,
                               bootstrap_samples: int = 1000) -> dict[str,Any]:
    paths=[Path(p_only_run),Path(pd_run)];delay_scan=[]
    for delay in range(9):
        groups=[_load_intervals(path,delay) for path in paths]
        coefficients,rmse,condition=_fit(groups)
        delay_scan.append({"delay_samples":delay,"delay_s":delay*.02,"a":float(coefficients[0]),
            "b":float(coefficients[1]),"c":float(coefficients[2]),"velocity_increment_rmse_m_s":rmse,
            "approximate_acceleration_rmse_m_s2":rmse/.02,"design_condition_number":condition})
    selected=min(delay_scan,key=lambda item:item["velocity_increment_rmse_m_s"])
    delay=int(selected["delay_samples"]);groups=[_load_intervals(path,delay) for path in paths]
    coefficients,rmse,condition=_fit(groups)
    leave_one_out=[]
    for test_index in range(2):
        train=[groups[1-test_index]];test_design,test_response=groups[test_index]
        held_coefficients,_,_=_fit(train);residual=test_response-test_design@held_coefficients
        leave_one_out.append({"test_run":str(paths[test_index]),"train_run":str(paths[1-test_index]),
            "a":float(held_coefficients[0]),"b":float(held_coefficients[1]),"c":float(held_coefficients[2]),
            "velocity_increment_rmse_m_s":float(np.sqrt(np.mean(residual**2)))})
    train_design=[];train_response=[];test_design=[];test_response=[]
    for design,response in groups:
        boundary=int(.7*len(response));train_design.append(design[:boundary]);train_response.append(response[:boundary])
        test_design.append(design[boundary:]);test_response.append(response[boundary:])
    train_coefficients=np.linalg.lstsq(np.vstack(train_design),np.concatenate(train_response),rcond=None)[0]
    def validation(designs,responses):
        design=np.vstack(designs);response=np.concatenate(responses);residual=response-design@train_coefficients
        return {"samples":len(response),"velocity_increment_rmse_m_s":float(np.sqrt(np.mean(residual**2))),
            "r_squared":float(1-np.sum(residual**2)/np.sum((response-response.mean())**2))}
    rng=np.random.default_rng(20261026);bootstrap=[];block=25
    for _ in range(bootstrap_samples):
        sampled=[]
        for design,response in groups:
            indices=[]
            while len(indices)<len(response):
                start=int(rng.integers(0,max(1,len(response)-block+1)))
                indices.extend(range(start,min(start+block,len(response))))
            chosen=np.asarray(indices[:len(response)]);sampled.append((design[chosen],response[chosen]))
        bootstrap.append(_fit(sampled)[0])
    distribution=np.asarray(bootstrap);low,median,high=np.percentile(distribution,[2.5,50,97.5],axis=0)
    a,b,c=(float(value) for value in coefficients)
    frequencies=[]
    for omega in (.9,1.1,1.3,1.5,math.sqrt(a*.1)):
        kp=omega**2/a;kv=(2*.7*omega-b)/a
        frequencies.append({"omega_n_rad_s":omega,"kp":kp,"kv":kv})
    candidate_kp=candidate_kv=.06
    positive=distribution[:,0]>0
    candidate_omega=np.sqrt(distribution[positive,0]*candidate_kp)
    candidate_zeta=(distribution[positive,1]+distribution[positive,0]*candidate_kv)/(2*candidate_omega)
    candidate={"kp":candidate_kp,"kv":candidate_kv,
        "nominal_omega_n_rad_s":math.sqrt(a*candidate_kp),
        "nominal_damping_ratio":(b+a*candidate_kv)/(2*math.sqrt(a*candidate_kp)),
        "bootstrap_95_percent_omega_n_rad_s":[float(np.percentile(candidate_omega,2.5)),float(np.percentile(candidate_omega,97.5))],
        "bootstrap_95_percent_damping_ratio":[float(np.percentile(candidate_zeta,2.5)),float(np.percentile(candidate_zeta,97.5))],
        "bootstrap_stable_fraction":float(np.mean(candidate_zeta>0)),
        "historical_state_replay":{
            paths[0].name:{"zero_reference":_candidate_replay(paths[0],candidate_kp,candidate_kv,False),
                           "ramp_tracking":_candidate_replay(paths[0],candidate_kp,candidate_kv,True)},
            paths[1].name:{"zero_reference":_candidate_replay(paths[1],candidate_kp,candidate_kv,False),
                           "ramp_tracking":_candidate_replay(paths[1],candidate_kp,candidate_kv,True)}}}
    report={"schema_version":1,"model":"z_dot=vz; vz_dot=a*(u-0.425)-b*vz+c",
        "selected_delay_s":selected["delay_s"],"sample_count":sum(len(group[1]) for group in groups),
        "estimate":{"a_m_s2_per_normalized_thrust":a,"b_per_s":b,"c_m_s2":c,
            "effective_equilibrium_thrust_if_model_literal":.425-c/a},
        "block_bootstrap_95_percent":{"a":[float(low[0]),float(high[0])],"b":[float(low[1]),float(high[1])],
            "c":[float(low[2]),float(high[2])],"median":[float(value) for value in median],
            "positive_a_fraction":float(np.mean(distribution[:,0]>0))},
        "fit":{"velocity_increment_rmse_m_s":rmse,"approximate_acceleration_rmse_m_s2":rmse/.02,
            "design_condition_number":condition},"delay_scan":delay_scan,"leave_one_flight_out":leave_one_out,
        "contiguous_70_30_validation":{"train":validation(train_design,train_response),
                                       "held_out":validation(test_design,test_response)},
        "pole_placement_zeta_0_7":frequencies,"recommended_candidate":candidate,
        "limitations":["closed-loop thrust is correlated with altitude and velocity",
            "a and c are strongly confounded and the fitted damping is not stable across flights",
            "the delay minimum is shallow and timestamps do not measure motor response directly",
            "the model is local, empirical, and does not include ground effect, battery voltage, or nonlinear thrust"],
        "interpretation":"Use for conservative gain screening only; coefficients are not established physical constants."}
    destination=Path(output_directory);destination.mkdir(parents=True,exist_ok=True)
    (destination/"vertical_system_id.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    return report
