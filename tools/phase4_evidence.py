"""Offline Phase 4 true-state, timing, initialization and prediction scores."""
import argparse
import json
from pathlib import Path

import numpy as np

from tools.analyze_mpc_batch import describe, read_csv, values
from velocity_control_framework.controllers.near_hover_model import world_to_fixed_yaw


def score(case):
    summary,_=describe(case)
    flight=next(case.glob("*/flight.csv"),None)
    if flight is None:
        return summary
    rows=read_csv(flight)
    tracking=[row for row in rows if row["command_thrust_normalized"]]
    summary["tracking_steps"]=len(tracking)
    diagnostics=[json.loads(r["controller_diagnostics_json"]) for r in tracking]
    solver=[d.get("mpc",{}) for d in diagnostics]
    modes=[d.get("active_controller","mpc") for d in solver]
    summary["active_controller_steps"]={mode:modes.count(mode) for mode in sorted(set(modes))}
    for key in ("g_norm","init_slack_rms","regularization_cost","qp_elapsed_s","tracking_cost","effort_cost","increment_cost"):
        entries=[d[key] for d in solver if key in d]
        if entries:
            summary[key+"_p50_p95_p99_max"]=np.percentile(entries,[50,95,99,100]).tolist()
    noise=[d["measurement_noise"] for d in diagnostics if d.get("measurement_noise")]
    summary["noise_samples"]=len(noise)
    if noise:
        summary["noise_component_rms"]=np.sqrt(np.mean(np.asarray([d["delta"] for d in noise])**2,axis=0)).tolist()
    summary["fallback_reasons"]=sorted({
        d["fallback_reason"] for row in rows
        if (d:=json.loads(row["controller_diagnostics_json"])).get("fallback_reason")
    })
    predictions=flight.with_name("predictions.jsonl")
    if not predictions.exists() or not tracking:
        return summary
    truth=read_csv(case/"sim_truth.csv")
    truth_t=values(truth,["monotonic_s"])[:,0]
    truth_p=values(truth,["x","y","z"])
    sent={int(row["step_index"]): row for row in tracking}
    last=float(tracking[-1]["estimated_timestamp_s"])
    sq={}
    with predictions.open() as stream:
        for line in stream:
            item=json.loads(line)
            if item["step_index"] not in sent:
                continue
            y=np.asarray(item["state"])[1:,:3]@world_to_fixed_yaw(item["yaw"])+item["origin"]
            origin=float(sent[item["step_index"]]["estimated_timestamp_s"])
            for step in (1,10,25):
                t=origin+step*.02
                if step>len(y) or t>last or t<truth_t[0] or t>truth_t[-1]:
                    continue
                actual=np.asarray([np.interp(t,truth_t,axis) for axis in truth_p.T])
                sq.setdefault(step,[]).append((y[step-1]-actual)**2)
    summary["online_truth_plan_error_mm"]={
        str(step):{"windows":len(err),"xyz_rmse":(1000*np.sqrt(np.mean(err,axis=0))).tolist()}
        for step,err in sq.items()
    }
    coverage=values(tracking,["estimated_velocity_x_m_s","estimated_velocity_y_m_s","estimated_velocity_z_m_s",
                             "estimated_roll_rad","estimated_pitch_rad","estimated_yaw_rad",
                             "command_roll_rate_rad_s","command_pitch_rate_rad_s","command_yaw_rate_rad_s",
                             "command_thrust_normalized"])
    summary["coverage_p01_p99_velocity_attitude_command"]=np.percentile(coverage,[1,99],axis=0).tolist()
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("batches",nargs="+",type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    result=[]
    for batch in args.batches:
        for manifest in sorted(batch.glob("*/experiment.json")):
            try:
                result.append(score(manifest.parent))
            except (ValueError,KeyError,IndexError) as error:
                case=json.loads(manifest.read_text())
                result.append({**case["case"],"status":case["status"],"analysis_error":str(error)})
    (args.output/"summary.json").write_text(json.dumps(result,indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for name,prefixes in (("mpc",("m0-","hybrid-","joint-")),("deepc",("deepc-",))):
        scored=[r for r in result if "tracking_rmse_xyz_yaw" in r and r["name"].startswith(prefixes)]
        if not scored:
            continue
        fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
        x=np.arange(len(scored))
        axes[0].bar(x,[1000*np.linalg.norm(r["tracking_rmse_xyz_yaw"][:3]) for r in scored])
        axes[1].bar(x,[r["solver_p99_ms"] for r in scored])
        axes[2].bar(x,[r.get("g_norm_p50_p95_p99_max",[0,0])[1] for r in scored])
        axes[0].set_ylabel("Truth XYZ RMSE [mm]")
        axes[1].set_ylabel("QP P99 [ms]")
        axes[2].set_ylabel("g norm P95")
        axes[2].set_xticks(x,[r["name"] for r in scored],rotation=30,ha="right")
        fig.tight_layout()
        fig.savefig(args.output/f"{name}-comparison.png")
        plt.close(fig)


if __name__=="__main__":
    main()
