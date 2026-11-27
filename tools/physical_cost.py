"""Shared realized cost for CrazySim tracking data, independent of solver penalties."""

import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from tools.analyze_mpc_batch import describe
from tools.shared_dataset import INPUT, OUTPUT, array, read_csv
from velocity_control_framework.backends.crazysim_command_adapter import CrazySimCommandAdapter as Adapter
from velocity_control_framework.controllers.near_hover_model import world_to_fixed_yaw


def realized(truth,reference,commands,q,r,s,thrust_eq,dt,frame_yaw):
    """Integrate physical cost over body-rate tracking, including the bridge."""
    error=truth-reference
    error[:,:3]=error[:,:3]@world_to_fixed_yaw(frame_yaw).T
    error[:,3:6]=error[:,3:6]@world_to_fixed_yaw(frame_yaw).T
    error[:,8]=(error[:,8]+np.pi)%(2*np.pi)-np.pi
    controls=commands-np.asarray([0.,0.,0.,thrust_eq])
    increments=np.diff(np.vstack((np.zeros(4),controls)),axis=0)
    terms=np.column_stack((np.square(error)@q,np.square(controls)@r,np.square(increments)@s))
    integral=dt@terms
    return integral.tolist()+[float(integral.sum())]


def analyze(folder):
    summary,_=describe(folder)
    f=next(folder.glob("*/flight.csv"))
    rows=[r for r in read_csv(f) if r[INPUT[-1]]]
    meta=json.loads(f.with_name("metadata.json").read_text())
    cfg=meta["controller_configuration"]
    sim=read_csv(folder/"sim_truth.csv")
    t=array(sim,["monotonic_s"])[:,0]
    obs=array(rows,["estimated_timestamp_s"])[:,0]
    angles=Rotation.from_quat(array(sim,["qx","qy","qz","qw"])).as_euler("xyz")
    angles[:,1]*=-1
    state=np.column_stack((array(sim,["x","y","z","vx","vy","vz"]),angles))
    aligned=np.column_stack([np.interp(obs,t,col) for col in state.T])
    refs=np.column_stack((array(rows,[f"reference_position_{a}_m" for a in "xyz"]+
                             [f"reference_velocity_{a}_m_s" for a in "xyz"]),
                         np.zeros((len(rows),2)),array(rows,["reference_yaw_rad"])))
    u=array(rows,INPUT)
    u[:,3]=[(Adapter._encode_thrust(v)-Adapter.RAW_THRUST_MIN) /
            (Adapter.RAW_THRUST_MAX-Adapter.RAW_THRUST_MIN) for v in u[:,3]]
    dt=array(rows,["control_dt_s"])[:,0]
    eq=cfg["model"]["equilibrium_thrust"]
    yaw=float(rows[0]["estimated_yaw_rad"])
    args=[refs,u,cfg["q"],cfg["r"],cfg["s"],eq,dt,yaw]
    names=("tracking","effort","increment","total")
    cost=realized(aligned,*args)
    estimate_cost=realized(array(rows,OUTPUT),*args)
    diagnostics=[json.loads(row["controller_diagnostics_json"]).get("mpc",{}) for row in rows]
    active={}
    for d in diagnostics:
        name=d.get("active_controller","mpc")
        active[name]=active.get(name,0)+1
    summary.update(physical_truth=dict(zip(names,cost)),physical_estimated=dict(zip(names,estimate_cost)),
        physical_truth_per_second=dict(zip(names,np.asarray(cost)/dt.sum())),
        active_controller_steps=active)
    actual_local=aligned.copy()
    rotation=world_to_fixed_yaw(yaw)
    actual_local[:,:3]=(actual_local[:,:3]-array(rows,OUTPUT)[0,:3])@rotation.T
    actual_local[:,3:6]=actual_local[:,3:6]@rotation.T
    actual_local[:,8]=(actual_local[:,8]-yaw+np.pi)%(2*np.pi)-np.pi
    summary["truth_soft_box_exceedance_steps"]=int(np.count_nonzero(
        np.any(np.abs(actual_local)>np.asarray(cfg["bounds"]),axis=1)))
    summary.pop("actual_input_model_rmse",None)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    p.add_argument("batches",type=Path,nargs="+")
    args=p.parse_args()
    result=[]
    for batch in args.batches:
        for row in sorted(batch.glob("*/experiment.json")):
            try:
                result.append(analyze(row.parent))
            except (ValueError,IndexError,KeyError,StopIteration) as error:
                meta=json.loads(row.read_text())
                result.append({**meta["case"],"status":meta["status"],"analysis_error":str(error)})
    args.output.write_text(json.dumps(result,indent=2))
    scored=[r for r in result if "physical_truth" in r]
    if not scored:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    x=np.arange(len(scored))
    offset=np.zeros(len(scored))
    for term in ("tracking","effort","increment"):
        level=np.array([r["physical_truth_per_second"][term] for r in scored])
        axes[0].bar(x,level,bottom=offset,label=term)
        offset+=level
    axes[0].set_ylabel("Realized truth cost / s")
    axes[0].legend()
    axes[1].bar(x,[r["solver_p99_ms"] for r in scored])
    axes[1].set(ylabel="QP P99 [ms]",xticks=x,xticklabels=[r["name"] for r in scored])
    fig.tight_layout()
    fig.savefig(args.output.with_suffix(".png"))


if __name__=="__main__":
    main()
