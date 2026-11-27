"""Generate presentation figures from archived summaries. No simulation entry points."""
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPO = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent/"figures"
BLUE, GREEN, GRAY, GOLD = "#275787", "#187e70", "#667481", "#bd6d17"
PALETTE = [BLUE, GREEN, GOLD, GRAY]
plt.rcParams.update({"font.size":11,"axes.spines.top":False,"axes.spines.right":False,
                     "axes.grid":True,"grid.alpha":.20,"axes.axisbelow":True,
                     "legend.frameon":False,"figure.dpi":120,"savefig.facecolor":"white"})


def data(path):
    snapshot = OUT.parent/"snapshots"/"log"/path
    return json.loads((snapshot if snapshot.exists() else REPO/"log"/path).read_text())


def make(n, title, width=11):
    fig,ax=plt.subplots(1,n,figsize=(width,4.8),layout="constrained",squeeze=False)
    fig.suptitle(title,fontsize=17,fontweight="bold")
    return fig,ax[0]


def save(fig,name):
    for suffix in ("png","svg"):
        fig.savefig(OUT/f"{name}.{suffix}",dpi=195)
    plt.close(fig)


def axis_labels(ax,labels):
    ax.set_xticks(np.arange(len(labels)),labels)


def bars(ax,labels,series):
    width=.73/len(series)
    for i,(label,y) in enumerate(series):
        pos=np.arange(len(labels))+(i-(len(series)-1)/2)*width
        y=np.asarray(y,dtype=float)
        ax.bar(pos[np.isfinite(y)],y[np.isfinite(y)],width=.93*width,label=label,color=PALETTE[i])
        if np.any(~np.isfinite(y)):
            ax.scatter(pos[~np.isfinite(y)],np.zeros(np.count_nonzero(~np.isfinite(y)))+.001,
                       marker="x",color="#a92d2d",s=68,zorder=5,clip_on=False)
    axis_labels(ax,labels)
    ax.margins(y=.18)


def model():
    raw=data("phase43-diagnosis/orders.json")["variants"]
    tested=data("phase43-diagnosis/physical.json")
    keys=["M0","joint-fir6","axis-r6-p6-y25-t12","axis-r10-p10-y16-t12","balanced-low-fir25"]
    labels=["M0","Joint\nFIR6","Axis FIR\n6/6/25/12","Axis FIR\n10/10/16/12","Weighted\nFIR25"]
    fig,a=make(3,"MPC model complexity, prediction and closed-loop cost | Phase 4.3",15)
    a[0].bar(np.arange(5),[raw[k]["state_dimension"] for k in keys],color=BLUE)
    axis_labels(a[0],labels)
    a[0].set(ylabel="Implemented state dimension",title="Model size")
    bars(a[1],labels,[(f"{n} steps",[1e3*np.sqrt(np.mean(np.square(raw[k]["validation_rmse"][n][-1][:3]))) for k in keys])
                       for n in ("25","40")])
    a[1].set(ylabel="Terminal position RMSE (mm per axis, RMS)",title="Shared validation replay")
    a[1].legend()
    lookup={r["name"]:r["physical_truth_per_second"]["total"] for r in tested}
    bars(a[2],labels,[(title,[lookup[f"{k.lower()}-{scene}"] for k in keys])
                       for title,scene in (("Hover","hover"),("+X step","x"),("Circle","circle"))])
    a[2].set(ylabel="Realized physical cost per second",title="N=25 nominal, one run per cell")
    a[2].legend()
    save(fig,"mpc_model_tradeoffs")


def phase2():
    raw=data("mpc-phase2-horizon-repeat/summary.json")
    steps=[15,25,40]
    groups={n:[r for r in raw if r["name"]=="baseline-circle" and r["horizon"]==n] for n in steps}
    fig,a=make(2,"MPC horizon sweep | Phase 2, matched preview circle")
    for i,axis in enumerate(("X","Y")):
        a[0].plot(steps,[1e3*mean(r["tracking_rmse_xyz_yaw"][i] for r in groups[n]) for n in steps],
                  "o-",label=axis,color=PALETTE[i])
    a[0].set(ylabel="Truth RMSE (mm)",xlabel="Prediction steps at 50 Hz",xticks=steps)
    a[0].legend()
    a[1].plot(steps,[mean(r["solver_p99_ms"] for r in groups[n]) for n in steps],"o-",color=GREEN)
    a[1].set(ylabel="Mean run-level QP P99 (ms)",xlabel="Prediction steps at 50 Hz",xticks=steps)
    save(fig,"prediction_horizon")
    raw=data("mpc-phase2-compare/summary.json")
    groups={m:[r for r in raw if r["mode"]==m] for m in ("constant","preview")}
    fig,a=make(2,"Known trajectory preview | Phase 2, matched N=25 circles")
    bars(a[0],["X","Y"],[(m.capitalize(),[1e3*mean(r["tracking_rmse_xyz_yaw"][i] for r in group)
                    for i in range(2)]) for m,group in groups.items()])
    a[0].set(ylabel="Mean truth RMSE across runs (mm)")
    a[0].legend()
    a[1].bar([0,1],[mean(r["phase_lag_s"] for r in rs) for rs in groups.values()],color=PALETTE[:2])
    axis_labels(a[1],["Constant","Preview"])
    a[1].set(ylabel="Mean run-level phase lag (s)")
    save(fig,"reference_preview")


def hankel():
    spectrum=data("phase3-v1/quality.json")["quality"]["spectrum"]
    fig,a=make(2,"Shared phase3-v1 training Hankel | scaled inputs")
    for (depth,s),color in zip(spectrum.items(),PALETTE):
        v=s["relative_spectrum"]
        a[0].semilogy(np.arange(1,len(v)+1),v,label=f"Depth {depth}",color=color)
    a[0].axhline(1e-4,color=GRAY,ls="--",label="Rank threshold")
    a[0].set(ylabel="Relative singular value",xlabel="Singular value index")
    a[0].legend()
    labels=list(spectrum)
    bars(a[1],labels,[("Numerical rank",[s["rank_1e-4"] for s in spectrum.values()]),
                      ("Rows",[s["rows"] for s in spectrum.values()])])
    a[1].set(xlabel="Hankel depth (samples)",ylabel="Rank / row count")
    a[1].legend()
    save(fig,"hankel_spectrum_PE")


def coords():
    records=data("phase3-offline-coordinates/results.json")["coordinates"]
    fig,a=make(2,"DeePC coordinate replay | historical circles, identical issued inputs",12)
    scenarios=["baseline-circle","scaled-circle","faster-circle"]
    for ax,n in zip(a,(25,40)):
        bars(ax,["Baseline","Scaled","Faster"],[(label,[records[f"N{n}_{m}"]["cases"][s]["0.001"]["pos_mm"][-1]
                for s in scenarios]) for m,label in (("takeover","Takeover"),("window","Window-local"),("increment","Increments"))])
        ax.set(title=f"{n} prediction steps",ylabel="Terminal XYZ RMSE norm vs estimator (mm)")
        ax.legend()
    save(fig,"deepc_coordinates")


def tini():
    d=data("phase51/offline.json")
    ts=[int(t) for t in d]
    fig,a=make(3,"DeePC initialization window | Phase 5.1 validation replay",13)
    for ax,i,unit in ((a[0],2,"Z RMSE (mm)"),(a[1],8,"Yaw RMSE (mrad)")):
        ax.plot(ts,[1e3*v["truth_rmse"][-1][i] for v in d.values()],"o-",color=BLUE)
        ax.set(ylabel=unit,xlabel="Initialization samples",xticks=ts)
    a[2].plot(ts,[v["joint_effective_condition"] for v in d.values()],"o-",color=GOLD)
    a[2].set(ylabel="Effective joint condition at 1e-4",xlabel="Initialization samples",xticks=ts)
    save(fig,"initialization_horizon")


def lambda_sensitivity():
    rows=[r for r in data("phase53/results-audit.json")["rows"] if "/screen/" in r["path"]]
    panels=[("hover",0,"L0 hover"),("hover",1,"L1 hover"),("circle",1,"L1 circle")]
    labels=["Baseline\nρ=.001, β=1","9× g penalty\nρ=.003, β=1","¼ init penalty\nρ=.001, β=.25"]
    fig,a=make(3,"DeePC penalty sensitivity | Phase 5.3 screening, all six settings",14)
    for ax,(scene,level,title) in zip(a,panels):
        series=[]
        for t in (6,12):
            costs=[]
            for m in ("baseline","g3","init-quarter"):
                r=next(r for r in rows if r["case"]["scene"]==scene and r["case"]["level"]==level
                       and r["case"]["candidate"]==f"t{t}-{m}")
                score=r["common_tracking"]
                costs.append(score["physical_cost_s"]["total"] if score else np.nan)
            series.append((f"T{t}",costs))
        bars(ax,labels,series)
        ax.set(title=title,ylabel="Fixed physical cost per second")
        ax.legend()
    save(fig,"lambda_beta_sensitivity")


def noise():
    rows=data("phase52/summary.json")["rows"]
    who=["m0","joint-fir6","t6","t12"]
    fig,a=make(2,"Nominal versus noisy controller performance | Phase 5.2 pair seeds")
    for ax,scene in zip(a,("hover","circle")):
        series=[]
        for level in (0,2):
            vals=[]
            for c in who:
                r=next(r for r in rows if r["scene"]==scene and r["level"]==level and r["candidate"]==c and r["original"])
                vals.append(r["score"]["physical_cost_s"]["total"] if r["score"] else np.nan)
            series.append((f"L{level}",vals))
        bars(ax,["M0","Joint FIR6","T6","T12"],series)
        ax.set(title=scene.capitalize()+(" (× = QP fallback)" if scene=="circle" else ""),
               ylabel="Fixed physical cost per second")
        ax.legend()
    save(fig,"nominal_vs_noisy")


def solver():
    stress=data("phase51/stress.json")
    raw=[[r[1] for k,v in stress.items() if k.endswith(mode) and "nominal" not in k
                         for r in v["records"]] for mode in ("cold","warm")]
    noisy=data("phase52/summary.json")["timing"]
    tuned=data("phase53/results-audit.json")["stats"]["pressure"]["qp_ms"]
    fig,a=make(2,"DeePC solver timing | distinct test protocols",12)
    a[0].boxplot(raw,tick_labels=["Cold","Warm"],showfliers=True)
    a[0].axhline(12,color=GOLD,ls="--")
    a[0].set(title="Phase 5.1 synthetic active-output stress",ylabel="QP elapsed (ms)")
    bars(a[1],["P50","P95","P99","Max"],[("P5.2 L2 pilot (all controllers)",noisy["L2"]["qp_ms"]),
             ("P5.3 L2 finalist stress",tuned)])
    a[1].axhline(12,color=GOLD,ls="--",label="12 ms budget")
    a[1].set(title="CrazySim closed-loop timing",ylabel="QP elapsed (ms)")
    a[1].legend()
    save(fig,"solver_timing")


if __name__=="__main__":
    OUT.mkdir(exist_ok=True)
    for chart in (model,phase2,hankel,coords,tini,lambda_sensitivity,noise,solver):
        chart()
