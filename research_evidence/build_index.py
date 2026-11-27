"""Build research CSVs from existing metadata and scored summaries. Never open sealed episodes."""
import argparse
import csv
import json
from pathlib import Path
import shlex


MISSING = "unavailable"
OFFLINE = {
    "OFF-P531-CV": ("5.3.1", "DeePC episode-grouped regularization cross-validation", "log/phase531/results/summary.json"),
    "OFF-P3-QUALITY": ("3", "shared dataset quality", "log/phase3-v1/quality.json"),
    "OFF-P3-COORD": ("3", "DeePC coordinate and MPC identification comparison", "log/phase3-offline-coordinates/results.json"),
    "OFF-P41-CONSISTENCY": ("4.1", "MPC implementation consistency", "log/phase41-diagnosis/recorded-equivalence.json"),
    "OFF-P42-ID": ("4.2", "low-frequency MPC identification", "log/phase42-diagnosis/identification.json"),
    "OFF-P42-HORIZON": ("4.2", "DeePC PE, predictions and horizon stress", "log/phase42-diagnosis/horizon-bench.json"),
    "OFF-P43-ORDER": ("4.3", "FIR order selection and prediction", "log/phase43-diagnosis/orders.json"),
    "OFF-P43-STRESS": ("4.3", "DeePC active-constraint stress", "log/phase43-diagnosis/stress.json"),
    "OFF-P51-INIT": ("5.1", "initialization window prediction and PE", "log/phase51/offline.json"),
    "OFF-P51-STRESS": ("5.1", "full DeePC QP certification and stress", "log/phase51/stress.json"),
    "OFF-P52-NOISE": ("5.2", "paired noise and initialization history audit", "log/phase52/noise-audit.json"),
    "OFF-P53-SENSITIVITY": ("5.3", "separate noise-channel diagnosis", "log/phase53/diagnosis.json"),
}
SUMMARIES = [
    "mpc-phase2-compare/summary.json","mpc-phase2-horizon-repeat/summary.json",
    "mpc-phase2-nominal/summary.json","phase4-summary/summary.json",
    "phase41-diagnosis/closed-loop.json","phase42-diagnosis/mpc-phase41-cost.json",
    "phase42-diagnosis/deepc-nominal-cost.json","phase43-diagnosis/physical.json",
    "phase43-diagnosis/deepc-physical.json","phase51/nominal-summary.json",
    "phase51/verification-summary.json","phase51/y-retry-summary.json",
    "phase52/noise-audit.json","phase53/results-audit.json",
]
OFFLINE_COMMANDS = {
    "OFF-P531-CV": ".venv/bin/python log/phase531/cross_validate.py log/phase531/folds.json log/phase531/replay-results",
    "OFF-P3-COORD": ".venv/bin/python log/phase3-offline-coordinates/coordinate_models.py log /tmp/phase3-coordinate-results.json",
    "OFF-P41-CONSISTENCY": ".venv/bin/python log/phase41-diagnosis/model_diagnosis.py log /tmp/phase41-diagnosis",
    "OFF-P42-ID": ".venv/bin/python log/phase42-diagnosis/identification.py log /tmp/phase42-identification.json",
    "OFF-P42-HORIZON": ".venv/bin/python log/phase42-diagnosis/horizon_bench.py /tmp/phase42-horizon.json",
    "OFF-P43-ORDER": ".venv/bin/python log/phase43-diagnosis/order_selection.py /tmp/phase43-orders.json",
    "OFF-P43-STRESS": ".venv/bin/python log/phase43-diagnosis/stress.py /tmp/phase43-stress.json",
    "OFF-P51-INIT": ".venv/bin/python log/phase51/offline.py /tmp/phase51-offline.json",
    "OFF-P51-STRESS": ".venv/bin/python log/phase51/stress.py /tmp/phase51-stress.json",
    "OFF-P52-NOISE": ".venv/bin/python log/phase52/audit_noise.py log/phase52 --output /tmp/phase52-audit.json",
    "OFF-P53-SENSITIVITY": ".venv/bin/python log/phase53/diagnose.py /tmp/phase53-diagnosis.json",
}


def read(path):
    return json.loads(path.read_text())


def compact(value):
    return json.dumps(value,ensure_ascii=False,separators=(",",":"),sort_keys=True)


def relative(path, repo):
    p = Path(path)
    return (p.relative_to(repo) if p.is_absolute() and p.is_relative_to(repo) else p).as_posix()


def phase(group):
    for label,prefixes in (("2",("mpc-phase2",)),("3",("phase3",)),("4.1",("phase41",)),
            ("4.2",("phase42",)),("4.3",("phase43",)),("4",("phase4",)),
            ("5.1",("phase51",)),("5.2",("phase52",)),("5.3",("phase53",))):
        if any(group.startswith(p) for p in prefixes):
            return label
    if group.startswith("20261126_16") and "outer-loop-mpc" in group:
        return "1"
    return "context_unverified_phase"


def write_csv(path, fields, rows):
    with path.open("w",encoding="utf-8",newline="") as f:
        writer = csv.DictWriter(f,fields,restval=MISSING)
        writer.writeheader()
        writer.writerows({k: MISSING if v is None else v for k,v in row.items()} for row in rows)


INDEX_FIELDS = """phase experiment_id experiment_name evidence_type controller configuration dataset_version
noise_profile noise_seed excitation_seed process_status success_failure_status fallback_count runner_deadline_misses
status_reason original_log_path recorded_git_commit reproduction_command supporting_source""".split()


def build_inventory(repo):
    log = repo/"log"
    manifest = read(log/"phase3-v1/manifest.json")
    episodes = {relative(v["raw"],repo):v for v in manifest["episodes"]}
    sealed = {k for k,v in episodes.items() if v["split"]=="test"}
    rejected = {f"log/phase3-shared-raw/{r['episode']}":r["reason"] for r in manifest["rejected"]}
    scoring = {}
    for name in SUMMARIES:
        path=log/name
        data=read(path)
        for row in data if isinstance(data,list) else data.get("rows",[]):
            raw=row.get("path",row.get("run"))
            if raw and not any(relative(raw,repo).startswith(k) for k in sealed):
                scoring[relative(raw,repo)] = (row,relative(path,repo))
    indexed=[]
    batch_paths=sorted(log.glob("*/**/experiment.json"))
    known=set()
    for path in batch_paths:
        run=relative(path.parent,repo)
        known.add(run)
        prefix=path.relative_to(log).parts[0]
        ids=dict(phase=phase(prefix),experiment_id=run.removeprefix("log/"),
            evidence_type="flight",original_log_path=run)
        if run in sealed:
            indexed.append(dict(**ids,experiment_name="sealed dataset episode",
                dataset_version=manifest["version"],success_failure_status="sealed_not_inspected",
                supporting_source="log/phase3-v1/manifest.json"))
            continue
        src=read(path)
        meta_path=next(iter(sorted(path.parent.glob("*/metadata.json"))),None)
        meta=read(meta_path) if meta_path else {}
        case=src.get("case",{})
        args=src.get("flight",[])
        who=meta.get("controller_type") or (args[args.index("--controller")+1] if "--controller" in args else MISSING)
        ctrl=meta.get("controller_configuration",{})
        ref=meta.get("reference",{})
        noise=ref.get("configuration",{}).get("measurement_noise") or {}
        scored,source=scoring.get(run,({},relative(path,repo)))
        events=next(iter(sorted(path.parent.glob("*/events.csv"))),None)
        fallback=scored.get("fallback")
        deadlines=scored.get("deadline_misses")
        if events and (fallback is None or deadlines is None):
            with events.open(newline="") as stream:
                entries=list(csv.DictReader(stream))
            fallback=sum(e["event"]=="phase_changed" and "FALLBACK_LANDING" in e["details_json"] for e in entries)
            deadlines=sum(e["event"]=="control_deadline_missed" for e in entries)
        raw=src.get("status",MISSING)
        expected="--force-solver-failure" in args
        reason=scored.get("fallback_reason") or src.get("error") or meta.get("termination_reason") or ""
        if run in rejected:
            status="dataset_rejected"; reason=rejected[run]
        elif raw=="started":
            status="incomplete_status_recorded_started"
        elif raw!="completed":
            status=raw
        elif fallback:
            status="intentional_fallback" if expected else "unexpected_fallback"
        elif deadlines or scored.get("feasible") is False:
            status="completed_ineligible"
        elif run in episodes:
            status="dataset_admitted"
        elif scored.get("feasible") is True:
            status="passed_feasibility"
        else:
            status="completed_feasibility_unavailable"
        command = {k:shlex.join(src[k]) for k in ("firmware","simulator","flight") if src.get(k)}
        indexed.append(dict(**ids,experiment_name=case.get("name",MISSING),controller=who,
            configuration=compact({"case":case,"controller":ctrl,"reference":ref}),
            dataset_version=manifest["version"] if run in episodes or run in rejected else (ctrl.get("dataset") or ("phase3-v1" if ctrl.get("identified") else MISSING)).removeprefix("log/"),
            noise_profile=compact({"profile":noise["profile"],"amplitude":noise["amplitude"]}) if noise else MISSING,
            noise_seed=noise.get("seed",MISSING),excitation_seed=ref.get("configuration",{}).get("excitation",{}).get("seed",MISSING)
                if ref.get("configuration",{}).get("excitation") else (case.get("seed",MISSING) if "excite" in case.get("name","") else MISSING),
            process_status=raw,success_failure_status=status,fallback_count=fallback,runner_deadline_misses=deadlines,
            status_reason=reason or MISSING,recorded_git_commit=meta.get("git_commit") or MISSING,
            reproduction_command=compact(command) if command else MISSING,supporting_source=source))
    for path in sorted(log.glob("20*/metadata.json")):
        run=relative(path.parent,repo)
        if run in known:
            continue
        meta=read(path)
        ref=meta.get("reference",{})
        with (path.parent/"events.csv").open(newline="") as stream:
            events=list(csv.DictReader(stream))
        fell=sum(e["event"]=="phase_changed" and "FALLBACK_LANDING" in e["details_json"] for e in events)
        misses=sum(e["event"]=="control_deadline_missed" for e in events)
        code=meta.get("status",MISSING)
        status=("intentional_fallback" if ref.get("configuration",{}).get("forced_solver_failure")
                else "unexpected_fallback") if fell else ("completed_deadline_event" if misses else code)
        indexed.append(dict(phase=phase(path.parent.name),experiment_id=path.parent.name,
            experiment_name=meta.get("experiment_name",MISSING),evidence_type="standalone_flight",
            controller=meta.get("controller_type",MISSING),configuration=compact({"controller":meta.get("controller_configuration"),"reference":ref}),
            process_status=code,success_failure_status=status,fallback_count=fell,runner_deadline_misses=misses,
            status_reason=meta.get("termination_reason") or MISSING,original_log_path=run,
            recorded_git_commit=meta.get("git_commit") or MISSING,supporting_source=relative(path,repo)))
    for tag,(label,name,path) in OFFLINE.items():
        if not (repo/path).exists():
            continue
        indexed.append(dict(phase=label,experiment_id=tag,experiment_name=name,
            evidence_type="offline_analysis",dataset_version="phase3-v1" if label!="5.2" else MISSING,
            success_failure_status="artifact_available",original_log_path=path,supporting_source=path,
            reproduction_command=OFFLINE_COMMANDS.get(tag,MISSING)))
    assert len({r["experiment_id"] for r in indexed})==len(indexed)
    return indexed


def regularization(repo):
    root=repo/"log/phase53"
    diag=read(root/"diagnosis.json")
    current=read(root/"runs/decisions.json")
    res=[]
    for r in read(root/"results-audit.json")["rows"]:
        c=r["case"]
        w=read(root/"configs"/f"{c['candidate']}-{c['weights']}.json")
        stage=Path(r["path"]).parent.name
        tracking=r.get("common_tracking") or {}
        diag_p95=r.get("deepc_p95") or {}
        past=w.get("past")
        rho=w.get("regularization")
        cost=tracking.get("physical_cost_s",{})
        rmse=tracking.get("truth_rmse_xyz_yaw",[]) or [None]*4
        qp=tracking.get("solver_ms") or [None]*4
        picked=c["candidate"] in current["confirmed"]
        note=("mpc_control" if not past else "confirmed" if picked else "screened_not_advanced")
        res.append(dict(experiment_id=relative(r["path"],repo).removeprefix("log/"),stage=stage,
            controller=c["controller"],candidate=c["candidate"],weight_setting=c["weights"],
            selection_outcome=note,scene=c["scene"],noise_level=c["level"],
            noise_profile="none" if c["level"]==0 else "iid-small",noise_amplitude=c["level"],noise_seed=c["seed"],
            dataset_version="phase3-v1" if past or c["candidate"]=="joint-fir6" else MISSING,horizon_steps=25,
            T_ini_steps=past,rho=rho,beta=w.get("initialization_weight"),
            lambda_g=(rho*diag[str(past)]["scaled_input_largest_singular_value"])**2 if past else None,
            Q=compact(w["q"]),R=compact(w["r"]),S=compact(w["s"]),
            evaluation_weights="original Q/R/S",score_samples=tracking.get("samples"),
            tracking_rmse_x_m=rmse[0],tracking_rmse_y_m=rmse[1],tracking_rmse_z_m=rmse[2],tracking_rmse_yaw_rad=rmse[3],
            tracking_cost_per_s=cost.get("tracking"),control_effort_cost_per_s=cost.get("effort"),
            input_increment_cost_per_s=cost.get("increment"),physical_cost_per_s=cost.get("total"),
            init_slack_p95=diag_p95.get("init_slack"),g_norm_p95=diag_p95.get("g_norm"),
            init_penalty_p95=diag_p95.get("init_penalty"),g_penalty_p95=diag_p95.get("g_penalty"),
            qp_p50_ms=qp[0],qp_p95_ms=qp[1],qp_p99_ms=qp[2],qp_max_ms=qp[3],
            full_run_qp_p99_ms=(r.get("solver_ms_percentiles") or [None]*4)[2],
            full_run_qp_max_ms=(r.get("solver_ms_percentiles") or [None]*4)[3],
            qp_overruns=len(r.get("qp_overruns",[])),runner_deadline_misses=r.get("deadline_misses"),
            fallback_count=r.get("fallback"),status=r["status"],eligible=bool(tracking and r.get("feasible") and not r.get("qp_overruns")),
            original_log_path=relative(r["path"],repo),source="log/phase53/results-audit.json"))
    assert len(res)==40
    return res


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo",type=Path,default=Path.cwd())
    p.add_argument("--output",type=Path,default=Path("research_evidence"))
    a=p.parse_args()
    repo=a.repo.resolve()
    index=build_inventory(repo)
    write_csv(a.output/"experiment_index.csv",INDEX_FIELDS,index)
    tuning=regularization(repo)
    write_csv(a.output/"regularization_trials.csv",list(tuning[0]),tuning)


if __name__=="__main__":
    main()
