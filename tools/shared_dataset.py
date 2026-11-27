"""Build and validate the Phase 3 shared offline I/O dataset."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.signal import welch
from scipy.spatial.transform import Rotation

from velocity_control_framework.backends.crazysim_command_adapter import CrazySimCommandAdapter
from velocity_control_framework.controllers.near_hover_model import (
    NearHoverModelConfig, discrete_model, world_to_fixed_yaw,
)

INPUT = ["command_roll_rate_rad_s", "command_pitch_rate_rad_s",
         "command_yaw_rate_rad_s", "command_thrust_normalized"]
OUTPUT = [f"estimated_position_{a}_m" for a in "xyz"] + [
    f"estimated_velocity_{a}_m_s" for a in "xyz"] + [
    f"estimated_{a}_rad" for a in ("roll", "pitch", "yaw")]
BASE = NearHoverModelConfig()


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def array(rows, keys):
    return np.asarray([[float(row[key]) for key in keys] for row in rows])


def read_episode(case_dir):
    case = json.loads((case_dir / "experiment.json").read_text())
    if case["status"] != "completed":
        raise ValueError(f"flight status is {case['status']}")
    flight = next(case_dir.glob("*/flight.csv"))
    meta = json.loads((flight.parent / "metadata.json").read_text())
    rows = [r for r in read_csv(flight) if r[INPUT[-1]]]
    diagnostics = [json.loads(r["controller_diagnostics_json"]) for r in rows]
    excit = [d["excitation"] for d in diagnostics]
    if len(rows) < 100 or any(d.get("fallback_reason") for d in diagnostics):
        raise ValueError("short tracking episode or fallback")
    if any(r["event"] == "control_deadline_missed" for r in read_csv(flight.parent / "events.csv")):
        raise ValueError("control deadline miss")
    tick = array(rows, ["monotonic_s"])[:, 0]
    obs = array(rows, ["estimated_timestamp_s"])[:, 0]
    sent = np.asarray([d["timing"]["send_completed_monotonic_s"] for d in diagnostics])
    jitter = np.abs(np.diff(tick) - BASE.sample_period_s)
    if np.max(jitter) > 0.005 or np.percentile(jitter, 99) > 0.002:
        raise ValueError("controller timing outside pilot limits")
    if np.any(np.diff(obs) < 0) or np.any(np.diff(sent) <= 0) or np.any(obs > sent) or np.max(sent - obs) > 0.04:
        raise ValueError("state/send ordering or age outside limits")
    truth = read_csv(case_dir / "sim_truth.csv")
    if len({r["agent"] for r in truth}) != 1:
        raise ValueError("truth missing or contains multiple agents")
    t = array(truth, ["monotonic_s"])[:, 0]
    if np.any(np.diff(t) <= 0) or obs[0] < t[0] or obs[-1] > t[-1]:
        raise ValueError("truth clock does not cover episode")
    ix = np.searchsorted(t, obs).clip(1, len(t) - 1)
    if np.max(t[ix] - t[ix - 1]) > 0.03:
        raise ValueError("truth interpolation gap exceeds 30 ms")
    angles = Rotation.from_quat(array(truth, ["qx", "qy", "qz", "qw"])).as_euler("xyz")
    angles[:, 1] *= -1
    angles[:, 2] = np.unwrap(angles[:, 2])
    xyzv = array(truth, ["x", "y", "z", "vx", "vy", "vz"])
    gt = np.column_stack([np.interp(obs, t, col) for col in np.column_stack((xyzv, angles)).T])
    y = array(rows, OUTPUT)
    origin, yaw = y[0, :3].copy(), y[0, 8]
    rotation = world_to_fixed_yaw(yaw)
    for state in (y, gt):
        state[:, :3] = (state[:, :3] - origin) @ rotation.T
        state[:, 3:6] = state[:, 3:6] @ rotation.T
        state[:, 8] = (state[:, 8] - yaw + np.pi) % (2 * np.pi) - np.pi
    if np.max(np.linalg.norm(gt[:, :3], axis=1)) > .15 or np.max(abs(gt[:, 6:9])) > .12:
        raise ValueError("tracking leaves near-hover dataset region")
    host = array(rows, INPUT)
    adapter = CrazySimCommandAdapter
    raw = np.asarray([adapter._encode_thrust(thrust) for thrust in host[:, 3]])
    u = host.copy()
    u[:, 3] = (raw - adapter.RAW_THRUST_MIN) / (adapter.RAW_THRUST_MAX - adapter.RAW_THRUST_MIN)
    config = meta["controller_configuration"]["model"]
    if config != asdict(BASE):
        raise ValueError("physics provenance mismatch")
    u[:, 3] -= config["equilibrium_thrust"]
    data = dict(u=u, y=y, truth=gt, host_command=host, raw_thrust=raw, tick=tick, observed=obs,
                sent=sent, dt=np.diff(tick, prepend=tick[0]), origin=origin, yaw_origin=yaw,
                measured_rates=array(rows, [f"estimated_body_rate_{axis}_rad_s" for axis in "pqr"]),
                injected=np.asarray([e["injected"] for e in excit]),
                baseline=np.asarray([e["baseline"] for e in excit]),
                saturated=np.asarray([e["saturated"] for e in excit]),
                repeated_observation=np.diff(obs, prepend=np.nan) == 0,
                valid=np.ones(len(tick), dtype=bool))
    if not all(np.all(np.isfinite(a)) for a in data.values()):
        raise ValueError("non-finite data")
    if np.mean(np.any(data["saturated"], axis=1)) > .01:
        raise ValueError("injection clipping exceeds 1% of samples")
    return data, case, meta


def hankel(episodes, channel, depth, offset=0, drop_last=0):
    """Return time-major block rows; columns never span episodes."""
    columns = []
    for episode in episodes:
        data = episode[channel][offset:]
        if drop_last:
            data = data[:-drop_last]
        if len(data) >= depth:
            windows = np.lib.stride_tricks.sliding_window_view(data, depth, axis=0)
            columns.append(windows.transpose(0, 2, 1).reshape(-1, depth * data.shape[1]).T)
    if not columns:
        raise ValueError("no complete Hankel windows")
    return np.concatenate(columns, axis=1)


def quality(episodes):
    u = np.concatenate([e["u"] for e in episodes])
    injection = np.concatenate([e["injected"] for e in episodes])
    baseline = np.concatenate([e["baseline"] for e in episodes])
    y = np.concatenate([e["truth"] for e in episodes])
    scale = np.maximum(u.std(axis=0), 1e-9)
    spectra = {}
    for depth in (40, 44, 55, 59):
        h = hankel(episodes, "u", depth) / np.tile(scale, depth)[:, None]
        s = np.linalg.svd(h, compute_uv=False)
        spectra[str(depth)] = {
            "rows": h.shape[0], "columns": h.shape[1], "rank_1e-4": int(np.sum(s > s[0] * 1e-4)),
            "relative_sigma_min": float(s[-1] / s[0]) if h.shape[1] >= h.shape[0] else 0.,
            "relative_spectrum": (s / s[0]).tolist(),
        }
    active = np.sqrt(np.mean(injection**2, axis=0))
    spectra_by_episode = [welch(e["u"], fs=50, nperseg=64, axis=0) for e in episodes]
    frequency = spectra_by_episode[0][0]
    power = np.mean([p for _, p in spectra_by_episode], axis=0)
    band_fraction = {
        f"{lo}-{hi}": (power[(frequency >= lo) & (frequency < hi)].sum(axis=0) /
                       np.maximum(power.sum(axis=0), 1e-20)).tolist()
        for lo, hi in ((0, 2), (2, 8), (8, 17), (17, 26))
    }
    return dict(samples=len(u), episodes=len(episodes), spectrum=spectra,
                input_std=scale.tolist(), input_corr=np.corrcoef(u.T).tolist(),
                injected_rms=active.tolist(),
                injected_effect_rms=np.sqrt(np.mean((np.concatenate([e["host_command"] for e in episodes])
                                                    - baseline)**2, axis=0)).tolist(),
                input_injection_corr=[
                    float(np.corrcoef(u[:, i], injection[:, i])[0, 1]) if active[i] > 0 else None
                    for i in range(4)],
                truth_min=y.min(axis=0).tolist(), truth_max=y.max(axis=0).tolist(),
                psd_band_fraction=band_fraction,
                repeated_observations=int(sum(e["repeated_observation"].sum() for e in episodes)),
                clipped_steps=int(sum(np.any(e["saturated"], axis=1).sum() for e in episodes)),
                jitter_p99_ms=float(1000 * np.percentile(np.concatenate([
                    abs(np.diff(e["tick"]) - .02) for e in episodes]), 99)),
                state_age_p99_ms=float(1000 * np.percentile(np.concatenate([
                    e["sent"] - e["observed"] for e in episodes]), 99)))


def matrices(episodes, past, future, u_scale, y_scale):
    depth = past + future
    u = hankel(episodes, "u", depth, drop_last=1) / np.tile(u_scale, depth)[:, None]
    y = hankel(episodes, "y", depth, offset=1) / np.tile(y_scale, depth)[:, None]
    return np.vstack((u, y[:past * 9])), y[past * 9:]


def predict(train, validation, future, past=6):
    u_scale = np.maximum(np.concatenate([e["u"] for e in train]).std(axis=0), 1e-5)
    y_scale = np.maximum(np.concatenate([e["y"] for e in train]).std(axis=0), 1e-4)
    x, target = matrices(train, past, future, u_scale, y_scale)
    x_test, _ = matrices(validation, past, future, u_scale, y_scale)
    a, s, b = np.linalg.svd(x, full_matrices=False)
    weight = s / (s**2 + (1e-3 * s[0])**2)
    dd = ((target @ b.T * weight) @ (a.T @ x_test)).T.reshape(-1, future, 9) * y_scale
    ad, bd = discrete_model(BASE)
    # Fit only the existing vertical gain and hover offset on training episodes.
    # Integrating five intervals avoids taking noisy one-tick acceleration.
    design, response = [], []
    for episode in train:
        for k in range(len(episode["u"]) - 5):
            design.append([.02 * np.sum(episode["u"][k:k + 5, 3]), .10])
            response.append(episode["y"][k + 5, 5] - episode["y"][k, 5])
    gain, bias = np.linalg.lstsq(design, response, rcond=None)[0]
    fitted = None
    if gain > 0 and 0 < BASE.equilibrium_thrust - bias / gain < 1:
        fitted = replace(BASE, thrust_gain_m_s2=float(gain),
                         equilibrium_thrust=float(BASE.equilibrium_thrust - bias / gain))
        ad_fit, bd_fit = discrete_model(fitted)
    physics, calibrated, persistence, expected, truth = [], [], [], [], []
    for episode in validation:
        for k in range(past, len(episode["y"]) - future):
            state = episode["y"][k]
            persistence.append(np.repeat(state[None, :], future, axis=0))
            rollout = []
            fit_rollout = []
            fit_state = state.copy()
            for u in episode["u"][k:k + future]:
                state = ad @ state + bd @ u
                rollout.append(state)
                if fitted is not None:
                    fit_u = u.copy()
                    fit_u[3] += BASE.equilibrium_thrust - fitted.equilibrium_thrust
                    fit_state = ad_fit @ fit_state + bd_fit @ fit_u
                    fit_rollout.append(fit_state)
            physics.append(rollout)
            calibrated.append(fit_rollout)
            expected.append(episode["y"][k + 1:k + future + 1])
            truth.append(episode["truth"][k + 1:k + future + 1])
    methods = [("physics", np.asarray(physics)), ("data", dd), ("persistence", np.asarray(persistence))]
    if fitted is not None:
        methods.append(("physics_train_fit", np.asarray(calibrated)))
    return {
        "windows": len(expected),
        "training_fit": asdict(fitted) if fitted else None,
        "estimated": {name: np.sqrt(np.mean((p - expected)**2, axis=0)).tolist()
                      for name, p in methods},
        "truth": {name: np.sqrt(np.mean((p - truth)**2, axis=0)).tolist()
                  for name, p in methods},
    }


def collect_batches(path):
    accepted, rejected = [], []
    for case_dir in sorted(p.parent for p in path.glob("*/experiment.json")):
        try:
            accepted.append((case_dir, *read_episode(case_dir)))
        except (ValueError, KeyError, OSError, StopIteration) as error:
            rejected.append({"episode": case_dir.name, "reason": str(error)})
    return accepted, rejected


def plot(output, episodes, metrics, validations):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    example = episodes[0]
    fig, axes = plt.subplots(4, 2, figsize=(12, 9))
    t = example["tick"] - example["tick"][0]
    for i, label in enumerate(("Roll rate", "Pitch rate", "Yaw rate", "Thrust deviation")):
        axes[i, 0].plot(t, example["u"][:, i], label="issued")
        axes[i, 0].plot(t, example["injected"][:, i], alpha=.7, label="injected")
        frequency, psd = zip(*(welch(e["u"][:, i], fs=50, nperseg=64) for e in episodes))
        axes[i, 1].semilogy(frequency[0], np.mean(psd, axis=0))
        axes[i, 0].set_ylabel(label)
    axes[0, 0].legend()
    axes[-1, 0].set_xlabel("Time [s]")
    axes[-1, 1].set_xlabel("Frequency [Hz]")
    fig.tight_layout()
    fig.savefig(output / "inputs.png")
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for d, item in metrics["spectrum"].items():
        axes[0, 0].semilogy(np.maximum(item["relative_spectrum"], 1e-16), label=f"depth {d}")
    axes[0, 0].set(title="Input Hankel spectrum", xlabel="Singular value index")
    axes[0, 0].legend()
    for e in episodes:
        axes[0, 1].plot(e["truth"][:, 0], e["truth"][:, 1], alpha=.6)
    axes[0, 1].set(title="Near-hover coverage", xlabel="X [m]", ylabel="Y [m]")
    axes[1, 0].plot(t, example["y"][:, :3])
    axes[1, 0].plot(t, example["truth"][:, :3], "--")
    axes[1, 0].set(title="XYZ estimate and truth", xlabel="Time [s]", ylabel="Position [m]")
    axes[1, 1].imshow(np.asarray(metrics["input_corr"]), vmin=-1, vmax=1)
    axes[1, 1].set(title="Cross-channel correlation", xlabel="Input channel", ylabel="Input channel")
    fig.tight_layout()
    fig.savefig(output / "quality.png")
    plt.close(fig)
    if validations:
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        full = validations[-1]
        for kind, ax in zip(("estimated", "truth"), axes):
            for name, matrix in full[kind].items():
                ax.plot(np.linalg.norm(np.asarray(matrix)[:, :3], axis=1), label=name)
            ax.set(title=f"{kind} position error", xlabel="Forecast step", ylabel="XYZ RMSE norm [m]")
            ax.legend()
        for name in ("physics", "data", "persistence"):
            axes[2].plot([v["episodes"] for v in validations],
                         [np.linalg.norm(v["truth"][name][-1][:3]) for v in validations], "o-", label=name)
        axes[2].set(title="40-step error vs dataset size", xlabel="Train episodes", ylabel="XYZ RMSE norm [m]")
        axes[2].legend()
        fig.tight_layout()
        fig.savefig(output / "prediction.png")
        plt.close(fig)
        fig, ax = plt.subplots()
        for depth in metrics["spectrum"]:
            ax.plot([v["episodes"] for v in validations], [v["rank"][depth] for v in validations],
                    "o-", label=f"depth {depth}")
        ax.set(xlabel="Training episodes", ylabel="Numerical rank")
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / "rank-vs-size.png")
        plt.close(fig)


def build(batch, output, version, simulator, past=6):
    accepted, rejected = collect_batches(batch)
    if not accepted:
        raise ValueError(f"no acceptable episodes: {rejected}")
    if output:
        output.mkdir(parents=True)
    grouped = {}
    seeds = set()
    manifest = {"version": version, "sample_period_s": .02, "episodes": [], "rejected": rejected,
                "scope": "host-issued final command with firmware thrust encoding reconstructed; firmware acceptance unobserved",
                "outputs": OUTPUT, "thrust_equilibrium": BASE.equilibrium_thrust}
    for path in (simulator, simulator / "crazyflie-firmware"):
        try:
            manifest[str(path)] = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
        except (OSError, subprocess.CalledProcessError):
            manifest[str(path)] = None
    if output:
        manifest["processing_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        project = Path(__file__).resolve().parents[1]
        sources = [project / "experiments/excitation.py", project / "experiments/run_flight.py",
                   project / "velocity_control_framework/controllers/outer_loop_mpc.py",
                   simulator / "crazyflie-firmware/sitl_make/build/cf2",
                   simulator / "crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py"]
        manifest["source_sha256"] = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources
        }
    for case_dir, data, info, meta in accepted:
        split = info["case"]["split"]
        seed = info["case"]["seed"]
        if seed in seeds:
            raise ValueError(f"duplicate episode seed: {seed}")
        seeds.add(seed)
        grouped.setdefault(split, []).append(data)
        item = {"id": meta["run_id"], "split": split, "case": info["case"], "raw": str(case_dir.resolve()),
                "samples": len(data["u"]), "controller": meta["controller_configuration"],
                "reference": meta["reference"], "git_commit": meta["git_commit"]}
        if output:
            item["processed"] = f"{case_dir.name}.npz"
            np.savez_compressed(output / item["processed"], **data)
        manifest["episodes"].append(item)
    training = grouped.get("train", grouped.get("pilot"))
    if not training:
        raise ValueError("training or pilot split is required")
    metrics = quality(training)
    prediction = {}
    learning = []
    if grouped.get("validation"):
        for future in (25, 40):
            prediction[str(future)] = predict(training, grouped["validation"], future, past)
            if output:
                depth = past + future
                u = hankel(training, "u", depth, drop_last=1)
                y = hankel(training, "y", depth, offset=1)
                np.savez_compressed(output / f"hankel-t{past}-n{future}.npz", up=u[:4 * past],
                                    uf=u[4 * past:], yp=y[:9 * past], yf=y[9 * past:])
        for size in sorted({min(k, len(training)) for k in (4, 8, len(training))}):
            r = {"episodes": size, **predict(training[:size], grouped["validation"], 40, past)}
            r["rank"] = {d: v["rank_1e-4"] for d, v in quality(training[:size])["spectrum"].items()}
            learning.append(r)
    result = {"quality": metrics, "prediction": prediction, "learning_curve": learning,
              "split_counts": {k: len(v) for k, v in grouped.items()}, "rejected": rejected,
              "t_ini": past, "pe_model_order_candidate": 9}
    result["acceptance"] = {
        "all_episodes_passed": not rejected,
        "pe_rank_passed": {
            str(n): metrics["spectrum"][str(past + n + 9)]["rank_1e-4"] == 4 * (past + n + 9)
            for n in (25, 40)
        },
        "held_out_vs_persistence": {
            n: {
                source: (np.asarray(report[source]["data"])[-1] <
                         np.asarray(report[source]["persistence"])[-1]).tolist()
                for source in ("estimated", "truth")
            } for n, report in prediction.items()
        },
        "test_scored": False,
    }
    if output:
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
        (output / "quality.json").write_text(json.dumps(result, indent=2))
        plot(output, training, metrics, learning)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--version", default="phase3-v1")
    parser.add_argument("--sim-root", type=Path, default=Path("/home/chris/CrazySim"))
    parser.add_argument("--t-ini", type=int, choices=(6, 10), default=6)
    parser.add_argument("--metrics-json", type=Path, help="write an additional report without duplicating episodes")
    args = parser.parse_args()
    result = build(args.batch, args.output, args.version, args.sim_root, args.t_ini)
    if args.metrics_json:
        with args.metrics_json.open("x") as stream:
            json.dump(result, stream, indent=2)
    quality_summary = {k: v for k, v in result["quality"].items() if k != "spectrum"}
    quality_summary["spectrum"] = {
        d: {k: v for k, v in item.items() if k != "relative_spectrum"}
        for d, item in result["quality"]["spectrum"].items()
    }
    print(json.dumps({"quality": quality_summary, "split_counts": result["split_counts"],
                      "rejected": result["rejected"]}, indent=2))


if __name__ == "__main__":
    main()
