from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import mean


ROOT=Path(__file__).resolve().parents[2]
P_RUN=ROOT/"calibration/runs/20261026T064847_690179Z_pitch-response-id"
PD_RUN=ROOT/"calibration/runs/20261026T065908_616181Z_cascaded-5c"
OUTPUT=Path(__file__).resolve().parent/"p_vs_pd"


def load(path):
    with (path/"flight_telemetry.csv").open(newline="",encoding="utf-8") as stream:
        rows=list(csv.DictReader(stream))
    return rows,json.loads((path/"metadata.json").read_text(encoding="utf-8"))


def value(row,key): return float(row[key])


def phase_metrics(rows,phase,elapsed_limit=None):
    selected=[row for row in rows if row["phase"]==phase and
              (elapsed_limit is None or value(row,"elapsed_s")<=elapsed_limit)]
    error=[value(row,"z_m")-value(row,"reference_z_m") for row in selected]
    return {"samples":len(selected),"duration_s":value(selected[-1],"elapsed_s")-value(selected[0],"elapsed_s"),
        "rmse_m":math.sqrt(mean(item*item for item in error)),"mean_error_m":mean(error),
        "maximum_overshoot_m":max(error),"minimum_error_m":min(error),
        "mean_vertical_velocity_m_s":mean(value(row,"vz_m_s") for row in selected),
        "mean_thrust":mean(value(row,"command_thrust_normalized") for row in selected)}


def extrema(rows):
    selected=[row for row in rows if row["phase"]=="HOVER"]
    altitude=[value(row,"z_m") for row in selected]
    smooth=[mean(altitude[max(0,index-3):min(len(altitude),index+4)]) for index in range(len(altitude))]
    result=[]
    for index in range(1,len(smooth)-1):
        kind="peak" if smooth[index-1]<smooth[index]>=smooth[index+1] else (
            "trough" if smooth[index-1]>smooth[index]<=smooth[index+1] else None)
        if kind and (smooth[index]>.20 or smooth[index]<.10):
            result.append({"kind":kind,"elapsed_s":value(selected[index],"elapsed_s"),
                           "altitude_m":smooth[index]})
    return result


def run_metrics(rows,metadata):
    target=metadata["captured_reference"]["ground_z_m"]+.1
    maximum=max(rows,key=lambda row:value(row,"z_m")-value(row,"reference_z_m"))
    crossing=next(row for row in rows if value(row,"z_m")>=target)
    crossing_index=rows.index(crossing)
    post=rows[crossing_index:]
    return {"samples":len(rows),"duration_s":value(rows[-1],"elapsed_s"),
        "termination_reason":metadata["termination_reason"],"takeoff":phase_metrics(rows,"TAKEOFF"),
        "hover":phase_metrics(rows,"HOVER"),"matched_hover":phase_metrics(rows,"HOVER",4.0001),
        "maximum_overshoot_m":value(maximum,"z_m")-value(maximum,"reference_z_m"),
        "maximum_overshoot_elapsed_s":value(maximum,"elapsed_s"),
        "maximum_altitude_m":value(maximum,"z_m"),"reference_at_maximum_m":value(maximum,"reference_z_m"),
        "target_crossing_elapsed_s":value(crossing,"elapsed_s"),
        "target_crossing_vertical_velocity_m_s":value(crossing,"vz_m_s"),
        "target_crossing_thrust":value(crossing,"command_thrust_normalized"),
        "maximum_ascent_velocity_m_s":max(value(row,"vz_m_s") for row in rows[:crossing_index+1]),
        "post_crossing_velocity_range_m_s":[min(value(row,"vz_m_s") for row in post),max(value(row,"vz_m_s") for row in post)],
        "thrust_range":[min(value(row,"command_thrust_normalized") for row in rows),
                        max(value(row,"command_thrust_normalized") for row in rows)],
        "altitude_saturation_samples":sum(int(row["altitude_saturated"]) for row in rows),
        "hover_extrema":extrema(rows)}


def polyline(rows,key,tmax,y0,y1,vmin,vmax):
    chosen=[row for row in rows if value(row,"elapsed_s")<=tmax]
    return " ".join(f'{70+value(row,"elapsed_s")/tmax*900:.1f},{y1-(value(row,key)-vmin)/(vmax-vmin)*(y1-y0):.1f}' for row in chosen)


def comparison_svg(path,p_rows,pd_rows):
    tmax=4.01
    panels=[("altitude/reference [m]","z_m",-.01,.33),("vertical velocity [m/s]","vz_m_s",-.22,.46),
            ("commanded thrust","command_thrust_normalized",.38,.45)]
    lines=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="720">','<rect width="100%" height="100%" fill="white"/>',
           '<text x="70" y="20" font-family="sans-serif" font-size="15">Measured P-only versus Kv=0.04 response (common 0–4.0 s window)</text>']
    for index,(label,key,vmin,vmax) in enumerate(panels):
        y0=40+index*215;y1=y0+175
        lines += [f'<rect x="70" y="{y0}" width="900" height="175" fill="none" stroke="#ccc"/>',
                  f'<text x="5" y="{y0+15}" font-family="sans-serif" font-size="11">{label}</text>',
                  f'<polyline fill="none" stroke="#2563eb" stroke-width="1.8" points="{polyline(p_rows,key,tmax,y0,y1,vmin,vmax)}"/>',
                  f'<polyline fill="none" stroke="#dc2626" stroke-width="1.8" points="{polyline(pd_rows,key,tmax,y0,y1,vmin,vmax)}"/>']
        if key=="z_m":
            lines.append(f'<polyline fill="none" stroke="#111" stroke-dasharray="5 4" points="{polyline(pd_rows,"reference_z_m",tmax,y0,y1,vmin,vmax)}"/>')
    lines += ['<text x="710" y="25" fill="#2563eb" font-family="sans-serif" font-size="12">P-only</text>',
              '<text x="780" y="25" fill="#dc2626" font-family="sans-serif" font-size="12">PD Kv=0.04</text>',
              '<text x="885" y="25" fill="#111" font-family="sans-serif" font-size="12">reference</text>',
              '<text x="520" y="710" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed seconds</text>','</svg>']
    path.write_text("\n".join(lines)+"\n",encoding="utf-8")


def context_svg(path,p_rows,pd_rows):
    width,height=1000,370;tmax=max(value(p_rows[-1],"elapsed_s"),value(pd_rows[-1],"elapsed_s"))
    allz=[value(row,"z_m") for row in p_rows+pd_rows];lo=min(allz);hi=max(allz)
    def points(rows,key):
        return " ".join(f'{70+value(row,"elapsed_s")/tmax*900:.1f},{320-(value(row,key)-lo)/(hi-lo)*270:.1f}' for row in rows)
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="370">','<rect width="100%" height="100%" fill="white"/>',
         '<text x="70" y="22" font-family="sans-serif" font-size="15">Full recorded altitude context; PD trace ends at horizontal-envelope abort</text>',
         '<rect x="70" y="50" width="900" height="270" fill="none" stroke="#ccc"/>',
         f'<polyline fill="none" stroke="#2563eb" points="{points(p_rows,"z_m")}"/>',
         f'<polyline fill="none" stroke="#dc2626" points="{points(pd_rows,"z_m")}"/>',
         f'<polyline fill="none" stroke="#111" stroke-dasharray="5 4" points="{points(p_rows,"reference_z_m")}"/>',
         '<text x="520" y="355" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed seconds</text>','</svg>']
    path.write_text("\n".join(svg)+"\n",encoding="utf-8")


def main():
    p_rows,p_meta=load(P_RUN);pd_rows,pd_meta=load(PD_RUN)
    p=run_metrics(p_rows,p_meta);pd=run_metrics(pd_rows,pd_meta)
    report={"p_only":p,"pd_kv_0_04":pd,
        "comparability":{"common_window_s":[0,4.0],"position_reference_profiles_match":True,
          "reference_velocity_difference":"P-only log used 0 m/s in TAKEOFF; PD used +0.10 m/s. Both use 0 m/s in HOVER.",
          "duration_limitation":"P-only includes full HOVER, LANDING, SETTLE, and LANDING_CONFIRM; PD aborted during HOVER at 4.00 s."}}
    OUTPUT.mkdir(parents=True,exist_ok=True)
    (OUTPUT/"metrics.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    comparison_svg(OUTPUT/"common_window_comparison.svg",p_rows,pd_rows)
    context_svg(OUTPUT/"full_duration_context.svg",p_rows,pd_rows)
    markdown=f'''# Measured altitude response: P-only versus PD

## Comparability

Both runs used the same altitude-position profile through the PD run: a 0.10 m/s ramp to 0.10 m during 0–1 s, followed by a fixed 0.10 m HOVER reference. P-only continued for {p["duration_s"]:.2f} s through landing and timed out during touchdown confirmation. PD ended at {pd["duration_s"]:.2f} s during HOVER because horizontal displacement exceeded 0.50 m. Quantitative controller comparison is therefore limited to TAKEOFF and the common HOVER interval, elapsed 1–4 s.

One intentional implementation difference is relevant: the P-only log carried `vz_ref=0` during TAKEOFF, while PD carried the corrected +0.10 m/s ramp velocity. Both used `vz_ref=0` in HOVER. Thus the measured change includes both Kv=0.04 and its intended takeoff velocity reference; it is not an isolated damping-only experiment.

## Metrics

| Metric | P-only | PD, Kv=0.04 | Change |
|---|---:|---:|---:|
| TAKEOFF altitude RMSE | {p["takeoff"]["rmse_m"]:.5f} m | {pd["takeoff"]["rmse_m"]:.5f} m | {(pd["takeoff"]["rmse_m"]-p["takeoff"]["rmse_m"])/p["takeoff"]["rmse_m"]*100:+.1f}% |
| HOVER RMSE, common 1–4 s | {p["matched_hover"]["rmse_m"]:.5f} m | {pd["matched_hover"]["rmse_m"]:.5f} m | {(pd["matched_hover"]["rmse_m"]-p["matched_hover"]["rmse_m"])/p["matched_hover"]["rmse_m"]*100:+.1f}% |
| HOVER RMSE, all available | {p["hover"]["rmse_m"]:.5f} m (4.0 s) | {pd["hover"]["rmse_m"]:.5f} m (3.0 s) | not duration-matched |
| Maximum overshoot vs instantaneous reference | {p["maximum_overshoot_m"]:.5f} m | {pd["maximum_overshoot_m"]:.5f} m | {pd["maximum_overshoot_m"]-p["maximum_overshoot_m"]:+.5f} m |
| Peak altitude / time | {p["maximum_altitude_m"]:.5f} m / {p["maximum_overshoot_elapsed_s"]:.2f} s | {pd["maximum_altitude_m"]:.5f} m / {pd["maximum_overshoot_elapsed_s"]:.2f} s | — |
| Target-crossing time | {p["target_crossing_elapsed_s"]:.2f} s | {pd["target_crossing_elapsed_s"]:.2f} s | {pd["target_crossing_elapsed_s"]-p["target_crossing_elapsed_s"]:+.2f} s |
| Vertical velocity at target crossing | {p["target_crossing_vertical_velocity_m_s"]:.3f} m/s | {pd["target_crossing_vertical_velocity_m_s"]:.3f} m/s | {pd["target_crossing_vertical_velocity_m_s"]-p["target_crossing_vertical_velocity_m_s"]:+.3f} m/s |
| Thrust at target crossing | {p["target_crossing_thrust"]:.5f} | {pd["target_crossing_thrust"]:.5f} | {pd["target_crossing_thrust"]-p["target_crossing_thrust"]:+.5f} |
| Maximum ascent velocity before crossing | {p["maximum_ascent_velocity_m_s"]:.3f} m/s | {pd["maximum_ascent_velocity_m_s"]:.3f} m/s | {pd["maximum_ascent_velocity_m_s"]-p["maximum_ascent_velocity_m_s"]:+.3f} m/s |
| Mean TAKEOFF thrust | {p["takeoff"]["mean_thrust"]:.5f} | {pd["takeoff"]["mean_thrust"]:.5f} | {pd["takeoff"]["mean_thrust"]-p["takeoff"]["mean_thrust"]:+.5f} |
| Mean common-HOVER thrust | {p["matched_hover"]["mean_thrust"]:.5f} | {pd["matched_hover"]["mean_thrust"]:.5f} | {pd["matched_hover"]["mean_thrust"]-p["matched_hover"]["mean_thrust"]:+.5f} |
| Commanded thrust range | {p["thrust_range"][0]:.5f}–{p["thrust_range"][1]:.5f} | {pd["thrust_range"][0]:.5f}–{pd["thrust_range"][1]:.5f} | — |
| Altitude saturation samples | {p["altitude_saturation_samples"]} | {pd["altitude_saturation_samples"]} | none |

The P-only HOVER trace has a smoothed peak of 0.30660 m at 1.92 s and trough of 0.06436 m at 3.82 s, giving a 0.24224 m peak-to-trough span and an approximate 3.80 s period from one half-cycle. PD has one smoothed peak, 0.31311 m at 2.30 s. It was still descending at abort (last logged altitude 0.12568 m, vz −0.13007 m/s), so no PD trough, period, or settling time is observable.

PD crossed 0.10 m 0.06 s earlier and with slightly greater upward velocity. Its peak was 0.00670 m higher and occurred 0.38 s later. Although the velocity term substantially lowered thrust during ascent, the recorded vehicle retained upward momentum longer. By the end of the common window PD was approaching the target from above, but it had not settled and the run ended before a rebound could be observed.

## Authority and conclusion

Neither controller saturated. P-only used 0.40428–0.42896 thrust; PD used 0.40164–0.43053, leaving 0.01947 upward and 0.02164 downward authority inside the unchanged 0.380–0.450 bounds. The PD command therefore had adequate unused authority; its result was not caused by command clipping.

**Measured verdict: deterioration over the available common window, with incomplete evidence about eventual settling.** TAKEOFF RMSE increased 11.5%, duration-matched HOVER RMSE increased 16.6%, maximum overshoot increased 6.8 mm, and ascent velocity increased. Kv=0.04 did not produce measurable improvement in this flight. Because the PD run ended before completing HOVER or one oscillation, it remains inconclusive whether PD would reduce subsequent cycles or settling time. No gain or limit change follows automatically from this comparison.

Plots: [common-window comparison](common_window_comparison.svg) and [full-duration context](full_duration_context.svg).
'''
    (OUTPUT/"report.md").write_text(markdown,encoding="utf-8")


if __name__=="__main__": main()
