"""Reproduce the Stage 5B first-flight diagnostic report using only the stdlib."""
from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import mean


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "calibration/runs/20261026T053025_672531Z_cascaded-5b"
OUT = Path(__file__).resolve().parent


def values(rows, key): return [float(row[key]) for row in rows]


def correlation(left, right):
    lm, rm = mean(left), mean(right)
    denominator = math.sqrt(sum((x-lm)**2 for x in left)*sum((y-rm)**2 for y in right))
    return sum((x-lm)*(y-rm) for x,y in zip(left,right))/denominator if denominator else None


def slope(x, y):
    xm, ym = mean(x), mean(y); denominator = sum((v-xm)**2 for v in x)
    return sum((a-xm)*(b-ym) for a,b in zip(x,y))/denominator if denominator else 0.0


def phase_metrics(rows):
    t=values(rows,"elapsed_s"); error=values(rows,"altitude_error_m")
    z=values(rows,"z_m"); reference=values(rows,"reference_z_m"); vz=values(rows,"vz_m_s")
    return {"samples":len(rows),"duration":t[-1]-t[0]+float(rows[-1]["dt_s"]),
            "rmse":math.sqrt(mean([v*v for v in error])),"mean_error":mean(error),
            "max_error":max(abs(v) for v in error),"mean_vz":mean(vz),
            "z_range":(min(z),max(z)),"reference_range":(min(reference),max(reference))}


def local_accelerations(rows, half_window=.20):
    t=values(rows,"elapsed_s"); vx=values(rows,"vx_m_s"); vy=values(rows,"vy_m_s")
    result=[]
    for index, center in enumerate(t):
        selected=[i for i,v in enumerate(t) if abs(v-center)<=half_window]
        if len(selected)<10: continue
        tx=[t[i] for i in selected]
        ax=slope(tx,[vx[i] for i in selected]); ay=slope(tx,[vy[i] for i in selected])
        yaw=float(rows[index]["yaw_rad"]); cosine,sine=math.cos(yaw),math.sin(yaw)
        result.append({"t":center,"ax_world":ax,"ay_world":ay,
                       "ax_body":cosine*ax+sine*ay,"ay_body":-sine*ax+cosine*ay,
                       "roll":float(rows[index]["roll_rad"]),"pitch":float(rows[index]["pitch_rad"])})
    return result


def svg_timeseries(path, title, panels, phase_boundary=2.000180647):
    width,height,left,right=1000,160*len(panels)+35,75,20
    all_t=[t for _,series in panels for _,points,_ in series for t,_ in points]; t0,t1=min(all_t),max(all_t)
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
         '<rect width="100%" height="100%" fill="white"/>',f'<text x="15" y="18" font-family="sans-serif" font-size="15">{title}</text>']
    for panel,(label,series) in enumerate(panels):
        top=28+panel*160;bottom=top+120; all_y=[v for _,points,_ in series for _,v in points];lo,hi=min(all_y),max(all_y)
        if lo==hi:lo-=.5;hi+=.5
        out += [f'<rect x="{left}" y="{top}" width="{width-left-right}" height="120" fill="none" stroke="#bbb"/>',
                f'<text x="4" y="{top+14}" font-family="sans-serif" font-size="11">{label}</text>']
        xphase=left+(phase_boundary-t0)/(t1-t0)*(width-left-right)
        out.append(f'<line x1="{xphase:.1f}" y1="{top}" x2="{xphase:.1f}" y2="{bottom}" stroke="#999" stroke-dasharray="4 4"/>')
        for name,points,color in series:
            coords=" ".join(f'{left+(x-t0)/(t1-t0)*(width-left-right):.1f},{bottom-(y-lo)/(hi-lo)*120:.1f}' for x,y in points)
            out.append(f'<polyline points="{coords}" fill="none" stroke="{color}"/><text x="{left+8}" y="{top+15+14*series.index((name,points,color))}" font-family="sans-serif" font-size="10" fill="{color}">{name}</text>')
    out.append('</svg>');path.write_text("\n".join(out)+"\n")


def xy_svg(path, rows):
    x=values(rows,"x_m");y=values(rows,"y_m");vx=values(rows,"vx_m_s");vy=values(rows,"vy_m_s");t=values(rows,"elapsed_s")
    width,height=1000,380;out=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">','<rect width="100%" height="100%" fill="white"/>','<text x="15" y="18" font-family="sans-serif" font-size="15">XY trajectory and velocity</text>']
    # Equal-scale XY trajectory.
    left,top,size=70,35,300;span=max(max(x)-min(x),max(y)-min(y),1e-9);cx=(max(x)+min(x))/2;cy=(max(y)+min(y))/2
    pts=" ".join(f'{left+(xx-(cx-span/2))/span*size:.1f},{top+size-(yy-(cy-span/2))/span*size:.1f}' for xx,yy in zip(x,y))
    out += [f'<rect x="{left}" y="{top}" width="{size}" height="{size}" fill="none" stroke="#bbb"/>',f'<polyline points="{pts}" fill="none" stroke="#2563eb"/>',f'<circle cx="{pts.split()[0].split(",")[0]}" cy="{pts.split()[0].split(",")[1]}" r="4" fill="#059669"/>',f'<circle cx="{pts.split()[-1].split(",")[0]}" cy="{pts.split()[-1].split(",")[1]}" r="4" fill="#dc2626"/>','<text x="75" y="52" font-family="sans-serif" font-size="10">green=start, red=last logged</text>']
    # Velocity time series.
    l2,r2,top2,bottom2=450,20,35,335;lo=min(vx+vy);hi=max(vx+vy)
    out.append(f'<rect x="{l2}" y="{top2}" width="{width-l2-r2}" height="{bottom2-top2}" fill="none" stroke="#bbb"/>')
    for name,data,color in [('vx',vx,'#dc2626'),('vy',vy,'#7c3aed')]:
        points=" ".join(f'{l2+(tt-t[0])/(t[-1]-t[0])*(width-l2-r2):.1f},{bottom2-(v-lo)/(hi-lo)*(bottom2-top2):.1f}' for tt,v in zip(t,data))
        out.append(f'<polyline points="{points}" fill="none" stroke="{color}"/><text x="{l2+8}" y="{top2+15+15*["vx","vy"].index(name)}" font-family="sans-serif" font-size="11" fill="{color}">{name} [m/s]</text>')
    out.append('</svg>');path.write_text("\n".join(out)+"\n")


def main():
    rows=list(csv.DictReader((RUN/"flight_telemetry.csv").open(newline="")))
    takeoff=[r for r in rows if r["phase"]=="TAKEOFF"];hover=[r for r in rows if r["phase"]=="HOVER"]
    metrics={name:phase_metrics(part) for name,part in (("TAKEOFF",takeoff),("HOVER",hover))}
    # Half-second hover windows expose the transient rather than labeling all hover as steady state.
    hover_start=float(hover[0]["elapsed_s"]); windows=[]
    for begin in (0,.5,1,1.5,2):
        selected=[r for r in hover if begin<=float(r["elapsed_s"])-hover_start<begin+.5]
        if not selected:continue
        tt=values(selected,"elapsed_s");zz=values(selected,"z_m")
        windows.append((begin,len(selected),mean(zz),mean(values(selected,"altitude_error_m")),
                        mean(values(selected,"vz_m_s")),slope(tt,zz),mean(values(selected,"command_thrust_normalized"))))
    accelerations=local_accelerations(rows)
    corr_roll_ay=correlation([a["roll"] for a in accelerations],[a["ay_body"] for a in accelerations])
    corr_pitch_ax=correlation([a["pitch"] for a in accelerations],[a["ax_body"] for a in accelerations])
    # Drift metrics.
    x0,y0=float(rows[0]["reference_x_m"]),float(rows[0]["reference_y_m"])
    dx,dy=float(rows[-1]["x_m"])-x0,float(rows[-1]["y_m"])-y0; direction=(dx/math.hypot(dx,dy),dy/math.hypot(dx,dy))
    aligned=sum(float(r["vx_m_s"])*direction[0]+float(r["vy_m_s"])*direction[1]>0 for r in rows)/len(rows)
    path=sum(math.hypot(float(rows[i]["x_m"])-float(rows[i-1]["x_m"]),float(rows[i]["y_m"])-float(rows[i-1]["y_m"])) for i in range(1,len(rows)))
    def mean_velocity(part):return mean(values(part,"vx_m_s")),mean(values(part,"vy_m_s"))
    # Exact P-only identities are expected, so correlations are descriptive, not independent evidence.
    e=values(rows,"altitude_error_m");vz=values(rows,"vz_m_s");thrust=values(rows,"command_thrust_normalized")
    hover_e=values(hover,"altitude_error_m");hover_vz=values(hover,"vz_m_s")

    t=values(rows,"elapsed_s")
    svg_timeseries(OUT/"first_custom_flight_altitude.svg","Altitude error and vertical velocity",[
        ("altitude [m]",[("actual",list(zip(t,values(rows,"z_m"))),"#2563eb"),("reference",list(zip(t,values(rows,"reference_z_m"))),"#111827")]),
        ("error / vz",[("error ref-z",list(zip(t,e)),"#dc2626"),("vz",list(zip(t,vz)),"#059669")])])
    xy_svg(OUT/"first_custom_flight_xy.svg",rows)
    at=[a["t"] for a in accelerations]
    svg_timeseries(OUT/"first_custom_flight_attitude_acceleration.svg","Attitude and 0.4 s local-regression acceleration",[
        ("attitude [rad]",[("roll",list(zip(t,values(rows,"roll_rad"))),"#dc2626"),("pitch",list(zip(t,values(rows,"pitch_rad"))),"#2563eb")]),
        ("body accel [m/s2]",[("ax body",list(zip(at,[a["ax_body"] for a in accelerations])),"#2563eb"),("ay body",list(zip(at,[a["ay_body"] for a in accelerations])),"#dc2626")])])

    def table_row(name,m):return f'| {name} | {m["samples"]} | {m["duration"]:.3f} | {m["rmse"]:.5f} | {m["mean_error"]:+.5f} | {m["max_error"]:.5f} | {m["mean_vz"]:+.5f} | {m["z_range"][0]:.4f}–{m["z_range"][1]:.4f} | {m["reference_range"][0]:.4f}–{m["reference_range"][1]:.4f} |'
    window_rows='\n'.join(f'| {b:.1f}–{b+.5:.1f} | {n} | {z:.4f} | {er:+.4f} | {v:+.4f} | {sl:+.4f} | {th:.4f} |' for b,n,z,er,v,sl,th in windows)
    text=f'''# First custom Cascaded PID flight diagnostics

Source: `{RUN.relative_to(ROOT)}`. Analysis uses all 207 logged command rows; the safety snapshot at 0.50326 m displacement is discussed separately because it was intentionally persisted before shutdown rather than written as a normal command row.

## TAKEOFF versus HOVER

Error is `reference - actual`; negative error means the vehicle is above reference.

| phase | samples | duration (s) | altitude RMSE (m) | mean error (m) | max abs error (m) | mean vz (m/s) | actual z range (m) | reference z range (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
{table_row("TAKEOFF",metrics["TAKEOFF"])}
{table_row("HOVER",metrics["HOVER"])}

Takeoff had the larger RMSE ({metrics["TAKEOFF"]["rmse"]:.5f} m versus {metrics["HOVER"]["rmse"]:.5f} m) and maximum error. The takeoff ramp lagged initially, then the vehicle overshot: actual altitude reached 0.3194 m while the reference ended near 0.1980 m. HOVER began during this overshoot, so its aggregate negative mean error ({metrics["HOVER"]["mean_error"]:+.5f} m) does not represent a settled offset.

![Altitude response](first_custom_flight_altitude.svg)

## Hover time windows

| seconds after HOVER start | samples | mean z (m) | mean error (m) | mean vz (m/s) | fitted z slope (m/s) | mean thrust |
|---|---:|---:|---:|---:|---:|---:|
{window_rows}

The 2.14 s hover record does not establish steady state. Altitude descended from the takeoff overshoot throughout nearly the whole interval. Mean error changed from -0.1159 m in the first 0.5 s to +0.0491 m in the final partial window; thus altitude did not remain consistently below reference. At termination z was 0.1465 m versus the 0.2000 m reference, and vz was -0.0669 m/s: it was below reference and still descending. There is no defensible steady-state offset estimate from this interval.

## Error, vertical velocity, thrust, and P contribution

Over the entire run, error–vz correlation was {correlation(e,vz):+.3f}; during HOVER it was {correlation(hover_e,hover_vz):+.3f}. Error–thrust correlation was {correlation(e,thrust):+.6f}. The last value is mechanically expected, not independent evidence: with unsaturated P-only control, `thrust = 0.425 + 0.1*error`, so altitude P and thrust are exact affine transforms of error.

Observations: takeoff showed lag followed by substantial overshoot; hover showed a continuing descent through the reference; no altitude or thrust command saturated. Hypotheses consistent with those observations include missing vertical-velocity damping and transient reference-tracking lag. An equilibrium-thrust mismatch may also contribute, but this flight does not isolate it. The data do not identify insufficient P gain: increasing P could reduce static error but could also worsen the observed overshoot without damping. Causal gain selection is not justified from this single short trajectory.

## Horizontal drift

The last logged displacement vector from the captured origin was ({dx:+.4f}, {dy:+.4f}) m, magnitude {math.hypot(dx,dy):.4f} m at {math.degrees(math.atan2(dy,dx)):.1f} degrees. The subsequent safety snapshot was 0.50326 m. Mean velocity vectors were TAKEOFF ({mean_velocity(takeoff)[0]:+.4f}, {mean_velocity(takeoff)[1]:+.4f}) m/s and HOVER ({mean_velocity(hover)[0]:+.4f}, {mean_velocity(hover)[1]:+.4f}) m/s; whole-run mean was ({mean_velocity(rows)[0]:+.4f}, {mean_velocity(rows)[1]:+.4f}) m/s.

The direct displacement was {math.hypot(dx,dy)/path:.3f} of the sampled XY path length, and {aligned*100:.1f}% of velocity samples projected in the final displacement direction. Drift was therefore predominantly toward negative X and negative Y, though not perfectly straight. Mean speed rose from {mean(values(takeoff,"horizontal_speed_m_s")):.4f} m/s in TAKEOFF to {mean(values(hover,"horizontal_speed_m_s")):.4f} m/s in HOVER. This is consistent with XY feedback being disabled: no control term attempted to return position or damp horizontal velocity. It does not establish the source of the drift; trim, small attitude bias, takeoff transients, floor/Flow Deck effects, and estimator behavior remain candidates.

![XY response](first_custom_flight_xy.svg)

## Attitude and estimated horizontal acceleration

Acceleration was estimated from local least-squares slopes of vx/vy over centered 0.4 s windows (typically about 20 samples), then rotated into the measured-yaw body frame. This suppresses pointwise differentiation noise but also smooths and phase-shifts short dynamics. Zero-lag correlations were roll versus body-Y acceleration {corr_roll_ay:+.3f}, and pitch versus body-X acceleration {corr_pitch_ax:+.3f}. Varying the total regression window from 0.3 to 0.8 s kept these correlations in approximately [-0.78, -0.63] and [-0.45, -0.32], respectively, so their signs are not a single-window artifact.

The roll/body-Y negative sign agrees with the controller mapping `roll_ref = -atan2(ay_body,g)`. The pitch/body-X result is opposite the controller's abstract `pitch_ref = atan2(ax_body,g)` mapping, but recorded pitch uses the Crazyflie legacy pitch convention and the command path performs a pitch sign conversion. This short closed-loop record cannot cleanly separate that coordinate convention, response lag, and coupled motion; it is not sufficient to identify a physical pitch-to-acceleration coefficient.

Maximum roll and pitch were only 0.0246 and 0.0680 rad, so a small persistent tilt or trim error could plausibly explain part of the accumulated drift. The trajectory cannot separate that effect from Flow Deck noise/filtering, aerodynamic transients, estimator lag, or yaw-coordinate coupling. Raw numerical acceleration is not used.

![Attitude and acceleration](first_custom_flight_attitude_acceleration.svg)

## Recommendation

**B — enable bounded XY position/velocity feedback first.** The direct reason the experiment ended was the horizontal boundary, not attitude, altitude, or command saturation. Bounded attitude response and zero rate saturation suggest enough margin to test a conservative XY loop, while uncontrolled mean horizontal velocity approximately doubled from TAKEOFF to HOVER and prevented collecting a longer altitude record. Adding XY control is therefore the most direct way to extend diagnostic time for subsequent altitude work.

Altitude dynamics still merit the next priority after horizontal containment: the overshoot and continuing descent strongly motivate vertical-velocity damping before increasing altitude P. This recommendation does not authorize gains or a flight and does not claim XY control will remove all drift.

## Limitations

- One 4.12 s run, only 2.14 s of HOVER, no landing, and no repeated trials.
- The HOVER phase began in a takeoff overshoot and never reached demonstrated equilibrium.
- P contribution and thrust are algebraically coupled, so their correlation with error is not causal evidence.
- Flow Deck estimates are filtered, serially correlated, and may degrade with surface, height, or motion.
- Local-regression acceleration is smoother than raw differentiation but remains an estimator-derived approximation.
- XY control was disabled, so this dataset cannot predict closed-loop XY stability or select XY gains.
'''
    (OUT/"first_custom_flight_diagnostics.md").write_text(text)


if __name__ == "__main__": main()
