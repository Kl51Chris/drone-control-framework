# Measured altitude response: P-only versus PD

## Comparability

Both runs used the same altitude-position profile through the PD run: a 0.10 m/s ramp to 0.10 m during 0–1 s, followed by a fixed 0.10 m HOVER reference. P-only continued for 9.74 s through landing and timed out during touchdown confirmation. PD ended at 4.00 s during HOVER because horizontal displacement exceeded 0.50 m. Quantitative controller comparison is therefore limited to TAKEOFF and the common HOVER interval, elapsed 1–4 s.

One intentional implementation difference is relevant: the P-only log carried `vz_ref=0` during TAKEOFF, while PD carried the corrected +0.10 m/s ramp velocity. Both used `vz_ref=0` in HOVER. Thus the measured change includes both Kv=0.04 and its intended takeoff velocity reference; it is not an isolated damping-only experiment.

## Metrics

| Metric | P-only | PD, Kv=0.04 | Change |
|---|---:|---:|---:|
| TAKEOFF altitude RMSE | 0.03848 m | 0.04291 m | +11.5% |
| HOVER RMSE, common 1–4 s | 0.13480 m | 0.15714 m | +16.6% |
| HOVER RMSE, all available | 0.11865 m (4.0 s) | 0.15714 m (3.0 s) | not duration-matched |
| Maximum overshoot vs instantaneous reference | 0.20717 m | 0.21400 m | +0.00683 m |
| Peak altitude / time | 0.30734 m / 1.94 s | 0.31404 m / 2.32 s | — |
| Target-crossing time | 0.74 s | 0.68 s | -0.06 s |
| Vertical velocity at target crossing | 0.399 m/s | 0.418 m/s | +0.019 m/s |
| Thrust at target crossing | 0.42171 | 0.40824 | -0.01347 |
| Maximum ascent velocity before crossing | 0.399 m/s | 0.436 m/s | +0.037 m/s |
| Mean TAKEOFF thrust | 0.42476 | 0.42047 | -0.00429 |
| Mean common-HOVER thrust | 0.41417 | 0.41097 | -0.00320 |
| Commanded thrust range | 0.40428–0.42896 | 0.40164–0.43053 | — |
| Altitude saturation samples | 0 | 0 | none |

The P-only HOVER trace has a smoothed peak of 0.30660 m at 1.92 s and trough of 0.06436 m at 3.82 s, giving a 0.24224 m peak-to-trough span and an approximate 3.80 s period from one half-cycle. PD has one smoothed peak, 0.31311 m at 2.30 s. It was still descending at abort (last logged altitude 0.12568 m, vz −0.13007 m/s), so no PD trough, period, or settling time is observable.

PD crossed 0.10 m 0.06 s earlier and with slightly greater upward velocity. Its peak was 0.00670 m higher and occurred 0.38 s later. Although the velocity term substantially lowered thrust during ascent, the recorded vehicle retained upward momentum longer. By the end of the common window PD was approaching the target from above, but it had not settled and the run ended before a rebound could be observed.

## Authority and conclusion

Neither controller saturated. P-only used 0.40428–0.42896 thrust; PD used 0.40164–0.43053, leaving 0.01947 upward and 0.02164 downward authority inside the unchanged 0.380–0.450 bounds. The PD command therefore had adequate unused authority; its result was not caused by command clipping.

**Measured verdict: deterioration over the available common window, with incomplete evidence about eventual settling.** TAKEOFF RMSE increased 11.5%, duration-matched HOVER RMSE increased 16.6%, maximum overshoot increased 6.8 mm, and ascent velocity increased. Kv=0.04 did not produce measurable improvement in this flight. Because the PD run ended before completing HOVER or one oscillation, it remains inconclusive whether PD would reduce subsequent cycles or settling time. No gain or limit change follows automatically from this comparison.

Plots: [common-window comparison](common_window_comparison.svg) and [full-duration context](full_duration_context.svg).
