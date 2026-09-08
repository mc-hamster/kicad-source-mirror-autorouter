# Three small online boards: routing benchmark

**Historical baseline.** The latest [production host-validation/repair results](PARITY-CLOSURE-CHECKLIST.md)
complete all three boards with zero new KiCad DRC violations. The 555 still fails
the via-count quality gate. Keep the original measurements below as before/after
evidence, not as the current build's performance.

Date: 2026-09-07. This is a measured comparison, not a claim of parity.
No production routing algorithm was changed for these measurements.

## Sources

All three designs are from [niyathiarun/Basic-PCB-Projects](https://github.com/niyathiarun/Basic-PCB-Projects),
pinned at `e5cc029002b4a4d235bbfd185e011e9debdb73df`:

- [5 V regulator](https://github.com/niyathiarun/Basic-PCB-Projects/blob/e5cc029002b4a4d235bbfd185e011e9debdb73df/project%201/5V%20regulated%20power%20supply%20pcbdesign.kicad_pcb) — `project 1/5V regulated power supply pcbdesign.kicad_pcb`.
- [555 astable oscillator](https://github.com/niyathiarun/Basic-PCB-Projects/blob/e5cc029002b4a4d235bbfd185e011e9debdb73df/project%202/555%20Astable%20multivibrator.kicad_pcb) — `project 2/555 Astable multivibrator.kicad_pcb`.
- [Two-transistor astable oscillator](https://github.com/niyathiarun/Basic-PCB-Projects/blob/e5cc029002b4a4d235bbfd185e011e9debdb73df/project%203/astable%20vibrator%28B%29.kicad_pcb) — `project 3/astable vibrator(B).kicad_pcb`.

Download identities are recorded in [online-simple-boards.json](benchmarks/online-simple-boards.json).
No source license was found, so the third-party boards are downloaded into the
ignored build directory for local evaluation, not vendored into the repository.
The regulator is a routing fixture only, not an endorsement of its mains-safety design.

The reference is the **official [Freerouting v2.3.0 release](https://github.com/freerouting/freerouting/releases/tag/v2.3.0)**,
not a locally modified development build. Both the GitHub release SHA-256 and the
JAR manifest revision are verified:

- Revision: `2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`
- JAR SHA-256: `3cf18d608437740bc497db6b8ef5888e2e60a08de0def20691d1bad0c0e0ee24`
- Native: the current Release build of `qa_autorouter_parity`, using the same
  native pipeline as the PCB Editor. Binary hashes and commands are in each run's provenance.
- Hardware: this computer, Apple M1 Max.

## Method

1. Fetch original PCB files and the supplied BJT `.kicad_pro` without alteration.
   The other two designs do not supply matching project/rule files; both engines
   are measured using the same KiCad-resolved input defaults.
2. Remove unlocked tracks/vias in memory. Saved normalized inputs were checked
   to contain **zero tracks and zero vias**. Component placement, enabled copper
   stack and zones remain; zones are refilled before export and after routing.
3. Export that same normalized input to DSN for Freerouting. Route the native
   snapshot directly through our production pipeline; neither engine uses a
   net/connection slice.
4. Run each engine serially, three repetitions per board, using four routing
   passes. Native retains eight attempts/request and 250,000 nodes/attempt.
   Each process has a 180-second safety timeout; no run timed out.
5. Disable final optimization in both engines to isolate routing. Fanout remains
   enabled. Freerouting uses one thread, a fresh private configuration, no inherited
   router environment overrides, and analytics disabled. Automatic neckdown is
   requested off; see the effective-width discrepancy below.
6. Apply/import the results into disposable KiCad boards, refill zones, rebuild
   connectivity, and run full KiCad DRC. Original PCB/project/rule hashes are
   checked again afterward. New DRC reports are fingerprinted against the input;
   unconnected-item reports are counted separately.
7. Save both output PCBs plus the normalized input. Reopen all six first-repeat
   routed outputs and verify actual connectivity and DRC category counts match
   the measurements. Copy supplied project/rule companions alongside saved boards.

**Timing:** table values are median *routing-core seconds*, excluding startup,
input loading, DSN/SES I/O, and host DRC. Native timing covers the pipeline
(including fanout). Freerouting timing sums its logged fanout and autoroute stage
durations, rounded by that engine to hundredths of a second. Missing reference
stage timings are not silently replaced with zero or process wall time. Raw
process wall timings are retained separately in provenance; do not compare
Java launch time against native core time. These are cold-process, small-board
measurements, not a warmed-JVM microbenchmark.

## Results

Paired columns are **Freerouting / native**. Connectivity and DRC outcomes, via
counts, and lengths were identical in all three repetitions.

| Board | Footprints / pads / nets | Initial missing connections | Core time, seconds | Remaining connections | New DRC reports | Vias |
|---|---|---:|---|---|---|---|
| 5 V regulator | 14 / 37 / 9 | 22 | 0.410 / 1.292 | 0 / 0 | 0 / 0 | 0 / 0 |
| 555 astable oscillator | 16 / 30 / 6 | 12 | 2.040 / 0.036 | 0 / 1 | 10 / 7 | 7 / 16 |
| Two-transistor astable oscillator | 10 / 22 / 6 | 16 | 0.290 / 0.017 | 0 / 0 | 0 / 0 | 0 / 0 |

The footprint count includes all PCB footprints, not just active circuit parts.
The 555 design has SMD components and a copper zone; the other two are through-hole designs.

| Board | Freerouting time range | Native time range | Freerouting / native track length |
|---|---|---|---|
| 5 V regulator | 0.410–0.450 s | 1.278–1.302 s | 319.203 / 337.810 mm |
| 555 astable oscillator | 1.860–2.050 s | 0.036–0.036 s | 184.248 / 103.599 mm |
| Two-transistor astable oscillator | 0.280–0.290 s | 0.017–0.019 s | 152.435 / 148.110 mm |

### Interpretation

- **Regulator:** both clean and electrically complete. Native core time is about
  3.15 times the reference, although still only 1.29 seconds on this tiny board.
- **BJT oscillator:** both clean and electrically complete. Native is faster on
  this easy case. This does not demonstrate scaling or general routing parity.
- **555 oscillator: native fails.** The worker reports 28/28 routing tasks and
  `worker_complete: true`, but KiCad still finds one missing connection plus
  seven dangling-via reports. The 0.036-second native time is therefore **not a
  successful routing speed result**. Native uses 16 vias versus seven in the
  reference. This is a small reproducible case for the known fanout/plane-model
  and false-completion gap.
- **555 reference result is electrically connected, not DRC-clean.** KiCad finds
  nine 0.150 mm tracks violating its 0.200 mm minimum width and one dangling via.
  The effective constraints across the DSN/SES path need investigation despite
  the explicit automatic-neckdown-off request. Do not call this a clean reference
  success or relax KiCad constraints to make the comparison pass.
- The test does **not** reproduce the minute-scale Arduino slowdown. It narrows
  the problem: very small THT cases work; a small SMD/zone case already exposes
  correctness and premature-stop problems. A larger scaling fixture is still
  necessary to diagnose the long-running searches.

The strict comparator passes the regulator and BJT board on all repeats and
fails the 555 board on every repeat. Final benchmark exit status is deliberately
nonzero because that failure is real.

## Reproduce

From the KiCad source checkout (not the protected reference checkout):

```sh
cmake --build build/autorouter --target qa_autorouter_parity -j4
python3 scripts/autorouter/benchmark_online_simple.py \
  --output-dir build/autorouter/online-simple-new-run --repeats 3
```

The output directory must be new. The script fetches checksum-pinned assets,
runs both engines serially, and produces `measurements.json`, `summary.json`,
per-process logs and all saved PCBs. A network connection and Java 25 are required.
No GUI automation or modification of an open user board is involved.

## Saved results on this computer

Permanent A/B fixtures are now preserved outside the build directory in
[autorouter-test-assets/online-simple](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/autorouter-test-assets/online-simple/README.md).
Each board has untouched originals, `unrouted.kicad_pcb`, `reference.kicad_pcb`
and `native.kicad_pcb`, project companions where supplied, and baseline evidence.
The local asset folder is Git-ignored and includes verified SHA-256 checksums.
Use copies for new runs; do not overwrite these baselines.

- **5 V regulator:** [unrouted input](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/regulated-5v/repeat-1/unrouted.kicad_pcb), [Freerouting result](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/regulated-5v/repeat-1/reference.kicad_pcb), [native result](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/regulated-5v/repeat-1/native.kicad_pcb).
- **555 astable oscillator:** [unrouted input](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/555-astable/repeat-1/unrouted.kicad_pcb), [Freerouting result](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/555-astable/repeat-1/reference.kicad_pcb), [native result](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/555-astable/repeat-1/native.kicad_pcb).
- **Two-transistor astable oscillator:** [unrouted input](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/bjt-astable/repeat-1/unrouted.kicad_pcb), [Freerouting result](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/bjt-astable/repeat-1/reference.kicad_pcb), [native result](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/runs/bjt-astable/repeat-1/native.kicad_pcb).

[Summary JSON](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/summary.json) · [All measurements](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/measurements.json) · [Saved-board reload checks](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/build/autorouter/online-simple-stable/reload-checks.json)

The earlier `build/autorouter/online-simple/` results used the pinned development
reference and are supplementary diagnostics. The official-release numbers in
this document come exclusively from `build/autorouter/online-simple-stable/`.
