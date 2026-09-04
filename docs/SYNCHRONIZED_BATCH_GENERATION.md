# Synchronized KS / REA / TTFF batch generation

## Purpose

A synchronized batch represents one common physical GNSS interval observed by multiple receiver boards or repeated receiver realizations. The batch generator therefore keeps the external environment fixed and varies only the deterministic random seed assigned to each realization.

The default batch contains:

- 8 KS realizations;
- 8 REA realizations;
- 8 TTFF realizations;
- 24 receiver logs in total.

All 24 runs receive the same GPS week/SOW start time and the same duration. The RINEX NAV input, receiver truth, sampling rate, elevation masks, atmosphere settings, REA cycle parameters, TTFF cycle parameters, and other common model settings come from one base simulator configuration.

A different realization index must **not** be implemented by shifting GPST, changing satellite geometry, modifying ephemeris, moving the receiver, or moving REA/TTFF event boundaries.

## Deterministic seed mapping

The batch tool uses the following mapping by default:

```text
KS_01   1001
...
KS_08   1008
REA_01  2001
...
REA_08  2008
TTFF_01 3001
...
TTFF_08 3008
```

The general rule is:

```text
seed = seed_base + scenario_index * seed_stride + (realization_index - 1)
```

where the scenario order is `KS`, `REA`, `TTFF`. The defaults are `seed_base=1001`, `seed_stride=1000`, and `count=8`.

The tool rejects overlapping scenario seed blocks and uint64 overflow. Seeds are fixed inputs: wall-clock time, process ID, thread scheduling, and launch order are not used to derive them.

A seed may influence only model behavior that is already explicitly seed-dependent, such as deterministic CN0 phase variation or stochastic receiver acquisition/reacquisition behavior. It does not authorize changes to real NAV/EPH/ION data or deterministic external truth.

## Command line

Linux example:

```bash
python3 tools/batch_generation/generate_batch.py \
  --simulator build/gnss-data-simulator \
  --config config/default_v1.json \
  --nav /path/to/real_broadcast_nav.rnx \
  --output-dir output/batch_001 \
  --week 2347 \
  --sow 436500
```

Windows example after a Release build:

```powershell
python tools\batch_generation\generate_batch.py `
  --simulator build\Release\gnss-data-simulator.exe `
  --config config\default_v1.json `
  --nav C:\data\real_broadcast_nav.rnx `
  --output-dir output\batch_001 `
  --week 2347 `
  --sow 436500
```

Useful options:

- `--count N`: realizations per scenario; default `8`.
- `--seed-base N`: seed for `KS_01`; default `1001`.
- `--seed-stride N`: offset between KS/REA/TTFF seed blocks; default `1000`.
- `--cn0-model FILE`: pass one calibrated CN0 model unchanged to every run.
- `--plan-only`: write the manifest and per-run effective configs without invoking the simulator.
- `--overwrite`: replace the planned per-realization directories and manifest.

## Output layout

Each realization has its own directory:

```text
batch_001/
  batch_manifest.json
  runs/
    KS_01/
      KS_01.log
      config.json
      scenario.json
      event_truth.csv
      observation_truth.csv
      solution_truth.csv
      run_manifest.json
    ...
    REA_01/
      ...
    ...
    TTFF_08/
      ...
```

The separate run directories are required because the simulator intentionally writes fixed-name truth/manifest sidecars beside each receiver log. Sharing one directory across 24 simulator invocations would overwrite those sidecars and destroy the evidence needed for synchronized comparison.

`batch_manifest.json` records the common GPST window, input SHA256 identities, deterministic seed schedule, effective-config hashes, receiver-log hashes, and per-run completion status. The simulator's own `run_manifest.json` remains the authoritative per-run record for the resolved configuration, simulator commit, pinned RTKLIB commit, real RINEX NAV identity, start time, and random seed.

## Reproducibility contract

For one supported reproducibility target, regenerating the same batch with the same:

- simulator build;
- pinned RTKLIB revision;
- base config bytes;
- real RINEX NAV bytes;
- optional CN0 model bytes;
- common GPST start;
- duration;
- seed schedule;

must reproduce the corresponding receiver logs and truth artifacts byte-for-byte.

Parallel or serial execution must not redefine this contract. The current V1 batch tool invokes runs serially, so launch scheduling cannot leak into output identity.

## CI regression

Normal Ubuntu and Windows pull-request CI runs `tests/integration/test_synchronized_batch.py` against the committed reduced real `BRD400DLR` RINEX 4.02 fixture. The test uses a short 1 Hz window rather than 8-hour production data, but it exercises the production batch tool and production simulator executable.

The regression generates two complete 24-run batches and verifies:

1. exactly 8 KS + 8 REA + 8 TTFF outputs;
2. one common start/end GPST and common physical configuration;
3. unique deterministic seed ranges;
4. identical real RINEX NAV identity across all runs;
5. identical REA signal transition timing across all 8 REA seeds;
6. identical TTFF power transition timing across all 8 TTFF seeds;
7. existing seed-dependent CN0 diversity on common KS observations;
8. unchanged satellite/receiver geometry, clocks, atmosphere, TGD/ISC and code-bias truth for those common observations;
9. byte-identical full rerun output for every corresponding seed.

The regression never modifies, retargets, interpolates, or fabricates navigation records.
