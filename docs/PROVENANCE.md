# Provenance of the published numbers

## Current results (`results/`)

`results/summary.json` records the SHA-256 of every file the evaluation imported
(`daa/*.py`, `config/default.json`, `scripts/run_eval.py`), hashed as raw bytes.
`.gitattributes` forces LF line endings, so Windows and Linux checkouts hash the
same. The evaluator also rejects a source change during the calculation before it
writes any output. `--n` must be a positive integer; use explicit `--out` and
`--docs` directories to keep a previous evaluation while running another one.

`results/mutation_report.json` records the hashes of the sources, tests and
configuration copied into the unmutated tree, and the SHA-256 of the evidence
archive and of each of its members. `results/mutation_evidence.zip` holds the C++
build log, pytest log and JUnit file of the control run and of each planted bug.
Before packing, `scripts/mutation_check.py` replaces absolute paths of the machine
that ran the gate with placeholders (`<tmp>/`, `<repo>/`, `<python>/`, `<home>/`,
`pytest-of-<user>`); a leftover home-directory path invalidates the run. Extract
the archive beside the report to resolve the relative paths in it.

`python scripts/make_readme.py --check` (run in CI on Ubuntu and Windows) fails if:

- the generated blocks of README.md differ from what `results/` renders to;
- any source hash in `summary.json` or `mutation_report.json` differs from the
  current checkout, i.e. the published numbers no longer describe the code;
- the evidence archive or one of its members does not match the report;
- one of the hand-written Findings no longer holds for `summary.json`
  (each Finding is re-tested in `check_findings`).

`python scripts/check_no_local_paths.py` (also in CI) fails if any tracked file, or
any member of a tracked ZIP, contains a home-directory path or a `pytest-of-<name>`
temp folder.

## Historical results (`docs/historical/`)

The October 2026 reliability review fixed tangency cancellation, future-report
leakage at tracker initialisation, emergency candidate selection, full-horizon
altitude screening, paired-input validation and Unicode MSVC builds. Its
6,000-encounter run and 29-case mutation gate are kept, byte for byte, in
`docs/historical/0e85663/`. Earlier results remain in commit
`3162ee8874f53e9970e1bfe4b4a8fdab32b2a3b7`. Changed sampling eligibility changes
the accepted encounters and later random draws, so old and new encounter indices
must not be paired even though the seed is the same. The `3162ee8` run also used
NumPy 1.24.3, so its aggregates cannot be compared with later runs as pure
algorithm changes.

`python scripts/check_archived_results.py` checks the historical files against
`docs/historical/manifest.json` and re-derives their recorded source hashes from
the fixed commit `0e85663`. The two historical runs were made from checkouts with
mixed LF/CRLF bytes; `manifest.json` records the line ending of each source file so
the original raw bytes can be rebuilt from Git without rewriting any historical
hash. JSON/CSV artifacts are hashed with CRLF normalised to LF, binaries exactly.
The mutation evidence ZIP of that gate is read from the Git object
`0e85663:results/mutation_evidence.zip`, not from the working tree, because its
logs contain local paths. The script needs the full history (CI checks out with
`fetch-depth: 0`); after a shallow clone, run `git fetch --unshallow` first.
It also reports which current sources differ from the historical run (today:
`daa/sim.py`, the altitude-constrained controller).
