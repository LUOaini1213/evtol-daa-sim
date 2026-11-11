# Historical results (source `0e85663`)

The files in [`0e85663/`](0e85663/) are the 6,000-encounter evaluation and the 29-case
mutation gate of
[source `0e8566360e02e7d2c5afb45018578ea80c05f13a`](https://github.com/LUOaini1213/evtol-daa-sim/tree/0e8566360e02e7d2c5afb45018578ea80c05f13a),
before the commanded-altitude correction described in
[ALTITUDE_CONSTRAINTS.md](../ALTITUDE_CONSTRAINTS.md). The README tables now come
from a fresh run of the current controller; these files are kept byte-for-byte so
the two can be compared.

| file | what it is |
|---|---|
| `0e85663/summary.json`, `sweep.csv`, `fail_cases.csv` | evaluation output of that source |
| `0e85663/soc_curve.png`, `lead_time_hist.png` | its figures |
| `0e85663/mutation_report.json` | its 29/29 mutation gate |
| `0e85663/mutation_report_first_run.json` | the first gate of the initial implementation (24/25) |
| `manifest.json` | SHA-256 of each file above, the source commit and its line-ending policy |
| `altitude_constraints_review.json` | the local validation receipt written for the altitude correction |

The mutation gate's log archive is not in the working tree: its logs contain
absolute paths of the machine that ran it. It stays in Git history at
`0e85663:results/mutation_evidence.zip`, and
`python scripts/check_archived_results.py` reads it from there and checks its
SHA-256 and all 90 member hashes. That script needs the full Git history; after a
shallow clone run `git fetch --unshallow` first.

How the historical hashes are reconstructed (raw checkout bytes, per-file LF/CRLF
policy) is described in [PROVENANCE.md](../PROVENANCE.md).
