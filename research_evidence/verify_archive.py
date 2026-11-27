"""Verify index links and Phase 5.3 archive payload without extraction."""
import csv
import hashlib
from pathlib import Path
import re
import tarfile


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent


def digest(stream):
    h = hashlib.sha256()
    for block in iter(lambda: stream.read(1024*1024),b""):
        h.update(block)
    return h.hexdigest()


def table(name):
    with (ROOT/name).open(newline="", encoding="utf-8") as f:
        reader=csv.DictReader(f)
        rows=list(reader)
        assert all(None not in r and None not in r.values() for r in rows), name
    return rows


def main():
    idx=table("experiment_index.csv")
    ids={r["experiment_id"] for r in idx}
    assert len(ids)==len(idx)
    for p in table("parameter_registry.csv"):
        for evidence in p["supporting_experiment_ids"].split("|"):
            assert evidence=="none" or evidence in ids, evidence
    for r in idx:
        # Leave sealed paths untouched. Metadata is sufficient for indexing.
        if r["success_failure_status"]!="sealed_not_inspected":
            assert (REPO/r["original_log_path"]).exists(), r["experiment_id"]
    trials=table("regularization_trials.csv")
    assert len(trials)==40 and len({r["experiment_id"] for r in trials})==40
    assert {r["experiment_id"] for r in trials}<ids
    assert sum(r["selection_outcome"]=="screened_not_advanced" for r in trials)==12
    for r in trials:
        parts=sum(float(r[k]) for k in ("tracking_cost_per_s","control_effort_cost_per_s","input_increment_cost_per_s"))
        assert abs(parts-float(r["physical_cost_per_s"]))<1e-10, r["experiment_id"]
    for md in ROOT.glob("*.md"):
        for link in re.findall(r"\]\(([^)]+)\)",md.read_text()):
            if not link.startswith(("https://","http://")):
                assert (ROOT/link.split("#",1)[0]).exists(), (md.name,link)
    archive=ROOT/"archive"
    expected={line.split(maxsplit=1)[1].removeprefix("log/"):line.split(maxsplit=1)[0]
              for line in (archive/"phase53-original-files.sha256").read_text().splitlines()}
    package=archive/"phase53-complete.tar.gz"
    with package.open("rb") as stream:
        assert digest(stream)==(archive/"phase53-complete.tar.gz.sha256").read_text().split()[0]
    seen=set()
    with tarfile.open(package,"r|gz") as tf:
        for member in tf:
            if member.isfile():
                with tf.extractfile(member) as data:
                    assert digest(data)==expected[member.name], member.name
                seen.add(member.name)
    assert seen==set(expected)
    print(f"Verified {len(idx)} index entries, {len(trials)} tuning trials, and {len(seen)} archived files.")


if __name__=="__main__":
    main()
