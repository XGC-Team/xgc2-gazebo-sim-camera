#!/usr/bin/env python3
"""Create an XGC2 trusted build-artifact manifest for Debian outputs."""
import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def field(path, name):
    return subprocess.check_output(["dpkg-deb", "-f", str(path), name], text=True).strip()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build")
    for option in ("deb-dir", "output-dir", "product", "product-version", "distribution", "architecture", "source-sha", "ci-run-id", "ci-workflow", "ci-workflow-ref"):
        build.add_argument("--" + option, required=True)
    args = parser.parse_args()
    debs = sorted(Path(args.deb_dir).glob("*.deb"))
    if not debs:
        raise SystemExit("no Debian artifacts found")
    entries = []
    for deb in debs:
        architecture = field(deb, "Architecture")
        if architecture not in (args.architecture, "all"):
            raise SystemExit("artifact architecture mismatch: " + deb.name)
        entries.append({
            "file": deb.name, "package": field(deb, "Package"),
            "version": field(deb, "Version"), "architecture": architecture,
            "sha256": sha256(deb), "size": deb.stat().st_size,
        })
    payload = {
        "schema": "xgc2.build-artifact.v1", "product": args.product,
        "source_sha": args.source_sha, "version": args.product_version,
        "distribution": args.distribution, "architecture": args.architecture,
        "ci": {"run_id": str(args.ci_run_id), "workflow": args.ci_workflow, "workflow_ref": args.ci_workflow_ref},
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "debs": entries,
    }
    output = Path(args.output_dir) / f"{args.product}_{args.distribution}_{args.architecture}.build.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
