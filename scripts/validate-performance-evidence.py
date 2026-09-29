#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator, FormatChecker


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def sha256_bytes(contents: bytes) -> str:
    return "sha256:" + hashlib.sha256(contents).hexdigest()


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: validate-performance-evidence.py <performance-evidence-repo> <evidence-root>",
            file=sys.stderr,
        )
        return 2

    contract_root = Path(sys.argv[1]).resolve()
    evidence_root = Path(sys.argv[2]).resolve()
    sys.path.insert(0, str(contract_root / "scripts"))
    from validate_schema import (  # type: ignore[import-not-found]
        profile_validation_errors,
        validation_errors,
    )

    evidence_schema = load_json(contract_root / "schema" / "performance-evidence.schema.json")
    profile_schema = load_json(contract_root / "schema" / "measurement-profile.schema.json")
    evidence_validator = Draft202012Validator(evidence_schema, format_checker=FormatChecker())
    profile_validator = Draft202012Validator(profile_schema, format_checker=FormatChecker())

    failures: list[str] = []
    profiles = sorted((evidence_root / "profiles").glob("*.json"))
    canonical = sorted((evidence_root / "canonical").rglob("*.json"))
    if not profiles:
        failures.append("no bundled measurement profiles found")
    if not canonical:
        failures.append("no canonical Performance Evidence documents found")

    for path in profiles:
        errors = profile_validation_errors(profile_validator, load_json(path))
        if errors:
            failures.append(f"profile {path.relative_to(evidence_root)}:\n  - " + "\n  - ".join(errors))

    hashes: dict[tuple[str, str], Path] = {}
    documents: list[tuple[Path, dict[str, Any]]] = []
    for path in canonical:
        raw = path.read_bytes()
        document = json.loads(raw)
        errors = validation_errors(evidence_validator, document)
        if errors:
            failures.append(f"evidence {path.relative_to(evidence_root)}:\n  - " + "\n  - ".join(errors))
            continue
        documents.append((path, document))
        hashes[(document["source"]["revision"], sha256_bytes(raw))] = path

        for artifact in document.get("artifacts", []):
            artifact_path = (evidence_root / artifact["path"]).resolve()
            try:
                artifact_path.relative_to(evidence_root)
            except ValueError:
                failures.append(
                    f"{path.relative_to(evidence_root)} references artifact outside evidence root: {artifact['path']}"
                )
                continue
            if not artifact_path.is_file():
                failures.append(
                    f"{path.relative_to(evidence_root)} references missing artifact: {artifact['path']}"
                )
                continue
            actual_hash = sha256_bytes(artifact_path.read_bytes())
            if actual_hash != artifact["sha256"]:
                failures.append(
                    f"{path.relative_to(evidence_root)} artifact hash mismatch for {artifact['path']}: "
                    f"expected {artifact['sha256']}, got {actual_hash}"
                )

    for path, document in documents:
        baseline = document.get("baseline")
        if baseline is None:
            continue
        key = (baseline["source_revision"], baseline["evidence_hash"])
        if key not in hashes:
            failures.append(
                f"{path.relative_to(evidence_root)} references unavailable baseline "
                f"{baseline['source_revision']} {baseline['evidence_hash']}"
            )

    if failures:
        print("physics-engine Performance Evidence validation failed:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1

    print(
        f"physics-engine Performance Evidence validation passed "
        f"({len(profiles)} profiles, {len(canonical)} canonical documents)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
