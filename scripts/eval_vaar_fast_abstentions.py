#!/usr/bin/env python3
"""Count and validate always-on VA fast-marginal abstention telemetry."""
from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path


START_RX = re.compile(
    r"\[vaar-fast-telemetry\]\s+version=(\d+)\s+enabled=(\d+)\s+benchmark=(\d+)"
)
EVENT_RX = re.compile(
    r"\[vaar-fast-abstain\].*?why=(\S+)\s+calls=(\d+)\s+abstain=(\d+)\s+"
    r"nuisance_not_psd=(\d+)"
)
SUMMARY_RX = re.compile(
    r"\[vaar-fast-summary\]\s+version=(\d+)\s+calls=(\d+)\s+ok=(\d+)\s+"
    r"abstain=(\d+)\s+nuisance_not_psd=(\d+)"
)


def parse_log(path: Path) -> dict[str, object]:
    text = path.read_text(errors="ignore")
    starts = START_RX.findall(text)
    if len(starts) != 1:
        raise ValueError(
            f"{path}: expected exactly one [vaar-fast-telemetry] marker, got {len(starts)}"
        )
    version, enabled, benchmark = map(int, starts[0])
    if version != 1 or enabled != 1:
        raise ValueError(f"{path}: unsupported or disabled telemetry marker")

    reasons: Counter[str] = Counter()
    previous_calls = 0
    nuisance_not_psd = 0
    events = EVENT_RX.findall(text)
    for event_index, (reason, calls_s, abstain_s, nuisance_s) in enumerate(events, 1):
        calls = int(calls_s)
        abstain = int(abstain_s)
        reported_nuisance = int(nuisance_s)
        if calls <= previous_calls:
            raise ValueError(f"{path}: non-increasing calls counter at abstention {event_index}")
        if abstain != event_index:
            raise ValueError(
                f"{path}: abstain counter {abstain} does not match event {event_index}"
            )
        reasons[reason] += 1
        if reason == "nuisance_not_psd":
            nuisance_not_psd += 1
        if reported_nuisance != nuisance_not_psd:
            raise ValueError(
                f"{path}: nuisance_not_psd counter {reported_nuisance} does not match "
                f"observed count {nuisance_not_psd}"
            )
        previous_calls = calls

    summaries = SUMMARY_RX.findall(text)
    if len(summaries) > 1:
        raise ValueError(f"{path}: expected at most one [vaar-fast-summary] line")
    summary = None
    if summaries:
        s_version, calls, successes, abstentions, reported_nuisance = map(int, summaries[0])
        if s_version != version:
            raise ValueError(f"{path}: telemetry version mismatch in summary")
        if calls != successes + abstentions:
            raise ValueError(f"{path}: summary calls != ok + abstain")
        if abstentions != len(events):
            raise ValueError(f"{path}: summary abstain count does not match event count")
        if reported_nuisance != nuisance_not_psd:
            raise ValueError(f"{path}: summary nuisance_not_psd count does not match events")
        if previous_calls > calls:
            raise ValueError(f"{path}: event calls counter exceeds summary calls")
        summary = {
            "calls": calls,
            "ok": successes,
            "abstain": abstentions,
            "nuisance_not_psd": reported_nuisance,
        }

    return {
        "log": str(path),
        "telemetry_version": version,
        "benchmark": bool(benchmark),
        "abstention_events": len(events),
        "nuisance_not_psd": nuisance_not_psd,
        "reasons": dict(sorted(reasons.items())),
        "lifetime_summary": summary,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate and count [vaar-fast-abstain] events without ground truth"
    )
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    try:
        rows = [parse_log(path) for path in args.logs]
    except (OSError, ValueError) as exc:
        raise SystemExit(f"REJECT: {exc}") from exc

    if args.json:
        print(json.dumps(rows, indent=2, sort_keys=True))
        return 0

    for row in rows:
        summary = row["lifetime_summary"]
        print(f"log: {row['log']}")
        print(
            f"benchmark={int(row['benchmark'])} "
            f"abstentions={row['abstention_events']} "
            f"nuisance_not_psd={row['nuisance_not_psd']}"
        )
        reasons = row["reasons"]
        print("reasons: " + (", ".join(f"{key}={value}" for key, value in reasons.items())
                              if reasons else "none"))
        if summary is None:
            print("lifetime_summary: unavailable (for example, external SIGINT)")
        else:
            print(
                "lifetime_summary: "
                f"calls={summary['calls']} ok={summary['ok']} abstain={summary['abstain']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
