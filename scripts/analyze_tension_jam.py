#!/usr/bin/env python3
import argparse
import re
from pathlib import Path


VALUE_RE = re.compile(r"TENSION_SAMPLE value=([-\d]+)")
BASELINE_RE = re.compile(r"Baseline updated to\s+(-?\d+)")
RAW_SAMPLES_RE = re.compile(r"raw_samples=\[([^\]]*)\]")
BASELINE_START_TEXT = "TENSION_UPDATE_BASELINE started in background thread"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze tension log with jam rules: median(3), "
            "jam_deviation_limit, jam_consecutive_frames."
        )
    )
    parser.add_argument("logfile", help="Path to tension_data.log")
    parser.add_argument(
        "--jam-deviation-limit",
        type=int,
        default=45000,
        help="Jam deviation threshold (default: 45000)",
    )
    parser.add_argument(
        "--jam-consecutive-frames",
        type=int,
        default=5,
        help="Consecutive jam frames to trigger event (default: 5)",
    )
    parser.add_argument(
        "--initial-baseline",
        type=int,
        default=None,
        help="Optional initial baseline before any 'Baseline updated to ...' line.",
    )
    parser.add_argument(
        "--no-skip-during-baseline-update",
        action="store_true",
        help="Do not skip jam check during baseline update thread window.",
    )
    parser.add_argument(
        "--recompute-baseline",
        action="store_true",
        help="Recompute baseline from raw_samples using cluster+EMA (new algorithm).",
    )
    parser.add_argument(
        "--baseline-cluster-eps",
        type=int,
        default=1200,
        help="Cluster epsilon for baseline recompute (default: 1200).",
    )
    parser.add_argument(
        "--baseline-cluster-min-points",
        type=int,
        default=4,
        help="Minimum primary cluster size (default: 4).",
    )
    parser.add_argument(
        "--baseline-ema-alpha",
        type=float,
        default=0.2,
        help="EMA alpha for baseline recompute (default: 0.2).",
    )
    parser.add_argument(
        "--max-normal-value",
        type=int,
        default=500000,
        help="Outlier cutoff abs(value) > max_normal_value (default: 500000).",
    )
    return parser.parse_args()


def median3(values):
    return sorted(values)[1]


def parse_int_list(s):
    s = s.strip()
    if not s:
        return []
    return [int(x.strip()) for x in s.split(",") if x.strip()]


def find_primary_cluster(values, eps):
    ordered = sorted(values)
    best_l, best_r = 0, -1
    l = 0
    for r in range(len(ordered)):
        while ordered[r] - ordered[l] > eps:
            l += 1
        if (r - l) > (best_r - best_l):
            best_l, best_r = l, r
    return ordered[best_l : best_r + 1]


def recompute_baseline_from_samples(samples, prev_baseline, args):
    abnormal = [v for v in samples if abs(v) > args.max_normal_value]
    if len(abnormal) / float(len(samples)) > 0.7:
        return None, "abnormal_ratio>70%"
    valid = [v for v in samples if abs(v) <= args.max_normal_value]
    if not valid:
        return None, "no_valid_samples"
    primary = find_primary_cluster(valid, args.baseline_cluster_eps)
    if len(primary) < args.baseline_cluster_min_points:
        return None, f"cluster_too_small({len(primary)})"
    center = int(round(sorted(primary)[len(primary) // 2]))
    if prev_baseline is None:
        return center, None
    baseline = int(round(
        args.baseline_ema_alpha * center
        + (1.0 - args.baseline_ema_alpha) * prev_baseline
    ))
    return baseline, None


def main():
    args = parse_args()
    lines = Path(args.logfile).read_text(errors="replace").splitlines()

    baseline = args.initial_baseline
    baseline_updating = False
    buf = []
    consecutive = 0
    sample_idx = 0
    jam_samples = 0
    jam_events = []
    baseline_updates_seen = 0
    baseline_updates_applied = 0
    baseline_updates_failed = 0

    for lineno, line in enumerate(lines, start=1):
        if BASELINE_START_TEXT in line:
            baseline_updating = True

        raw_match = RAW_SAMPLES_RE.search(line)
        if args.recompute_baseline and raw_match:
            baseline_updates_seen += 1
            raw_samples = parse_int_list(raw_match.group(1))
            if raw_samples:
                new_baseline, err = recompute_baseline_from_samples(
                    raw_samples, baseline, args
                )
                if err is None:
                    baseline = new_baseline
                    baseline_updates_applied += 1
                    baseline_updating = False
                    consecutive = 0
                else:
                    baseline_updates_failed += 1

        baseline_match = BASELINE_RE.search(line)
        if baseline_match and not args.recompute_baseline:
            baseline = int(baseline_match.group(1))
            baseline_updating = False
            consecutive = 0

        value_match = VALUE_RE.search(line)
        if not value_match:
            continue
        value = int(value_match.group(1))
        sample_idx += 1

        buf.append(value)
        if len(buf) > 3:
            buf.pop(0)
        if baseline is None or len(buf) < 3:
            continue

        smoothed = median3(buf)
        delta = smoothed - baseline
        if baseline < 0:
            fast_dev = max(0, -delta)
        else:
            fast_dev = max(0, delta)

        is_jam = fast_dev > args.jam_deviation_limit
        if baseline_updating and not args.no_skip_during_baseline_update:
            is_jam = False

        if is_jam:
            jam_samples += 1
            consecutive += 1
        else:
            consecutive = 0

        if consecutive == args.jam_consecutive_frames:
            jam_events.append(
                {
                    "line": lineno,
                    "sample_index": sample_idx,
                    "baseline": baseline,
                    "median3": smoothed,
                    "fast_dev": fast_dev,
                }
            )

    print(f"logfile: {args.logfile}")
    print(f"jam_deviation_limit: {args.jam_deviation_limit}")
    print(f"jam_consecutive_frames: {args.jam_consecutive_frames}")
    print(f"skip_during_baseline_update: {not args.no_skip_during_baseline_update}")
    print(f"recompute_baseline: {args.recompute_baseline}")
    if args.recompute_baseline:
        print(
            "baseline_recompute_params: "
            f"eps={args.baseline_cluster_eps}, "
            f"min_points={args.baseline_cluster_min_points}, "
            f"ema_alpha={args.baseline_ema_alpha}, "
            f"max_normal_value={args.max_normal_value}"
        )
        print(
            f"baseline_updates: seen={baseline_updates_seen}, "
            f"applied={baseline_updates_applied}, failed={baseline_updates_failed}"
        )
    print(f"jam_samples: {jam_samples}")
    print(f"jam_events: {len(jam_events)}")

    if jam_events:
        print("\ntrigger_events:")
        for i, event in enumerate(jam_events, start=1):
            print(
                f"{i}. line={event['line']} sample={event['sample_index']} "
                f"baseline={event['baseline']} median3={event['median3']} "
                f"fast_dev={event['fast_dev']}"
            )


if __name__ == "__main__":
    main()
