#!/usr/bin/env python3
"""Runs kitti_evaluation over sequences x seeds x scenarios x methods and aggregates the results.

usage: kitti_benchmark.py <benchmark.yaml> [--jobs N] [--results results.jsonl] [--binary path]

The benchmark file lists the dataset, the sequences, the seeds, the scenarios (initial error,
kidnapping, ...) and the methods; every scenario and method is a set of kitti_evaluation options
(see tools/config/benchmark.yaml). Results are appended to the JSON lines file (one line per run,
written by kitti_evaluation summary=), so an interrupted benchmark resumes where it stopped.
"""
import argparse
import itertools
import json
import os
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import yaml


def run_key(sequence, seed, scenario, method):
    return f"{sequence}/{seed}/{scenario}/{method}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("benchmark")
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--results", default="benchmark_results.jsonl")
    parser.add_argument("--binary", default=None)
    args = parser.parse_args()
    with open(args.benchmark) as f:
        bench = yaml.safe_load(f)
    binary = args.binary or bench["binary"]
    dataset = os.path.expanduser(bench["dataset"])
    common = bench.get("common", {})

    done = set()
    if os.path.exists(args.results):
        with open(args.results) as f:
            for line in f:
                d = json.loads(line)
                done.add(d.get("run", ""))

    runs = []
    for sequence, seed, (scenario, scenario_options), (method, method_options) in itertools.product(
        bench["sequences"], bench["seeds"], bench["scenarios"].items(), bench["methods"].items()
    ):
        key = run_key(sequence, seed, scenario, method)
        if key in done:
            continue
        options = {**common, **(scenario_options or {}), **(method_options or {}), "seed": seed}
        runs.append((key, str(sequence), options))
    print(f"{len(runs)} runs to do, {len(done)} done", file=sys.stderr)

    def execute(run):
        key, sequence, options = run
        summary = f"{args.results}.{abs(hash(key))}.tmp"
        command = [binary, dataset, sequence, f"summary={summary}"] + [
            f"{k}={v}" for k, v in options.items()
        ]
        completed = subprocess.run(command, capture_output=True, text=True)
        if completed.returncode != 0 or not os.path.exists(summary):
            print(f"FAILED {key}: {completed.stderr[-500:]}", file=sys.stderr)
            return None
        with open(summary) as f:
            result = json.loads(f.readline())
        os.remove(summary)
        result["run"] = key
        result["scenario"] = key.split("/")[2]
        result["method"] = key.split("/")[3]
        return result

    with ThreadPoolExecutor(args.jobs) as pool, open(args.results, "a") as out:
        for k, result in enumerate(pool.map(execute, runs)):
            if result is not None:
                out.write(json.dumps(result) + "\n")
                out.flush()
            print(f"{k + 1}/{len(runs)}", file=sys.stderr, end="\r")
    report(args.results, bench)


def report(results_path, bench):
    with open(results_path) as f:
        results = [json.loads(line) for line in f]
    methods = list(bench["methods"])
    scenarios = list(bench["scenarios"])

    def cell(values, fmt="{:.3f}"):
        if not values:
            return "-"
        return fmt.format(statistics.mean(values))

    print("\n## Per scenario (mean over sequences and seeds)\n")
    header = (
        "| scenario | method | runs | RMSE [m] | p95 [m] | max [m] | rot RMSE [deg] | converged | wrong mode "
        "| NEES | 95 % cov. | evals | accept. | distinct | ms mean / p95 / p99 |"
    )
    print(header)
    print("|" + "---|" * (header.count("|") - 1))
    for scenario in scenarios:
        for method in methods:
            rs = [r for r in results if r["scenario"] == scenario and r["method"] == method]
            if not rs:
                continue
            ok = sum(
                1
                for r in rs
                if (r["recovery_s"] >= 0 if r.get("kidnap_frame", 0) > 0 else r["convergence_s"] >= 0)
            )
            print(
                f"| {scenario} | {method} | {len(rs)} | {cell([r['rmse'] for r in rs])} "
                f"| {cell([r['p95'] for r in rs])} | {cell([r['max'] for r in rs])} "
                f"| {cell([r['rotation_rmse_deg'] for r in rs])} | {ok}/{len(rs)} "
                f"| {sum(1 for r in rs if r['final_error'] > 2.0)}/{len(rs)} "
                f"| {cell([r['nees'] for r in rs], '{:.2f}')} "
                f"| {cell([r['coverage95'] for r in rs], '{:.2f}')} "
                f"| {cell([r['evaluations'] for r in rs], '{:.1f}')} "
                f"| {cell([r['acceptance'] for r in rs], '{:.2f}')} "
                f"| {cell([r['distinct'] for r in rs], '{:.2f}')} "
                f"| {cell([r['ms_mean'] for r in rs], '{:.1f}')} / {cell([r['ms_p95'] for r in rs], '{:.1f}')}"
                f" / {cell([r['ms_p99'] for r in rs], '{:.1f}')} |"
            )
    print("\n## Translation RMSE [m] per sequence (first scenario, mean over seeds)\n")
    sequences = [str(s) for s in bench["sequences"]]
    print("| method | " + " | ".join(sequences) + " |")
    print("|---|" + "---|" * len(sequences))
    for method in methods:
        row = []
        for sequence in sequences:
            rs = [
                r
                for r in results
                if r["method"] == method and r["sequence"] == sequence and r["scenario"] == scenarios[0]
            ]
            row.append(cell([r["rmse"] for r in rs]))
        print(f"| {method} | " + " | ".join(row) + " |")


if __name__ == "__main__":
    main()
