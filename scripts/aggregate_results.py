#!/usr/bin/env python3
"""
aggregate_results.py

Combines per-run CSVs from run_nav2_experiments.py into a summary table:
mean +/- std of time-to-goal and path length, plus success rate, grouped
by planner. Prints the table and writes it to summary.csv.

Usage:
    python3 aggregate_results.py results_dwb.csv results_rpp.csv
    python3 aggregate_results.py results_dwb.csv results_rpp.csv --output summary.csv
"""

import argparse
import pandas as pd


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    df["success"] = df["success"].astype(str).str.lower().isin(["true", "1"])

    grouped = df.groupby("planner").agg(
        n_runs=("run_id", "count"),
        success_rate=("success", "mean"),
        time_mean_s=("time_to_goal_s", "mean"),
        time_std_s=("time_to_goal_s", "std"),
        path_mean_m=("path_length_m", "mean"),
        path_std_m=("path_length_m", "std"),
    ).reset_index()

    # Only average time/path over successful runs (failed runs don't have a
    # meaningful time-to-goal / path length). Comment this out if you'd
    # rather average over all runs regardless of outcome.
    successful = df[df["success"]]
    succ_grouped = successful.groupby("planner").agg(
        time_mean_s_success_only=("time_to_goal_s", "mean"),
        time_std_s_success_only=("time_to_goal_s", "std"),
        path_mean_m_success_only=("path_length_m", "mean"),
        path_std_m_success_only=("path_length_m", "std"),
    ).reset_index()

    merged = grouped.merge(succ_grouped, on="planner", how="left")
    return merged.round(3)


def main():
    parser = argparse.ArgumentParser(description="Aggregate Nav2 experiment CSVs.")
    parser.add_argument("csv_files", nargs="+", help="One or more per-run CSV files.")
    parser.add_argument("--output", default="summary.csv", help="Output summary CSV path.")
    args = parser.parse_args()

    frames = [pd.read_csv(f) for f in args.csv_files]
    df = pd.concat(frames, ignore_index=True)

    summary = summarize(df)
    print(summary.to_string(index=False))
    summary.to_csv(args.output, index=False)
    print(f"\nSummary written to {args.output}")


if __name__ == "__main__":
    main()
