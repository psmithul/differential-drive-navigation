import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import platform

import numpy as np

from .robot import SCENARIOS
from .simulation import run, summarize, TRACE_COLUMNS
from .world import World


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("Must be positive")
    return number


def main():
    parser = argparse.ArgumentParser(description="Differential-drive navigation simulation")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Run one trial and save its trajectory")
    demo.add_argument("--scenario", choices=SCENARIOS, default="nominal")
    demo.add_argument("--policy", choices=("fixed", "adaptive"), default="adaptive")
    benchmark = commands.add_parser("benchmark", help="Compare both controllers on paired seeds")
    benchmark.add_argument("--trials", type=positive_int, default=50)
    benchmark.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    for command in (demo, benchmark):
        command.add_argument("--seed", type=int, default=0)
        command.add_argument("--output", type=Path, default=Path("results"))
        command.add_argument("--max-time", type=float, default=120.0)
    args = parser.parse_args()
    if args.seed < 0:
        parser.error("Seed must be nonnegative")
    if not np.isfinite(args.max_time) or args.max_time < 0.1:
        parser.error("max-time must be finite and at least 0.1 seconds")
    args.output.mkdir(parents=True, exist_ok=True)
    if args.command == "demo":
        from .plots import plot_run
        result, history, path = run(args.seed, args.scenario, args.policy, args.max_time, trace=True)
        np.savetxt(args.output / "trajectory.csv", history, delimiter=",", header=",".join(TRACE_COLUMNS), comments="")
        (args.output / "trial.json").write_text(json.dumps(asdict(result), indent=2) + "\n")
        plot_run(World(), history, path, result, args.output / "trajectory.png")
        print(json.dumps(asdict(result), indent=2))
    else:
        from .plots import plot_summary
        results = []
        for scenario in dict.fromkeys(args.scenarios):
            for policy in ("fixed", "adaptive"):
                for seed in range(args.seed, args.seed + args.trials):
                    result, _, _ = run(seed, scenario, policy, args.max_time)
                    results.append(result)
                print(f"{scenario}: {policy}, {args.trials} trials", flush=True)
        with (args.output / "trials.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=asdict(results[0]))
            writer.writeheader()
            writer.writerows(asdict(r) for r in results)
        summary = summarize(results)
        metadata = {"python": platform.python_version(), "numpy": np.__version__, "seed": args.seed,
                    "trials_per_group": args.trials, "dt_s": 0.1, "max_time_s": args.max_time,
                    "scenarios": {name: asdict(SCENARIOS[name]) for name in dict.fromkeys(args.scenarios)}}
        (args.output / "summary.json").write_text(json.dumps({"settings": metadata, "groups": summary}, indent=2) + "\n")
        plot_summary(summary, args.output / "comparison.png")
        for row in summary:
            print(f"{row['scenario']:10} {row['policy']:8} success={row['success_rate']:.0%} "
                  f"collisions={row['collisions']} RMSE={row['mean_position_rmse_m']:.3f} m")


if __name__ == "__main__":
    main()
