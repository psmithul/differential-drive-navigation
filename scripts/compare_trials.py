import argparse
import csv
import json
import math
from pathlib import Path
import random
import statistics


def read_trials(path):
    trials = {}
    with Path(path).open(newline="") as stream:
        for row in csv.DictReader(stream):
            key = row["scenario"], row["policy"], int(row["seed"])
            if key in trials:
                raise ValueError(f"Duplicate trial: {key}")
            if row["success"] not in ("True", "False"):
                raise ValueError(f"Invalid success value: {key}")
            if (row["success"] == "True") != (row["outcome"] == "success"):
                raise ValueError(f"Success and outcome disagree: {key}")
            if int(row["collisions"]) != int(row["outcome"] == "collision"):
                raise ValueError(f"Collision and outcome disagree: {key}")
            for field in ("position_rmse_m", "elapsed_s"):
                if not math.isfinite(float(row[field])) or float(row[field]) < 0:
                    raise ValueError(f"Invalid {field}: {key}")
            trials[key] = row
    if not trials:
        raise ValueError("No trials to compare")
    return trials


def compare(before, after):
    if before.keys() != after.keys():
        raise ValueError("Both runs must contain exactly the same scenario, policy, and seed keys")
    groups = sorted({key[:2] for key in before})
    output = []
    for scenario, policy in groups:
        keys = sorted(key for key in before if key[:2] == (scenario, policy))
        changes = [int(after[k]["success"] == "True") - int(before[k]["success"] == "True") for k in keys]
        rng = random.Random(202511)
        bootstrap = sorted(statistics.mean(rng.choices(changes, k=len(keys))) for _ in range(4000))
        output.append({
            "scenario": scenario, "policy": policy, "pairs": len(keys),
            "before_successes": sum(before[k]["success"] == "True" for k in keys),
            "after_successes": sum(after[k]["success"] == "True" for k in keys),
            "gained_successes": changes.count(1), "lost_successes": changes.count(-1),
            "success_rate_change": statistics.mean(changes),
            "paired_bootstrap_ci95": [bootstrap[99], bootstrap[3899]],
            "before_collisions": sum(int(before[k]["collisions"]) for k in keys),
            "after_collisions": sum(int(after[k]["collisions"]) for k in keys),
            "mean_position_rmse_change_m": statistics.mean(
                float(after[k]["position_rmse_m"]) - float(before[k]["position_rmse_m"]) for k in keys),
        })
    return output


def main():
    parser = argparse.ArgumentParser(description="Compare two navigation runs using matched seeds")
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(read_trials(args.before), read_trials(args.after))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    for row in result:
        print(f"{row['scenario']:10} {row['policy']:8} "
              f"{row['before_successes']} -> {row['after_successes']} successes / {row['pairs']}")


if __name__ == "__main__":
    main()
