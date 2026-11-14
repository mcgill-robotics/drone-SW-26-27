import argparse
import csv
import json
import math


def summarize(values):
    v = sorted(values)
    if not v or any(not math.isfinite(x) or x < 0 for x in v):
        raise ValueError("Require finite nonnegative latency samples")
    return dict(
        count=len(v),
        p50=v[math.ceil(0.5 * len(v)) - 1],
        p95=v[math.ceil(0.95 * len(v)) - 1],
        p99=v[math.ceil(0.99 * len(v)) - 1],
        max=v[-1],
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv")
    a = p.parse_args()
    with open(a.csv) as f:
        result = summarize([float(r["latency_ms"]) for r in csv.DictReader(f)])
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
